import asyncio
from collections.abc import Callable
from typing import Optional

from .config import (
    ARRIVAL_RADIUS_M,
    DEFAULT_ALTITUDES,
    DEFAULT_LANE_SPACING_M,
    DEFAULT_SEARCH_HEIGHT_M,
    DEFAULT_SEARCH_WIDTH_M,
    VEHICLES,
    WAYPOINT_TIMEOUT_SECONDS,
)

from .coverage import build_search_plan
from .events import SwarmEvent
from .models import (
    MissionState,
    MissionStatus,
    RouteSegment,
    UAVState,
)
from .recovery import (
    build_recovery_segment,
    choose_recovery_vehicle,
)
from .vehicle import (
    Vehicle,
    VehicleUnavailableError,
)


EventSink = Callable[[SwarmEvent], None]


class SwarmCoordinator:
    def __init__(
        self,
        event_sink: Optional[EventSink] = None,
        width_m: float = DEFAULT_SEARCH_WIDTH_M,
        height_m: float = DEFAULT_SEARCH_HEIGHT_M,
        lane_spacing_m: float = DEFAULT_LANE_SPACING_M,
    ):
        self._event_sink = (
            event_sink
            if event_sink is not None
            else lambda event: None
        )

        self.width_m = width_m
        self.height_m = height_m
        self.lane_spacing_m = lane_spacing_m

        self.status = MissionStatus()

        self.fleet = {
            name: Vehicle(
                name=name,
                connection_string=connection,
                cruise_altitude=DEFAULT_ALTITUDES[name],
                event_sink=self._event_sink,
            )
            for name, connection in VEHICLES.items()
        }

        self.origin_lat: Optional[float] = None
        self.origin_lon: Optional[float] = None

        self.plan: dict[str, RouteSegment] = {}

        self.queues: dict[
            str,
            asyncio.Queue[RouteSegment],
        ] = {}

        self.workers: dict[
            str,
            asyncio.Task,
        ] = {}

        self.current_segments: dict[
            str,
            Optional[RouteSegment],
        ] = {
            name: None
            for name in self.fleet
        }

        self.current_indices: dict[
            str,
            int,
        ] = {
            name: 0
            for name in self.fleet
        }

        self._stop_event = asyncio.Event()
        self._active_event = asyncio.Event()

        # Ensures two simultaneous failures cannot mutate
        # recovery queues at the same time.
        self._recovery_lock = asyncio.Lock()

    # ==================================================
    # EVENTS
    # ==================================================

    def _emit(
        self,
        event_type: str,
        message: str,
        **data,
    ):
        self._event_sink(
            SwarmEvent(
                event_type=event_type,
                source="SWARM",
                message=message,
                data=data,
            )
        )

    def _set_mission_state(
        self,
        state: MissionState,
    ):
        previous = self.status.state
        self.status.state = state

        self._emit(
            "mission.state_changed",
            f"{previous.value} -> {state.value}",
            previous_state=previous.value,
            state=state.value,
        )

    # ==================================================
    # PUBLIC CONTROL API
    # ==================================================

    async def wait_until_active(self):
        await self._active_event.wait()

    def inject_failure(
        self,
        vehicle_name: str,
        reason: str = "operator-injected failure",
    ):
        if vehicle_name not in self.fleet:
            raise KeyError(
                f"Unknown vehicle: {vehicle_name}"
            )

        vehicle = self.fleet[vehicle_name]

        if vehicle.status.state == UAVState.FAILED:
            return

        vehicle.request_failure(reason)

        self._emit(
            "mission.failure_injected",
            f"Failure injected into {vehicle_name}",
            vehicle=vehicle_name,
            reason=reason,
        )

    # ==================================================
    # MISSION
    # ==================================================

    async def run(self):
        self._set_mission_state(
            MissionState.INITIALIZING
        )

        await self._connect_fleet()
        await self._initialize_fleet()

        self._build_plan()

        self._set_mission_state(
            MissionState.READY
        )

        self._set_mission_state(
            MissionState.TAKING_OFF
        )

        await asyncio.gather(
            *[
                vehicle.launch()
                for vehicle in self.fleet.values()
            ]
        )

        self._emit(
            "mission.fleet_airborne",
            "All available UAVs airborne",
        )

        self._create_task_queues()

        self._set_mission_state(
            MissionState.ACTIVE
        )

        self._active_event.set()

        self._start_workers()

        await self._wait_for_search_completion()

        self._stop_event.set()

        await asyncio.gather(
            *self.workers.values(),
            return_exceptions=True,
        )

        if self.status.state == MissionState.ABORTED:
            await self._land_survivors()
            return

        self._set_mission_state(
            MissionState.LANDING
        )

        await self._land_survivors()

        self._set_mission_state(
            MissionState.COMPLETED
        )

        self._emit(
            "mission.completed",
            "Search objective completed",
            failed_uavs=list(
                self.status.failed_uavs
            ),
            recovery_count=(
                self.status.recovery_count
            ),
        )

    # ==================================================
    # INITIALIZATION
    # ==================================================

    async def _connect_fleet(self):
        await asyncio.gather(
            *[
                vehicle.connect()
                for vehicle in self.fleet.values()
            ]
        )

    async def _initialize_fleet(self):
        homes = await asyncio.gather(
            *[
                vehicle.initialize()
                for vehicle in self.fleet.values()
            ]
        )

        vehicle_names = list(
            self.fleet.keys()
        )

        home_map = dict(
            zip(vehicle_names, homes)
        )

        self.origin_lat, self.origin_lon = (
            home_map["UAV-1"]
        )

        self._emit(
            "mission.origin_ready",
            "Shared search origin established",
            latitude=self.origin_lat,
            longitude=self.origin_lon,
        )

    # ==================================================
    # PLANNING
    # ==================================================

    def _build_plan(self):
        self.plan = build_search_plan(
            width_m=self.width_m,
            height_m=self.height_m,
            lane_spacing_m=self.lane_spacing_m,
            vehicle_names=list(
                self.fleet.keys()
            ),
        )

        self.status.total_waypoints = sum(
            len(segment.waypoints)
            for segment in self.plan.values()
        )

        for name, segment in self.plan.items():
            self._emit(
                "mission.route_assigned",
                (
                    f"{name} assigned "
                    f"{len(segment.waypoints)} waypoints"
                ),
                vehicle=name,
                segment_id=segment.segment_id,
                waypoint_count=len(
                    segment.waypoints
                ),
            )

    def _create_task_queues(self):
        self.queues = {
            name: asyncio.Queue()
            for name in self.fleet
        }

        for name, segment in self.plan.items():
            self.queues[name].put_nowait(
                segment
            )

    # ==================================================
    # WORKERS
    # ==================================================

    def _start_workers(self):
        self.workers = {
            name: asyncio.create_task(
                self._vehicle_worker(name)
            )
            for name in self.fleet
        }

    async def _vehicle_worker(
        self,
        name: str,
    ):
        vehicle = self.fleet[name]
        queue = self.queues[name]

        while not self._stop_event.is_set():

            # Failure while idle / between route segments.
            if (
                vehicle.failure_requested
                and vehicle.status.state
                != UAVState.FAILED
            ):
                await vehicle.fail(
                    vehicle.failure_reason
                    or "vehicle unavailable"
                )

                await self._recover_queued_work(
                    failed_vehicle=name,
                )

                return

            try:
                segment = await asyncio.wait_for(
                    queue.get(),
                    timeout=0.25,
                )

            except asyncio.TimeoutError:
                continue

            self.current_segments[name] = segment
            self.current_indices[name] = 0

            try:
                if segment.kind == "recovery":
                    vehicle.transition_to(
                        UAVState.RECOVERING
                    )
                else:
                    vehicle.transition_to(
                        UAVState.SEARCHING
                    )

                self._emit(
                    "mission.segment_started",
                    (
                        f"{name} started "
                        f"{segment.segment_id}"
                    ),
                    vehicle=name,
                    segment_id=segment.segment_id,
                    kind=segment.kind,
                )

                for index, waypoint in enumerate(
                    segment.waypoints
                ):
                    self.current_indices[name] = index

                    error = await vehicle.fly_to(
                        waypoint=waypoint,
                        origin_lat=self.origin_lat,
                        origin_lon=self.origin_lon,
                        arrival_radius=(
                            ARRIVAL_RADIUS_M
                        ),
                        timeout=(
                            WAYPOINT_TIMEOUT_SECONDS
                        ),
                    )

                    self.status.completed_waypoints += 1

                    self._emit(
                        "mission.waypoint_completed",
                        (
                            f"{name} completed "
                            f"waypoint {index + 1}/"
                            f"{len(segment.waypoints)}"
                        ),
                        vehicle=name,
                        segment_id=segment.segment_id,
                        waypoint_index=index,
                        error_m=error,
                    )

                vehicle.transition_to(
                    UAVState.AIRBORNE
                )

                self._emit(
                    "mission.segment_completed",
                    (
                        f"{name} completed "
                        f"{segment.segment_id}"
                    ),
                    vehicle=name,
                    segment_id=segment.segment_id,
                )

            except VehicleUnavailableError as exc:

                await vehicle.fail(
                    str(exc)
                )

                await self._recover_failure(
                    failed_vehicle=name,
                    segment=segment,
                    current_index=(
                        self.current_indices[name]
                    ),
                )

                return

            finally:
                self.current_segments[name] = None
                queue.task_done()

    # ==================================================
    # FAILURE RECOVERY
    # ==================================================

    async def _recover_failure(
        self,
        failed_vehicle: str,
        segment: RouteSegment,
        current_index: int,
    ):
        """
        Recover:
        1. unfinished part of the current segment
        2. any complete segments already queued behind it
        """

        recovery = build_recovery_segment(
            failed_vehicle=failed_vehicle,
            segment=segment,
            current_index=current_index,
        )

        segments: list[RouteSegment] = []

        if recovery.waypoints:
            segments.append(recovery)

        # Anything already waiting behind the active segment
        # must also survive this vehicle failure.
        queue = self.queues[failed_vehicle]

        while not queue.empty():
            queued_segment = queue.get_nowait()

            segments.append(
                queued_segment
            )

            queue.task_done()

        await self._assign_recovery_work(
            failed_vehicle,
            segments,
        )

    async def _recover_queued_work(
        self,
        failed_vehicle: str,
    ):
        """
        Vehicle failed while idle, before starting its next
        queued route segment.
        """

        queue = self.queues[failed_vehicle]
        segments: list[RouteSegment] = []

        while not queue.empty():
            segment = queue.get_nowait()
            segments.append(segment)
            queue.task_done()

        await self._assign_recovery_work(
            failed_vehicle,
            segments,
        )

    async def _assign_recovery_work(
        self,
        failed_vehicle: str,
        segments: list[RouteSegment],
    ):
        async with self._recovery_lock:

            if (
                failed_vehicle
                not in self.status.failed_uavs
            ):
                self.status.failed_uavs.append(
                    failed_vehicle
                )

            self._set_mission_state(
                MissionState.RECOVERING
            )

            self._emit(
                "mission.recovery_started",
                (
                    f"Recovering work from "
                    f"{failed_vehicle}"
                ),
                failed_vehicle=failed_vehicle,
                segment_count=len(segments),
            )

            for segment in segments:

                candidates = [
                    name
                    for name, vehicle
                    in self.fleet.items()
                    if (
                        name != failed_vehicle
                        and vehicle.status.healthy
                        and not vehicle.failure_requested
                        and vehicle.status.state
                        != UAVState.FAILED
                    )
                ]

                if not candidates:
                    self._set_mission_state(
                        MissionState.ABORTED
                    )

                    self._emit(
                        "mission.aborted",
                        (
                            "No healthy UAVs remain "
                            "for recovery"
                        ),
                    )

                    self._stop_event.set()
                    return

                queue_lengths = {
                    name: (
                        self.queues[name].qsize()
                        + (
                            1
                            if self.current_segments[
                                name
                            ] is not None
                            else 0
                        )
                    )
                    for name in candidates
                }

                target = choose_recovery_vehicle(
                    candidates=candidates,
                    queue_lengths=queue_lengths,
                )

                await self.queues[target].put(
                    segment
                )

                self.status.recovery_count += 1

                self._emit(
                    "mission.route_reassigned",
                    (
                        f"{segment.segment_id} "
                        f"reassigned to {target}"
                    ),
                    failed_vehicle=failed_vehicle,
                    recovery_vehicle=target,
                    segment_id=segment.segment_id,
                    waypoint_count=len(
                        segment.waypoints
                    ),
                )

            if self.status.state != MissionState.ABORTED:
                self._set_mission_state(
                    MissionState.ACTIVE
                )

    # ==================================================
    # COMPLETION
    # ==================================================

    async def _wait_for_search_completion(
        self,
    ):
        while True:

            if self.status.state == MissionState.ABORTED:
                return

            queued_work = any(
                not queue.empty()
                for queue in self.queues.values()
            )

            active_work = any(
                segment is not None
                for segment
                in self.current_segments.values()
            )

            if not queued_work and not active_work:
                return

            await asyncio.sleep(0.1)

    async def _land_survivors(self):
        survivors = [
            vehicle
            for vehicle in self.fleet.values()
            if (
                vehicle.status.state
                != UAVState.FAILED
            )
        ]

        await asyncio.gather(
            *[
                vehicle.land()
                for vehicle in survivors
            ],
            return_exceptions=True,
        )