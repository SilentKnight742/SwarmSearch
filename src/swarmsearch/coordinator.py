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
            for name, connection
            in VEHICLES.items()
        }

        self.origin_lat: Optional[float] = None
        self.origin_lon: Optional[float] = None

        self.plan: dict[
            str,
            RouteSegment,
        ] = {}

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

        if previous == state:
            return

        self.status.state = state

        self._emit(
            "mission.state_changed",
            (
                f"{previous.value} "
                f"-> {state.value}"
            ),
            previous_state=previous.value,
            state=state.value,
        )

    # ==================================================
    # PUBLIC API
    # ==================================================

    async def wait_until_active(self):
        await self._active_event.wait()

    def inject_failure(
        self,
        vehicle_name: str,
        reason: str = (
            "operator-injected failure"
        ),
    ):
        if vehicle_name not in self.fleet:
            raise KeyError(
                f"Unknown vehicle: "
                f"{vehicle_name}"
            )

        vehicle = self.fleet[
            vehicle_name
        ]

        if (
            vehicle.status.state
            in {
                UAVState.FAILED,
                UAVState.LANDED,
            }
        ):
            return

        vehicle.request_failure(
            reason
        )

        self._emit(
            "mission.failure_injected",
            (
                f"Failure injected "
                f"into {vehicle_name}"
            ),
            vehicle=vehicle_name,
            reason=reason,
            phase=vehicle.status.state.value,
        )

    # ==================================================
    # MAIN MISSION
    # ==================================================

    async def run(self):
        self._set_mission_state(
            MissionState.INITIALIZING
        )

        await self._connect_fleet()
        await self._initialize_fleet()

        if (
            self.status.state
            == MissionState.ABORTED
        ):
            return

        self._build_plan()

        self._create_task_queues()

        self._set_mission_state(
            MissionState.READY
        )

        self._set_mission_state(
            MissionState.TAKING_OFF
        )

        await asyncio.gather(
            *[
                self._launch_vehicle(
                    name,
                    vehicle,
                )
                for name, vehicle
                in self.fleet.items()
                if (
                    vehicle.status.state
                    != UAVState.FAILED
                )
            ],
            return_exceptions=True,
        )

        if (
            self.status.state
            == MissionState.ABORTED
        ):
            await self._land_survivors()
            return

        healthy_airborne = [
            vehicle
            for vehicle
            in self.fleet.values()
            if (
                vehicle.status.healthy
                and vehicle.status.state
                == UAVState.AIRBORNE
            )
        ]

        if not healthy_airborne:
            self._set_mission_state(
                MissionState.ABORTED
            )

            self._emit(
                "mission.aborted",
                (
                    "No healthy UAVs "
                    "became airborne"
                ),
            )

            return

        self._emit(
            "mission.fleet_airborne",
            (
                f"{len(healthy_airborne)} "
                f"healthy UAV(s) airborne"
            ),
            healthy_count=(
                len(healthy_airborne)
            ),
        )

        self._set_mission_state(
            MissionState.ACTIVE
        )

        self._active_event.set()

        self._start_workers()

        await self._wait_for_search_completion()

        self._stop_event.set()

        if self.workers:
            await asyncio.gather(
                *self.workers.values(),
                return_exceptions=True,
            )

        if (
            self.status.state
            == MissionState.ABORTED
        ):
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
    # CONNECTION / INITIALIZATION
    # ==================================================

    async def _connect_fleet(self):
        results = await asyncio.gather(
            *[
                self._connect_vehicle(
                    name,
                    vehicle,
                )
                for name, vehicle
                in self.fleet.items()
            ],
            return_exceptions=True,
        )

        if all(
            isinstance(
                result,
                Exception,
            )
            for result in results
        ):
            self._set_mission_state(
                MissionState.ABORTED
            )

            self._emit(
                "mission.aborted",
                (
                    "No UAVs connected "
                    "successfully"
                ),
            )

    async def _connect_vehicle(
        self,
        name: str,
        vehicle: Vehicle,
    ):
        try:
            await vehicle.connect()

        except VehicleUnavailableError as exc:
            await vehicle.fail(
                str(exc)
            )

            self._record_failed_vehicle(
                name
            )

        except Exception as exc:
            self._emit(
                "mission.vehicle_error",
                (
                    f"{name} connection "
                    f"failed: {exc}"
                ),
                vehicle=name,
            )

            if (
                vehicle.status.state
                != UAVState.FAILED
            ):
                try:
                    await vehicle.fail(
                        str(exc)
                    )
                except Exception:
                    pass

            self._record_failed_vehicle(
                name
            )

    async def _initialize_fleet(self):
        initialization_results = (
            await asyncio.gather(
                *[
                    self._initialize_vehicle(
                        name,
                        vehicle,
                    )
                    for name, vehicle
                    in self.fleet.items()
                    if (
                        vehicle.status.state
                        != UAVState.FAILED
                    )
                ]
            )
        )

        homes = {
            name: home
            for name, home
            in initialization_results
            if home is not None
        }

        if not homes:
            self._set_mission_state(
                MissionState.ABORTED
            )

            self._emit(
                "mission.aborted",
                (
                    "No UAV obtained a valid "
                    "navigation solution"
                ),
            )

            return

        origin_name = next(
            iter(homes)
        )

        (
            self.origin_lat,
            self.origin_lon,
        ) = homes[origin_name]

        self._emit(
            "mission.origin_ready",
            (
                f"Shared search origin "
                f"established from "
                f"{origin_name}"
            ),
            vehicle=origin_name,
            latitude=self.origin_lat,
            longitude=self.origin_lon,
        )

    async def _initialize_vehicle(
        self,
        name: str,
        vehicle: Vehicle,
    ):
        try:
            home = await vehicle.initialize()

            return name, home

        except VehicleUnavailableError as exc:
            await vehicle.fail(
                str(exc)
            )

            self._record_failed_vehicle(
                name
            )

            return name, None

        except Exception as exc:
            self._emit(
                "mission.vehicle_error",
                (
                    f"{name} initialization "
                    f"failed: {exc}"
                ),
                vehicle=name,
            )

            try:
                await vehicle.fail(
                    str(exc)
                )
            except Exception:
                pass

            self._record_failed_vehicle(
                name
            )

            return name, None

    # ==================================================
    # PLANNING
    # ==================================================

    def _build_plan(self):
        available_names = [
            name
            for name, vehicle
            in self.fleet.items()
            if (
                vehicle.status.state
                != UAVState.FAILED
            )
        ]

        self.plan = build_search_plan(
            width_m=self.width_m,
            height_m=self.height_m,
            lane_spacing_m=self.lane_spacing_m,
            vehicle_names=available_names,
        )

        self.status.total_waypoints = sum(
            len(
                segment.waypoints
            )
            for segment
            in self.plan.values()
        )

        for (
            name,
            segment,
        ) in self.plan.items():
            self._emit(
                "mission.route_assigned",
                (
                    f"{name} assigned "
                    f"{len(segment.waypoints)} "
                    f"waypoints"
                ),
                vehicle=name,
                segment_id=(
                    segment.segment_id
                ),
                waypoint_count=len(
                    segment.waypoints
                ),
            )

    def _create_task_queues(self):
        self.queues = {
            name: asyncio.Queue()
            for name in self.fleet
        }

        for (
            name,
            segment,
        ) in self.plan.items():
            self.queues[
                name
            ].put_nowait(
                segment
            )

    # ==================================================
    # TAKEOFF
    # ==================================================

    async def _launch_vehicle(
        self,
        name: str,
        vehicle: Vehicle,
    ):
        try:
            await vehicle.launch()

        except VehicleUnavailableError as exc:
            await vehicle.fail(
                str(exc)
            )

            self._record_failed_vehicle(
                name
            )

            await self._recover_queued_work(
                failed_vehicle=name,
            )

        except Exception as exc:
            self._emit(
                "mission.vehicle_error",
                (
                    f"{name} takeoff "
                    f"failed: {exc}"
                ),
                vehicle=name,
            )

            try:
                await vehicle.fail(
                    str(exc)
                )
            except Exception:
                pass

            self._record_failed_vehicle(
                name
            )

            await self._recover_queued_work(
                failed_vehicle=name,
            )

    # ==================================================
    # WORKERS
    # ==================================================

    def _start_workers(self):
        self.workers = {
            name: asyncio.create_task(
                self._vehicle_worker(
                    name
                )
            )
            for name, vehicle
            in self.fleet.items()
            if (
                vehicle.status.healthy
                and vehicle.status.state
                == UAVState.AIRBORNE
            )
        }

    async def _vehicle_worker(
        self,
        name: str,
    ):
        vehicle = self.fleet[
            name
        ]

        queue = self.queues[
            name
        ]

        while (
            not self._stop_event.is_set()
        ):
            if (
                vehicle.failure_requested
                and vehicle.status.state
                != UAVState.FAILED
            ):
                await vehicle.fail(
                    vehicle.failure_reason
                    or "vehicle unavailable"
                )

                self._record_failed_vehicle(
                    name
                )

                await self._recover_queued_work(
                    failed_vehicle=name,
                )

                return

            try:
                segment = (
                    await asyncio.wait_for(
                        queue.get(),
                        timeout=0.25,
                    )
                )

            except asyncio.TimeoutError:
                continue

            self.current_segments[
                name
            ] = segment

            self.current_indices[
                name
            ] = 0

            try:
                if (
                    segment.kind
                    == "recovery"
                ):
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
                    segment_id=(
                        segment.segment_id
                    ),
                    kind=segment.kind,
                )

                for (
                    index,
                    waypoint,
                ) in enumerate(
                    segment.waypoints
                ):
                    self.current_indices[
                        name
                    ] = index

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
                            f"waypoint "
                            f"{index + 1}/"
                            f"{len(segment.waypoints)}"
                        ),
                        vehicle=name,
                        segment_id=(
                            segment.segment_id
                        ),
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
                    segment_id=(
                        segment.segment_id
                    ),
                )

            except VehicleUnavailableError as exc:
                await vehicle.fail(
                    str(exc)
                )

                self._record_failed_vehicle(
                    name
                )

                await self._recover_failure(
                    failed_vehicle=name,
                    segment=segment,
                    current_index=(
                        self.current_indices[
                            name
                        ]
                    ),
                )

                return

            finally:
                self.current_segments[
                    name
                ] = None

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
        recovery = (
            build_recovery_segment(
                failed_vehicle=(
                    failed_vehicle
                ),
                segment=segment,
                current_index=(
                    current_index
                ),
            )
        )

        segments: list[
            RouteSegment
        ] = []

        if recovery.waypoints:
            segments.append(
                recovery
            )

        queue = self.queues[
            failed_vehicle
        ]

        while (
            not queue.empty()
        ):
            queued_segment = (
                queue.get_nowait()
            )

            segments.append(
                self._mark_as_recovery(
                    queued_segment
                )
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
        if (
            failed_vehicle
            not in self.queues
        ):
            return

        queue = self.queues[
            failed_vehicle
        ]

        segments: list[
            RouteSegment
        ] = []

        while (
            not queue.empty()
        ):
            segment = (
                queue.get_nowait()
            )

            segments.append(
                self._mark_as_recovery(
                    segment
                )
            )

            queue.task_done()

        await self._assign_recovery_work(
            failed_vehicle,
            segments,
        )

    def _mark_as_recovery(
        self,
        segment: RouteSegment,
    ) -> RouteSegment:
        if (
            segment.kind
            == "recovery"
        ):
            return segment

        return RouteSegment(
            segment_id=(
                f"{segment.segment_id}"
                f"-recovery"
            ),
            source_uav=(
                segment.source_uav
            ),
            waypoints=list(
                segment.waypoints
            ),
            kind="recovery",
        )

    async def _assign_recovery_work(
        self,
        failed_vehicle: str,
        segments: list[RouteSegment],
    ):
        async with (
            self._recovery_lock
        ):
            self._record_failed_vehicle(
                failed_vehicle
            )

            previous_state = (
                self.status.state
            )

            self._set_mission_state(
                MissionState.RECOVERING
            )

            self._emit(
                "mission.recovery_started",
                (
                    "Recovering work "
                    f"from {failed_vehicle}"
                ),
                failed_vehicle=(
                    failed_vehicle
                ),
                segment_count=len(
                    segments
                ),
            )

            for segment in segments:
                candidates = [
                    name
                    for name, vehicle
                    in self.fleet.items()
                    if (
                        name
                        != failed_vehicle
                        and vehicle.status.healthy
                        and not (
                            vehicle.failure_requested
                        )
                        and (
                            vehicle.status.state
                            != UAVState.FAILED
                        )
                    )
                ]

                if not candidates:
                    self._set_mission_state(
                        MissionState.ABORTED
                    )

                    self._emit(
                        "mission.aborted",
                        (
                            "No healthy UAVs "
                            "remain for recovery"
                        ),
                    )

                    self._stop_event.set()

                    return

                queue_lengths = {
                    name: (
                        self.queues[
                            name
                        ].qsize()
                        + (
                            1
                            if (
                                self.current_segments[
                                    name
                                ]
                                is not None
                            )
                            else 0
                        )
                    )
                    for name
                    in candidates
                }

                target = (
                    choose_recovery_vehicle(
                        candidates=(
                            candidates
                        ),
                        queue_lengths=(
                            queue_lengths
                        ),
                    )
                )

                await self.queues[
                    target
                ].put(
                    segment
                )

                self.status.recovery_count += 1

                self._emit(
                    "mission.route_reassigned",
                    (
                        f"{segment.segment_id} "
                        f"reassigned to "
                        f"{target}"
                    ),
                    failed_vehicle=(
                        failed_vehicle
                    ),
                    recovery_vehicle=(
                        target
                    ),
                    segment_id=(
                        segment.segment_id
                    ),
                    waypoint_count=len(
                        segment.waypoints
                    ),
                )

            if (
                self.status.state
                != MissionState.ABORTED
            ):
                restore_state = (
                    MissionState.ACTIVE
                    if (
                        previous_state
                        in {
                            MissionState.ACTIVE,
                            MissionState.RECOVERING,
                        }
                    )
                    else previous_state
                )

                self._set_mission_state(
                    restore_state
                )

    # ==================================================
    # COMPLETION
    # ==================================================

    async def _wait_for_search_completion(
        self,
    ):
        while True:
            if (
                self.status.state
                == MissionState.ABORTED
            ):
                return

            queued_work = any(
                not queue.empty()
                for queue
                in self.queues.values()
            )

            active_work = any(
                segment is not None
                for segment
                in self.current_segments.values()
            )

            if (
                not queued_work
                and not active_work
            ):
                return

            await asyncio.sleep(
                0.1
            )

    # ==================================================
    # LANDING
    # ==================================================

    async def _land_survivors(self):
        survivors = [
            (
                name,
                vehicle,
            )
            for name, vehicle
            in self.fleet.items()
            if (
                vehicle.status.state
                != UAVState.FAILED
            )
        ]

        await asyncio.gather(
            *[
                self._land_vehicle(
                    name,
                    vehicle,
                )
                for (
                    name,
                    vehicle,
                ) in survivors
            ],
            return_exceptions=True,
        )

    async def _land_vehicle(
        self,
        name: str,
        vehicle: Vehicle,
    ):
        try:
            await vehicle.land()

        except VehicleUnavailableError as exc:
            await vehicle.fail(
                str(exc)
            )

            self._record_failed_vehicle(
                name
            )

            self._emit(
                "mission.landing_failure",
                (
                    f"{name} became "
                    f"unavailable during landing"
                ),
                vehicle=name,
            )

        except Exception as exc:
            self._emit(
                "mission.vehicle_error",
                (
                    f"{name} landing "
                    f"failed: {exc}"
                ),
                vehicle=name,
            )

    # ==================================================
    # HELPERS
    # ==================================================

    def _record_failed_vehicle(
        self,
        name: str,
    ):
        if (
            name
            not in self.status.failed_uavs
        ):
            self.status.failed_uavs.append(
                name
            )