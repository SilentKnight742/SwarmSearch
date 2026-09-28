import asyncio

from math import hypot
from typing import Optional

from swarmsearch.coordinator import (
    SwarmCoordinator,
)

from swarmsearch.events import (
    SwarmEvent,
)

from .event_broker import (
    EventBroker,
)


class MissionRuntime:
    def __init__(
        self,
        broker: EventBroker,
    ):
        self.broker = broker

        self.coordinator: Optional[
            SwarmCoordinator
        ] = None

        self.mission_task: Optional[
            asyncio.Task
        ] = None

        self.mission_config: Optional[
            dict
        ] = None

    @property
    def mission_running(
        self,
    ) -> bool:
        return (
            self.mission_task
            is not None
            and not self.mission_task.done()
        )

    @property
    def fleet_armed(
        self,
    ) -> bool:
        if self.coordinator is None:
            return False

        return any(
            vehicle.status.armed
            for vehicle
            in self.coordinator.fleet.values()
        )

    @property
    def accepts_new_mission(
        self,
    ) -> bool:
        return (
            not self.mission_running
            and not self.fleet_armed
        )

    def status(
        self,
    ) -> dict:
        return {
            "mission_running": (
                self.mission_running
            ),
            "fleet_armed": (
                self.fleet_armed
            ),
            "accepts_new_mission": (
                self.accepts_new_mission
            ),
        }

    def snapshot(
        self,
    ) -> dict:
        return {
            "runtime": (
                self.status()
            ),
            "mission_config": (
                self.mission_config
            ),
            "plan": (
                self._plan_snapshot()
            ),
            "world": (
                self.broker.snapshot()
            ),
        }

    def _plan_snapshot(
        self,
    ) -> dict:
        if self.coordinator is None:
            return {}

        snapshot = {}

        for (
            vehicle_name,
            segment,
        ) in (
            self.coordinator
            .plan
            .items()
        ):
            start = (
                self.coordinator
                .start_positions
                .get(vehicle_name)
            )

            entry = (
                segment.waypoints[0]
                if segment.waypoints
                else None
            )

            transit_distance = None

            if (
                start is not None
                and entry is not None
            ):
                transit_distance = (
                    hypot(
                        entry.east
                        - start.east,
                        entry.north
                        - start.north,
                    )
                )

            snapshot[
                vehicle_name
            ] = {
                "segment_id": (
                    segment.segment_id
                ),
                "source_uav": (
                    segment.source_uav
                ),
                "kind": (
                    segment.kind
                ),
                "start": (
                    {
                        "east": (
                            start.east
                        ),
                        "north": (
                            start.north
                        ),
                    }
                    if start
                    is not None
                    else None
                ),
                "entry": (
                    {
                        "east": (
                            entry.east
                        ),
                        "north": (
                            entry.north
                        ),
                    }
                    if entry
                    is not None
                    else None
                ),
                "transit_distance_m": (
                    transit_distance
                ),
                "waypoints": [
                    {
                        "east": (
                            waypoint.east
                        ),
                        "north": (
                            waypoint.north
                        ),
                    }
                    for waypoint
                    in segment.waypoints
                ],
            }

        return snapshot

    async def start_mission(
        self,
        width_m: float,
        height_m: float,
        lane_spacing_m: float,
    ):
        if self.mission_running:
            raise RuntimeError(
                "A mission is already running."
            )

        if self.fleet_armed:
            raise RuntimeError(
                "The previous fleet is still "
                "physically armed or landing."
            )

        await (
            self._close_previous_fleet()
        )

        self.broker.reset()

        self.mission_config = {
            "width_m": width_m,
            "height_m": height_m,
            "lane_spacing_m": (
                lane_spacing_m
            ),
        }

        self.coordinator = (
            SwarmCoordinator(
                event_sink=(
                    self.broker.publish
                ),
                width_m=width_m,
                height_m=height_m,
                lane_spacing_m=(
                    lane_spacing_m
                ),
            )
        )

        self.mission_task = (
            asyncio.create_task(
                self._run_mission()
            )
        )

    async def _run_mission(
        self,
    ):
        try:
            await (
                self.coordinator.run()
            )

        except asyncio.CancelledError:
            raise

        except Exception as exc:
            self.broker.publish(
                SwarmEvent(
                    event_type=(
                        "mission.runtime_error"
                    ),
                    source="SWARM",
                    message=str(exc),
                    data={
                        "error_type": (
                            type(exc).__name__
                        ),
                    },
                )
            )

            await (
                self._safe_withdraw_fleet(
                    reason=(
                        "mission runtime error"
                    )
                )
            )

    def inject_failure(
        self,
        vehicle_name: str,
        reason: str,
    ):
        if self.coordinator is None:
            raise RuntimeError(
                "No mission has been started."
            )

        if not self.mission_running:
            raise RuntimeError(
                "There is no active mission."
            )

        self.coordinator.inject_failure(
            vehicle_name=vehicle_name,
            reason=reason,
        )

    async def _safe_withdraw_fleet(
        self,
        reason: str,
    ):
        if self.coordinator is None:
            return

        await asyncio.gather(
            *[
                vehicle.fail(
                    reason
                )
                for vehicle
                in self.coordinator.fleet.values()
                if (
                    vehicle.status.state.value
                    not in {
                        "failed",
                        "landed",
                    }
                )
            ],
            return_exceptions=True,
        )

    async def release_session(
        self,
    ):
        """
        Called when the public simulator lease ends.

        Prefer allowing a short mission to finish naturally.
        If it does not finish within the safety window, withdraw
        the fleet and cancel the mission task.
        """

        if self.coordinator is None:
            self.broker.reset()
            self.mission_config = None
            return

        task = self.mission_task

        if (
            task is not None
            and not task.done()
        ):
            try:
                await asyncio.wait_for(
                    asyncio.shield(
                        task
                    ),
                    timeout=120,
                )

            except asyncio.TimeoutError:
                await (
                    self._safe_withdraw_fleet(
                        reason=(
                            "simulator session ended"
                        )
                    )
                )

                task.cancel()

                try:
                    await task

                except asyncio.CancelledError:
                    pass

        if self.fleet_armed:
            disarm_waiters = [
                vehicle.wait_until_disarmed(
                    timeout=90
                )
                for vehicle
                in self.coordinator.fleet.values()
                if vehicle.status.armed
            ]

            if disarm_waiters:
                try:
                    await asyncio.wait_for(
                        asyncio.gather(
                            *disarm_waiters,
                            return_exceptions=True,
                        ),
                        timeout=95,
                    )

                except asyncio.TimeoutError:
                    pass

        await (
            self._close_previous_fleet()
        )

        self.broker.reset()

        self.mission_config = None

    async def _close_previous_fleet(
        self,
    ):
        if self.coordinator is None:
            return

        coordinator = (
            self.coordinator
        )

        for vehicle in (
            coordinator.fleet.values()
        ):
            try:
                vehicle.close()

            except Exception:
                pass

        for vehicle in (
            coordinator.fleet.values()
        ):
            connection = (
                vehicle.connection
            )

            if connection is None:
                continue

            try:
                connection.close()

            except Exception:
                pass

            finally:
                vehicle.connection = None

        await asyncio.sleep(
            0.25
        )

        self.coordinator = None
        self.mission_task = None

    async def shutdown(
        self,
    ):
        await self.release_session()