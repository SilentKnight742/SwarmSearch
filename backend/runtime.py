import asyncio
from typing import Optional

from swarmsearch.coordinator import (
    SwarmCoordinator,
)
from swarmsearch.events import SwarmEvent

from .event_broker import EventBroker


class MissionRuntime:
    """
    Owns the active SwarmCoordinator and manages mission lifecycles.

    Important rule:

        one coordinator / MAVLink connection set exists at a time.

    Before a new mission is created, the previous receiver threads
    and their underlying MAVLink TCP sockets are explicitly closed.
    """

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

    # ==================================================
    # RUNTIME STATUS
    # ==================================================

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

    # ==================================================
    # SNAPSHOT
    # ==================================================

    def snapshot(
        self,
    ) -> dict:
        return {
            "runtime": self.status(),
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

        return {
            vehicle_name: {
                "segment_id": (
                    segment.segment_id
                ),
                "source_uav": (
                    segment.source_uav
                ),
                "kind": (
                    segment.kind
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
            for (
                vehicle_name,
                segment,
            )
            in self.coordinator.plan.items()
        }

    # ==================================================
    # MISSION START
    # ==================================================

    async def start_mission(
        self,
        width_m: float,
        height_m: float,
        lane_spacing_m: float,
    ):
        # ------------------------------------------------
        # Authoritative concurrency protection.
        # ------------------------------------------------

        if self.mission_running:
            raise RuntimeError(
                "A mission is already running."
            )

        # A mission task can finish before a failed aircraft
        # physically reaches the ground.
        #
        # Do not recycle its MAVLink connection until every
        # vehicle is physically disarmed.
        if self.fleet_armed:
            raise RuntimeError(
                "The previous fleet is still "
                "physically armed or landing."
            )

        # ------------------------------------------------
        # Dispose of the previous mission completely.
        # ------------------------------------------------

        await self._close_previous_fleet()

        self.broker.reset()

        self.mission_config = {
            "width_m": width_m,
            "height_m": height_m,
            "lane_spacing_m": (
                lane_spacing_m
            ),
        }

        # ------------------------------------------------
        # New mission = new coordinator = new vehicles =
        # new MAVLink TCP connections.
        # ------------------------------------------------

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
            await self.coordinator.run()

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

            await self._safe_withdraw_fleet(
                reason=(
                    "mission runtime error"
                )
            )

    # ==================================================
    # FAILURE INJECTION
    # ==================================================

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

    # ==================================================
    # SAFETY
    # ==================================================

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

    # ==================================================
    # CONNECTION CLEANUP
    # ==================================================

    async def _close_previous_fleet(
        self,
    ):
        """
        Completely dispose of the previous mission's vehicle layer.

        Vehicle.close() stops SwarmSearch's dedicated receiver thread.

        We must ALSO close pymavlink's underlying TCP socket.

        Without the second step, ArduPilot SITL can retain the old
        TCP client while the next mission creates another connection,
        resulting in a connection that receives some MAVLink traffic
        but never obtains the heartbeat required by Vehicle.connect().
        """

        if self.coordinator is None:
            return

        coordinator = (
            self.coordinator
        )

        # ----------------------------------------------
        # 1. Stop receiver threads first.
        # ----------------------------------------------

        for vehicle in (
            coordinator.fleet.values()
        ):
            try:
                vehicle.close()

            except Exception:
                # Cleanup should continue for the remaining
                # vehicles even if one receiver fails to stop.
                pass

        # ----------------------------------------------
        # 2. Close the actual pymavlink TCP sockets.
        # ----------------------------------------------

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

        # Give SITL / the local TCP stack a brief opportunity
        # to observe the disconnect before establishing the
        # next set of clients.
        await asyncio.sleep(
            0.25
        )

        self.coordinator = None
        self.mission_task = None

    # ==================================================
    # BACKEND SHUTDOWN
    # ==================================================

    async def shutdown(
        self,
    ):
        if self.coordinator is None:
            return

        # If the backend is stopped while aircraft are in flight,
        # first request their normal SwarmSearch failure safety
        # behavior (LAND / DISARM depending on phase).
        if self.fleet_armed:
            await self._safe_withdraw_fleet(
                reason="backend shutdown"
            )

            # LAND mode is persistent inside ArduPilot once accepted.
            # This delay gives the command/state transition a chance
            # to reach the autopilot before we close the TCP links.
            await asyncio.sleep(
                0.5
            )

        await self._close_previous_fleet()