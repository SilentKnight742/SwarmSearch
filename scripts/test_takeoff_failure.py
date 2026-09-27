import asyncio

from swarmsearch.coordinator import SwarmCoordinator
from swarmsearch.models import UAVState


IMPORTANT_EVENTS = {
    "mission.state_changed",
    "mission.route_assigned",
    "mission.failure_injected",
    "mission.recovery_started",
    "mission.route_reassigned",
    "mission.fleet_airborne",
    "mission.segment_started",
    "mission.segment_completed",
    "mission.completed",
    "mission.aborted",
    "uav.takeoff_started",
    "uav.airborne",
    "uav.failure_requested",
    "uav.failed",
    "uav.failure_land_requested",
}


def print_event(event):
    if event.event_type not in IMPORTANT_EVENTS:
        return

    print(
        f"[{event.source}] "
        f"{event.event_type}: "
        f"{event.message}"
    )


async def wait_for_state(
    coordinator: SwarmCoordinator,
    vehicle_name: str,
    state: UAVState,
    timeout: float = 30.0,
):
    start = asyncio.get_running_loop().time()

    while True:
        vehicle = coordinator.fleet[vehicle_name]

        if vehicle.status.state == state:
            return

        if (
            asyncio.get_running_loop().time()
            - start
            >= timeout
        ):
            raise TimeoutError(
                f"{vehicle_name} did not reach "
                f"{state.value} within {timeout}s. "
                f"Current state: "
                f"{vehicle.status.state.value}"
            )

        await asyncio.sleep(0.05)


async def main():
    coordinator = SwarmCoordinator(
        event_sink=print_event,
    )

    mission_task = asyncio.create_task(
        coordinator.run()
    )

    print()
    print("==============================")
    print(" WAITING FOR UAV-2 TAKEOFF")
    print("==============================")
    print()

    await wait_for_state(
        coordinator,
        "UAV-2",
        UAVState.TAKING_OFF,
    )

    # Give UAV-2 enough time to leave the ground so this is
    # genuinely a mid-climb failure rather than a pre-takeoff one.
    await asyncio.sleep(2.0)

    vehicle = coordinator.fleet["UAV-2"]

    print()
    print("==============================")
    print(" TAKEOFF FAILURE INJECTION")
    print("==============================")
    print(
        f"UAV-2 altitude before failure: "
        f"{vehicle.status.altitude:.2f}m"
    )
    print()

    coordinator.inject_failure(
        "UAV-2",
        reason="takeoff-phase test failure",
    )

    await mission_task

    print()
    print("==============================")
    print("         FINAL STATUS")
    print("==============================")
    print(
        f"Mission: "
        f"{coordinator.status.state.value}"
    )
    print(
        f"UAV-2 state: "
        f"{coordinator.fleet['UAV-2'].status.state.value}"
    )
    print(
        f"Failed UAVs: "
        f"{coordinator.status.failed_uavs}"
    )
    print(
        f"Recovery assignments: "
        f"{coordinator.status.recovery_count}"
    )

    assert (
        coordinator.fleet["UAV-2"].status.state
        == UAVState.FAILED
    )

    assert (
        "UAV-2"
        in coordinator.status.failed_uavs
    )

    assert (
        coordinator.status.recovery_count
        >= 1
    )

    print()
    print("TAKEOFF FAILURE TEST: PASS")


if __name__ == "__main__":
    asyncio.run(main())