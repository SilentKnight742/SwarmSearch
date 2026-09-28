import asyncio

from swarmsearch.coordinator import SwarmCoordinator
from swarmsearch.models import (
    MissionState,
    UAVState,
)


IMPORTANT_EVENTS = {
    "mission.state_changed",
    "mission.failure_injected",
    "mission.recovery_started",
    "mission.route_reassigned",
    "mission.completed",
    "mission.aborted",
    "uav.arming_started",
    "uav.failure_requested",
    "uav.failed",
    "uav.failure_disarm_requested",
    "uav.failure_disarmed",
    "uav.failure_disarm_failed",
}


def print_event(event):
    if event.event_type not in IMPORTANT_EVENTS:
        return

    print(
        f"[{event.source}] "
        f"{event.event_type}: "
        f"{event.message}"
    )


async def wait_for_arming(
    coordinator: SwarmCoordinator,
    vehicle_name: str,
    timeout: float = 90.0,
):
    start = asyncio.get_running_loop().time()

    while True:
        vehicle = coordinator.fleet[
            vehicle_name
        ]

        if (
            vehicle.status.state
            == UAVState.ARMING
        ):
            return

        if (
            coordinator.status.state
            == MissionState.ABORTED
        ):
            raise RuntimeError(
                "Mission aborted before arming."
            )

        if (
            asyncio.get_running_loop().time()
            - start
            >= timeout
        ):
            raise TimeoutError(
                f"{vehicle_name} did not enter "
                f"ARMING within {timeout}s. "
                f"Current state: "
                f"{vehicle.status.state.value}"
            )

        await asyncio.sleep(0.01)


async def main():
    coordinator = SwarmCoordinator(
        event_sink=print_event,
    )

    mission_task = asyncio.create_task(
        coordinator.run()
    )

    print()
    print("==============================")
    print(" WAITING FOR UAV-2 ARMING")
    print("==============================")
    print()

    await wait_for_arming(
        coordinator,
        "UAV-2",
    )

    vehicle = coordinator.fleet[
        "UAV-2"
    ]

    print()
    print("==============================")
    print(" ARMING FAILURE INJECTION")
    print("==============================")
    print(
        f"State before failure: "
        f"{vehicle.status.state.value}"
    )
    print(
        f"Armed before failure: "
        f"{vehicle.status.armed}"
    )
    print()

    coordinator.inject_failure(
        "UAV-2",
        reason=(
            "arming-phase test failure"
        ),
    )

    await mission_task

    # Even if the ARM command crossed the failure request,
    # the later DISARM command must leave the UAV physically safe.
    await vehicle.wait_until_disarmed(
        timeout=20,
    )

    await asyncio.sleep(0.5)

    print()
    print("==============================")
    print("      PHYSICAL UAV STATUS")
    print("==============================")
    print(
        f"Mission state: "
        f"{vehicle.status.state.value}"
    )
    print(
        f"Altitude: "
        f"{vehicle.status.altitude:.2f}m"
    )
    print(
        f"Armed: "
        f"{vehicle.status.armed}"
    )
    print(
        f"Healthy: "
        f"{vehicle.status.healthy}"
    )

    print()
    print("==============================")
    print("         FINAL STATUS")
    print("==============================")
    print(
        f"Mission: "
        f"{coordinator.status.state.value}"
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
        vehicle.status.state
        == UAVState.FAILED
    )

    assert (
        vehicle.status.armed
        is False
    )

    assert (
        abs(vehicle.status.altitude)
        < 0.5
    )

    assert (
        "UAV-2"
        in coordinator.status.failed_uavs
    )

    assert (
        coordinator.status.recovery_count
        >= 1
    )

    assert (
        coordinator.status.state
        == MissionState.COMPLETED
    )

    print()
    print(
        "ARMING FAILURE RECOVERY: PASS"
    )
    print(
        "SAFE GROUND DISARM: PASS"
    )


if __name__ == "__main__":
    asyncio.run(main())