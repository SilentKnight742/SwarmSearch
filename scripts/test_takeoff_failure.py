import asyncio

from swarmsearch.coordinator import SwarmCoordinator
from swarmsearch.models import MissionState, UAVState


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
    "uav.failure_land_failed",
}


def print_event(event):
    if event.event_type not in IMPORTANT_EVENTS:
        return

    print(
        f"[{event.source}] "
        f"{event.event_type}: "
        f"{event.message}"
    )


async def wait_for_mission_takeoff(
    coordinator: SwarmCoordinator,
    timeout: float = 90.0,
):
    """
    Wait for initialization, home acquisition and EKF readiness
    to complete.

    Fresh SITL instances can take substantially longer than
    already-warm vehicles.
    """

    start = asyncio.get_running_loop().time()

    while True:
        state = coordinator.status.state

        if state == MissionState.TAKING_OFF:
            return

        if state == MissionState.ABORTED:
            raise RuntimeError(
                "Mission aborted before takeoff."
            )

        elapsed = (
            asyncio.get_running_loop().time()
            - start
        )

        if elapsed >= timeout:
            raise TimeoutError(
                "Mission did not enter TAKING_OFF "
                f"within {timeout}s. "
                f"Current state: {state.value}"
            )

        await asyncio.sleep(0.05)


async def wait_for_mid_climb(
    coordinator: SwarmCoordinator,
    vehicle_name: str,
    minimum_altitude: float = 1.0,
    timeout: float = 20.0,
):
    """
    Wait until the selected UAV is physically climbing and
    has exceeded minimum_altitude while still TAKING_OFF.
    """

    start = asyncio.get_running_loop().time()

    while True:
        vehicle = coordinator.fleet[
            vehicle_name
        ]

        if (
            vehicle.status.state
            == UAVState.TAKING_OFF
            and vehicle.status.altitude
            >= minimum_altitude
        ):
            return

        if (
            vehicle.status.state
            == UAVState.FAILED
        ):
            raise RuntimeError(
                f"{vehicle_name} failed before "
                "the planned test injection."
            )

        if (
            vehicle.status.state
            == UAVState.AIRBORNE
        ):
            raise RuntimeError(
                f"{vehicle_name} completed takeoff "
                "before a mid-climb fault could "
                "be injected."
            )

        elapsed = (
            asyncio.get_running_loop().time()
            - start
        )

        if elapsed >= timeout:
            raise TimeoutError(
                f"{vehicle_name} did not reach "
                f"{minimum_altitude:.1f}m during "
                f"TAKING_OFF within {timeout}s. "
                f"Current state: "
                f"{vehicle.status.state.value}, "
                f"altitude: "
                f"{vehicle.status.altitude:.2f}m"
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
    print(" WAITING FOR MISSION TAKEOFF")
    print("==============================")
    print()

    await wait_for_mission_takeoff(
        coordinator
    )

    print()
    print("==============================")
    print(" WAITING FOR UAV-2 MID-CLIMB")
    print("==============================")
    print()

    await wait_for_mid_climb(
        coordinator,
        "UAV-2",
        minimum_altitude=1.0,
    )

    vehicle = coordinator.fleet[
        "UAV-2"
    ]

    altitude_before_failure = (
        vehicle.status.altitude
    )

    print()
    print("==============================")
    print(" TAKEOFF FAILURE INJECTION")
    print("==============================")
    print(
        f"UAV-2 altitude before failure: "
        f"{altitude_before_failure:.2f}m"
    )
    print()

    coordinator.inject_failure(
        "UAV-2",
        reason=(
            "mid-climb takeoff test failure"
        ),
    )

    # The remaining healthy fleet should continue
    # executing the mission.
    await mission_task

    print()
    print("==============================")
    print(" WAITING FOR FAILED UAV LAND")
    print("==============================")
    print()
    print(
        "Mission has finished, but UAV-2's "
        "receiver remains active."
    )
    print(
        "Waiting for ArduPilot LAND to reach "
        "physical disarm..."
    )
    print()

    # This wait intentionally ignores the UAV's FAILED
    # mission state. The dedicated MAVLink receiver still
    # observes heartbeat and altitude while LAND executes.
    await vehicle.wait_until_disarmed(
        timeout=90.0,
    )

    # Give GLOBAL_POSITION_INT a moment to reflect the
    # final ground-state altitude after disarm.
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
        f"Flight mode: "
        f"{vehicle.status.flight_mode}"
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
        f"UAV-2 mission state: "
        f"{vehicle.status.state.value}"
    )
    print(
        f"Failed UAVs: "
        f"{coordinator.status.failed_uavs}"
    )
    print(
        f"Recovery assignments: "
        f"{coordinator.status.recovery_count}"
    )

    assert altitude_before_failure >= 1.0

    assert (
        vehicle.status.state
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

    assert (
        coordinator.status.state
        == MissionState.COMPLETED
    )

    # This is the important new assertion:
    # the failed UAV physically completed ArduPilot LAND.
    assert (
        vehicle.status.armed
        is False
    )

    # SITL relative altitude can end slightly above or
    # below zero because of estimator noise.
    assert abs(
        vehicle.status.altitude
    ) < 0.5

    print()
    print(
        "MID-CLIMB FAILURE RECOVERY: PASS"
    )
    print(
        "SAFE FAILED-UAV DESCENT: PASS"
    )


if __name__ == "__main__":
    asyncio.run(main())