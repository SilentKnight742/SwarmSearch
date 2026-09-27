import asyncio

from swarmsearch.coordinator import (
    SwarmCoordinator,
)


def print_event(event):
    important = {
        "mission.state_changed",
        "mission.route_assigned",
        "mission.segment_started",
        "mission.segment_completed",
        "mission.failure_injected",
        "uav.failure_requested",
        "uav.failed",
        "mission.recovery_started",
        "mission.route_reassigned",
        "mission.aborted",
        "mission.completed",
    }

    if event.event_type not in important:
        return

    print(
        f"[{event.source}] "
        f"{event.event_type}: "
        f"{event.message}"
    )


async def main():
    coordinator = SwarmCoordinator(
        event_sink=print_event,
    )

    mission = asyncio.create_task(
        coordinator.run()
    )

    # Wait until search execution really begins.
    await coordinator.wait_until_active()

    # Let the fleet begin covering its zones.
    await asyncio.sleep(8)

    print()
    print("==============================")
    print("   OPERATOR INJECTS FAILURE")
    print("==============================")
    print()

    coordinator.inject_failure(
        "UAV-2",
        reason="demo operator fault injection",
    )

    await mission

    print()
    print("==============================")
    print("       FINAL STATUS")
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


if __name__ == "__main__":
    asyncio.run(main())