import asyncio

from swarmsearch.config import (
    VEHICLES,
    DEFAULT_ALTITUDES,
)

from swarmsearch.vehicle import Vehicle


IMPORTANT_EVENTS = {
    "uav.state_changed",
    "uav.connecting",
    "uav.connected",
    "uav.home_wait",
    "uav.home_ready",
    "uav.ekf_wait",
    "uav.ekf_ready",
    "uav.ready",
    "uav.mode_requested",
    "uav.mode_changed",
    "uav.arming_started",
    "uav.armed",
    "uav.takeoff_started",
    "uav.airborne",
    "uav.landing_started",
    "uav.landed",
    "uav.failed",
}


def print_event(event):
    if event.event_type not in IMPORTANT_EVENTS:
        return

    print(
        f"[{event.source}] "
        f"{event.event_type}: "
        f"{event.message}"
    )


async def main():
    fleet = {
        name: Vehicle(
            name=name,
            connection_string=connection,
            cruise_altitude=(
                DEFAULT_ALTITUDES[name]
            ),
            event_sink=print_event,
        )
        for name, connection
        in VEHICLES.items()
    }

    try:
        print()
        print("=== CONNECT ===")

        await asyncio.gather(
            *[
                vehicle.connect()
                for vehicle
                in fleet.values()
            ]
        )

        print()
        print("=== INITIALIZE ===")

        await asyncio.gather(
            *[
                vehicle.initialize()
                for vehicle
                in fleet.values()
            ]
        )

        print()
        print("=== TAKEOFF ===")

        await asyncio.gather(
            *[
                vehicle.launch()
                for vehicle
                in fleet.values()
            ]
        )

        print()
        print("=== FLEET AIRBORNE ===")

        for vehicle in fleet.values():
            print(
                f"{vehicle.name}: "
                f"{vehicle.status.altitude:.1f}m "
                f"| {vehicle.status.flight_mode}"
            )

        await asyncio.sleep(5)

        print()
        print("=== LAND ===")

        await asyncio.gather(
            *[
                vehicle.land()
                for vehicle
                in fleet.values()
            ]
        )

        print()
        print("=== COMPLETE ===")

    finally:
        for vehicle in fleet.values():
            vehicle.close()


if __name__ == "__main__":
    asyncio.run(main())