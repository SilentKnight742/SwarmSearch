import asyncio

from swarmsearch.config import (
    VEHICLES,
    DEFAULT_ALTITUDES,
)

from swarmsearch.vehicle import Vehicle


def print_event(event):
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

    print("\n=== CONNECT ===")

    await asyncio.gather(
        *[
            vehicle.connect()
            for vehicle in fleet.values()
        ]
    )

    print("\n=== INITIALIZE ===")

    await asyncio.gather(
        *[
            vehicle.initialize()
            for vehicle in fleet.values()
        ]
    )

    print("\n=== TAKEOFF ===")

    await asyncio.gather(
        *[
            vehicle.launch()
            for vehicle in fleet.values()
        ]
    )

    print("\n=== FLEET AIRBORNE ===")

    await asyncio.sleep(5)

    print("\n=== LAND ===")

    await asyncio.gather(
        *[
            vehicle.land()
            for vehicle in fleet.values()
        ]
    )

    print("\n=== COMPLETE ===")


if __name__ == "__main__":
    asyncio.run(main())