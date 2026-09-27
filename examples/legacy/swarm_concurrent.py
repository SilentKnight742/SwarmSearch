import asyncio
import time

from pymavlink import mavutil

from connect import (
    request_message_interval,
    wait_for_home,
    set_mode,
    arm,
    takeoff,
    land,
)


VEHICLES = {
    "UAV-1": "tcp:127.0.0.1:5760",
    "UAV-2": "tcp:127.0.0.1:5770",
    "UAV-3": "tcp:127.0.0.1:5780",
}

TARGET_ALTITUDE = 10


def connect_vehicle(name, connection_string):
    print(f"[{name}] Connecting...")

    vehicle = mavutil.mavlink_connection(connection_string)
    vehicle.wait_heartbeat(timeout=30)

    print(
        f"[{name}] CONNECTED "
        f"(sysid={vehicle.target_system})"
    )

    request_message_interval(
        vehicle,
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        5,
    )

    return vehicle


def prepare_vehicle(name, vehicle):
    print(f"[{name}] Waiting for navigation...")
    wait_for_home(vehicle)
    print(f"[{name}] READY")


def launch_vehicle(name, vehicle):
    print(f"[{name}] TAKEOFF starting")

    set_mode(vehicle, "GUIDED")
    arm(vehicle)
    takeoff(vehicle, TARGET_ALTITUDE)

    print(f"[{name}] AIRBORNE")


def land_vehicle(name, vehicle):
    print(f"[{name}] LANDING")
    land(vehicle)
    print(f"[{name}] LANDED")


async def main():
    # --------------------------------------------------
    # 1. Connect concurrently
    # --------------------------------------------------

    print("\n=== CONNECTING FLEET ===")

    connection_results = await asyncio.gather(
        *[
            asyncio.to_thread(
                connect_vehicle,
                name,
                connection_string,
            )
            for name, connection_string in VEHICLES.items()
        ]
    )

    vehicles = dict(
        zip(VEHICLES.keys(), connection_results)
    )

    print("\n=== FLEET CONNECTED ===")

    # --------------------------------------------------
    # 2. Wait for all UAVs to become flight-ready
    # --------------------------------------------------

    await asyncio.gather(
        *[
            asyncio.to_thread(
                prepare_vehicle,
                name,
                vehicle,
            )
            for name, vehicle in vehicles.items()
        ]
    )

    print("\n=== FLEET READY ===")

    # --------------------------------------------------
    # 3. Take off concurrently
    # --------------------------------------------------

    takeoff_start = time.time()

    await asyncio.gather(
        *[
            asyncio.to_thread(
                launch_vehicle,
                name,
                vehicle,
            )
            for name, vehicle in vehicles.items()
        ]
    )

    takeoff_duration = time.time() - takeoff_start

    print("\n==========================")
    print("      SWARM AIRBORNE")
    print("==========================")
    print(
        f"Fleet takeoff completed in "
        f"{takeoff_duration:.2f}s"
    )

    print("\nHolding position for 10 seconds...")
    await asyncio.sleep(10)

    # --------------------------------------------------
    # 4. Land concurrently
    # --------------------------------------------------

    landing_start = time.time()

    await asyncio.gather(
        *[
            asyncio.to_thread(
                land_vehicle,
                name,
                vehicle,
            )
            for name, vehicle in vehicles.items()
        ]
    )

    landing_duration = time.time() - landing_start

    print("\n==========================")
    print("       FLEET LANDED")
    print("==========================")
    print(
        f"Fleet landing completed in "
        f"{landing_duration:.2f}s"
    )


if __name__ == "__main__":
    asyncio.run(main())