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
    get_position,
    offset_position,
    distance_m,
    goto,
)

from coverage import (
    partition_rectangle,
    generate_lawnmower_path,
)


VEHICLES = {
    "UAV-1": "tcp:127.0.0.1:5760",
    "UAV-2": "tcp:127.0.0.1:5770",
    "UAV-3": "tcp:127.0.0.1:5780",
}


ALTITUDES = {
    "UAV-1": 10,
    "UAV-2": 12,
    "UAV-3": 14,
}


SEARCH_WIDTH_M = 60
SEARCH_HEIGHT_M = 30
LANE_SPACING_M = 15

ARRIVAL_RADIUS_M = 2.0
WAYPOINT_TIMEOUT = 30


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
    altitude = ALTITUDES[name]

    print(
        f"[{name}] TAKEOFF → "
        f"{altitude} m"
    )

    set_mode(vehicle, "GUIDED")

    arm(vehicle)

    takeoff(
        vehicle,
        altitude,
    )

    print(
        f"[{name}] AIRBORNE @ "
        f"{altitude} m"
    )


def wait_for_waypoint(
    name,
    vehicle,
    target_lat,
    target_lon,
    waypoint_number,
    total_waypoints,
):
    start = time.time()
    last_log = 0

    while time.time() - start < WAYPOINT_TIMEOUT:

        msg = vehicle.recv_match(
            type="GLOBAL_POSITION_INT",
            blocking=True,
            timeout=1,
        )

        if msg is None:
            continue

        current_lat = msg.lat / 1e7
        current_lon = msg.lon / 1e7

        remaining = distance_m(
            current_lat,
            current_lon,
            target_lat,
            target_lon,
        )

        if time.time() - last_log >= 2:
            print(
                f"[{name}] "
                f"WP {waypoint_number}/{total_waypoints} "
                f"| {remaining:.1f} m remaining"
            )

            last_log = time.time()

        if remaining <= ARRIVAL_RADIUS_M:

            print(
                f"[{name}] "
                f"WP {waypoint_number}/{total_waypoints} "
                f"REACHED "
                f"(error={remaining:.2f} m)"
            )

            return

    raise TimeoutError(
        f"{name} failed to reach waypoint "
        f"{waypoint_number}"
    )


def execute_search_route(
    name,
    vehicle,
    route,
    origin_lat,
    origin_lon,
):
    altitude = ALTITUDES[name]

    print(
        f"[{name}] SEARCH START "
        f"| {len(route)} waypoints"
    )

    for index, (east_m, north_m) in enumerate(
        route,
        start=1,
    ):

        target_lat, target_lon = offset_position(
            origin_lat,
            origin_lon,
            north_m=north_m,
            east_m=east_m,
        )

        print(
            f"[{name}] "
            f"WP {index}/{len(route)} "
            f"→ east={east_m:.1f}m "
            f"north={north_m:.1f}m"
        )

        goto(
            vehicle,
            target_lat,
            target_lon,
            altitude,
        )

        wait_for_waypoint(
            name,
            vehicle,
            target_lat,
            target_lon,
            index,
            len(route),
        )

    print(f"[{name}] SEARCH COMPLETE")


def land_vehicle(name, vehicle):
    print(f"[{name}] LANDING")

    land(vehicle)

    print(f"[{name}] LANDED")


async def main():

    # --------------------------------------------------
    # CONNECT
    # --------------------------------------------------

    print("\n=== CONNECTING SWARM ===")

    results = await asyncio.gather(
        *[
            asyncio.to_thread(
                connect_vehicle,
                name,
                connection_string,
            )
            for name, connection_string
            in VEHICLES.items()
        ]
    )

    vehicles = dict(
        zip(VEHICLES.keys(), results)
    )

    # --------------------------------------------------
    # INITIALIZE
    # --------------------------------------------------

    print("\n=== INITIALIZING SWARM ===")

    await asyncio.gather(
        *[
            asyncio.to_thread(
                prepare_vehicle,
                name,
                vehicle,
            )
            for name, vehicle
            in vehicles.items()
        ]
    )

    print("\n=== SWARM READY ===")

    # --------------------------------------------------
    # SHARED SEARCH ORIGIN
    # --------------------------------------------------

    origin_lat, origin_lon, _ = get_position(
        vehicles["UAV-1"]
    )

    print(
        "\nShared search origin:"
        f"\n  lat={origin_lat:.7f}"
        f"\n  lon={origin_lon:.7f}"
    )

    # --------------------------------------------------
    # PLAN SEARCH REGION
    # --------------------------------------------------

    zones = partition_rectangle(
        SEARCH_WIDTH_M,
        SEARCH_HEIGHT_M,
        len(vehicles),
    )

    routes = {}

    for name, zone in zip(
        vehicles.keys(),
        zones,
    ):
        routes[name] = generate_lawnmower_path(
            zone,
            lane_spacing_m=LANE_SPACING_M,
        )

        print(
            f"[{name}] Zone {zone} "
            f"| {len(routes[name])} waypoints"
        )

    # --------------------------------------------------
    # TAKE OFF
    # --------------------------------------------------

    print("\n=== FLEET TAKEOFF ===")

    await asyncio.gather(
        *[
            asyncio.to_thread(
                launch_vehicle,
                name,
                vehicle,
            )
            for name, vehicle
            in vehicles.items()
        ]
    )

    print("\n=== SWARM AIRBORNE ===")

    # --------------------------------------------------
    # SEARCH CONCURRENTLY
    # --------------------------------------------------

    print("\n=== SEARCH START ===")

    mission_start = time.time()

    await asyncio.gather(
        *[
            asyncio.to_thread(
                execute_search_route,
                name,
                vehicle,
                routes[name],
                origin_lat,
                origin_lon,
            )
            for name, vehicle
            in vehicles.items()
        ]
    )

    mission_duration = time.time() - mission_start

    print("\n==============================")
    print("     SEARCH COMPLETE")
    print("==============================")

    print(
        f"Mission time: "
        f"{mission_duration:.2f}s"
    )

    # --------------------------------------------------
    # LAND
    # --------------------------------------------------

    print("\n=== LANDING FLEET ===")

    await asyncio.gather(
        *[
            asyncio.to_thread(
                land_vehicle,
                name,
                vehicle,
            )
            for name, vehicle
            in vehicles.items()
        ]
    )

    print("\n==============================")
    print("     MISSION COMPLETE")
    print("==============================")


if __name__ == "__main__":
    asyncio.run(main())