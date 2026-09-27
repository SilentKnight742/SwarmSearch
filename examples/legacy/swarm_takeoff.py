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
    print(f"\n[{name}] Connecting to {connection_string}...")

    vehicle = mavutil.mavlink_connection(connection_string)
    vehicle.wait_heartbeat(timeout=30)

    print(
        f"[{name}] Connected | "
        f"sysid={vehicle.target_system}"
    )

    request_message_interval(
        vehicle,
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        5,
    )

    return vehicle


def main():
    vehicles = {}

    # -------------------------------------------------
    # 1. Connect to the entire fleet
    # -------------------------------------------------

    for name, connection_string in VEHICLES.items():
        vehicles[name] = connect_vehicle(
            name,
            connection_string,
        )

    print("\n=== ALL VEHICLES CONNECTED ===")

    # -------------------------------------------------
    # 2. Wait until every UAV is actually flight-ready
    # -------------------------------------------------

    for name, vehicle in vehicles.items():
        print(f"\n[{name}] Initializing...")
        wait_for_home(vehicle)

    print("\n=== ALL VEHICLES NAVIGATION-READY ===")

    # -------------------------------------------------
    # 3. Take off each UAV
    #
    # Arm + takeoff are kept together deliberately.
    # ArduPilot may auto-disarm if takeoff does not
    # follow arming quickly enough.
    # -------------------------------------------------

    for name, vehicle in vehicles.items():

        print(f"\n========== {name} ==========")

        set_mode(vehicle, "GUIDED")

        arm(vehicle)

        takeoff(
            vehicle,
            TARGET_ALTITUDE,
        )

        print(
            f"[{name}] Airborne at "
            f"{TARGET_ALTITUDE} m"
        )

    print("\n================================")
    print("      SWARM AIRBORNE")
    print("================================")

    print("\nHolding for 10 seconds...")
    time.sleep(10)

    # -------------------------------------------------
    # 4. Land everything
    # -------------------------------------------------

    print("\nLanding fleet...")

    for name, vehicle in vehicles.items():
        print(f"\n[{name}] Landing...")
        land(vehicle)

    print("\n================================")
    print("      ALL UAVs LANDED")
    print("================================")


if __name__ == "__main__":
    main()