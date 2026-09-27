from pymavlink import mavutil


VEHICLES = {
    "UAV-1": "tcp:127.0.0.1:5760",
    "UAV-2": "tcp:127.0.0.1:5770",
    "UAV-3": "tcp:127.0.0.1:5780",
}


def connect_vehicle(name, connection_string):
    print(f"[{name}] Connecting to {connection_string}...")

    vehicle = mavutil.mavlink_connection(connection_string)

    vehicle.wait_heartbeat(timeout=30)

    print(
        f"[{name}] Connected | "
        f"System ID: {vehicle.target_system} | "
        f"Component ID: {vehicle.target_component}"
    )

    return vehicle


def main():
    vehicles = {}

    for name, connection_string in VEHICLES.items():
        try:
            vehicles[name] = connect_vehicle(
                name,
                connection_string,
            )

        except Exception as e:
            print(f"[{name}] Connection failed: {e}")

    print()
    print("----- SWARM DISCOVERY -----")

    for name, vehicle in vehicles.items():
        print(
            f"{name}: "
            f"sysid={vehicle.target_system}, "
            f"component={vehicle.target_component}"
        )

    print(f"\nVehicles discovered: {len(vehicles)}")


if __name__ == "__main__":
    main()