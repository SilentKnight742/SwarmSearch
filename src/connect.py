from pymavlink import mavutil
import time
import math

CONNECTION_STRING = "tcp:127.0.0.1:5760"
TARGET_ALTITUDE = 10

def request_message_interval(vehicle, message_id, frequency_hz):
    interval_us = int(1_000_000 / frequency_hz)

    print(
        f"Requesting MAVLink message {message_id} "
        f"at {frequency_hz} Hz..."
    )

    vehicle.mav.command_long_send(
        vehicle.target_system,
        0,  # target component; 0 = autopilot/default target
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval_us,
        0,
        0,
        0,
        0,
        0,
    )

def connect():
    print(f"Connecting to SITL at {CONNECTION_STRING}...")

    vehicle = mavutil.mavlink_connection(CONNECTION_STRING)

    print("Waiting for heartbeat...")
    vehicle.wait_heartbeat(timeout=30)

    print(
        f"Connected! System ID: {vehicle.target_system}, "
        f"Component ID: {vehicle.target_component}"
    )

    # Request GLOBAL_POSITION_INT at 5 Hz
    request_message_interval(
        vehicle,
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        5,
    )

    print("Waiting for position telemetry...")

    msg = vehicle.recv_match(
        type="GLOBAL_POSITION_INT",
        blocking=True,
        timeout=10,
    )

    if msg is None:
        raise RuntimeError("GLOBAL_POSITION_INT still not being received.")

    print(
        f"Position telemetry received: "
        f"lat={msg.lat / 1e7:.7f}, "
        f"lon={msg.lon / 1e7:.7f}, "
        f"relative_alt={msg.relative_alt / 1000:.2f} m"
    )

    return vehicle


def set_mode(vehicle, mode_name):
    mode_mapping = vehicle.mode_mapping()

    if mode_name not in mode_mapping:
        raise RuntimeError(
            f"Mode {mode_name} unavailable. "
            f"Available modes: {list(mode_mapping.keys())}"
        )

    mode_id = mode_mapping[mode_name]

    print(f"Setting mode to {mode_name}...")

    vehicle.mav.set_mode_send(
        vehicle.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )

    while True:
        heartbeat = vehicle.recv_match(
            type="HEARTBEAT",
            blocking=True,
            timeout=5,
        )

        if heartbeat is None:
            raise TimeoutError("Timed out waiting for mode change.")

        current_mode = mavutil.mode_string_v10(heartbeat)

        if current_mode == mode_name:
            print(f"Mode changed to {mode_name}.")
            return

def get_position(vehicle, timeout=10):
    start = time.time()

    while time.time() - start < timeout:
        msg = vehicle.recv_match(
            type="GLOBAL_POSITION_INT",
            blocking=True,
            timeout=1,
        )

        if msg is None:
            continue

        if msg.lat == 0 or msg.lon == 0:
            continue

        return (
            msg.lat / 1e7,
            msg.lon / 1e7,
            msg.relative_alt / 1000.0,
        )

    raise TimeoutError("Could not get current vehicle position.")

def offset_position(lat, lon, north_m=0, east_m=0):
    earth_radius = 6_378_137.0

    delta_lat = north_m / earth_radius
    delta_lon = east_m / (
        earth_radius * math.cos(math.radians(lat))
    )

    new_lat = lat + math.degrees(delta_lat)
    new_lon = lon + math.degrees(delta_lon)

    return new_lat, new_lon

def distance_m(lat1, lon1, lat2, lon2):
    earth_radius = 6_378_137.0

    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)

    delta_lat = math.radians(lat2 - lat1)
    delta_lon = math.radians(lon2 - lon1)

    a = (
        math.sin(delta_lat / 2) ** 2
        + math.cos(lat1_rad)
        * math.cos(lat2_rad)
        * math.sin(delta_lon / 2) ** 2
    )

    c = 2 * math.atan2(
        math.sqrt(a),
        math.sqrt(1 - a),
    )

    return earth_radius * c

def arm(vehicle, timeout=10):
    print("Arming...")

    vehicle.mav.command_long_send(
        vehicle.target_system,
        mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,  # arm
        0,
        0,
        0,
        0,
        0,
        0,
    )

    start_time = time.time()

    while time.time() - start_time < timeout:

        msg = vehicle.recv_match(
            type=["COMMAND_ACK", "HEARTBEAT", "STATUSTEXT"],
            blocking=True,
            timeout=1,
        )

        if msg is None:
            continue

        msg_type = msg.get_type()

        if msg_type == "COMMAND_ACK":
            if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                print(f"Arm command ACK result: {msg.result}")

                if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    # Don't immediately quit; STATUSTEXT may follow
                    print("Arming command was not accepted.")

        elif msg_type == "STATUSTEXT":
            print(f"[ArduPilot] {msg.text}")

        elif msg_type == "HEARTBEAT":
            armed = bool(
                msg.base_mode
                & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )

            if armed:
                print("Vehicle armed.")
                return

    raise RuntimeError(
        "Vehicle failed to arm. "
        "Check the ArduPilot status messages above."
    )

def wait_for_navigation(vehicle, timeout=30):
    print("Waiting for GPS/navigation initialization...")

    start_time = time.time()
    gps_ready = False
    position_ready = False

    while time.time() - start_time < timeout:

        msg = vehicle.recv_match(
            type=["GPS_RAW_INT", "GLOBAL_POSITION_INT", "STATUSTEXT"],
            blocking=True,
            timeout=1,
        )

        if msg is None:
            continue

        msg_type = msg.get_type()

        if msg_type == "GPS_RAW_INT":
            if msg.fix_type >= 3:
                gps_ready = True
                print(
                    f"GPS fix acquired: type={msg.fix_type}, "
                    f"satellites={msg.satellites_visible}"
                )

        elif msg_type == "GLOBAL_POSITION_INT":
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7

            if msg.lat != 0 and msg.lon != 0:
                position_ready = True
                print(
                    f"Position ready: "
                    f"lat={lat:.7f}, lon={lon:.7f}"
                )

        elif msg_type == "STATUSTEXT":
            print(f"[ArduPilot] {msg.text}")

        if gps_ready and position_ready:
            print("Navigation system ready.")
            return

    raise TimeoutError(
        "Navigation did not become ready within "
        f"{timeout} seconds."
    )

def wait_for_home(vehicle, timeout=30):
    print("Waiting for EKF/home initialization...")

    start = time.time()
    last_request = 0

    while time.time() - start < timeout:

        # Request HOME_POSITION roughly once per second.
        if time.time() - last_request >= 1:
            vehicle.mav.command_long_send(
                vehicle.target_system,
                0,
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                0,
                mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            last_request = time.time()

        msg = vehicle.recv_match(
            type=["HOME_POSITION", "STATUSTEXT"],
            blocking=True,
            timeout=1,
        )

        if msg is None:
            continue

        if msg.get_type() == "STATUSTEXT":
            print(f"[ArduPilot] {msg.text}")
            continue

        if msg.get_type() == "HOME_POSITION":
            lat = msg.latitude / 1e7
            lon = msg.longitude / 1e7

            if msg.latitude != 0 and msg.longitude != 0:
                print(
                    f"Home position ready: "
                    f"lat={lat:.7f}, lon={lon:.7f}"
                )
                print("EKF/home initialization complete.")
                return

    raise TimeoutError(
        f"Home position was not established within {timeout} seconds."
    )

def takeoff(vehicle, altitude):
    print(f"Taking off to {altitude} m...")

    vehicle.mav.command_long_send(
        vehicle.target_system,
        mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        altitude,
    )

    start_time = time.time()

    while time.time() - start_time < 30:
        msg = vehicle.recv_match(
            type="GLOBAL_POSITION_INT",
            blocking=True,
            timeout=2,
        )

        if msg is None:
            print("Waiting for altitude telemetry...")
            continue

        relative_altitude = msg.relative_alt / 1000.0

        print(f"Altitude: {relative_altitude:.2f} m")

        if relative_altitude >= altitude * 0.95:
            print("Target altitude reached.")
            return

    raise TimeoutError(
        f"Vehicle did not reach {altitude} m within 30 seconds."
    )

def goto(vehicle, lat, lon, altitude):
    print(
        f"Sending position target:\n"
        f"  lat={lat:.7f}\n"
        f"  lon={lon:.7f}\n"
        f"  alt={altitude:.1f} m"
    )

    vehicle.mav.set_position_target_global_int_send(
        0,
        vehicle.target_system,
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,

        # Position only.
        0x0DF8,

        int(lat * 1e7),
        int(lon * 1e7),
        altitude,

        # Velocity ignored
        0,
        0,
        0,

        # Acceleration ignored
        0,
        0,
        0,

        # Yaw / yaw rate ignored
        0,
        0,
    )

def wait_until_arrival(
    vehicle,
    target_lat,
    target_lon,
    arrival_radius=1.5,
    timeout=30,
):
    print("Flying to target...")

    start = time.time()
    last_print = 0

    while time.time() - start < timeout:
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

        # Don't spam at 5 Hz.
        if time.time() - last_print >= 1:
            print(f"Distance remaining: {remaining:.2f} m")
            last_print = time.time()

        if remaining <= arrival_radius:
            print(
                f"Target reached. "
                f"Final horizontal error: {remaining:.2f} m"
            )
            return

    raise TimeoutError(
        "Vehicle failed to reach target within "
        f"{timeout} seconds."
    )

def land(vehicle):
    print("Landing...")

    set_mode(vehicle, "LAND")

    while vehicle.motors_armed():
        msg = vehicle.recv_match(
            type="GLOBAL_POSITION_INT",
            blocking=True,
            timeout=5,
        )

        if msg:
            altitude = msg.relative_alt / 1000.0
            print(f"Altitude: {altitude:.2f} m")

    print("Landed and disarmed.")


def main():
    vehicle = connect()

    wait_for_home(vehicle)

    set_mode(vehicle, "GUIDED")

    arm(vehicle)

    takeoff(vehicle, TARGET_ALTITUDE)

    print("Takeoff complete.")

    current_lat, current_lon, current_alt = get_position(vehicle)

    print(
        f"Current position: "
        f"{current_lat:.7f}, {current_lon:.7f}"
    )

    # Generate target 20 metres north.
    target_lat, target_lon = offset_position(
        current_lat,
        current_lon,
        north_m=20,
    )

    print(
        f"Target position: "
        f"{target_lat:.7f}, {target_lon:.7f}"
    )

    goto(
        vehicle,
        target_lat,
        target_lon,
        TARGET_ALTITUDE,
    )

    wait_until_arrival(
        vehicle,
        target_lat,
        target_lon,
    )

    print("Hovering at target for 3 seconds...")
    time.sleep(3)

    land(vehicle)

    print("Mission complete.")


if __name__ == "__main__":
    main()