import asyncio
import time

from pymavlink import mavutil

from connect import (
    request_message_interval,
    set_mode,
    arm,
    takeoff,
    land,
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

# Simulated fault: UAV-2 fails after it has completed 2 waypoints.
FAILURE_UAV = "UAV-2"
FAIL_AFTER_COMPLETED = 2


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


def wait_for_home_position(vehicle, timeout=30):
    """
    Wait until ArduPilot has established HOME_POSITION and return
    (latitude, longitude).

    We intentionally use HOME_POSITION instead of the vehicle's current
    GPS position so the swarm has a stable shared mission origin even if
    a vehicle has moved before the mission begins.
    """
    print("Waiting for EKF/home initialization...")

    start = time.time()
    last_request = 0.0

    while time.time() - start < timeout:
        # HOME_POSITION is not necessarily streamed continuously, so request it.
        if time.time() - last_request >= 1.0:
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

        lat = msg.latitude / 1e7
        lon = msg.longitude / 1e7

        if msg.latitude != 0 and msg.longitude != 0:
            print(
                f"Home position ready: "
                f"lat={lat:.7f}, lon={lon:.7f}"
            )
            print("EKF/home initialization complete.")
            return lat, lon

    raise TimeoutError(
        f"Home position was not established within {timeout} seconds."
    )


def prepare_vehicle(name, vehicle):
    home = wait_for_home_position(vehicle)

    wait_for_position_estimate(vehicle)

    print(f"[{name}] READY")

    return home


def launch_vehicle(name, vehicle):
    altitude = ALTITUDES[name]

    print(f"[{name}] TAKEOFF -> {altitude}m")

    set_mode(vehicle, "GUIDED")
    arm(vehicle)
    takeoff(vehicle, altitude)

    print(f"[{name}] AIRBORNE @ {altitude}m")


def execute_waypoint(
    name,
    vehicle,
    waypoint,
    origin_lat,
    origin_lon,
):
    east_m, north_m = waypoint

    target_lat, target_lon = offset_position(
        origin_lat,
        origin_lon,
        north_m=north_m,
        east_m=east_m,
    )

    goto(
        vehicle,
        target_lat,
        target_lon,
        ALTITUDES[name],
    )

    start = time.time()

    while time.time() - start < WAYPOINT_TIMEOUT:
        msg = vehicle.recv_match(
            type="GLOBAL_POSITION_INT",
            blocking=True,
            timeout=1,
        )

        if msg is None:
            continue

        remaining = distance_m(
            msg.lat / 1e7,
            msg.lon / 1e7,
            target_lat,
            target_lon,
        )

        if remaining <= ARRIVAL_RADIUS_M:
            return remaining

    raise TimeoutError(
        f"{name} failed to reach waypoint {waypoint}"
    )

def wait_for_position_estimate(vehicle, timeout=30):
    print("Waiting for valid EKF position estimate...")

    # EKF_STATUS_REPORT = message 193
    request_message_interval(
        vehicle,
        193,
        2,
    )

    start = time.time()

    while time.time() - start < timeout:
        msg = vehicle.recv_match(
            type=["EKF_STATUS_REPORT", "STATUSTEXT"],
            blocking=True,
            timeout=1,
        )

        if msg is None:
            continue

        if msg.get_type() == "STATUSTEXT":
            print(f"[ArduPilot] {msg.text}")
            continue

        flags = msg.flags

        # ArduPilotMega EKF_STATUS_FLAGS
        EKF_ATTITUDE = 1
        EKF_VELOCITY_HORIZ = 2
        EKF_POS_HORIZ_ABS = 16
        EKF_POS_VERT_ABS = 32
        EKF_CONST_POS_MODE = 128
        EKF_UNINITIALIZED = 1024
        EKF_GPS_GLITCHING = 32768

        required = (
            EKF_ATTITUDE
            | EKF_VELOCITY_HORIZ
            | EKF_POS_HORIZ_ABS
            | EKF_POS_VERT_ABS
        )

        required_ready = (flags & required) == required

        unhealthy = (
            flags & EKF_CONST_POS_MODE
            or flags & EKF_UNINITIALIZED
            or flags & EKF_GPS_GLITCHING
        )

        if required_ready and not unhealthy:
            print(
                f"EKF position estimate ready "
                f"(flags={flags})"
            )
            return

    raise TimeoutError(
        "EKF did not obtain a valid position estimate "
        f"within {timeout} seconds."
    )

async def vehicle_worker(
    name,
    vehicle,
    queue,
    failure_queue,
    origin_lat,
    origin_lon,
):
    """
    Execute this vehicle's task queue.

    If this is FAILURE_UAV, inject a simulated failure after
    FAIL_AFTER_COMPLETED waypoints. The worker reports:
      - the last successfully completed waypoint
      - all still-unfinished ordered waypoints

    That lets the coordinator rebuild the failed UAV's exact unfinished
    coverage path rather than distributing isolated points.
    """
    completed = 0
    last_completed = None

    while True:
        task = await queue.get()

        if task is None:
            queue.task_done()
            return

        # -------------------------------------------------
        # FAULT INJECTION
        # -------------------------------------------------
        if (
            name == FAILURE_UAV
            and completed >= FAIL_AFTER_COMPLETED
        ):
            # The task just removed from the queue has NOT been flown.
            remaining_tasks = [task]

            # Drain the rest in their original order.
            while not queue.empty():
                pending = queue.get_nowait()

                if pending is not None:
                    remaining_tasks.append(pending)

                queue.task_done()

            # Mark the current unflown task as removed from the failed
            # vehicle's queue.
            queue.task_done()

            print()
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print(f"[FAULT] {name} FAILURE INJECTED")
            print(
                f"[FAULT] {len(remaining_tasks)} "
                f"waypoints unfinished"
            )
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print()

            # Withdraw this vehicle from the active mission.
            await asyncio.to_thread(
                set_mode,
                vehicle,
                "LAND",
            )

            await failure_queue.put(
                {
                    "vehicle": name,
                    "last_completed": last_completed,
                    "remaining": remaining_tasks,
                }
            )

            return

        # -------------------------------------------------
        # NORMAL EXECUTION
        # -------------------------------------------------
        east_m, north_m = task

        print(
            f"[{name}] Executing "
            f"east={east_m:.1f} "
            f"north={north_m:.1f}"
        )

        error = await asyncio.to_thread(
            execute_waypoint,
            name,
            vehicle,
            task,
            origin_lat,
            origin_lon,
        )

        completed += 1
        last_completed = task

        print(
            f"[{name}] waypoint complete "
            f"| error={error:.2f}m"
        )

        queue.task_done()


async def recovery_coordinator(
    queues,
    failure_queue,
):
    """
    Preserve the failed UAV's exact unfinished coverage path.

    Instead of round-robin distributing individual waypoints, choose one
    surviving UAV and append:

        last_completed -> remaining_wp_1 -> remaining_wp_2 -> ...

    The first point is a repositioning point. From there, the survivor
    retraces the exact ordered coverage segment that the failed UAV would
    have flown.
    """
    report = await failure_queue.get()

    failed_vehicle = report["vehicle"]
    last_completed = report["last_completed"]
    remaining = report["remaining"]

    survivors = [
        name
        for name in queues
        if name != failed_vehicle
    ]

    print(
        f"[SWARM] {failed_vehicle} removed "
        f"from active fleet"
    )

    # Pick the survivor with the lightest currently queued workload.
    # This is intentionally simple; the important correctness property is
    # that the recovery route remains ordered and intact.
    recovery_vehicle = min(
        survivors,
        key=lambda name: queues[name].qsize(),
    )

    if last_completed is not None:
        recovery_route = [
            last_completed,
            *remaining,
        ]
    else:
        recovery_route = list(remaining)

    print(
        f"[SWARM] Preserving failed coverage path "
        f"with {len(recovery_route)} route points"
    )

    print(
        f"[SWARM] Recovery assigned to "
        f"{recovery_vehicle}"
    )

    if last_completed is not None:
        print(
            f"[SWARM] {recovery_vehicle} will first "
            f"reposition to the failed UAV's last "
            f"completed waypoint {last_completed}"
        )

    for waypoint in recovery_route:
        await queues[recovery_vehicle].put(waypoint)

    print(
        f"[SWARM] {recovery_vehicle} receives "
        f"{len(recovery_route)} ordered recovery points"
    )

    failure_queue.task_done()

    return {
        "failed_vehicle": failed_vehicle,
        "recovery_vehicle": recovery_vehicle,
        "recovery_route": recovery_route,
    }


async def safe_land_vehicle(name, vehicle):
    """
    Land a vehicle only if it is still armed.

    The failed UAV is already placed into LAND when the fault is injected,
    so by mission end it may already be disarmed.
    """
    if not vehicle.motors_armed():
        print(f"[{name}] Already landed/disarmed")
        return

    print(f"[{name}] LANDING")
    await asyncio.to_thread(land, vehicle)
    print(f"[{name}] LANDED")


async def main():
    # ==========================================
    # CONNECT
    # ==========================================
    print("\n=== CONNECTING SWARM ===")

    results = await asyncio.gather(
        *[
            asyncio.to_thread(
                connect_vehicle,
                name,
                connection,
            )
            for name, connection in VEHICLES.items()
        ]
    )

    vehicles = dict(
        zip(VEHICLES.keys(), results)
    )

    # ==========================================
    # INITIALIZE + CAPTURE TRUE HOME POSITIONS
    # ==========================================
    home_results = await asyncio.gather(
        *[
            asyncio.to_thread(
                prepare_vehicle,
                name,
                vehicle,
            )
            for name, vehicle in vehicles.items()
        ]
    )

    homes = dict(
        zip(vehicles.keys(), home_results)
    )

    print("\n=== SWARM READY ===")

    # IMPORTANT:
    # Use UAV-1's HOME_POSITION, not UAV-1's current GPS position.
    origin_lat, origin_lon = homes["UAV-1"]

    print(
        f"[SWARM] Shared mission origin "
        f"(UAV-1 HOME) "
        f"{origin_lat:.7f}, {origin_lon:.7f}"
    )

    # ==========================================
    # PLAN SEARCH
    # ==========================================
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
            f"[{name}] assigned "
            f"{len(routes[name])} waypoints"
        )

    # ==========================================
    # TAKEOFF
    # ==========================================
    print("\n=== FLEET TAKEOFF ===")

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

    print("\n=== SWARM AIRBORNE ===")

    # ==========================================
    # CREATE DYNAMIC TASK QUEUES
    # ==========================================
    queues = {
        name: asyncio.Queue()
        for name in vehicles
    }

    for name, route in routes.items():
        for waypoint in route:
            await queues[name].put(waypoint)

    failure_queue = asyncio.Queue()

    # ==========================================
    # START VEHICLE WORKERS
    # ==========================================
    workers = {}

    for name, vehicle in vehicles.items():
        workers[name] = asyncio.create_task(
            vehicle_worker(
                name,
                vehicle,
                queues[name],
                failure_queue,
                origin_lat,
                origin_lon,
            )
        )

    recovery_task = asyncio.create_task(
        recovery_coordinator(
            queues,
            failure_queue,
        )
    )

    mission_start = time.time()

    # Wait for the injected failure and for recovery tasks to be appended.
    recovery_report = await recovery_task

    # Wait for all original + recovery work to finish.
    await asyncio.gather(
        *[
            queue.join()
            for queue in queues.values()
        ]
    )

    mission_duration = time.time() - mission_start

    # ==========================================
    # STOP SURVIVING WORKERS
    # ==========================================
    for name, worker in workers.items():
        if worker.done():
            continue

        await queues[name].put(None)

    await asyncio.gather(
        *workers.values(),
        return_exceptions=True,
    )

    print()
    print("==============================")
    print(" SEARCH OBJECTIVE COMPLETED")
    print("==============================")
    print(
        f"Mission duration: "
        f"{mission_duration:.2f}s"
    )
    print(
        f"Failed vehicle: "
        f"{recovery_report['failed_vehicle']}"
    )
    print(
        f"Recovery vehicle: "
        f"{recovery_report['recovery_vehicle']}"
    )
    print(
        "Coverage-path recovery: SUCCESS"
    )

    # ==========================================
    # LAND / CONFIRM ALL VEHICLES SAFE
    # ==========================================
    print("\n=== LANDING FLEET ===")

    await asyncio.gather(
        *[
            safe_land_vehicle(
                name,
                vehicle,
            )
            for name, vehicle in vehicles.items()
        ],
        return_exceptions=True,
    )

    print("\n=== MISSION COMPLETE ===")


if __name__ == "__main__":
    asyncio.run(main())
