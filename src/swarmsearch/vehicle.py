import asyncio
import threading
import time
from collections import defaultdict, deque
from collections.abc import Callable
from typing import Optional

from pymavlink import mavutil

from connect import (
    offset_position,
    distance_m,
)

from .events import SwarmEvent
from .models import (
    UAVState,
    UAVStatus,
    Waypoint,
)
from .state_machine import validate_transition


EventSink = Callable[[SwarmEvent], None]


class VehicleUnavailableError(RuntimeError):
    """
    Raised when a vehicle has been withdrawn from the mission
    while an operation is still executing.
    """

    pass


class Vehicle:
    def __init__(
        self,
        name: str,
        connection_string: str,
        cruise_altitude: float,
        event_sink: Optional[EventSink] = None,
    ):
        self.name = name
        self.connection_string = connection_string
        self.cruise_altitude = cruise_altitude

        self.connection = None

        self.home_lat: Optional[float] = None
        self.home_lon: Optional[float] = None

        self.status = UAVStatus(
            name=name,
            system_id=0,
        )

        self._event_sink = (
            event_sink
            if event_sink is not None
            else lambda event: None
        )

        self._failure_requested = threading.Event()
        self._failure_reason: Optional[str] = None

        self._receiver_stop = threading.Event()

        self._receiver_thread: Optional[
            threading.Thread
        ] = None

        self._condition = threading.Condition()

        self._heartbeat_received = False

        self._home_position: Optional[
            tuple[float, float]
        ] = None

        self._ekf_flags: Optional[int] = None

        self._command_acks = defaultdict(
            deque
        )

    # ==================================================
    # EVENTS
    # ==================================================

    def _emit(
        self,
        event_type: str,
        message: str,
        **data,
    ):
        self._event_sink(
            SwarmEvent(
                event_type=event_type,
                source=self.name,
                message=message,
                data=data,
            )
        )

    # ==================================================
    # STATE MACHINE
    # ==================================================

    def _set_state(
        self,
        new_state: UAVState,
    ):
        old_state = self.status.state

        validate_transition(
            old_state,
            new_state,
        )

        self.status.state = new_state

        self._emit(
            "uav.state_changed",
            (
                f"{old_state.value} "
                f"-> {new_state.value}"
            ),
            previous_state=old_state.value,
            state=new_state.value,
        )

    def transition_to(
        self,
        state: UAVState,
    ):
        self._set_state(
            state
        )

    # ==================================================
    # FAILURE CONTROL
    # ==================================================

    @property
    def failure_requested(self) -> bool:
        return self._failure_requested.is_set()

    @property
    def failure_reason(self) -> Optional[str]:
        return self._failure_reason

    def request_failure(
        self,
        reason: str = "failure injected",
    ):
        if self.status.state in {
            UAVState.FAILED,
            UAVState.LANDED,
        }:
            return

        if self._failure_requested.is_set():
            return

        self._failure_reason = reason
        self._failure_requested.set()

        self._emit(
            "uav.failure_requested",
            reason,
        )

        with self._condition:
            self._condition.notify_all()

    def _raise_if_failure_requested(
        self,
    ):
        if self._failure_requested.is_set():
            raise VehicleUnavailableError(
                self._failure_reason
                or f"{self.name} unavailable"
            )

    # ==================================================
    # CONNECTION
    # ==================================================

    async def connect(self):
        self._set_state(
            UAVState.INITIALIZING
        )

        await asyncio.to_thread(
            self._connect_blocking
        )

    def _connect_blocking(self):
        self._emit(
            "uav.connecting",
            (
                f"Connecting to "
                f"{self.connection_string}"
            ),
        )

        self.connection = (
            mavutil.mavlink_connection(
                self.connection_string
            )
        )

        self._start_receiver()

        self._wait_for_heartbeat(
            timeout=30
        )

        self._request_message_interval(
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            5,
        )

        self._emit(
            "uav.connected",
            (
                f"Connected "
                f"(sysid={self.status.system_id})"
            ),
            system_id=self.status.system_id,
        )

    # ==================================================
    # DEDICATED MAVLINK RECEIVER
    # ==================================================

    def _start_receiver(self):
        if (
            self._receiver_thread is not None
            and self._receiver_thread.is_alive()
        ):
            return

        self._receiver_stop.clear()

        self._receiver_thread = (
            threading.Thread(
                target=self._receiver_loop,
                name=(
                    f"{self.name}-mavlink-rx"
                ),
                daemon=True,
            )
        )

        self._receiver_thread.start()

    def _receiver_loop(self):
        while not self._receiver_stop.is_set():
            try:
                msg = (
                    self.connection.recv_match(
                        blocking=True,
                        timeout=0.5,
                    )
                )

            except Exception as exc:
                if self._receiver_stop.is_set():
                    return

                self._emit(
                    "uav.receiver_error",
                    (
                        "MAVLink receiver error: "
                        f"{exc}"
                    ),
                )

                time.sleep(0.1)
                continue

            if msg is None:
                continue

            msg_type = msg.get_type()

            if msg_type == "BAD_DATA":
                continue

            if msg_type == "HEARTBEAT":
                self._handle_heartbeat(
                    msg
                )

            elif (
                msg_type
                == "GLOBAL_POSITION_INT"
            ):
                self._handle_position(
                    msg
                )

            elif (
                msg_type
                == "HOME_POSITION"
            ):
                self._handle_home_position(
                    msg
                )

            elif (
                msg_type
                == "EKF_STATUS_REPORT"
            ):
                self._handle_ekf(
                    msg
                )

            elif (
                msg_type
                == "COMMAND_ACK"
            ):
                self._handle_command_ack(
                    msg
                )

            elif (
                msg_type
                == "STATUSTEXT"
            ):
                self._handle_statustext(
                    msg
                )

    def _handle_heartbeat(
        self,
        msg,
    ):
        system_id = (
            msg.get_srcSystem()
        )

        armed = bool(
            msg.base_mode
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )

        try:
            flight_mode = (
                mavutil.mode_string_v10(
                    msg
                )
            )

        except Exception:
            flight_mode = None

        with self._condition:
            self.status.system_id = (
                system_id
            )

            self.status.armed = armed

            self.status.flight_mode = (
                flight_mode
            )

            self._heartbeat_received = (
                True
            )

            self.connection.target_system = (
                system_id
            )

            try:
                self.connection.target_component = (
                    msg.get_srcComponent()
                )

            except Exception:
                pass

            self._condition.notify_all()

    def _handle_position(
        self,
        msg,
    ):
        latitude = (
            msg.lat / 1e7
        )

        longitude = (
            msg.lon / 1e7
        )

        altitude = (
            msg.relative_alt / 1000.0
        )

        with self._condition:
            self.status.latitude = (
                latitude
            )

            self.status.longitude = (
                longitude
            )

            self.status.altitude = (
                altitude
            )

            self._condition.notify_all()

        self._emit(
            "uav.telemetry",
            "Telemetry update",
            latitude=latitude,
            longitude=longitude,
            altitude=altitude,
            armed=self.status.armed,
            flight_mode=(
                self.status.flight_mode
            ),
            mission_state=(
                self.status.state.value
            ),
            healthy=(
                self.status.healthy
            ),
        )

    def _handle_home_position(
        self,
        msg,
    ):
        if (
            msg.latitude == 0
            or msg.longitude == 0
        ):
            return

        latitude = (
            msg.latitude / 1e7
        )

        longitude = (
            msg.longitude / 1e7
        )

        with self._condition:
            self._home_position = (
                latitude,
                longitude,
            )

            self._condition.notify_all()

    def _handle_ekf(
        self,
        msg,
    ):
        with self._condition:
            self._ekf_flags = (
                msg.flags
            )

            self._condition.notify_all()

    def _handle_command_ack(
        self,
        msg,
    ):
        with self._condition:
            self._command_acks[
                msg.command
            ].append(
                msg.result
            )

            self._condition.notify_all()

    def _handle_statustext(
        self,
        msg,
    ):
        try:
            text = msg.text

            if isinstance(
                text,
                bytes,
            ):
                text = text.decode(
                    errors="replace"
                )

            text = text.rstrip(
                "\x00"
            )

        except Exception:
            return

        self._emit(
            "uav.autopilot_text",
            text,
        )

    # ==================================================
    # LOW-LEVEL WAIT HELPERS
    # ==================================================

    def _wait_for_heartbeat(
        self,
        timeout: float,
    ):
        deadline = (
            time.monotonic()
            + timeout
        )

        with self._condition:
            while not self._heartbeat_received:
                self._raise_if_failure_requested()

                remaining = (
                    deadline
                    - time.monotonic()
                )

                if remaining <= 0:
                    raise TimeoutError(
                        f"{self.name}: "
                        "heartbeat timeout."
                    )

                self._condition.wait(
                    timeout=min(
                        0.25,
                        remaining,
                    )
                )

    def _wait_for_predicate(
        self,
        predicate,
        timeout: float,
        timeout_message: str,
        failure_sensitive: bool = True,
    ):
        deadline = (
            time.monotonic()
            + timeout
        )

        with self._condition:
            while not predicate():

                if failure_sensitive:
                    self._raise_if_failure_requested()

                remaining = (
                    deadline
                    - time.monotonic()
                )

                if remaining <= 0:
                    raise TimeoutError(
                        timeout_message
                    )

                self._condition.wait(
                    timeout=min(
                        0.20,
                        remaining,
                    )
                )

    # ==================================================
    # MESSAGE REQUESTS
    # ==================================================

    def _request_message_interval(
        self,
        message_id: int,
        frequency_hz: float,
    ):
        if frequency_hz <= 0:
            interval_us = -1

        else:
            interval_us = int(
                1_000_000
                / frequency_hz
            )

        self.connection.mav.command_long_send(
            self.connection.target_system,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
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

    def _request_message(
        self,
        message_id: int,
    ):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            message_id,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    # ==================================================
    # INITIALIZATION
    # ==================================================

    async def initialize(self):
        if self.connection is None:
            raise RuntimeError(
                f"{self.name} is not connected."
            )

        home = await asyncio.to_thread(
            self._initialize_blocking
        )

        (
            self.home_lat,
            self.home_lon,
        ) = home

        self._set_state(
            UAVState.READY
        )

        self._emit(
            "uav.ready",
            "Navigation system ready",
            home_lat=self.home_lat,
            home_lon=self.home_lon,
        )

        return home

    def _initialize_blocking(self):
        home = (
            self._wait_for_home_position()
        )

        self._wait_for_position_estimate()

        return home

    def _wait_for_home_position(
        self,
        timeout: float = 45,
    ):
        self._emit(
            "uav.home_wait",
            "Waiting for home position",
        )

        deadline = (
            time.monotonic()
            + timeout
        )

        next_request = 0.0

        while True:
            self._raise_if_failure_requested()

            with self._condition:
                if (
                    self._home_position
                    is not None
                ):
                    home = (
                        self._home_position
                    )

                    self._emit(
                        "uav.home_ready",
                        (
                            "Home position "
                            "established"
                        ),
                        latitude=home[0],
                        longitude=home[1],
                    )

                    return home

            now = time.monotonic()

            if now >= next_request:
                self._request_message(
                    mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION
                )

                next_request = (
                    now + 1.0
                )

            if now >= deadline:
                raise TimeoutError(
                    f"{self.name}: "
                    "home position "
                    "not established."
                )

            with self._condition:
                self._condition.wait(
                    timeout=0.20
                )

    def _wait_for_position_estimate(
        self,
        timeout: float = 45,
    ):
        self._request_message_interval(
            193,
            2,
        )

        self._emit(
            "uav.ekf_wait",
            (
                "Waiting for EKF "
                "position estimate"
            ),
        )

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

        deadline = (
            time.monotonic()
            + timeout
        )

        while True:
            self._raise_if_failure_requested()

            with self._condition:
                flags = (
                    self._ekf_flags
                )

                if flags is not None:
                    ready = (
                        flags & required
                    ) == required

                    unhealthy = bool(
                        flags
                        & (
                            EKF_CONST_POS_MODE
                            | EKF_UNINITIALIZED
                            | EKF_GPS_GLITCHING
                        )
                    )

                    if (
                        ready
                        and not unhealthy
                    ):
                        self._emit(
                            "uav.ekf_ready",
                            (
                                "EKF position "
                                "estimate ready"
                            ),
                            flags=flags,
                        )

                        return

            if (
                time.monotonic()
                >= deadline
            ):
                raise TimeoutError(
                    f"{self.name}: "
                    "EKF position estimate "
                    "not ready."
                )

            with self._condition:
                self._condition.wait(
                    timeout=0.20
                )

    # ==================================================
    # MODE CONTROL
    # ==================================================

    def _set_mode_blocking(
        self,
        mode_name: str,
        timeout: float = 10,
        failure_sensitive: bool = True,
    ):
        if failure_sensitive:
            self._raise_if_failure_requested()

        mapping = (
            self.connection.mode_mapping()
        )

        if (
            mapping is None
            or mode_name not in mapping
        ):
            raise RuntimeError(
                f"{self.name}: mode "
                f"{mode_name} unavailable."
            )

        mode_id = mapping[
            mode_name
        ]

        self.connection.mav.set_mode_send(
            self.connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )

        self._emit(
            "uav.mode_requested",
            (
                f"Mode requested: "
                f"{mode_name}"
            ),
            mode=mode_name,
        )

        self._wait_for_predicate(
            predicate=lambda: (
                self.status.flight_mode
                == mode_name
            ),
            timeout=timeout,
            timeout_message=(
                f"{self.name}: "
                f"failed to enter "
                f"{mode_name} mode."
            ),
            failure_sensitive=(
                failure_sensitive
            ),
        )

        self._emit(
            "uav.mode_changed",
            (
                f"Mode changed to "
                f"{mode_name}"
            ),
            mode=mode_name,
        )

    # ==================================================
    # COMMAND ACK HELPERS
    # ==================================================

    def _clear_command_acks(
        self,
        command: int,
    ):
        with self._condition:
            self._command_acks[
                command
            ].clear()

    def _wait_for_command_ack(
        self,
        command: int,
        timeout: float = 5,
        failure_sensitive: bool = True,
    ) -> Optional[int]:
        deadline = (
            time.monotonic()
            + timeout
        )

        with self._condition:
            while True:
                queue = (
                    self._command_acks[
                        command
                    ]
                )

                if queue:
                    return (
                        queue.popleft()
                    )

                if failure_sensitive:
                    self._raise_if_failure_requested()

                remaining = (
                    deadline
                    - time.monotonic()
                )

                if remaining <= 0:
                    return None

                self._condition.wait(
                    timeout=min(
                        0.20,
                        remaining,
                    )
                )

    # ==================================================
    # LAUNCH
    # ==================================================

    async def launch(self):
        if self.connection is None:
            raise RuntimeError(
                f"{self.name} not connected."
            )

        self._raise_if_failure_requested()

        self._set_state(
            UAVState.ARMING
        )

        await asyncio.to_thread(
            self._arm_blocking
        )

        self._raise_if_failure_requested()

        self._set_state(
            UAVState.TAKING_OFF
        )

        self._emit(
            "uav.takeoff_started",
            (
                f"Taking off to "
                f"{self.cruise_altitude:.1f}m"
            ),
            target_altitude=(
                self.cruise_altitude
            ),
        )

        await asyncio.to_thread(
            self._takeoff_blocking,
            self.cruise_altitude,
        )

        self._raise_if_failure_requested()

        self._set_state(
            UAVState.AIRBORNE
        )

        self._emit(
            "uav.airborne",
            (
                f"Airborne at "
                f"{self.status.altitude:.1f}m"
            ),
            altitude=(
                self.status.altitude
            ),
        )

    # ==================================================
    # ARMING
    # ==================================================

    def _arm_blocking(
        self,
        timeout: float = 20,
    ):
        self._raise_if_failure_requested()

        self._set_mode_blocking(
            "GUIDED",
            failure_sensitive=True,
        )

        self._raise_if_failure_requested()

        command = (
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        )

        self._clear_command_acks(
            command
        )

        self.connection.mav.command_long_send(
            self.connection.target_system,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            command,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )

        self._emit(
            "uav.arming_started",
            "Arming motors",
        )

        ack = (
            self._wait_for_command_ack(
                command,
                timeout=5,
            )
        )

        if ack is not None:
            accepted = {
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            }

            if ack not in accepted:
                raise RuntimeError(
                    f"{self.name}: "
                    "arm command rejected "
                    f"with result {ack}."
                )

        self._wait_for_predicate(
            predicate=lambda: (
                self.status.armed
            ),
            timeout=timeout,
            timeout_message=(
                f"{self.name}: "
                "failed to arm."
            ),
        )

        self._emit(
            "uav.armed",
            "Vehicle armed",
        )

    # ==================================================
    # TAKEOFF
    # ==================================================

    def _takeoff_blocking(
        self,
        target_altitude: float,
        timeout: float = 45,
    ):
        self._raise_if_failure_requested()

        command = (
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        )

        self._clear_command_acks(
            command
        )

        self.connection.mav.command_long_send(
            self.connection.target_system,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            command,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            target_altitude,
        )

        ack = (
            self._wait_for_command_ack(
                command,
                timeout=5,
            )
        )

        if ack is not None:
            accepted = {
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            }

            if ack not in accepted:
                raise RuntimeError(
                    f"{self.name}: "
                    "takeoff rejected "
                    f"with result {ack}."
                )

        self._wait_for_predicate(
            predicate=lambda: (
                self.status.altitude
                >= target_altitude * 0.95
            ),
            timeout=timeout,
            timeout_message=(
                f"{self.name}: "
                "takeoff timeout."
            ),
        )

    # ==================================================
    # WAYPOINT NAVIGATION
    # ==================================================

    async def fly_to(
        self,
        waypoint: Waypoint,
        origin_lat: float,
        origin_lon: float,
        arrival_radius: float = 2.0,
        timeout: float = 30,
    ) -> float:
        self._raise_if_failure_requested()

        return await asyncio.to_thread(
            self._fly_to_blocking,
            waypoint,
            origin_lat,
            origin_lon,
            arrival_radius,
            timeout,
        )

    def _fly_to_blocking(
        self,
        waypoint: Waypoint,
        origin_lat: float,
        origin_lon: float,
        arrival_radius: float,
        timeout: float,
    ) -> float:
        self._raise_if_failure_requested()

        (
            target_lat,
            target_lon,
        ) = offset_position(
            origin_lat,
            origin_lon,
            north_m=waypoint.north,
            east_m=waypoint.east,
        )

        self._send_position_target(
            target_lat,
            target_lon,
            self.cruise_altitude,
        )

        self._emit(
            "uav.waypoint_started",
            (
                f"Flying to "
                f"east={waypoint.east:.1f}, "
                f"north={waypoint.north:.1f}"
            ),
            east=waypoint.east,
            north=waypoint.north,
        )

        deadline = (
            time.monotonic()
            + timeout
        )

        while True:
            self._raise_if_failure_requested()

            with self._condition:
                latitude = (
                    self.status.latitude
                )

                longitude = (
                    self.status.longitude
                )

                if (
                    latitude is not None
                    and longitude is not None
                ):
                    remaining = (
                        distance_m(
                            latitude,
                            longitude,
                            target_lat,
                            target_lon,
                        )
                    )

                    if (
                        remaining
                        <= arrival_radius
                    ):
                        self.status.completed_waypoints += 1

                        self._emit(
                            "uav.waypoint_reached",
                            (
                                "Waypoint reached "
                                f"(error="
                                f"{remaining:.2f}m)"
                            ),
                            east=waypoint.east,
                            north=waypoint.north,
                            error_m=remaining,
                        )

                        return remaining

            if (
                time.monotonic()
                >= deadline
            ):
                raise TimeoutError(
                    f"{self.name}: "
                    "waypoint timeout."
                )

            with self._condition:
                self._condition.wait(
                    timeout=0.10
                )

    def _send_position_target(
        self,
        latitude: float,
        longitude: float,
        altitude: float,
    ):
        type_mask = (
            0b0000111111111000
        )

        self.connection.mav.set_position_target_global_int_send(
            0,
            self.connection.target_system,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            int(latitude * 1e7),
            int(longitude * 1e7),
            altitude,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

        self._emit(
            "uav.position_target_sent",
            "Position target sent",
            latitude=latitude,
            longitude=longitude,
            altitude=altitude,
        )

    # ==================================================
    # LANDING
    # ==================================================

    async def land(self):
        if self.connection is None:
            return

        if not self.status.armed:
            if (
                self.status.state
                not in {
                    UAVState.FAILED,
                    UAVState.LANDED,
                }
            ):
                if (
                    self.status.state
                    != UAVState.LANDING
                ):
                    self._set_state(
                        UAVState.LANDING
                    )

                self._set_state(
                    UAVState.LANDED
                )

            return

        self._raise_if_failure_requested()

        if (
            self.status.state
            != UAVState.LANDING
        ):
            self._set_state(
                UAVState.LANDING
            )

        self._emit(
            "uav.landing_started",
            "Landing",
        )

        await asyncio.to_thread(
            self._land_blocking,
            True,
        )

        self._raise_if_failure_requested()

        self.status.altitude = 0.0

        self._set_state(
            UAVState.LANDED
        )

        self._emit(
            "uav.landed",
            "Landed and disarmed",
        )

    def _land_blocking(
        self,
        failure_sensitive: bool,
        timeout: float = 90,
    ):
        self._set_mode_blocking(
            "LAND",
            failure_sensitive=(
                failure_sensitive
            ),
        )

        self._wait_for_predicate(
            predicate=lambda: (
                not self.status.armed
            ),
            timeout=timeout,
            timeout_message=(
                f"{self.name}: "
                "landing timeout."
            ),
            failure_sensitive=(
                failure_sensitive
            ),
        )

    # ==================================================
    # FAILURE FINALIZATION
    # ==================================================

    async def fail(
        self,
        reason: str = "failure injected",
    ):
        if (
            self.status.state
            == UAVState.FAILED
        ):
            return

        if (
            self.status.state
            == UAVState.LANDED
        ):
            return

        failure_phase = (
            self.status.state
        )

        self._failure_reason = reason
        self._failure_requested.set()

        self.status.healthy = False
        self.status.failure_reason = reason

        self._set_state(
            UAVState.FAILED
        )

        self._emit(
            "uav.failed",
            reason,
            failure_phase=(
                failure_phase.value
            ),
        )

        if self.connection is None:
            return

        # ----------------------------------------------
        # GROUND / ARMING FAILURE
        # ----------------------------------------------
        #
        # If an ARM command was already sent, sending DISARM after it
        # closes the race where our cached heartbeat has not yet
        # reported the armed state.
        #

        if failure_phase in {
            UAVState.READY,
            UAVState.ARMING,
        }:
            if (
                failure_phase
                == UAVState.ARMING
            ):
                try:
                    await asyncio.to_thread(
                        self._disarm_after_failure_blocking
                    )

                except Exception as exc:
                    self._emit(
                        "uav.failure_disarm_failed",
                        (
                            "Unable to guarantee "
                            "safe disarm after "
                            f"arming failure: {exc}"
                        ),
                    )

            return

        # ----------------------------------------------
        # AIRBORNE / FLIGHT FAILURE
        # ----------------------------------------------

        should_land = (
            self.status.armed
            or self.status.altitude > 0.5
            or failure_phase
            in {
                UAVState.TAKING_OFF,
                UAVState.AIRBORNE,
                UAVState.SEARCHING,
                UAVState.RECOVERING,
                UAVState.RETURNING,
                UAVState.LANDING,
            }
        )

        if should_land:
            try:
                await asyncio.to_thread(
                    self._land_after_failure_blocking
                )

            except Exception as exc:
                self._emit(
                    "uav.failure_land_failed",
                    (
                        "Unable to request "
                        "safe LAND after failure: "
                        f"{exc}"
                    ),
                )

    def _disarm_after_failure_blocking(
        self,
        timeout: float = 10,
    ):
        command = (
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        )

        self._clear_command_acks(
            command
        )

        self.connection.mav.command_long_send(
            self.connection.target_system,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            command,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

        self._emit(
            "uav.failure_disarm_requested",
            (
                "Safe DISARM requested "
                "after arming-phase failure"
            ),
        )

        ack = (
            self._wait_for_command_ack(
                command,
                timeout=3,
                failure_sensitive=False,
            )
        )

        if ack is not None:
            accepted = {
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            }

            if ack not in accepted:
                raise RuntimeError(
                    "disarm command rejected "
                    f"with result {ack}"
                )

        self._wait_for_predicate(
            predicate=lambda: (
                not self.status.armed
            ),
            timeout=timeout,
            timeout_message=(
                f"{self.name}: "
                "vehicle remained armed "
                "after failure."
            ),
            failure_sensitive=False,
        )

        self._emit(
            "uav.failure_disarmed",
            (
                "Vehicle confirmed "
                "disarmed after failure"
            ),
        )

    def _land_after_failure_blocking(
        self,
    ):
        self._set_mode_blocking(
            "LAND",
            timeout=10,
            failure_sensitive=False,
        )

        self._emit(
            "uav.failure_land_requested",
            (
                "Safe LAND requested "
                "after fault injection"
            ),
        )

    # ==================================================
    # PHYSICAL STATE HELPERS
    # ==================================================

    async def wait_until_disarmed(
        self,
        timeout: float = 90,
    ):
        await asyncio.to_thread(
            self._wait_until_disarmed_blocking,
            timeout,
        )

    def _wait_until_disarmed_blocking(
        self,
        timeout: float,
    ):
        self._wait_for_predicate(
            predicate=lambda: (
                not self.status.armed
            ),
            timeout=timeout,
            timeout_message=(
                f"{self.name}: "
                "did not disarm."
            ),
            failure_sensitive=False,
        )

    # ==================================================
    # SHUTDOWN
    # ==================================================

    def close(self):
        self._receiver_stop.set()

        with self._condition:
            self._condition.notify_all()

        thread = self._receiver_thread

        if (
            thread is not None
            and thread.is_alive()
        ):
            thread.join(
                timeout=2
            )