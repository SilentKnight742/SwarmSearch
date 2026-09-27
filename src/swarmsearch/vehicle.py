import asyncio
import threading
import time
from collections.abc import Callable
from typing import Optional

from pymavlink import mavutil

from connect import (
    request_message_interval,
    set_mode,
    offset_position,
    distance_m,
    goto,
)

from .events import SwarmEvent
from .models import UAVState, UAVStatus, Waypoint
from .state_machine import validate_transition


EventSink = Callable[[SwarmEvent], None]


class VehicleUnavailableError(RuntimeError):
    """
    Raised when the coordinator marks a UAV unavailable
    while an operation is still running.
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

    # ==================================================
    # EVENTS / STATE
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
            f"{old_state.value} -> {new_state.value}",
            previous_state=old_state.value,
            state=new_state.value,
        )

    def transition_to(
        self,
        state: UAVState,
    ):
        self._set_state(state)

    # ==================================================
    # FAILURE FLAG
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

    def _raise_if_failure_requested(self):
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
            f"Connecting to {self.connection_string}",
        )

        vehicle = mavutil.mavlink_connection(
            self.connection_string
        )

        start = time.time()

        while time.time() - start < 30:
            self._raise_if_failure_requested()

            msg = vehicle.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=1,
            )

            if msg is not None:
                break

        else:
            raise TimeoutError(
                f"{self.name}: heartbeat timeout."
            )

        self.connection = vehicle
        self.status.system_id = (
            vehicle.target_system
        )

        request_message_interval(
            vehicle,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            5,
        )

        self._emit(
            "uav.connected",
            (
                f"Connected "
                f"(sysid={vehicle.target_system})"
            ),
            system_id=vehicle.target_system,
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

        self.home_lat, self.home_lon = home

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
        home = self._wait_for_home_position()

        self._wait_for_position_estimate()

        return home

    def _wait_for_home_position(
        self,
        timeout=30,
    ):
        vehicle = self.connection

        self._emit(
            "uav.home_wait",
            "Waiting for home position",
        )

        start = time.time()
        last_request = 0.0

        while time.time() - start < timeout:
            self._raise_if_failure_requested()

            if (
                time.time() - last_request
                >= 1
            ):
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
                type=[
                    "HOME_POSITION",
                    "STATUSTEXT",
                ],
                blocking=True,
                timeout=1,
            )

            if msg is None:
                continue

            if (
                msg.get_type()
                == "STATUSTEXT"
            ):
                continue

            if (
                msg.latitude != 0
                and msg.longitude != 0
            ):
                lat = (
                    msg.latitude / 1e7
                )

                lon = (
                    msg.longitude / 1e7
                )

                self._emit(
                    "uav.home_ready",
                    "Home position established",
                    latitude=lat,
                    longitude=lon,
                )

                return lat, lon

        raise TimeoutError(
            f"{self.name}: home position "
            f"not established."
        )

    def _wait_for_position_estimate(
        self,
        timeout=30,
    ):
        vehicle = self.connection

        request_message_interval(
            vehicle,
            193,
            2,
        )

        self._emit(
            "uav.ekf_wait",
            "Waiting for EKF position estimate",
        )

        start = time.time()

        while time.time() - start < timeout:
            self._raise_if_failure_requested()

            msg = vehicle.recv_match(
                type=[
                    "EKF_STATUS_REPORT",
                    "STATUSTEXT",
                ],
                blocking=True,
                timeout=1,
            )

            if msg is None:
                continue

            if (
                msg.get_type()
                == "STATUSTEXT"
            ):
                continue

            flags = msg.flags

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

            if ready and not unhealthy:
                self._emit(
                    "uav.ekf_ready",
                    "EKF position estimate ready",
                    flags=flags,
                )

                return

        raise TimeoutError(
            f"{self.name}: EKF position "
            f"estimate not ready."
        )

    # ==================================================
    # ARMING
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
            altitude=self.status.altitude,
        )

    def _arm_blocking(
        self,
        timeout=15,
    ):
        self._raise_if_failure_requested()

        set_mode(
            self.connection,
            "GUIDED",
        )

        self._raise_if_failure_requested()

        self.connection.mav.command_long_send(
            self.connection.target_system,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
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

        start = time.time()

        while time.time() - start < timeout:
            self._raise_if_failure_requested()

            msg = self.connection.recv_match(
                type=[
                    "COMMAND_ACK",
                    "HEARTBEAT",
                    "STATUSTEXT",
                ],
                blocking=True,
                timeout=1,
            )

            if msg is None:
                continue

            if (
                msg.get_type()
                == "COMMAND_ACK"
            ):
                if (
                    msg.command
                    == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
                ):
                    accepted_results = {
                        mavutil.mavlink.MAV_RESULT_ACCEPTED,
                        mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
                    }

                    if (
                        msg.result
                        not in accepted_results
                    ):
                        raise RuntimeError(
                            f"{self.name}: arm command "
                            f"rejected with result "
                            f"{msg.result}."
                        )

            elif (
                msg.get_type()
                == "HEARTBEAT"
            ):
                armed = bool(
                    msg.base_mode
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )

                if armed:
                    self._emit(
                        "uav.armed",
                        "Vehicle armed",
                    )

                    return

        raise TimeoutError(
            f"{self.name}: failed to arm "
            f"within {timeout}s."
        )

    # ==================================================
    # TAKEOFF
    # ==================================================

    def _takeoff_blocking(
        self,
        target_altitude: float,
        timeout=45,
    ):
        self._raise_if_failure_requested()

        self.connection.mav.command_long_send(
            self.connection.target_system,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            target_altitude,
        )

        start = time.time()

        while time.time() - start < timeout:
            self._raise_if_failure_requested()

            msg = self.connection.recv_match(
                type=[
                    "GLOBAL_POSITION_INT",
                    "COMMAND_ACK",
                    "STATUSTEXT",
                ],
                blocking=True,
                timeout=1,
            )

            if msg is None:
                continue

            if (
                msg.get_type()
                == "COMMAND_ACK"
            ):
                if (
                    msg.command
                    == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
                ):
                    accepted_results = {
                        mavutil.mavlink.MAV_RESULT_ACCEPTED,
                        mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
                    }

                    if (
                        msg.result
                        not in accepted_results
                    ):
                        raise RuntimeError(
                            f"{self.name}: takeoff "
                            f"rejected with result "
                            f"{msg.result}."
                        )

                continue

            if (
                msg.get_type()
                == "STATUSTEXT"
            ):
                continue

            latitude = (
                msg.lat / 1e7
            )

            longitude = (
                msg.lon / 1e7
            )

            altitude = (
                msg.relative_alt
                / 1000.0
            )

            self.status.latitude = latitude
            self.status.longitude = longitude
            self.status.altitude = altitude

            self._emit(
                "uav.telemetry",
                "Telemetry update",
                latitude=latitude,
                longitude=longitude,
                altitude=altitude,
            )

            if (
                altitude
                >= target_altitude * 0.95
            ):
                return

        raise TimeoutError(
            f"{self.name}: takeoff timeout."
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

        target_lat, target_lon = (
            offset_position(
                origin_lat,
                origin_lon,
                north_m=waypoint.north,
                east_m=waypoint.east,
            )
        )

        goto(
            self.connection,
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

        start = time.time()

        while time.time() - start < timeout:
            self._raise_if_failure_requested()

            msg = self.connection.recv_match(
                type="GLOBAL_POSITION_INT",
                blocking=True,
                timeout=1,
            )

            if msg is None:
                continue

            latitude = (
                msg.lat / 1e7
            )

            longitude = (
                msg.lon / 1e7
            )

            altitude = (
                msg.relative_alt
                / 1000.0
            )

            self.status.latitude = latitude
            self.status.longitude = longitude
            self.status.altitude = altitude

            self._emit(
                "uav.telemetry",
                "Telemetry update",
                latitude=latitude,
                longitude=longitude,
                altitude=altitude,
            )

            remaining = distance_m(
                latitude,
                longitude,
                target_lat,
                target_lon,
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
                        f"(error={remaining:.2f}m)"
                    ),
                    east=waypoint.east,
                    north=waypoint.north,
                    error_m=remaining,
                )

                return remaining

        raise TimeoutError(
            f"{self.name}: waypoint timeout."
        )

    # ==================================================
    # LANDING
    # ==================================================

    async def land(self):
        if self.connection is None:
            return

        if (
            not self.connection.motors_armed()
        ):
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
            self._land_blocking
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
        timeout=60,
    ):
        self._raise_if_failure_requested()

        set_mode(
            self.connection,
            "LAND",
        )

        start = time.time()

        while time.time() - start < timeout:
            self._raise_if_failure_requested()

            msg = self.connection.recv_match(
                type=[
                    "GLOBAL_POSITION_INT",
                    "HEARTBEAT",
                    "STATUSTEXT",
                ],
                blocking=True,
                timeout=1,
            )

            if msg is None:
                continue

            if (
                msg.get_type()
                == "GLOBAL_POSITION_INT"
            ):
                latitude = (
                    msg.lat / 1e7
                )

                longitude = (
                    msg.lon / 1e7
                )

                altitude = (
                    msg.relative_alt
                    / 1000.0
                )

                self.status.latitude = latitude
                self.status.longitude = longitude
                self.status.altitude = altitude

                self._emit(
                    "uav.telemetry",
                    "Telemetry update",
                    latitude=latitude,
                    longitude=longitude,
                    altitude=altitude,
                )

                continue

            if (
                msg.get_type()
                == "HEARTBEAT"
            ):
                armed = bool(
                    msg.base_mode
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )

                if not armed:
                    self.status.altitude = 0.0
                    return

        raise TimeoutError(
            f"{self.name}: landing timeout."
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
        )

        # In V1 a fault means "withdraw this UAV from the mission".
        #
        # If communications are still available and the aircraft is
        # armed, request LAND as its safe withdrawal action.
        if (
            self.connection is not None
            and self.connection.motors_armed()
        ):
            try:
                await asyncio.to_thread(
                    set_mode,
                    self.connection,
                    "LAND",
                )

                self._emit(
                    "uav.failure_land_requested",
                    (
                        "Safe LAND requested "
                        "after fault injection"
                    ),
                )

            except Exception as exc:
                self._emit(
                    "uav.failure_land_failed",
                    (
                        "Unable to request safe LAND "
                        f"after failure: {exc}"
                    ),
                )