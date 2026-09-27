import asyncio
import threading
import time
from collections.abc import Callable
from typing import Optional

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

from .events import SwarmEvent
from .models import UAVState, UAVStatus, Waypoint
from .state_machine import validate_transition


EventSink = Callable[[SwarmEvent], None]


class VehicleUnavailableError(RuntimeError):
    """
    Raised when the coordinator marks a UAV unavailable
    while it is executing an active operation.
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

        # Failure injection is represented as a thread-safe flag.
        #
        # This lets the coordinator request a failure without starting
        # another MAVLink telemetry reader on this vehicle.
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
        """
        Public state transition used by the coordinator.

        The actual transition still goes through validation.
        """
        self._set_state(state)

    # ==================================================
    # FAILURE REQUEST STATE
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
        """
        Request that this UAV become unavailable.

        This does not immediately consume telemetry or issue MAVLink
        commands. The currently active vehicle operation detects this
        flag and exits through VehicleUnavailableError.
        """

        if self.status.state == UAVState.FAILED:
            return

        if self._failure_requested.is_set():
            return

        self._failure_reason = reason
        self._failure_requested.set()

        self._emit(
            "uav.failure_requested",
            reason,
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

        vehicle.wait_heartbeat(
            timeout=30
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
    # NAVIGATION READINESS
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

            if self._failure_requested.is_set():
                raise VehicleUnavailableError(
                    self._failure_reason
                    or f"{self.name} unavailable"
                )

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
            193,  # EKF_STATUS_REPORT
            2,
        )

        self._emit(
            "uav.ekf_wait",
            "Waiting for EKF position estimate",
        )

        start = time.time()

        while time.time() - start < timeout:

            if self._failure_requested.is_set():
                raise VehicleUnavailableError(
                    self._failure_reason
                    or f"{self.name} unavailable"
                )

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
    # TAKEOFF
    # ==================================================

    async def launch(self):
        if self.connection is None:
            raise RuntimeError(
                f"{self.name} not connected."
            )

        if self._failure_requested.is_set():
            raise VehicleUnavailableError(
                self._failure_reason
                or f"{self.name} unavailable"
            )

        self._set_state(
            UAVState.ARMING
        )

        await asyncio.to_thread(
            self._arm_blocking
        )

        if self._failure_requested.is_set():
            raise VehicleUnavailableError(
                self._failure_reason
                or f"{self.name} unavailable"
            )

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

        # Still uses the proven legacy takeoff implementation.
        #
        # It is not yet fully interruptible while physically climbing.
        # We will replace this in the next stage.
        await asyncio.to_thread(
            takeoff,
            self.connection,
            self.cruise_altitude,
        )

        if self._failure_requested.is_set():
            raise VehicleUnavailableError(
                self._failure_reason
                or f"{self.name} unavailable"
            )

        self.status.altitude = (
            self.cruise_altitude
        )

        self._set_state(
            UAVState.AIRBORNE
        )

        self._emit(
            "uav.airborne",
            (
                f"Airborne at "
                f"{self.cruise_altitude:.1f}m"
            ),
            altitude=(
                self.cruise_altitude
            ),
        )

    def _arm_blocking(self):
        if self._failure_requested.is_set():
            raise VehicleUnavailableError(
                self._failure_reason
                or f"{self.name} unavailable"
            )

        set_mode(
            self.connection,
            "GUIDED",
        )

        if self._failure_requested.is_set():
            raise VehicleUnavailableError(
                self._failure_reason
                or f"{self.name} unavailable"
            )

        arm(
            self.connection
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

        if self._failure_requested.is_set():
            raise VehicleUnavailableError(
                self._failure_reason
                or f"{self.name} unavailable"
            )

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

        if self._failure_requested.is_set():
            raise VehicleUnavailableError(
                self._failure_reason
                or f"{self.name} unavailable"
            )

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

            # --------------------------------------------------
            # RUNTIME FAILURE CHECK
            # --------------------------------------------------
            #
            # This is intentionally checked inside the existing
            # telemetry consumer instead of creating another thread
            # that calls recv_match() on the same connection.
            #
            if self._failure_requested.is_set():
                raise VehicleUnavailableError(
                    self._failure_reason
                    or f"{self.name} unavailable"
                )

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
                != UAVState.FAILED
            ):
                if (
                    self.status.state
                    != UAVState.LANDED
                ):
                    self._set_state(
                        UAVState.LANDED
                    )

            return

        if (
            self.status.state
            != UAVState.FAILED
        ):
            self._set_state(
                UAVState.LANDING
            )

        self._emit(
            "uav.landing_started",
            "Landing",
        )

        # Still uses the proven legacy landing implementation.
        #
        # Full mid-landing failure interruption comes next.
        await asyncio.to_thread(
            land,
            self.connection,
        )

        self.status.altitude = 0.0

        if (
            self.status.state
            != UAVState.FAILED
        ):
            self._set_state(
                UAVState.LANDED
            )

        self._emit(
            "uav.landed",
            "Landed and disarmed",
        )

    # ==================================================
    # FAILURE
    # ==================================================

    async def fail(
        self,
        reason: str = "failure injected",
    ):
        """
        Finalize a previously requested failure.

        V1 semantics:
        - remove the UAV from the active mission
        - mark it FAILED
        - if communication still exists and the vehicle is armed,
          request a safe LAND
        """

        if (
            self.status.state
            == UAVState.FAILED
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