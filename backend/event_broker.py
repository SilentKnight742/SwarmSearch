import asyncio
import copy
from dataclasses import asdict
from typing import Optional

from swarmsearch.events import SwarmEvent
from swarmsearch.geometry import (
    geo_to_local,
)


class EventBroker:
    """
    Bridges SwarmSearch events into the asyncio application layer.

    Vehicle telemetry originates from dedicated MAVLink receiver
    threads. asyncio.Queue objects should only be modified from their
    owning event loop, so publish() crosses that boundary using
    call_soon_threadsafe().
    """

    def __init__(
        self,
        subscriber_queue_size: int = 512,
        timeline_size: int = 200,
    ):
        self._loop: Optional[
            asyncio.AbstractEventLoop
        ] = None

        self._subscriber_queue_size = (
            subscriber_queue_size
        )

        self._timeline_size = (
            timeline_size
        )

        self._subscribers: set[
            asyncio.Queue
        ] = set()

        self._state = (
            self._initial_state()
        )

    # ==================================================
    # LIFECYCLE
    # ==================================================

    def start(self):
        self._loop = (
            asyncio.get_running_loop()
        )

    def stop(self):
        self._loop = None
        self._subscribers.clear()

    # ==================================================
    # STATE
    # ==================================================

    @staticmethod
    def _initial_state() -> dict:
        return {
            "mission": {
                "state": "idle",
                "failed_uavs": [],
                "recovery_count": 0,
            },
            "origin": None,
            "uavs": {},
            "timeline": [],
        }

    def reset(self):
        """
        Reset world state before a new mission begins.

        The previous vehicle receiver threads must already have been
        closed before calling this.
        """

        self._state = (
            self._initial_state()
        )

    def snapshot(self) -> dict:
        return copy.deepcopy(
            self._state
        )

    # ==================================================
    # PUBLISHING
    # ==================================================

    def publish(
        self,
        event: SwarmEvent,
    ):
        """
        Safe to call from both asyncio tasks and MAVLink threads.
        """

        loop = self._loop

        if (
            loop is None
            or loop.is_closed()
        ):
            return

        payload = asdict(
            event
        )

        loop.call_soon_threadsafe(
            self._handle_event,
            payload,
        )

    def _handle_event(
        self,
        payload: dict,
    ):
        payload = self._enrich_event(
            payload
        )

        self._update_snapshot(
            payload
        )

        self._update_timeline(
            payload
        )

        for queue in list(
            self._subscribers
        ):
            # We care more about current position than replaying an
            # enormous telemetry backlog to a slow browser.
            if queue.full():
                try:
                    queue.get_nowait()

                except asyncio.QueueEmpty:
                    pass

            try:
                queue.put_nowait(
                    payload
                )

            except asyncio.QueueFull:
                pass

    # ==================================================
    # LOCAL COORDINATES
    # ==================================================

    def _enrich_event(
        self,
        payload: dict,
    ) -> dict:
        if (
            payload["event_type"]
            != "uav.telemetry"
        ):
            return payload

        data = payload["data"]

        origin = self._state[
            "origin"
        ]

        if origin is None:
            data["east"] = None
            data["north"] = None

            return payload

        latitude = data.get(
            "latitude"
        )

        longitude = data.get(
            "longitude"
        )

        if (
            latitude is None
            or longitude is None
        ):
            data["east"] = None
            data["north"] = None

            return payload

        east, north = geo_to_local(
            latitude=latitude,
            longitude=longitude,
            origin_latitude=(
                origin["latitude"]
            ),
            origin_longitude=(
                origin["longitude"]
            ),
        )

        data["east"] = east
        data["north"] = north

        return payload

    # ==================================================
    # SNAPSHOT REDUCER
    # ==================================================

    def _ensure_uav(
        self,
        name: str,
    ) -> dict:
        uavs = self._state[
            "uavs"
        ]

        if name not in uavs:
            uavs[name] = {
                "name": name,
                "state": "disconnected",
                "latitude": None,
                "longitude": None,
                "east": None,
                "north": None,
                "altitude": 0.0,
                "target_altitude": None,
                "armed": False,
                "flight_mode": None,
                "healthy": True,
                "failure_reason": None,
            }

        return uavs[name]

    def _update_snapshot(
        self,
        event: dict,
    ):
        event_type = event[
            "event_type"
        ]

        source = event[
            "source"
        ]

        data = event[
            "data"
        ]

        # ----------------------------------------------
        # Mission state
        # ----------------------------------------------

        if (
            event_type
            == "mission.state_changed"
        ):
            self._state[
                "mission"
            ]["state"] = data[
                "state"
            ]

        elif (
            event_type
            == "mission.origin_ready"
        ):
            self._state[
                "origin"
            ] = {
                "latitude": data[
                    "latitude"
                ],
                "longitude": data[
                    "longitude"
                ],
            }

        elif (
            event_type
            == "mission.route_reassigned"
        ):
            self._state[
                "mission"
            ]["recovery_count"] += 1

        elif (
            event_type
            == "mission.completed"
        ):
            self._state[
                "mission"
            ]["failed_uavs"] = list(
                data.get(
                    "failed_uavs",
                    [],
                )
            )

            self._state[
                "mission"
            ]["recovery_count"] = (
                data.get(
                    "recovery_count",
                    self._state[
                        "mission"
                    ]["recovery_count"],
                )
            )

        # ----------------------------------------------
        # UAV state
        # ----------------------------------------------

        if not source.startswith(
            "UAV-"
        ):
            return

        uav = self._ensure_uav(
            source
        )

        if (
            event_type
            == "uav.state_changed"
        ):
            uav["state"] = data[
                "state"
            ]

        elif (
            event_type
            == "uav.takeoff_started"
        ):
            uav[
                "target_altitude"
            ] = data.get(
                "target_altitude"
            )

        elif (
            event_type
            == "uav.telemetry"
        ):
            uav["latitude"] = (
                data.get(
                    "latitude"
                )
            )

            uav["longitude"] = (
                data.get(
                    "longitude"
                )
            )

            uav["east"] = (
                data.get(
                    "east"
                )
            )

            uav["north"] = (
                data.get(
                    "north"
                )
            )

            uav["altitude"] = (
                data.get(
                    "altitude",
                    0.0,
                )
            )

            uav["armed"] = (
                data.get(
                    "armed",
                    False,
                )
            )

            uav["flight_mode"] = (
                data.get(
                    "flight_mode"
                )
            )

            uav["healthy"] = (
                data.get(
                    "healthy",
                    uav["healthy"],
                )
            )

            mission_state = data.get(
                "mission_state"
            )

            if mission_state:
                uav["state"] = (
                    mission_state
                )

        elif (
            event_type
            == "uav.failed"
        ):
            uav["state"] = "failed"
            uav["healthy"] = False

            uav[
                "failure_reason"
            ] = event[
                "message"
            ]

            failed_uavs = (
                self._state[
                    "mission"
                ]["failed_uavs"]
            )

            if (
                source
                not in failed_uavs
            ):
                failed_uavs.append(
                    source
                )

    # ==================================================
    # TIMELINE
    # ==================================================

    def _update_timeline(
        self,
        payload: dict,
    ):
        if payload[
            "event_type"
        ] in {
            "uav.telemetry",
            "uav.autopilot_text",
        }:
            return

        timeline = self._state[
            "timeline"
        ]

        timeline.append(
            copy.deepcopy(
                payload
            )
        )

        if (
            len(timeline)
            > self._timeline_size
        ):
            del timeline[
                : len(timeline)
                - self._timeline_size
            ]

    # ==================================================
    # SUBSCRIBERS
    # ==================================================

    def subscribe(
        self,
    ) -> asyncio.Queue:
        queue = asyncio.Queue(
            maxsize=(
                self._subscriber_queue_size
            )
        )

        self._subscribers.add(
            queue
        )

        return queue

    def unsubscribe(
        self,
        queue: asyncio.Queue,
    ):
        self._subscribers.discard(
            queue
        )