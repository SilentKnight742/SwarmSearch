import asyncio
import json
import time

import websockets


WS_URL = "ws://127.0.0.1:8000/ws"

PRINT_INTERVAL_SECONDS = 0.5


async def main():
    print()
    print("==============================")
    print(" SWARMSEARCH WEBSOCKET WATCH")
    print("==============================")
    print()

    last_print = 0.0

    async with websockets.connect(
        WS_URL
    ) as websocket:

        while True:
            raw_message = (
                await websocket.recv()
            )

            message = json.loads(
                raw_message
            )

            message_type = (
                message.get("type")
            )

            # ------------------------------------------
            # Initial complete snapshot
            # ------------------------------------------

            if (
                message_type
                == "snapshot"
            ):
                data = message["data"]

                runtime = data.get(
                    "runtime",
                    {}
                )

                world = data.get(
                    "world",
                    {}
                )

                mission = world.get(
                    "mission",
                    {}
                )

                print(
                    "SNAPSHOT"
                )

                print(
                    "Mission:",
                    mission.get(
                        "state"
                    ),
                )

                print(
                    "Running:",
                    runtime.get(
                        "mission_running"
                    ),
                )

                print(
                    "Accepts new mission:",
                    runtime.get(
                        "accepts_new_mission"
                    ),
                )

                print()

                continue

            # ------------------------------------------
            # Incremental events
            # ------------------------------------------

            if (
                message_type
                != "event"
            ):
                continue

            event = message[
                "data"
            ]

            event_type = event[
                "event_type"
            ]

            source = event[
                "source"
            ]

            data = event.get(
                "data",
                {}
            )

            # ------------------------------------------
            # Throttled live telemetry
            # ------------------------------------------

            if (
                event_type
                == "uav.telemetry"
            ):
                now = time.monotonic()

                if (
                    now - last_print
                    < PRINT_INTERVAL_SECONDS
                ):
                    continue

                last_print = now

                east = data.get(
                    "east"
                )

                north = data.get(
                    "north"
                )

                altitude = data.get(
                    "altitude"
                )

                state = data.get(
                    "mission_state"
                )

                mode = data.get(
                    "flight_mode"
                )

                armed = data.get(
                    "armed"
                )

                east_text = (
                    f"{east:6.1f}"
                    if east is not None
                    else "   n/a"
                )

                north_text = (
                    f"{north:6.1f}"
                    if north is not None
                    else "   n/a"
                )

                altitude_text = (
                    f"{altitude:5.1f}"
                    if altitude is not None
                    else "  n/a"
                )

                print(
                    f"{source:<5} | "
                    f"{str(state):<11} | "
                    f"E {east_text} | "
                    f"N {north_text} | "
                    f"ALT {altitude_text}m | "
                    f"{str(mode):<6} | "
                    f"armed={armed}"
                )

                continue

            # ------------------------------------------
            # Important non-telemetry events
            # ------------------------------------------

            if event_type in {
                "mission.state_changed",
                "mission.failure_injected",
                "mission.recovery_started",
                "mission.route_reassigned",
                "mission.completed",
                "mission.aborted",
                "uav.failed",
                "uav.failure_land_requested",
                "uav.failure_disarm_requested",
            }:
                print(
                    f"[{source}] "
                    f"{event_type}: "
                    f"{event['message']}"
                )


if __name__ == "__main__":
    asyncio.run(
        main()
    )