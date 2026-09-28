#!/usr/bin/env bash

set -Eeuo pipefail


ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/tools/ardupilot}"

ARDUPILOT_VENV="$ARDUPILOT_DIR/.venv"


if [[ ! -d "$ARDUPILOT_DIR" ]]; then
    echo "ArduPilot directory not found:"
    echo "  $ARDUPILOT_DIR"
    exit 1
fi


if [[ -x "$ARDUPILOT_VENV/bin/python" ]]; then
    ARDUPILOT_PYTHON="$ARDUPILOT_VENV/bin/python"

    export PATH="$ARDUPILOT_VENV/bin:$PATH"

elif [[ -x "/usr/bin/python3" ]]; then
    ARDUPILOT_PYTHON="/usr/bin/python3"

    export PATH="$HOME/.local/bin:$PATH"

else
    echo "No usable Python installation found for ArduPilot."
    exit 1
fi


unset VIRTUAL_ENV
unset PYTHONHOME
unset PYTHONPATH


cd "$ARDUPILOT_DIR"


exec "$ARDUPILOT_PYTHON" \
    ./Tools/autotest/sim_vehicle.py \
    -v Copter \
    -w \
    --count 3 \
    --auto-sysid \
    --location CMAC \
    --auto-offset-line 90,10 \
    --no-mavproxy