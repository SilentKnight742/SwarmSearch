#!/usr/bin/env bash

set -e

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/tools/ardupilot}"
ARDUPILOT_VENV="$ARDUPILOT_DIR/.venv"
ARDUPILOT_PYTHON="$ARDUPILOT_VENV/bin/python"

if [[ ! -x "$ARDUPILOT_PYTHON" ]]; then
    echo "ArduPilot Python not found:"
    echo "  $ARDUPILOT_PYTHON"
    exit 1
fi

# Important:
# The user may currently have the SwarmSearch venv activated.
# sim_vehicle.py launches Waf subprocesses which discover `python`
# via PATH, so make the ArduPilot venv the first Python environment.
export PATH="$ARDUPILOT_VENV/bin:$PATH"

# Prevent the active SwarmSearch venv from leaking into ArduPilot tooling.
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