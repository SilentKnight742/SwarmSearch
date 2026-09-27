#!/usr/bin/env bash

set -e

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/tools/ardupilot}"

cd "$ARDUPILOT_DIR"

./Tools/autotest/sim_vehicle.py \
    -v Copter \
    -w \
    --count 3 \
    --auto-sysid \
    --location CMAC \
    --auto-offset-line 90,10 \
    --no-mavproxy