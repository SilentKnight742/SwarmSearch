#!/usr/bin/env bash

set -Eeuo pipefail


ROOT_DIR="$(
    cd "$(dirname "${BASH_SOURCE[0]}")/.."
    pwd
)"

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/tools/ardupilot}"

PROJECT_VENV="$ROOT_DIR/.venv"
PROJECT_PYTHON="$PROJECT_VENV/bin/python"

LOG_DIR="$ROOT_DIR/logs"
SITL_LOG="$LOG_DIR/sitl.log"
BACKEND_LOG="$LOG_DIR/backend.log"

SITL_PID=""
BACKEND_PID=""
FRONTEND_PID=""

CLEANUP_DONE=0


cd "$ROOT_DIR"

mkdir -p "$LOG_DIR"


cleanup() {
    if (( CLEANUP_DONE )); then
        return
    fi

    CLEANUP_DONE=1

    echo
    echo "Stopping SwarmSearch..."

    if [[ -n "$FRONTEND_PID" ]]; then
        kill "$FRONTEND_PID" \
            2>/dev/null || true
    fi

    if [[ -n "$BACKEND_PID" ]]; then
        kill "$BACKEND_PID" \
            2>/dev/null || true
    fi

    if [[ -n "$SITL_PID" ]]; then
        kill "$SITL_PID" \
            2>/dev/null || true
    fi

    wait 2>/dev/null || true
}


trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM


require_command() {
    if ! command -v "$1" \
        >/dev/null 2>&1
    then
        echo "Missing required command: $1"
        exit 1
    fi
}


require_command git
require_command python3
require_command node
require_command npm
require_command sha256sum


python3 - <<'PY'
import sys

if sys.version_info < (3, 11):
    raise SystemExit(
        "SwarmSearch requires Python 3.11 or newer."
    )
PY


NODE_MAJOR="$(
    node -p \
    "Number(process.versions.node.split('.')[0])"
)"

if (( NODE_MAJOR < 20 )); then
    echo "SwarmSearch requires Node.js 20 or newer."
    exit 1
fi


# ======================================================
# ARDUPILOT
# ======================================================

if [[ ! -d "$ARDUPILOT_DIR/.git" ]]; then
    echo
    echo "ArduPilot was not found."
    echo "Installing it at:"
    echo "  $ARDUPILOT_DIR"
    echo

    mkdir -p "$(dirname "$ARDUPILOT_DIR")"

    git clone \
        --recurse-submodules \
        https://github.com/ArduPilot/ardupilot.git \
        "$ARDUPILOT_DIR"

    echo
    echo "Installing ArduPilot prerequisites."
    echo "This may request sudo access and may take a while."
    echo

    (
        cd "$ARDUPILOT_DIR"

        Tools/environment_install/install-prereqs-ubuntu.sh \
            -y
    )

    if [[ -f "$HOME/.profile" ]]; then
        # shellcheck disable=SC1090
        source "$HOME/.profile" || true
    fi
fi


# ======================================================
# PYTHON ENVIRONMENT
# ======================================================

VENV_CREATED=0

if [[ ! -x "$PROJECT_PYTHON" ]]; then
    echo
    echo "Creating SwarmSearch Python environment..."

    python3 -m venv "$PROJECT_VENV"

    VENV_CREATED=1
fi


PYPROJECT_FILE="$ROOT_DIR/pyproject.toml"
PYTHON_STAMP="$PROJECT_VENV/.swarmsearch-pyproject"

PYPROJECT_HASH="$(
    sha256sum "$PYPROJECT_FILE" |
        awk '{print $1}'
)"

INSTALLED_PYPROJECT_HASH=""

if [[ -f "$PYTHON_STAMP" ]]; then
    INSTALLED_PYPROJECT_HASH="$(
        cat "$PYTHON_STAMP"
    )"
fi


if [[
    "$VENV_CREATED" -eq 1
    || "$PYPROJECT_HASH" != "$INSTALLED_PYPROJECT_HASH"
]]; then
    echo
    echo "Installing Python dependencies..."

    "$PROJECT_PYTHON" \
        -m pip install \
        --quiet \
        --upgrade pip

    "$PROJECT_PYTHON" \
        -m pip install \
        --quiet \
        -e "${ROOT_DIR}[dev]"

    echo "$PYPROJECT_HASH" \
        > "$PYTHON_STAMP"
fi


# ======================================================
# FRONTEND DEPENDENCIES
# ======================================================

FRONTEND_DIR="$ROOT_DIR/frontend"
LOCK_FILE="$FRONTEND_DIR/package-lock.json"
NPM_STAMP="$FRONTEND_DIR/node_modules/.swarmsearch-lock"

LOCK_HASH="$(
    sha256sum "$LOCK_FILE" |
        awk '{print $1}'
)"

INSTALLED_LOCK_HASH=""

if [[ -f "$NPM_STAMP" ]]; then
    INSTALLED_LOCK_HASH="$(
        cat "$NPM_STAMP"
    )"
fi


if [[
    ! -d "$FRONTEND_DIR/node_modules"
    || "$LOCK_HASH" != "$INSTALLED_LOCK_HASH"
]]; then
    echo
    echo "Installing frontend dependencies..."

    npm \
        --prefix "$FRONTEND_DIR" \
        ci

    echo "$LOCK_HASH" \
        > "$NPM_STAMP"
fi


# ======================================================
# START ARDUPILOT SITL
# ======================================================

echo
echo "Starting 3-vehicle ArduPilot SITL fleet..."

export ARDUPILOT_DIR

bash "$ROOT_DIR/scripts/start_sitl.sh" \
    >"$SITL_LOG" \
    2>&1 &

SITL_PID=$!


wait_for_listen_port() {
    local port="$1"
    local timeout_seconds="$2"

    for ((i = 0; i < timeout_seconds; i++)); do
        if ! kill -0 "$SITL_PID" \
            2>/dev/null
        then
            echo
            echo "ArduPilot SITL exited unexpectedly."
            echo
            tail -n 40 "$SITL_LOG"
            exit 1
        fi

        if command -v ss \
            >/dev/null 2>&1
        then
            if ss -ltn \
                2>/dev/null |
                grep -Eq ":${port}[[:space:]]"
            then
                return 0
            fi
        else
            if (( i >= 15 )); then
                return 0
            fi
        fi

        sleep 1
    done

    echo
    echo "Timed out waiting for SITL TCP port $port."
    echo
    tail -n 40 "$SITL_LOG"
    exit 1
}


wait_for_listen_port 5760 180
wait_for_listen_port 5770 180
wait_for_listen_port 5780 180


echo "SITL fleet ready."


# ======================================================
# START BACKEND
# ======================================================

echo
echo "Starting FastAPI backend..."

"$PROJECT_PYTHON" \
    -m uvicorn \
    backend.main:app \
    --host 127.0.0.1 \
    --port 8000 \
    >"$BACKEND_LOG" \
    2>&1 &

BACKEND_PID=$!


wait_for_backend() {
    for ((i = 0; i < 60; i++)); do
        if ! kill -0 "$BACKEND_PID" \
            2>/dev/null
        then
            echo
            echo "Backend exited unexpectedly."
            echo
            tail -n 40 "$BACKEND_LOG"
            exit 1
        fi

        if "$PROJECT_PYTHON" - <<'PY' \
            >/dev/null 2>&1
import urllib.request

with urllib.request.urlopen(
    "http://127.0.0.1:8000/api/health",
    timeout=1,
) as response:
    if response.status != 200:
        raise RuntimeError(response.status)
PY
        then
            return 0
        fi

        sleep 1
    done

    echo
    echo "Timed out waiting for the backend."
    echo
    tail -n 40 "$BACKEND_LOG"
    exit 1
}


wait_for_backend

echo "Backend ready."


# ======================================================
# START FRONTEND
# ======================================================

echo
echo "======================================================"
echo " SwarmSearch is ready"
echo "======================================================"
echo
echo " Mission Control:"
echo "   http://127.0.0.1:5173"
echo
echo " Backend:"
echo "   http://127.0.0.1:8000"
echo
echo " Logs:"
echo "   $SITL_LOG"
echo "   $BACKEND_LOG"
echo
echo " Press Ctrl+C to stop everything."
echo
echo "======================================================"
echo


npm \
    --prefix "$FRONTEND_DIR" \
    run dev \
    -- \
    --host 127.0.0.1 &

FRONTEND_PID=$!


wait "$FRONTEND_PID"