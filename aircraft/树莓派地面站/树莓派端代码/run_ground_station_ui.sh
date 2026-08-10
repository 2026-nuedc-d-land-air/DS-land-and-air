#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)

SUPERVISE=0
if [ "${1:-}" = "--supervise" ]; then
    SUPERVISE=1
    shift
fi

if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
    echo "no local graphical session; run from the Raspberry Pi desktop" >&2
    exit 2
fi

# Debian's installed PyQt6 build on the target does not include the Wayland
# platform plugin, while the Labwc desktop provides a verified XWayland :0.
# This selects Qt's local xcb backend only; it has no VLC/RTSP dependency.
if [ -z "${QT_QPA_PLATFORM:-}" ] && [ -n "${DISPLAY:-}" ]; then
    export QT_QPA_PLATFORM=xcb
fi

if [ "$SUPERVISE" -eq 0 ]; then
    exec python3 "$SCRIPT_DIR/ground_station_ui.py" "$@"
fi

# This wrapper only restarts the local receive-only UI after an abnormal exit.
# It starts no VLC/RTSP/network process.
DELAY=2
while :; do
    STARTED=$(date +%s)
    if python3 "$SCRIPT_DIR/ground_station_ui.py" "$@"; then
        exit 0
    else
        STATUS=$?
    fi
    RUNTIME=$(($(date +%s) - STARTED))
    if [ "$RUNTIME" -ge 30 ]; then
        DELAY=2
    fi
    echo "ground-station UI exited status=$STATUS runtime=${RUNTIME}s; restarting in ${DELAY}s" >&2
    sleep "$DELAY"
    if [ "$DELAY" -lt 15 ]; then
        DELAY=$((DELAY * 2))
        if [ "$DELAY" -gt 15 ]; then DELAY=15; fi
    fi
done
