#!/bin/sh
set -eu

DURATION=${1:-600}
INTERVAL=${2:-10}

case "$DURATION:$INTERVAL" in
    *[!0-9:]*|0:*|*:0)
        echo "usage: $0 [positive-duration-seconds] [positive-interval-seconds]" >&2
        exit 2
        ;;
esac

PID=$(pgrep -o -f '^python3 .*/ground_station_ui.py' || true)
if [ -z "$PID" ]; then
    echo "result: FAIL - ground_station_ui.py is not running" >&2
    exit 1
fi

STATE_HOME=${XDG_STATE_HOME:-"$HOME/.local/state"}
LOG_PATH="$STATE_HOME/d-task-ground-station/ground_station_ui.log"
START=$(date +%s)
DEADLINE=$((START + DURATION))

echo "ui_pid: $PID"
echo "duration_seconds: $DURATION"
echo "interval_seconds: $INTERVAL"

while [ "$(date +%s)" -lt "$DEADLINE" ]; do
    if ! kill -0 "$PID" 2>/dev/null; then
        echo "result: FAIL - UI process exited" >&2
        exit 1
    fi

    PROCESS=$(ps -p "$PID" -o etimes=,%cpu=,rss= | xargs)
    if [ -r /sys/class/thermal/thermal_zone0/temp ]; then
        TEMP_MILLI=$(sed -n '1p' /sys/class/thermal/thermal_zone0/temp)
        TEMP_C=$(awk "BEGIN { printf \"%.1f\", $TEMP_MILLI / 1000 }")
    else
        TEMP_C="unavailable"
    fi
    METRICS=$(grep ' metrics ' "$LOG_PATH" 2>/dev/null | tail -n 1 || true)
    echo "sample: process=[$PROCESS] temp_c=$TEMP_C"
    if [ -n "$METRICS" ]; then
        echo "latest_$METRICS"
    fi
    sleep "$INTERVAL"
done

if ! kill -0 "$PID" 2>/dev/null; then
    echo "result: FAIL - UI process exited at deadline" >&2
    exit 1
fi

echo "result: PASS"
