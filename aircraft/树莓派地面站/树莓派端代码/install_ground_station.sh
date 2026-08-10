#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
echo "install_ground_station.sh installs the offline V2.3 UI (RTSP/VLC removed)"
exec "$SCRIPT_DIR/install_ground_station_ui.sh" "$@"
