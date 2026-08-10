#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
BIN_DIR=${XDG_BIN_HOME:-"$HOME/.local/bin"}
CONFIG_DIR=${XDG_CONFIG_HOME:-"$HOME/.config"}
AUTOSTART_DIR="$CONFIG_DIR/autostart"
DESKTOP_PATH="$AUTOSTART_DIR/d-task-ground-station.desktop"

for dependency in python3; do
    if ! command -v "$dependency" >/dev/null 2>&1; then
        echo "error: required command not found: $dependency" >&2
        exit 1
    fi
done
if ! python3 -c 'import PyQt6, serial' >/dev/null 2>&1; then
    echo "error: install python3-pyqt6 and python3-serial first" >&2
    exit 1
fi

FILES="ground_station_ui.py ground_station_ui_core.py ground_station_protocol.py ground_station_serial.py stage5_serial_probe.py run_ground_station_ui.sh"
for name in $FILES; do
    if [ ! -f "$SCRIPT_DIR/$name" ]; then
        echo "error: installer file is missing: $SCRIPT_DIR/$name" >&2
        exit 1
    fi
done
if [ ! -f "$SCRIPT_DIR/assets/field_map.png" ]; then
    echo "error: official field map is missing: $SCRIPT_DIR/assets/field_map.png" >&2
    exit 1
fi
if [ ! -f "$SCRIPT_DIR/maixcam-ground-station-ui.desktop.in" ]; then
    echo "error: desktop template is missing" >&2
    exit 1
fi

mkdir -p "$BIN_DIR" "$BIN_DIR/assets" "$AUTOSTART_DIR"
install -m 0755 "$SCRIPT_DIR/ground_station_ui.py" "$BIN_DIR/ground_station_ui.py"
install -m 0644 "$SCRIPT_DIR/ground_station_ui_core.py" "$BIN_DIR/ground_station_ui_core.py"
install -m 0644 "$SCRIPT_DIR/ground_station_protocol.py" "$BIN_DIR/ground_station_protocol.py"
install -m 0644 "$SCRIPT_DIR/ground_station_serial.py" "$BIN_DIR/ground_station_serial.py"
install -m 0755 "$SCRIPT_DIR/stage5_serial_probe.py" "$BIN_DIR/stage5_serial_probe.py"
install -m 0755 "$SCRIPT_DIR/run_ground_station_ui.sh" "$BIN_DIR/ground_station_ui"
install -m 0644 "$SCRIPT_DIR/assets/field_map.png" "$BIN_DIR/assets/field_map.png"

ESCAPED_PATH=$(printf '%s' "$BIN_DIR/ground_station_ui" | sed 's/[&|\\]/\\&/g')
TEMP_DESKTOP=$(mktemp "${TMPDIR:-/tmp}/d-task-ground-station.XXXXXX")
trap 'rm -f "$TEMP_DESKTOP"' EXIT HUP INT TERM
sed "s|@EXEC_PATH@|$ESCAPED_PATH|g" "$SCRIPT_DIR/maixcam-ground-station-ui.desktop.in" > "$TEMP_DESKTOP"
install -m 0644 "$TEMP_DESKTOP" "$DESKTOP_PATH"

# Remove only known historical RTSP autostart entries/files installed by this
# project.  No packages, network settings or unrelated user files are changed.
rm -f "$AUTOSTART_DIR/maixcam-ground-station.desktop"
rm -f "$BIN_DIR/ground_station_supervisor.py" "$BIN_DIR/restore_ground_station_supervisor"

echo "installed V2.3 ground station: $BIN_DIR/ground_station_ui.py"
echo "installed official field map:  $BIN_DIR/assets/field_map.png"
echo "installed read-only probe:     $BIN_DIR/stage5_serial_probe.py"
echo "installed offline autostart:   $DESKTOP_PATH"
echo "no VLC/RTSP dependency or task-control sender is installed"
