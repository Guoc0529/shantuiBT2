#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# One-click launcher for the autonomous loader behavior-tree node and
# its two test scripts.
#
# ‑ Terminal 1: launches the state-machine (behavior-tree) using roslaunch.
# ‑ Terminal 2: starts the two test scripts in separate tabs of a new window.
# -----------------------------------------------------------------------------

# --- Configuration -----------------------------------------------------------
WS_DIR="$(dirname "$(readlink -f "$0")")"   # assume script lives in workspace
WS_DIR="${WS_DIR%/}"                           # trim trailing slash if any
# If you prefer a hard-coded path, uncomment and edit the next line
# WS_DIR="$HOME/shantui/behaviorTreev4_ws"

SETUP_FILE="$WS_DIR/devel/setup.bash"
LAUNCH_CMD="roslaunch autonomous_loader_bt autonomous_loader_bt.launch"
TEST_CMD_1="rosrun autonomous_loader_bt mock_spading_service.py"
TEST_CMD_2="rosrun autonomous_loader_bt manual_intervention_tester.py"

if [ ! -f "$SETUP_FILE" ]; then
  echo "[ERROR] Cannot find workspace setup file: $SETUP_FILE" >&2
  exit 1
fi

# Helper to open a new gnome-terminal window running a given command string
open_terminal() {
  local title="$1"; shift
  local cmd="$@"
  # Use gnome-terminal if available, otherwise fall back to xterm
  if command -v gnome-terminal &>/dev/null; then
    gnome-terminal --title="$title" -- bash -c "$cmd; exec bash"
  elif command -v xterm &>/dev/null; then
    xterm -T "$title" -e "bash -c '$cmd; exec bash'" &
  else
    echo "[ERROR] Neither gnome-terminal nor xterm is available." >&2
    exit 1
  fi
}

# Terminal 1: run the behavior-tree (state-machine)
open_terminal "AutonomousLoaderBT" "cd $WS_DIR && source $SETUP_FILE && $LAUNCH_CMD"

sleep 2

# Terminals 2 & 3: run test scripts in separate windows with delays
echo "Launching mock spading service..."
open_terminal "Mock Spading Service" "cd $WS_DIR && source $SETUP_FILE && $TEST_CMD_1"

echo "Waiting 2 seconds..."
sleep 2

echo "Launching manual intervention tester..."
open_terminal "Manual Intervention" "cd $WS_DIR && source $SETUP_FILE && $TEST_CMD_2"
