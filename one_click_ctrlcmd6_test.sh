#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# One-click launcher for ctrlCmd=6 (End Task) automated testing.
#
# - Terminal 1: launches the state-machine (behavior-tree)
# - Terminal 2: mock spading service
# - Terminal 3: automated ctrlCmd=6 test script
# -----------------------------------------------------------------------------

WS_DIR="$(dirname "$(readlink -f "$0")")"
WS_DIR="${WS_DIR%/}"

SETUP_FILE="$WS_DIR/devel/setup.bash"
LAUNCH_CMD="roslaunch autonomous_loader_bt autonomous_loader_bt.launch"
TEST_CMD_1="rosrun autonomous_loader_bt mock_spading_service.py"
TEST_CMD_2="rosrun autonomous_loader_bt test_ctrlcmd6_end_task.py"

if [ ! -f "$SETUP_FILE" ]; then
  echo "[ERROR] Cannot find workspace setup file: $SETUP_FILE" >&2
  exit 1
fi

open_terminal() {
  local title="$1"; shift
  local cmd="$@"
  if command -v gnome-terminal &>/dev/null; then
    gnome-terminal --title="$title" -- bash -c "$cmd; exec bash"
  elif command -v xterm &>/dev/null; then
    xterm -T "$title" -e "bash -c '$cmd; exec bash'" &
  else
    echo "[ERROR] Neither gnome-terminal nor xterm is available." >&2
    exit 1
  fi
}

echo "Launching AutonomousLoaderBT..."
open_terminal "AutonomousLoaderBT" "cd $WS_DIR && source $SETUP_FILE && $LAUNCH_CMD"

sleep 2

echo "Launching mock spading service..."
open_terminal "Mock Spading Service" "cd $WS_DIR && source $SETUP_FILE && $TEST_CMD_1"

sleep 2

echo "Launching ctrlCmd=6 test..."
open_terminal "CtrlCmd6 Test" "cd $WS_DIR && source $SETUP_FILE && $TEST_CMD_2"
