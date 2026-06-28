#!/usr/bin/env python3
"""
Lightweight GUI to send NavigateAction goals from sites.json.

Constraints:
- No start-side handling; only goal-side selection matters.
- Pile percent & y_offset are only needed for unload-load and no-load modes.
- Goal=load or goal=unload must provide the corresponding ID.
"""

import sys
import time
import re
from typing import Dict, Optional

from PyQt5 import QtCore, QtWidgets
import actionlib
import rospy
from autonomous_loader_msgs.msg import NavigateAction, NavigateGoal

from sites_publisher import (
    MAP_FRAME,
    SITES_PATH,
    build_goal,
    filter_entries,
    format_entry,
    load_sites,
    select_load_entry,
    select_unload_entry,
)


MODES = ["load-unload", "unload-load", "no-load", "no-unload"]
FEEDBACK_THROTTLE_SEC = 0.5


def derive_status(mode: str):
    if mode == "load-unload":
        return 0, 1
    elif mode == "unload-load":
        return 1, 0
    elif mode == "no-load":
        return 2, 0
    elif mode == "no-unload":
        return 2, 1
    return 2, 2


def canonicalize_id(raw: str, point_type: str, known_ids):
    """
    Normalize user input to canonical id form like load-01/unload-02.
    Accepts raw numbers (1 -> load-01) and already-prefixed ids.
    """
    value = (raw or "").strip()
    if not value:
        return ""

    prefix = f"{point_type}-"
    if value.isdigit():
        value = f"{prefix}{int(value):02d}"
    elif value.startswith(prefix) and value[len(prefix) :].isdigit():
        value = f"{prefix}{int(value[len(prefix) :]):02d}"

    if known_ids and value not in known_ids:
        available = ", ".join(sorted(known_ids))
        raise RuntimeError(f"Unknown {point_type} id '{value}'. Available: {available}")
    return value


def parse_id_number(id_str: str, label: str) -> int:
    """
    Extract trailing digits from an id like load-02/unload-05; raise if missing.
    """
    if not id_str:
        return 0
    match = re.search(r"(\d+)$", id_str)
    if not match:
        raise RuntimeError(f"{label} missing numeric suffix: '{id_str}'")
    return int(match.group(1))


class ActionWorker(QtCore.QThread):
    log = QtCore.pyqtSignal(str)
    done = QtCore.pyqtSignal(bool, str)

    def __init__(
        self,
        mode: str,
        load_id: str,
        unload_id: str,
        pile_percent: Optional[float],
        y_offset: Optional[float],
        wait_timeout: float,
        result_timeout: float,
        parent=None,
    ):
        super().__init__(parent)
        self.mode = mode
        self.load_id = load_id
        self.unload_id = unload_id
        self.pile_percent = pile_percent
        self.y_offset = y_offset
        self.wait_timeout = wait_timeout
        self.result_timeout = result_timeout
        self._cancel_requested = False
        self._client = None
        self._last_feedback_time = 0.0

    def cancel(self):
        self._cancel_requested = True
        if self._client:
            self._client.cancel_goal()
            self.log.emit("Cancel requested.")

    def _select_goal_entry(self, entries):
        if self.mode in ("unload-load", "no-load"):
            if not self.load_id:
                raise RuntimeError("load_id is required for this mode.")
            candidates = filter_entries(entries, "load", self.load_id)
            if self.pile_percent is None or self.y_offset is None:
                raise RuntimeError("pile_percent and y_offset are required for this mode.")
            goal_entry = select_load_entry(candidates, self.pile_percent, self.y_offset)
        else:
            if not self.unload_id:
                raise RuntimeError("unload_id is required for this mode.")
            candidates = filter_entries(entries, "unload", self.unload_id)
            goal_entry = select_unload_entry(candidates)
        return goal_entry

    def _feedback_cb(self, feedback):
        if self._cancel_requested:
            return
        now = time.monotonic()
        if now - self._last_feedback_time < FEEDBACK_THROTTLE_SEC:
            return
        self._last_feedback_time = now
        self.log.emit(f"feedback: dist_to_goal={feedback.distance_to_goal:.2f}, plan_succeeded={feedback.plan_succeeded}")

    def run(self):
        try:
            if not rospy.core.is_initialized():
                rospy.init_node("test_action_gui", anonymous=True, disable_signals=True)

            entries = load_sites(SITES_PATH)
            goal_entry = self._select_goal_entry(entries)
            last_status, current_status = derive_status(self.mode)

            self.log.emit(f"Goal entry: {format_entry(goal_entry)}")
            self.log.emit(f"Status (last/current): {last_status}/{current_status}")

            client = actionlib.SimpleActionClient("navigate", NavigateAction)
            self._client = client
            self.log.emit("Waiting for navigate action server...")
            if not client.wait_for_server(rospy.Duration(self.wait_timeout)):
                raise RuntimeError("Navigate action server not available within timeout.")

            goal_msg = NavigateGoal()
            goal_msg.end_pose = build_goal(goal_entry)
            goal_msg.end_pose.header.frame_id = MAP_FRAME
            goal_msg.last_status = last_status
            goal_msg.current_status = current_status

            load_num = parse_id_number(self.load_id, "load_id")
            unload_num = parse_id_number(self.unload_id, "unload_id")
            goal_msg.last_id = load_num if last_status == 0 else (unload_num if last_status == 1 else 0)
            goal_msg.current_id = load_num if current_status == 0 else (unload_num if current_status == 1 else 0)
            self.log.emit(f"IDs (last_id={goal_msg.last_id}, current_id={goal_msg.current_id})")

            self.log.emit("Sending goal...")
            client.send_goal(goal_msg, feedback_cb=self._feedback_cb)
            finished = client.wait_for_result(rospy.Duration(self.result_timeout))
            if not finished:
                client.cancel_goal()
                raise RuntimeError("Timed out waiting for result.")

            if self._cancel_requested:
                self.done.emit(False, "Canceled.")
                return

            result = client.get_result()
            summary = (
                f"Result: success={result.success} dist_err={result.final_distance_error:.3f} "
                f"angle_err={result.final_angle_error:.3f} err_code={result.error_code} msg={result.error_msg}"
            )
            self.log.emit(summary)
            self.done.emit(True, summary)
        except Exception as exc:
            self.done.emit(False, str(exc))


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Test Action GUI")
        self.resize(900, 600)

        self.entries = []
        self.load_ids = []
        self.unload_ids = []
        self.current_thread: Optional[ActionWorker] = None
        self.last_args: Optional[Dict] = None

        self._build_ui()
        self._reload_sites()
        self._update_field_state()

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        main_layout = QtWidgets.QHBoxLayout()
        central.setLayout(main_layout)

        form_layout = QtWidgets.QFormLayout()
        form_layout.setLabelAlignment(QtCore.Qt.AlignRight)

        self.mode_combo = QtWidgets.QComboBox()
        self.mode_combo.addItems(MODES)
        self.mode_combo.currentIndexChanged.connect(self._update_field_state)
        form_layout.addRow("Mode:", self.mode_combo)

        self.load_label = QtWidgets.QLabel("Load ID (goal=load):")
        self.load_id_input = QtWidgets.QComboBox()
        self.load_id_input.setEditable(True)
        form_layout.addRow(self.load_label, self.load_id_input)

        self.unload_label = QtWidgets.QLabel("Unload ID (goal=unload):")
        self.unload_id_input = QtWidgets.QComboBox()
        self.unload_id_input.setEditable(True)
        form_layout.addRow(self.unload_label, self.unload_id_input)

        self.pile_input = QtWidgets.QDoubleSpinBox()
        self.pile_input.setRange(-1000.0, 1000.0)
        self.pile_input.setDecimals(4)
        self.pile_input.setSingleStep(0.05)
        form_layout.addRow("Pile %:", self.pile_input)

        self.y_offset_input = QtWidgets.QDoubleSpinBox()
        self.y_offset_input.setRange(-1000.0, 1000.0)
        self.y_offset_input.setDecimals(4)
        self.y_offset_input.setSingleStep(0.05)
        form_layout.addRow("y_offset:", self.y_offset_input)

        self.wait_timeout_input = QtWidgets.QDoubleSpinBox()
        self.wait_timeout_input.setRange(0.1, 120.0)
        self.wait_timeout_input.setValue(10.0)
        self.wait_timeout_input.setSingleStep(1.0)
        form_layout.addRow("wait_for_server (s):", self.wait_timeout_input)

        self.result_timeout_input = QtWidgets.QDoubleSpinBox()
        self.result_timeout_input.setRange(1.0, 3600.0)
        self.result_timeout_input.setValue(300.0)
        self.result_timeout_input.setSingleStep(10.0)
        form_layout.addRow("result timeout (s):", self.result_timeout_input)

        self.reload_button = QtWidgets.QPushButton("Reload sites.json")
        self.reload_button.clicked.connect(self._reload_sites)

        self.send_button = QtWidgets.QPushButton("Send")
        self.send_button.clicked.connect(self._send)

        self.send_again_button = QtWidgets.QPushButton("Send again")
        self.send_again_button.setEnabled(False)
        self.send_again_button.clicked.connect(self._send_again)

        self.cancel_button = QtWidgets.QPushButton("Cancel")
        self.cancel_button.setEnabled(False)
        self.cancel_button.clicked.connect(self._cancel_current)

        btn_layout = QtWidgets.QHBoxLayout()
        btn_layout.addWidget(self.reload_button)
        btn_layout.addStretch()
        btn_layout.addWidget(self.send_button)
        btn_layout.addWidget(self.send_again_button)
        btn_layout.addWidget(self.cancel_button)

        left_panel = QtWidgets.QVBoxLayout()
        left_panel.addLayout(form_layout)
        left_panel.addLayout(btn_layout)

        self.log_view = QtWidgets.QTextEdit()
        self.log_view.setReadOnly(True)
        left_panel.addWidget(self.log_view, stretch=1)

        lists_layout = QtWidgets.QVBoxLayout()
        load_group = QtWidgets.QGroupBox("Load entries")
        unload_group = QtWidgets.QGroupBox("Unload entries")

        self.load_list = QtWidgets.QListWidget()
        self.load_list.itemClicked.connect(self._select_load_from_list)
        load_layout = QtWidgets.QVBoxLayout()
        load_layout.addWidget(self.load_list)
        load_group.setLayout(load_layout)

        self.unload_list = QtWidgets.QListWidget()
        self.unload_list.itemClicked.connect(self._select_unload_from_list)
        unload_layout = QtWidgets.QVBoxLayout()
        unload_layout.addWidget(self.unload_list)
        unload_group.setLayout(unload_layout)

        lists_layout.addWidget(load_group, stretch=1)
        lists_layout.addWidget(unload_group, stretch=1)

        main_layout.addLayout(left_panel, stretch=2)
        main_layout.addLayout(lists_layout, stretch=1)

    def _append_log(self, text: str):
        self.log_view.append(text)
        self.log_view.ensureCursorVisible()

    def _update_field_state(self):
        mode = self.mode_combo.currentText()
        goal_is_load = mode in ("unload-load", "no-load")
        load_needed = goal_is_load
        unload_needed = not goal_is_load
        pile_needed = mode in ("unload-load", "no-load")

        self.load_id_input.setEnabled(load_needed)
        self.unload_id_input.setEnabled(unload_needed)
        self.pile_input.setEnabled(pile_needed)
        self.y_offset_input.setEnabled(pile_needed)

        self.load_label.setText("Load ID (required)" if load_needed else "Load ID (disabled)")
        self.unload_label.setText("Unload ID (required)" if unload_needed else "Unload ID (disabled)")

    def _reload_sites(self):
        try:
            self.entries = load_sites(SITES_PATH)
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Error", str(exc))
            return

        self.load_list.clear()
        self.unload_list.clear()
        prev_load = self.load_id_input.currentText()
        prev_unload = self.unload_id_input.currentText()
        load_ids = set()
        unload_ids = set()
        for entry in self.entries:
            text = format_entry(entry)
            item = QtWidgets.QListWidgetItem(text)
            if entry.get("type") == "load":
                self.load_list.addItem(item)
                if entry.get("id"):
                    load_ids.add(entry.get("id"))
            elif entry.get("type") == "unload":
                self.unload_list.addItem(item)
                if entry.get("id"):
                    unload_ids.add(entry.get("id"))

        self.load_ids = sorted(load_ids)
        self.unload_ids = sorted(unload_ids)
        self.load_id_input.blockSignals(True)
        self.unload_id_input.blockSignals(True)
        self.load_id_input.clear()
        self.unload_id_input.clear()
        self.load_id_input.addItems(self.load_ids)
        self.unload_id_input.addItems(self.unload_ids)
        if prev_load:
            self.load_id_input.setCurrentText(prev_load)
        if prev_unload:
            self.unload_id_input.setCurrentText(prev_unload)
        self.load_id_input.blockSignals(False)
        self.unload_id_input.blockSignals(False)
        self._append_log(f"Loaded {len(self.entries)} entries from {SITES_PATH}")

    def _select_load_from_list(self, item):
        text = item.text()
        if "id=" in text:
            self.load_id_input.setCurrentText(text.split("id=")[1].split()[0])

    def _select_unload_from_list(self, item):
        text = item.text()
        if "id=" in text:
            self.unload_id_input.setCurrentText(text.split("id=")[1].split()[0])

    def _collect_args(self):
        mode = self.mode_combo.currentText()
        load_id_raw = self.load_id_input.currentText()
        unload_id_raw = self.unload_id_input.currentText()
        load_id = canonicalize_id(load_id_raw, "load", set(self.load_ids))
        unload_id = canonicalize_id(unload_id_raw, "unload", set(self.unload_ids))
        args = {
            "mode": mode,
            "load_id": load_id,
            "unload_id": unload_id,
            "pile_percent": self.pile_input.value() if self.pile_input.isEnabled() else None,
            "y_offset": self.y_offset_input.value() if self.y_offset_input.isEnabled() else None,
            "wait_timeout": self.wait_timeout_input.value(),
            "result_timeout": self.result_timeout_input.value(),
        }

        goal_is_load = mode in ("unload-load", "no-load")
        if goal_is_load and not load_id:
            raise RuntimeError("Load ID is required for this mode.")
        if not goal_is_load and not unload_id:
            raise RuntimeError("Unload ID is required for this mode.")
        if mode in ("unload-load", "no-load"):
            if args["pile_percent"] is None or args["y_offset"] is None:
                raise RuntimeError("Pile percent and y_offset are required for this mode.")
        return args

    def _send(self):
        if self.current_thread:
            QtWidgets.QMessageBox.information(self, "Busy", "Action already running.")
            return
        try:
            args = self._collect_args()
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "Invalid input", str(exc))
            return
        self._start_thread(args)
        self.last_args = args
        self.send_again_button.setEnabled(True)

    def _send_again(self):
        if not self.last_args:
            return
        if self.current_thread:
            QtWidgets.QMessageBox.information(self, "Busy", "Action already running.")
            return
        self._start_thread(self.last_args)

    def _start_thread(self, args: Dict):
        worker = ActionWorker(**args)
        self.current_thread = worker
        self.send_button.setEnabled(False)
        self.send_again_button.setEnabled(False)
        self.cancel_button.setEnabled(True)

        worker.log.connect(self._append_log)
        worker.done.connect(self._on_finished)
        worker.start()
        self._append_log(f"Sending mode={args['mode']}")

    def _cancel_current(self):
        if self.current_thread:
            self.current_thread.cancel()

    def _on_finished(self, success: bool, message: str):
        self._append_log(message)
        self.send_button.setEnabled(True)
        self.send_again_button.setEnabled(bool(self.last_args))
        self.cancel_button.setEnabled(False)
        self.current_thread = None
        if not success:
            QtWidgets.QMessageBox.warning(self, "Action result", message)


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
