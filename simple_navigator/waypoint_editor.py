#!/usr/bin/env python3
"""Qt-based Waypoint Editor GUI for Simple Navigator.

This module provides a graphical interface for:
- Setting and editing waypoints interactively
- Saving waypoints to YAML configuration files
- Publishing waypoints directly to the navigator node
- Monitoring robot position via TF
- Sending go signals to start navigation
"""

import sys
import os
import yaml
import math
from typing import Optional, Dict, List

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QMessageBox,
    QFileDialog,
    QGroupBox,
    QTabWidget,
    QDoubleSpinBox,
    QCheckBox,
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Odometry


class Waypoint:
    """Simple waypoint data class."""

    def __init__(self, name: str, x: float, y: float, yaw: float):
        self.name = name
        self.x = x
        self.y = y
        self.yaw = yaw

    def to_dict(self) -> dict:
        return {"x": self.x, "y": self.y, "yaw": self.yaw}

    @classmethod
    def from_dict(cls, name: str, data: dict) -> "Waypoint":
        return cls(
            name=name,
            x=float(data.get("x", 0.0)),
            y=float(data.get("y", 0.0)),
            yaw=float(data.get("yaw", 0.0)),
        )


class RosInterface(Node, QObject):
    """ROS2 interface for the waypoint editor."""

    tf_update_signal = pyqtSignal(float, float, float)  # x, y, yaw
    odom_update_signal = pyqtSignal(float, float, float)  # x, y, yaw

    def __init__(self):
        Node.__init__(self, "waypoint_editor_node")
        QObject.__init__(self)

        # Publishers
        self.target_pose_pub = self.create_publisher(PoseStamped, "target_pose", 10)
        self.go_pub = self.create_publisher(Bool, "go", 10)

        # Subscribers
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Optional: subscribe to odometry for alternative position tracking
        self.odom_sub = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )

        self._current_x = 0.0
        self._current_y = 0.0
        self._current_yaw = 0.0
        self._has_position = False

        # Timer for TF updates
        self.tf_timer = self.create_timer(0.1, self.tf_callback)  # 10 Hz

    def tf_callback(self):
        """Periodically check TF for robot position."""
        try:
            transform = self.tf_buffer.lookup_transform(
                "odom", "base_link", rclpy.time.Time()
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            q = transform.transform.rotation
            yaw = self.quaternion_to_yaw(q)

            self._current_x = x
            self._current_y = y
            self._current_yaw = yaw
            self._has_position = True

            self.tf_update_signal.emit(x, y, yaw)

        except Exception as e:
            # TF not available yet
            pass

    def odom_callback(self, msg: Odometry):
        """Callback for odometry messages."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q)

        self.odom_update_signal.emit(x, y, yaw)

    def quaternion_to_yaw(self, q) -> float:
        """Convert quaternion to yaw angle."""
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def publish_target_pose(
        self, x: float, y: float, yaw: float, frame_id: str = "odom"
    ):
        """Publish target pose to navigator."""
        from transforms3d.euler import euler2quat as quaternion_from_euler

        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.target_pose_pub.publish(msg)
        self.get_logger().info(
            f"Published target pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )

    def publish_go(self, go: bool = True):
        """Publish go signal."""
        msg = Bool()
        msg.data = go
        self.go_pub.publish(msg)
        self.get_logger().info(f"Published go signal: {go}")

    def get_current_position(self) -> tuple:
        """Get current robot position."""
        return self._current_x, self._current_y, self._current_yaw

    def has_position(self) -> bool:
        """Check if position is available."""
        return self._has_position


class WaypointEditorWindow(QMainWindow):
    """Main waypoint editor window."""

    def __init__(self, ros_node: RosInterface):
        super().__init__()
        self.ros_node = ros_node
        self.waypoints: Dict[str, Waypoint] = {}

        self.init_ui()
        self.start_ros_timer()

    def init_ui(self):
        """Initialize the user interface."""
        self.setWindowTitle("Simple Navigator - Waypoint Editor")
        self.setMinimumSize(900, 700)

        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Tab widget
        tab_widget = QTabWidget()
        main_layout.addWidget(tab_widget)

        # Waypoint Editor Tab
        waypoint_tab = QWidget()
        tab_widget.addTab(waypoint_tab, "Waypoint Editor")
        waypoint_layout = QVBoxLayout(waypoint_tab)

        # Current position group
        pos_group = QGroupBox("Current Robot Position (from TF)")
        pos_layout = QGridLayout()
        self.pos_x_label = QLabel("X: -- m")
        self.pos_y_label = QLabel("Y: -- m")
        self.pos_yaw_label = QLabel("Yaw: -- rad")
        self.pos_status_label = QLabel("Status: Waiting for TF...")
        self.pos_status_label.setStyleSheet("color: orange")

        pos_layout.addWidget(self.pos_x_label, 0, 0)
        pos_layout.addWidget(self.pos_y_label, 0, 1)
        pos_layout.addWidget(self.pos_yaw_label, 0, 2)
        pos_layout.addWidget(self.pos_status_label, 0, 3)

        # Button to set current position as waypoint
        self.use_current_pos_btn = QPushButton("Use Current Position")
        self.use_current_pos_btn.clicked.connect(self.use_current_position)
        pos_layout.addWidget(self.use_current_pos_btn, 0, 4)

        pos_group.setLayout(pos_layout)
        waypoint_layout.addWidget(pos_group)

        # Input group
        input_group = QGroupBox("Add/Edit Waypoint")
        input_layout = QGridLayout()

        input_layout.addWidget(QLabel("Name:"), 0, 0)
        self.name_input = QLineEdit()
        self.name_input.setPlaceholderText("e.g., home, point_a")
        input_layout.addWidget(self.name_input, 0, 1)

        input_layout.addWidget(QLabel("X (m):"), 1, 0)
        self.x_input = QDoubleSpinBox()
        self.x_input.setRange(-1000, 1000)
        self.x_input.setDecimals(3)
        self.x_input.setValue(0.0)
        input_layout.addWidget(self.x_input, 1, 1)

        input_layout.addWidget(QLabel("Y (m):"), 2, 0)
        self.y_input = QDoubleSpinBox()
        self.y_input.setRange(-1000, 1000)
        self.y_input.setDecimals(3)
        self.y_input.setValue(0.0)
        input_layout.addWidget(self.y_input, 2, 1)

        input_layout.addWidget(QLabel("Yaw (rad):"), 3, 0)
        self.yaw_input = QDoubleSpinBox()
        self.yaw_input.setRange(-math.pi, math.pi)
        self.yaw_input.setDecimals(3)
        self.yaw_input.setValue(0.0)
        input_layout.addWidget(self.yaw_input, 3, 1)

        # Yaw in degrees
        input_layout.addWidget(QLabel("Yaw (deg):"), 4, 0)
        self.yaw_deg_input = QDoubleSpinBox()
        self.yaw_deg_input.setRange(-180, 180)
        self.yaw_deg_input.setDecimals(1)
        self.yaw_deg_input.setValue(0.0)
        self.yaw_deg_input.valueChanged.connect(self.yaw_deg_changed)
        input_layout.addWidget(self.yaw_deg_input, 4, 1)

        input_group.setLayout(input_layout)
        waypoint_layout.addWidget(input_group)

        # Button group
        btn_layout = QHBoxLayout()
        self.add_btn = QPushButton("Add Waypoint")
        self.add_btn.clicked.connect(self.add_waypoint)
        btn_layout.addWidget(self.add_btn)

        self.update_btn = QPushButton("Update Selected")
        self.update_btn.clicked.connect(self.update_waypoint)
        btn_layout.addWidget(self.update_btn)

        self.delete_btn = QPushButton("Delete Selected")
        self.delete_btn.clicked.connect(self.delete_waypoint)
        btn_layout.addWidget(self.delete_btn)

        self.clear_btn = QPushButton("Clear All")
        self.clear_btn.clicked.connect(self.clear_waypoints)
        btn_layout.addWidget(self.clear_btn)

        waypoint_layout.addLayout(btn_layout)

        # Table
        self.waypoint_table = QTableWidget()
        self.waypoint_table.setColumnCount(4)
        self.waypoint_table.setHorizontalHeaderLabels(
            ["Name", "X (m)", "Y (m)", "Yaw (rad)"]
        )
        self.waypoint_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.waypoint_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.waypoint_table.itemSelectionChanged.connect(self.on_selection_changed)
        waypoint_layout.addWidget(self.waypoint_table)

        # File operations group
        file_group = QGroupBox("Configuration File")
        file_layout = QHBoxLayout()

        self.load_btn = QPushButton("Load YAML")
        self.load_btn.clicked.connect(self.load_yaml)
        file_layout.addWidget(self.load_btn)

        self.save_btn = QPushButton("Save to YAML")
        self.save_btn.clicked.connect(self.save_yaml)
        file_layout.addWidget(self.save_btn)

        file_group.setLayout(file_layout)
        waypoint_layout.addWidget(file_group)

        # Control tab
        control_tab = QWidget()
        tab_widget.addTab(control_tab, "Navigation Control")
        control_layout = QVBoxLayout(control_tab)

        # Target waypoint selection
        target_group = QGroupBox("Send Target to Navigator")
        target_layout = QVBoxLayout()

        self.target_info_label = QLabel(
            "Selected waypoint will be published to /target_pose"
        )
        target_layout.addWidget(self.target_info_label)

        self.publish_target_btn = QPushButton("Publish Target Pose")
        self.publish_target_btn.clicked.connect(self.publish_target)
        self.publish_target_btn.setStyleSheet(
            "background-color: #4CAF50; color: white; padding: 10px;"
        )
        target_layout.addWidget(self.publish_target_btn)

        target_group.setLayout(target_layout)
        control_layout.addWidget(target_group)

        # Go signal group
        go_group = QGroupBox("Navigation Control")
        go_layout = QVBoxLayout()

        self.go_btn = QPushButton("GO - Start Navigation")
        self.go_btn.clicked.connect(lambda: self.send_go_signal(True))
        self.go_btn.setStyleSheet(
            "background-color: #2196F3; color: white; padding: 15px; font-weight: bold;"
        )
        go_layout.addWidget(self.go_btn)

        self.stop_btn = QPushButton("STOP - Cancel Navigation")
        self.stop_btn.clicked.connect(lambda: self.send_go_signal(False))
        self.stop_btn.setStyleSheet(
            "background-color: #f44336; color: white; padding: 15px;"
        )
        go_layout.addWidget(self.stop_btn)

        go_group.setLayout(go_layout)
        control_layout.addWidget(go_group)

        # Status group
        status_group = QGroupBox("Status")
        status_layout = QVBoxLayout()

        self.status_label = QLabel("Ready")
        status_layout.addWidget(self.status_label)

        status_group.setLayout(status_layout)
        control_layout.addWidget(status_group)

        control_layout.addStretch()

    def start_ros_timer(self):
        """Start Qt timer for ROS updates."""
        self.ros_node.tf_update_signal.connect(self.update_position_display)
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(100)  # 10 Hz

    def spin_ros(self):
        """Spin ROS node."""
        rclpy.spin_once(self.ros_node, timeout_sec=0)

    def update_position_display(self, x: float, y: float, yaw: float):
        """Update position display from TF."""
        self.pos_x_label.setText(f"X: {x:.3f} m")
        self.pos_y_label.setText(f"Y: {y:.3f} m")
        self.pos_yaw_label.setText(f"Yaw: {yaw:.3f} rad")
        self.pos_status_label.setText("Status: TF OK")
        self.pos_status_label.setStyleSheet("color: green")

    def use_current_position(self):
        """Set input fields to current robot position."""
        if self.ros_node.has_position():
            x, y, yaw = self.ros_node.get_current_position()
            self.x_input.setValue(x)
            self.y_input.setValue(y)
            self.yaw_input.setValue(yaw)
            self.yaw_deg_input.setValue(math.degrees(yaw))
            self.status_label.setText(f"Loaded current position: ({x:.2f}, {y:.2f})")
        else:
            QMessageBox.warning(self, "No Position", "TF position not available yet")

    def yaw_deg_changed(self, value: float):
        """Sync yaw rad input when deg changes."""
        self.yaw_input.setValue(math.radians(value))

    def add_waypoint(self):
        """Add waypoint from input fields."""
        name = self.name_input.text().strip()
        if not name:
            QMessageBox.warning(self, "Invalid Name", "Please enter a waypoint name")
            return

        if name in self.waypoints:
            reply = QMessageBox.question(
                self,
                "Waypoint Exists",
                f"Waypoint '{name}' already exists. Overwrite?",
                QMessageBox.Yes | QMessageBox.No,
            )
            if reply != QMessageBox.Yes:
                return

        x = self.x_input.value()
        y = self.y_input.value()
        yaw = self.yaw_input.value()

        self.waypoints[name] = Waypoint(name, x, y, yaw)
        self.refresh_table()
        self.status_label.setText(f"Added waypoint: {name}")

    def update_waypoint(self):
        """Update selected waypoint."""
        selected_rows = self.waypoint_table.selectedItems()
        if not selected_rows:
            QMessageBox.warning(
                self, "No Selection", "Please select a waypoint to update"
            )
            return

        row = selected_rows[0].row()
        old_name = self.waypoint_table.item(row, 0).text()

        name = self.name_input.text().strip()
        if not name:
            QMessageBox.warning(self, "Invalid Name", "Please enter a waypoint name")
            return

        x = self.x_input.value()
        y = self.y_input.value()
        yaw = self.yaw_input.value()

        if old_name != name:
            del self.waypoints[old_name]

        self.waypoints[name] = Waypoint(name, x, y, yaw)
        self.refresh_table()
        self.status_label.setText(f"Updated waypoint: {name}")

    def delete_waypoint(self):
        """Delete selected waypoint."""
        selected_rows = self.waypoint_table.selectedItems()
        if not selected_rows:
            QMessageBox.warning(
                self, "No Selection", "Please select a waypoint to delete"
            )
            return

        row = selected_rows[0].row()
        name = self.waypoint_table.item(row, 0).text()

        reply = QMessageBox.question(
            self,
            "Confirm Delete",
            f"Delete waypoint '{name}'?",
            QMessageBox.Yes | QMessageBox.No,
        )
        if reply == QMessageBox.Yes:
            del self.waypoints[name]
            self.refresh_table()
            self.status_label.setText(f"Deleted waypoint: {name}")

    def clear_waypoints(self):
        """Clear all waypoints."""
        reply = QMessageBox.question(
            self,
            "Confirm Clear",
            "Delete all waypoints?",
            QMessageBox.Yes | QMessageBox.No,
        )
        if reply == QMessageBox.Yes:
            self.waypoints.clear()
            self.refresh_table()
            self.status_label.setText("Cleared all waypoints")

    def on_selection_changed(self):
        """Load selected waypoint into input fields."""
        selected_rows = self.waypoint_table.selectedItems()
        if selected_rows:
            row = selected_rows[0].row()
            name = self.waypoint_table.item(row, 0).text()
            waypoint = self.waypoints[name]

            self.name_input.setText(name)
            self.x_input.setValue(waypoint.x)
            self.y_input.setValue(waypoint.y)
            self.yaw_input.setValue(waypoint.yaw)
            self.yaw_deg_input.setValue(math.degrees(waypoint.yaw))

    def refresh_table(self):
        """Refresh the waypoint table."""
        self.waypoint_table.setRowCount(0)
        for name, waypoint in sorted(self.waypoints.items()):
            row = self.waypoint_table.rowCount()
            self.waypoint_table.insertRow(row)
            self.waypoint_table.setItem(row, 0, QTableWidgetItem(waypoint.name))
            self.waypoint_table.setItem(row, 1, QTableWidgetItem(f"{waypoint.x:.3f}"))
            self.waypoint_table.setItem(row, 2, QTableWidgetItem(f"{waypoint.y:.3f}"))
            self.waypoint_table.setItem(row, 3, QTableWidgetItem(f"{waypoint.yaw:.3f}"))

    def load_yaml(self):
        """Load waypoints from YAML file."""
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Load Waypoint Configuration",
            "",
            "YAML Files (*.yaml);;All Files (*)",
        )
        if file_path:
            try:
                with open(file_path, "r") as f:
                    config = yaml.safe_load(f)

                if config and "waypoints" in config:
                    self.waypoints.clear()
                    for name, data in config["waypoints"].items():
                        self.waypoints[name] = Waypoint.from_dict(name, data)
                    self.refresh_table()
                    self.status_label.setText(
                        f"Loaded {len(self.waypoints)} waypoints from {file_path}"
                    )
                else:
                    QMessageBox.warning(
                        self, "Invalid Format", "No 'waypoints' section found in YAML"
                    )

            except Exception as e:
                QMessageBox.critical(self, "Load Error", f"Failed to load file: {e}")

    def save_yaml(self):
        """Save waypoints to YAML file."""
        if not self.waypoints:
            QMessageBox.warning(self, "No Waypoints", "No waypoints to save")
            return

        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Waypoint Configuration",
            "waypoints.yaml",
            "YAML Files (*.yaml);;All Files (*)",
        )
        if file_path:
            try:
                config = {
                    "waypoints": {
                        name: wp.to_dict() for name, wp in self.waypoints.items()
                    }
                }
                with open(file_path, "w") as f:
                    yaml.dump(config, f, default_flow_style=False)
                self.status_label.setText(
                    f"Saved {len(self.waypoints)} waypoints to {file_path}"
                )
            except Exception as e:
                QMessageBox.critical(self, "Save Error", f"Failed to save file: {e}")

    def publish_target(self):
        """Publish selected waypoint as target pose."""
        selected_rows = self.waypoint_table.selectedItems()
        if not selected_rows:
            QMessageBox.warning(
                self, "No Selection", "Please select a waypoint to publish"
            )
            return

        row = selected_rows[0].row()
        name = self.waypoint_table.item(row, 0).text()
        waypoint = self.waypoints[name]

        self.ros_node.publish_target_pose(waypoint.x, waypoint.y, waypoint.yaw)
        self.status_label.setText(f"Published target: {name}")

    def send_go_signal(self, go: bool):
        """Send go signal to navigator."""
        self.ros_node.publish_go(go)
        if go:
            self.status_label.setText("GO signal sent - Navigation started")
        else:
            self.status_label.setText("STOP signal sent - Navigation cancelled")


def main():
    """Main entry point."""
    rclpy.init()

    ros_node = RosInterface()
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)

    # Start executor in background
    import threading

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Create Qt application
    app = QApplication(sys.argv)
    app.setFont(QFont("Arial", 10))

    window = WaypointEditorWindow(ros_node)
    window.show()

    # Run Qt event loop
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
