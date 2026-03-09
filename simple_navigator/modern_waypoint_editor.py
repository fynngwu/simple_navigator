#!/usr/bin/env python3
"""Modern Waypoint Editor with 2D Map Visualization.

This module provides a modern graphical interface with:
- Interactive 2D map visualization
- Real-time robot position tracking
- Click-to-add waypoints
- Modern dark theme UI
- Navigation controls
"""

import sys
import math
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QListWidget, QGraphicsView, QGraphicsScene, QGraphicsPolygonItem, QGraphicsEllipseItem
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QPointF
from PyQt5.QtGui import QFont, QPolygonF, QBrush, QPen, QColor, QPainter

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

# --- Modern Style CSS ---
MODERN_STYLE = """
QMainWindow {
    background-color: #1e1e2e;
}
QWidget {
    color: #cdd6f4;
    font-family: 'Segoe UI', Arial, sans-serif;
}
QGraphicsView {
    background-color: #181825;
    border: 2px solid #313244;
    border-radius: 10px;
}
QListWidget {
    background-color: #181825;
    border: 2px solid #313244;
    border-radius: 8px;
    padding: 5px;
    font-size: 14px;
}
QListWidget::item:selected {
    background-color: #89b4fa;
    color: #1e1e2e;
    border-radius: 4px;
}
QPushButton {
    background-color: #313244;
    color: #cdd6f4;
    border: none;
    border-radius: 8px;
    padding: 12px;
    font-size: 14px;
    font-weight: bold;
}
QPushButton:hover {
    background-color: #45475a;
}
QPushButton#goBtn {
    background-color: #a6e3a1;
    color: #1e1e2e;
}
QPushButton#goBtn:hover {
    background-color: #94e289;
}
QPushButton#stopBtn {
    background-color: #f38ba8;
    color: #1e1e2e;
}
QPushButton#stopBtn:hover {
    background-color: #f0719b;
}
QLabel#titleLabel {
    font-size: 18px;
    font-weight: bold;
    color: #89b4fa;
}
"""


class RosInterface(Node, QObject):
    """ROS2 interface for the modern waypoint editor."""

    tf_update_signal = pyqtSignal(float, float, float)

    def __init__(self):
        Node.__init__(self, "modern_waypoint_editor")
        QObject.__init__(self)

        self.target_pose_pub = self.create_publisher(PoseStamped, "target_pose", 10)
        self.go_pub = self.create_publisher(Bool, "go", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(0.05, self.tf_callback)  # 20Hz for smooth updates

    def tf_callback(self):
        """Periodically check TF for robot position."""
        try:
            transform = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time())
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            q = transform.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.tf_update_signal.emit(x, y, yaw)
        except Exception:
            pass

    def publish_target_pose(self, x: float, y: float):
        """Publish target pose to navigator."""
        msg = PoseStamped()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        self.target_pose_pub.publish(msg)

    def publish_go(self, go: bool = True):
        """Publish go signal."""
        msg = Bool()
        msg.data = go
        self.go_pub.publish(msg)


class MapView(QGraphicsView):
    """Interactive 2D coordinate system view."""

    map_clicked = pyqtSignal(float, float)

    def __init__(self):
        super().__init__()
        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.setRenderHint(QPainter.Antialiasing)
        self.setSceneRect(-300, -300, 600, 600)  # 600x600 pixels

        # Scale: 1 meter = 50 pixels
        self.scale_factor = 50.0
        self.draw_grid()

        # Robot item (green triangle)
        self.robot_item = QGraphicsPolygonItem(QPolygonF([QPointF(15, 0), QPointF(-10, -10), QPointF(-10, 10)]))
        self.robot_item.setBrush(QBrush(QColor("#a6e3a1")))
        self.robot_item.setPen(QPen(Qt.NoPen))
        self.scene.addItem(self.robot_item)

        self.waypoint_items = {}

    def draw_grid(self):
        """Draw coordinate grid."""
        pen = QPen(QColor("#313244"))
        pen.setWidth(1)
        for i in range(-300, 301, int(self.scale_factor)):
            self.scene.addLine(i, -300, i, 300, pen)
            self.scene.addLine(-300, i, 300, i, pen)
        # Coordinate axes
        axis_pen = QPen(QColor("#585b70"))
        axis_pen.setWidth(2)
        self.scene.addLine(0, -300, 0, 300, axis_pen)
        self.scene.addLine(-300, 0, 300, 0, axis_pen)

    def update_robot(self, x, y, yaw):
        """Update robot position and orientation."""
        # ROS to screen coordinates (screen Y is down, ROS Y is up)
        px, py = x * self.scale_factor, -y * self.scale_factor
        self.robot_item.setPos(px, py)
        # Screen rotation is clockwise, ROS yaw is counterclockwise
        self.robot_item.setRotation(-math.degrees(yaw))

    def mousePressEvent(self, event):
        """Handle mouse clicks on the map."""
        if event.button() == Qt.LeftButton:
            pos = self.mapToScene(event.pos())
            ros_x = pos.x() / self.scale_factor
            ros_y = -pos.y() / self.scale_factor
            self.map_clicked.emit(ros_x, ros_y)

    def add_waypoint_visual(self, name, x, y):
        """Add visual waypoint marker on the map."""
        px, py = x * self.scale_factor, -y * self.scale_factor
        dot = QGraphicsEllipseItem(px - 5, py - 5, 10, 10)
        dot.setBrush(QBrush(QColor("#89b4fa")))
        dot.setPen(QPen(Qt.NoPen))
        self.scene.addItem(dot)
        self.waypoint_items[name] = dot

    def clear_waypoints(self):
        """Clear all waypoint markers."""
        for item in self.waypoint_items.values():
            self.scene.removeItem(item)
        self.waypoint_items.clear()


class ModernWaypointEditor(QMainWindow):
    """Modern waypoint editor with 2D visualization."""

    def __init__(self, ros_node: RosInterface):
        super().__init__()
        self.ros_node = ros_node
        self.waypoints = {}
        self.wp_count = 1

        self.init_ui()
        self.ros_node.tf_update_signal.connect(self.map_view.update_robot)
        self.ros_node.tf_update_signal.connect(self.update_status)

    def init_ui(self):
        """Initialize the user interface."""
        self.setWindowTitle("Modern Navigator Dashboard")
        self.setMinimumSize(950, 650)
        self.setStyleSheet(MODERN_STYLE)

        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QHBoxLayout(main_widget)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(20)

        # === Left: Map Area ===
        map_layout = QVBoxLayout()
        self.map_view = MapView()
        self.map_view.map_clicked.connect(self.handle_map_click)
        map_layout.addWidget(self.map_view)

        self.status_label = QLabel("Waiting for Odometry...")
        self.status_label.setStyleSheet("color: #a6e3a1; font-weight: bold;")
        map_layout.addWidget(self.status_label)
        layout.addLayout(map_layout, stretch=2)

        # === Right: Control Panel ===
        control_layout = QVBoxLayout()

        title = QLabel("Navigation Control")
        title.setObjectName("titleLabel")
        control_layout.addWidget(title)

        self.wp_list = QListWidget()
        control_layout.addWidget(self.wp_list)

        clear_btn = QPushButton("Clear Waypoints")
        clear_btn.clicked.connect(self.clear_waypoints)
        control_layout.addWidget(clear_btn)

        control_layout.addSpacing(20)

        pub_btn = QPushButton("Publish Selected Target")
        pub_btn.clicked.connect(self.publish_selected)
        control_layout.addWidget(pub_btn)

        go_btn = QPushButton("START NAVIGATION")
        go_btn.setObjectName("goBtn")
        go_btn.clicked.connect(lambda: self.ros_node.publish_go(True))
        control_layout.addWidget(go_btn)

        stop_btn = QPushButton("EMERGENCY STOP")
        stop_btn.setObjectName("stopBtn")
        stop_btn.clicked.connect(lambda: self.ros_node.publish_go(False))
        control_layout.addWidget(stop_btn)

        layout.addLayout(control_layout, stretch=1)

    def handle_map_click(self, x, y):
        """Handle map click to add waypoint."""
        name = f"WP_{self.wp_count}"
        self.wp_count += 1
        self.waypoints[name] = (x, y)
        self.wp_list.addItem(f"{name}: ({x:.2f}, {y:.2f})")
        self.map_view.add_waypoint_visual(name, x, y)

    def clear_waypoints(self):
        """Clear all waypoints."""
        self.waypoints.clear()
        self.wp_list.clear()
        self.map_view.clear_waypoints()
        self.wp_count = 1

    def publish_selected(self):
        """Publish selected waypoint as target."""
        selected = self.wp_list.currentItem()
        if selected:
            name = selected.text().split(":")[0]
            x, y = self.waypoints[name]
            self.ros_node.publish_target_pose(x, y)

    def update_status(self, x, y, yaw):
        """Update status label with robot position."""
        self.status_label.setText(f"Robot Position: X: {x:.2f}m | Y: {y:.2f}m | Yaw: {math.degrees(yaw):.1f}°")


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
    window = ModernWaypointEditor(ros_node)
    window.show()

    # Run Qt event loop
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
