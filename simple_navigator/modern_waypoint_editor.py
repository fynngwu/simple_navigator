import math
import sys
import threading

import matplotlib

matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure

import rclpy
from geometry_msgs.msg import PoseStamped
from PyQt5.QtCore import QObject, Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


def build_target_pose(
    x: float,
    y: float,
    yaw: float,
    stamp=None,
    frame_id: str = "odom",
) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    if stamp is not None:
        msg.header.stamp = stamp
    msg.pose.position.x = x
    msg.pose.position.y = y
    qx, qy, qz, qw = quaternion_from_yaw(yaw)
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg


class TargetPublisherNode(Node, QObject):
    pose_signal = pyqtSignal(float, float, float)

    def __init__(self):
        Node.__init__(self, "target_goal_editor")
        QObject.__init__(self)

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("target_topic", "target_pose")

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.target_topic = self.get_parameter("target_topic").value

        self.publisher = self.create_publisher(PoseStamped, self.target_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.05, self._emit_robot_pose)
        self._tf_warned = False
        self.get_logger().info(
            "Target goal editor ready; "
            f"publishing to /{self.target_topic}, TF={self.odom_frame}->{self.base_frame}"
        )

    def _emit_robot_pose(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
            q = transform.transform.rotation
            self.pose_signal.emit(
                transform.transform.translation.x,
                transform.transform.translation.y,
                yaw_from_quaternion(q.x, q.y, q.z, q.w),
            )
            self._tf_warned = False
        except Exception:
            if not self._tf_warned:
                self.get_logger().warn(
                    f"Waiting for TF: {self.odom_frame} -> {self.base_frame}..."
                )
                self._tf_warned = True

    def publish_target(self, x: float, y: float, yaw: float) -> None:
        msg = build_target_pose(
            x,
            y,
            yaw,
            self.get_clock().now().to_msg(),
            frame_id=self.odom_frame,
        )
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published target pose: frame={self.odom_frame}, x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )


class TargetGoalEditor(QMainWindow):
    def __init__(self, ros_node: TargetPublisherNode):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("Advanced Target Goal Publisher")
        self.resize(1000, 600)

        main_widget = QWidget()
        main_layout = QHBoxLayout(main_widget)

        self._init_control_panel(main_layout)
        self._init_plot_panel(main_layout)

        self.setCentralWidget(main_widget)
        self.ros_node.pose_signal.connect(self._update_robot_pose)
        self._update_target_plot_from_spinbox()

    def _init_control_panel(self, parent_layout) -> None:
        control_panel = QWidget()
        layout = QVBoxLayout(control_panel)
        layout.setAlignment(Qt.AlignTop)
        control_panel.setMaximumWidth(350)

        status_group = QGroupBox(
            f"Robot TF Status ({self.ros_node.odom_frame} -> {self.ros_node.base_frame})"
        )
        status_layout = QVBoxLayout()
        self.robot_pose_label = QLabel("Waiting for TF data...")
        self.robot_pose_label.setWordWrap(True)
        status_layout.addWidget(self.robot_pose_label)
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)

        input_group = QGroupBox("Target Configuration")
        form = QFormLayout()
        self.x_input = self._create_spinbox(-50.0, 50.0, 0.0)
        self.y_input = self._create_spinbox(-50.0, 50.0, 0.0)
        self.yaw_input = self._create_spinbox(-math.pi, math.pi, 0.0)
        self.x_input.valueChanged.connect(self._update_target_plot_from_spinbox)
        self.y_input.valueChanged.connect(self._update_target_plot_from_spinbox)
        self.yaw_input.valueChanged.connect(self._update_target_plot_from_spinbox)
        form.addRow("Target X (m)", self.x_input)
        form.addRow("Target Y (m)", self.y_input)
        form.addRow("Target Yaw (rad)", self.yaw_input)
        input_group.setLayout(form)
        layout.addWidget(input_group)

        publish_button = QPushButton("Publish Target Pose")
        publish_button.setMinimumHeight(50)
        publish_button.clicked.connect(self.publish_target)
        layout.addWidget(publish_button)

        parent_layout.addWidget(control_panel)

    def _init_plot_panel(self, parent_layout) -> None:
        plot_panel = QWidget()
        layout = QVBoxLayout(plot_panel)

        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.toolbar = NavigationToolbar(self.canvas, self)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_title("2D Pose Visualization (Click to set target X/Y)")
        self.ax.set_xlabel("X (meters)")
        self.ax.set_ylabel("Y (meters)")
        self.ax.grid(True, linestyle="--", alpha=0.6)
        self.ax.set_aspect("equal", adjustable="datalim")
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)

        self.robot_quiver = self.ax.quiver(
            0,
            0,
            1,
            0,
            color="blue",
            scale=10,
            pivot="tail",
            label="Current Robot",
        )
        self.target_quiver = self.ax.quiver(
            0,
            0,
            1,
            0,
            color="red",
            scale=10,
            pivot="tail",
            label="Target Goal",
        )
        self.ax.legend(loc="upper right")
        self.canvas.mpl_connect("button_press_event", self.on_canvas_click)

        layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas)
        parent_layout.addWidget(plot_panel)

    def _create_spinbox(self, minimum: float, maximum: float, value: float) -> QDoubleSpinBox:
        widget = QDoubleSpinBox()
        widget.setDecimals(3)
        widget.setRange(minimum, maximum)
        widget.setSingleStep(0.1)
        widget.setValue(value)
        return widget

    def on_canvas_click(self, event) -> None:
        if event.inaxes != self.ax or self.toolbar.mode != "":
            return
        if event.button != 1 or event.xdata is None or event.ydata is None:
            return

        self.x_input.blockSignals(True)
        self.y_input.blockSignals(True)
        self.x_input.setValue(event.xdata)
        self.y_input.setValue(event.ydata)
        self.x_input.blockSignals(False)
        self.y_input.blockSignals(False)
        self._update_target_plot_from_spinbox()

    def _update_target_plot_from_spinbox(self) -> None:
        x = self.x_input.value()
        y = self.y_input.value()
        yaw = self.yaw_input.value()
        dx = math.cos(yaw)
        dy = math.sin(yaw)
        self.target_quiver.set_offsets([[x, y]])
        self.target_quiver.set_UVC(dx, dy)
        self.canvas.draw_idle()

    def _update_robot_pose(self, x: float, y: float, yaw: float) -> None:
        self.robot_pose_label.setText(
            f"<b>X:</b> {x:.3f} m<br>"
            f"<b>Y:</b> {y:.3f} m<br>"
            f"<b>Yaw:</b> {yaw:.3f} rad ({math.degrees(yaw):.1f}°)"
        )
        dx = math.cos(yaw)
        dy = math.sin(yaw)
        self.robot_quiver.set_offsets([[x, y]])
        self.robot_quiver.set_UVC(dx, dy)
        self.canvas.draw_idle()

    def publish_target(self) -> None:
        self.ros_node.publish_target(
            self.x_input.value(),
            self.y_input.value(),
            self.yaw_input.value(),
        )


def main():
    rclpy.init()
    ros_node = TargetPublisherNode()
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    app = QApplication(sys.argv)
    window = TargetGoalEditor(ros_node)
    window.show()

    try:
        exit_code = app.exec_()
    finally:
        executor.shutdown()
        thread.join(timeout=1.0)
        ros_node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except ExternalShutdownException:
                pass
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
