# Simple Navigator

A lightweight ROS2 navigation module for holonomic robots with waypoint management.

## Features

- Waypoint configuration via YAML file
- Go-signal triggered navigation
- Holonomic wheel control (omnidirectional)
- TF-based localization
- Extensible controller architecture (PID, MPC, Pure Pursuit ready)

## Installation

### 1. Clone and Build

```bash
cd ~/ros2_ws/src
git clone <repository_url>
cd ..
colcon build --packages-select simple_navigator
source install/setup.bash
```

### 2. Install Dependencies

```bash
# Python dependencies
pip3 install transforms3d PyQt5 PyYAML

# Or on Ubuntu/Debian
sudo apt install python3-pyqt5 python3-yaml
```

## Dependencies

- ROS2 (tested on Humble/Iron)
- Python 3.8+
- `transforms3d` - TF transformations (install via: `pip3 install transforms3d`)
- `PyQt5` - GUI framework (install via: `pip3 install PyQt5`)
- `PyYAML` - YAML file handling (install via: `pip3 install PyYAML`)

## Usage

### 1. Configure Waypoints

Edit `config/waypoints.yaml` to define your waypoints:

```yaml
waypoints:
  home:
    x: 0.0
    y: 0.0
    yaw: 0.0
  point_a:
    x: 1.0
    y: 0.0
    yaw: 1.5708
```

### 2. Launch the Node

```bash
ros2 launch simple_navigator navigator.launch.py
```

Or with custom config:

```bash
ros2 launch simple_navigator navigator.launch.py config_file:=/path/to/config.yaml
```

### 3. Set Target Waypoint

**Option A: Publish Pose via Command Line**

```bash
# Publish target pose (x=1.0, y=0.0, yaw=90°)
ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'odom'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7071, w: 0.7071}}}"
```

**Option B: Use Pre-defined Waypoint Name**

```bash
# The node loads waypoints from config/waypoints.yaml
# home, point_a, point_b, point_c are available by default
```

**Option C: Add Waypoint via Python API**

```python
from simple_navigator.waypoint_manager import WaypointManager

manager = WaypointManager()
manager.add_waypoint("my_point", x=2.5, y=1.0, yaw=0.785)
waypoint = manager.get_waypoint("my_point")
print(f"Waypoint: {waypoint}")
```

### 4. Start Navigation

**Command Line:**
```bash
# Send go signal
ros2 topic pub /go std_msgs/msg/Bool "{data: true}" --once

# Or keep publishing to maintain go state
ros2 topic pub /go std_msgs/msg/Bool "{data: true}" -r 1
```

**Python API:**
```python
import rclpy
from std_msgs.msg import Bool

rclpy.init()
node = rclpy.create_node('go_sender')
pub = node.create_publisher(Bool, 'go', 10)
pub.publish(Bool(data=True))
```

### 5. Monitor Progress

**Watch goal reached status:**
```bash
ros2 topic echo /goal_reached
```

**Monitor robot velocity commands:**
```bash
ros2 topic echo /cmd_vel
```

**Check available topics:**
```bash
ros2 topic list
ros2 topic info /go
ros2 topic info /target_pose
ros2 topic info /goal_reached
```

**The robot will:**
1. Move to the target waypoint
2. Stop when within tolerance (default: 0.05m position, 0.05rad yaw)
3. Publish `True` to `/goal_reached`
4. Wait for next `go` signal

### 6. Complete Workflow Example

```bash
# Terminal 1: Launch navigator
ros2 launch simple_navigator navigator.launch.py

# Terminal 2: Set target and start navigation
# Set target pose
ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'odom'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}" --once

# Wait a moment, then send go signal
sleep 1
ros2 topic pub /go std_msgs/msg/Bool "{data: true}" --once

# Monitor progress
ros2 topic echo /goal_reached
```

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Publish | Velocity commands |
| `/go` | `std_msgs/Bool` | Subscribe | Start navigation |
| `/target_pose` | `geometry_msgs/PoseStamped` | Subscribe | Set target waypoint |
| `/goal_reached` | `std_msgs/Bool` | Publish | Goal reached notification |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `config_file` | string | `""` | Path to waypoints config |
| `control_frequency` | double | 20.0 | Control loop frequency (Hz) |
| `base_frame` | string | `base_link` | Robot base TF frame |
| `odom_frame` | string | `odom` | Odometry TF frame |
| `position_tolerance` | double | 0.05 | Goal position tolerance (m) |
| `yaw_tolerance` | double | 0.05 | Goal yaw tolerance (rad) |
| `default_waypoint` | string | `home` | Default waypoint name |

## Extending the Controller

The `Controller` abstract base class allows different control strategies:

```python
from simple_navigator.controller import Controller, RobotState, VelocityCommand

class MPCController(Controller):
    def compute_velocity(self, current_state, target_x, target_y, target_yaw):
        # Your MPC implementation
        pass
    
    def reset(self):
        pass
    
    def set_parameters(self, **kwargs):
        pass
```

## Architecture

```
simple_navigator/
├── config/
│   └── waypoints.yaml       # Waypoint configuration
├── launch/
│   └── navigator.launch.py  # Launch file
├── simple_navigator/
│   ├── __init__.py
│   ├── navigator_node.py    # Main ROS2 node
│   ├── waypoint_manager.py  # Waypoint management
│   ├── controller.py        # Abstract controller interface
│   └── simple_controller.py # PID implementation
├── package.xml
├── setup.py
└── setup.cfg
```

## GUI Tool: Waypoint Editor

A PyQt5-based graphical interface for waypoint management and navigation control.

### Features

- **Interactive Waypoint Editor**: Add, edit, delete waypoints visually
- **TF Position Display**: Real-time robot position from TF
- **YAML Import/Export**: Load and save waypoint configurations
- **Direct Topic Publishing**: Send waypoints directly to navigator
- **Navigation Control**: GO/STOP buttons for navigation

### Installation

```bash
# Install PyQt5 (if not already installed)
pip3 install PyQt5

# Or on Ubuntu/Debian
sudo apt install python3-pyqt5
```

### Usage

```bash
# Launch the waypoint editor
ros2 run simple_navigator waypoint_editor
```

### Interface Overview

**Waypoint Editor Tab:**
- Current robot position display (from TF)
- Input fields for waypoint name, X, Y, Yaw (rad/deg)
- Waypoint table with selection
- Load/Save YAML configuration buttons

**Navigation Control Tab:**
- Publish selected waypoint as target
- GO button to start navigation
- STOP button to cancel navigation
- Status display

### Workflow

1. Launch navigator node in one terminal
2. Launch waypoint editor in another terminal
3. Use "Use Current Position" to capture robot's current location
4. Add waypoints or load from YAML
5. Select a waypoint and click "Publish Target Pose"
6. Click "GO - Start Navigation" to begin

## Future Extensions

- **Path Planning**: Add global/local planner interface
- **MPC Controller**: Model Predictive Control for better tracking
- **Pure Pursuit**: For path following applications
- **Obstacle Avoidance**: Integration with costmaps
- **Multi-waypoint missions**: Queue-based waypoint execution

## Testing

### Run Unit Tests

```bash
# Install pytest (if not already installed)
pip3 install pytest

# Run tests
colcon test --packages-select simple_navigator --event-handlers console_direct+
# Or directly
python3 test/test_navigator.py
```

Test coverage:
- WaypointManager: CRUD operations, temporary waypoints
- Controller: Robot state, goal detection, velocity commands
- SimpleController: PID control, velocity limits, deadband, navigation simulation

### Integration Test with Mock Robot

The package includes a mock robot node for testing without real hardware.

```bash
# Terminal 1: Launch integration test (mock_robot + navigator)
ros2 launch simple_navigator test_integration.launch.py

# Terminal 2: Send target and go signal
ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'odom'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}" --once

ros2 topic pub /go std_msgs/msg/Bool "{data: true}" --once

# Terminal 3: Monitor topics
ros2 topic echo /cmd_vel
ros2 topic echo /goal_reached
ros2 topic echo /odom
```

### Run Waypoint Editor with Mock Robot

```bash
# Terminal 1: Launch mock robot only
ros2 run simple_navigator mock_robot

# Terminal 2: Launch navigator
ros2 launch simple_navigator navigator.launch.py

# Terminal 3: Launch Qt waypoint editor (optional, requires display)
ros2 run simple_navigator waypoint_editor
```

## Architecture

```
simple_navigator/
├── config/
│   └── waypoints.yaml       # Waypoint configuration
├── launch/
│   ├── navigator.launch.py  # Main launch file
│   └── test_integration.launch.py  # Integration test launch
├── simple_navigator/
│   ├── __init__.py
│   ├── navigator_node.py    # Main ROS2 node
│   ├── waypoint_manager.py  # Waypoint management
│   ├── controller.py        # Abstract controller interface
│   ├── simple_controller.py # PID implementation
│   ├── waypoint_editor.py   # Qt GUI tool
│   └── mock_robot.py        # Mock robot for simulation
├── test/
│   └── test_navigator.py    # Unit tests
├── package.xml
├── setup.py
└── setup.cfg
```

## License

MIT