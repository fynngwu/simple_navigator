# Simple Navigator

A lightweight ROS2 navigation module for holonomic robots with waypoint management.

## Features

- Waypoint configuration via YAML file
- Go-signal triggered navigation
- Holonomic wheel control (omnidirectional)
- TF-based localization
- Extensible controller architecture (PID, MPC, Pure Pursuit ready)

## Installation

```bash
cd ~/ros2_ws/src
git clone <repository_url>
cd ..
colcon build --packages-select simple_navigator
source install/setup.bash
```

## Dependencies

- ROS2 (tested on Humble/Iron)
- Python 3.8+
- `tf-transformations` (install via: `pip3 install transforms3d`)

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

**Option A: By Name (from config file)**

```bash
ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'odom'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}"
```

**Option B: Use Waypoint Name (programmatically)**

In your code, call the service or use the API.

### 4. Start Navigation

Publish a `True` to the `/go` topic:

```bash
ros2 topic pub /go std_msgs/msg/Bool "{data: true}" --once
```

### 5. Monitor Progress

The robot will:
1. Move to the target waypoint
2. Stop when within tolerance
3. Publish `True` to `/goal_reached`
4. Wait for next `go` signal

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

## Future Extensions

- **Path Planning**: Add global/local planner interface
- **MPC Controller**: Model Predictive Control for better tracking
- **Pure Pursuit**: For path following applications
- **Obstacle Avoidance**: Integration with costmaps
- **Multi-waypoint missions**: Queue-based waypoint execution

## License

MIT