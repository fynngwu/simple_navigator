# simple_navigator

Minimal ROS2 navigation demo for a holonomic robot. The package contains:

- `navigator`: generates an SE(2) trajectory from current pose to target pose and tracks it
- `mock_robot`: simulates a holonomic robot and publishes `odom -> base_link`
- `target_goal_editor`: PyQt5 + Matplotlib tool for publishing `target_pose`

## Project Structure

```text
simple_navigator/
├── config/
│   ├── target.yaml
│   ├── target_diag.yaml
│   ├── target_origin.yaml
│   └── target_spin.yaml
├── launch/
│   ├── modern_editor.launch.py
│   └── navigator.launch.py
├── simple_navigator/
│   ├── config_loader.py
│   ├── controller.py
│   ├── math_utils.py
│   ├── mock_robot.py
│   ├── modern_waypoint_editor.py
│   ├── navigator_node.py
│   └── trajectory.py
└── test/
    ├── test_integration_navigator.py
    └── test_navigator.py
```

## Quick Start

Load ROS and set local cache/log directories:

```bash
source /opt/ros/humble/setup.bash
export UV_CACHE_DIR=/home/wufy/git_resp/simple_navigator/.uv-cache
export ROS_LOG_DIR=/tmp/ros_logs
```

Start the mock robot:

```bash
uv run --no-sync mock_robot
```

Start the navigator with one of the YAML targets:

```bash
uv run --no-sync navigator --ros-args -p config_file:=/home/wufy/git_resp/simple_navigator/config/target_diag.yaml
```

Start the goal editor:

```bash
uv run --no-sync target_goal_editor --ros-args -p odom_frame:=odom -p base_frame:=base_link
```

You can also publish targets from the command line:

```bash
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: odom}, pose: {position: {x: 0.8, y: -0.2, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.3826834, w: 0.9238795}}}"
```

Observe outputs:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /goal_reached
ros2 topic echo /odom
```

## Important Parameters

The main tuning file is any YAML under [config](/home/wufy/git_resp/simple_navigator/config).

### Target Pose

```yaml
target:
  x: 1.0
  y: 0.0
  yaw: 1.57
```

This sets the initial target loaded when `navigator` starts.

### Speed Limits

These are the most important parameters for motion speed and are read from YAML by `navigator`.

```yaml
controller:
  max_linear_velocity: 1.0
  max_angular_velocity: 2.0
```

Behavior:

- `max_linear_velocity` sets both `max_vx` and `max_vy`
- `max_angular_velocity` sets `max_vyaw`
- If you need different translational limits, use:

```yaml
controller:
  max_linear_velocity: 1.0
  max_vx: 1.4
  max_vy: 0.8
  max_angular_velocity: 2.0
```

Then:

- trajectory duration uses these limits
- controller saturation uses these limits
- final `cmd_vel` clamp uses these limits

### Tracking Gains

```yaml
controller:
  kp_x: 2.5
  kp_y: 2.5
  kp_yaw: 3.0
```

These determine how aggressively the robot tracks the reference trajectory.

### Trajectory Timing

```yaml
trajectory:
  duration_scale: 0.8
  min_duration: 0.2
```

These directly affect how fast the planned trajectory is.

- smaller `duration_scale` -> faster motion
- smaller `min_duration` -> allows short moves to complete faster

### Goal Tolerances

```yaml
navigation:
  position_tolerance: 0.05
  yaw_tolerance: 0.05
```

These control when the goal is considered reached.

### Frame Names

```yaml
frames:
  base_frame: "base_link"
  odom_frame: "odom"
```

These affect both TF lookup in the navigator and the editor launch parameters.

## Recommended Tuning Order

If the robot is too slow:

1. Increase `controller.max_linear_velocity`
2. Increase `controller.max_angular_velocity`
3. Decrease `trajectory.duration_scale`
4. Increase `controller.kp_x`, `controller.kp_y`, `controller.kp_yaw`

Example faster config:

```yaml
trajectory:
  duration_scale: 0.7
  min_duration: 0.2

controller:
  kp_x: 2.5
  kp_y: 2.5
  kp_yaw: 3.0
  max_linear_velocity: 1.0
  max_angular_velocity: 2.5
```

## Tests

Unit tests:

```bash
python3 -m unittest discover -s test -p 'test_navigator.py'
```

Integration tests:

```bash
ROS_LOG_DIR=/tmp/ros_logs python3 -m unittest discover -s test -p 'test_integration_navigator.py'
```
