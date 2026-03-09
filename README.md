# Simple Navigator - ROS2 导航控制器

一个简单易用的 ROS2 全向移动机器人导航控制器，提供现代化 GUI 界面和 2D 地图可视化。

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Python](https://img.shields.io/badge/Python-3.10+-green)
![License](https://img.shields.io/badge/License-MIT-yellow)

## ✨ 特性

- 🎯 **简单的导航控制** - 基于航点的点对点导航
- 🗺️ **2D 地图可视化** - 实时显示机器人位置和航点
- 🎨 **现代化 GUI** - 暗色主题，直观易用
- 🤖 **模拟机器人** - 内置模拟机器人用于测试
- ⚙️ **灵活配置** - YAML 配置文件管理航点
- 🖱️ **交互式操作** - 点击地图添加航点，无需手动输入坐标

## 📋 系统要求

- Ubuntu 22.04 或类似 Linux 系统
- ROS2 Humble
- Python 3.10+
- PyQt5

## 🚀 快速开始

### 安装依赖

```bash
# 安装 ROS2 Humble (如果尚未安装)
# 参考: http://docs.ros.org/en/humble/Installation.html

# 安装 Python 依赖
pip3 install PyQt5 transforms3d
```

### 构建项目

```bash
# 克隆或下载项目
cd /path/to/simple_navigator

# 构建 ROS2 包
colcon build --symlink-install

# 加载环境
source install/setup.bash
```

### 启动方式

#### 方式 1: 一键启动完整系统（推荐）

```bash
# 启动模拟机器人 + 导航器 + 现代化 GUI 界面
ros2 launch simple_navigator modern_editor.launch.py
```

#### 方式 2: 分步启动（调试用）

**终端 1 - 启动模拟机器人：**
```bash
ros2 run simple_navigator mock_robot
```

**终端 2 - 启动导航控制器：**
```bash
ros2 run simple_navigator navigator
```

**终端 3 - 启动现代化 GUI 界面：**
```bash
ros2 run simple_navigator modern_waypoint_editor
```

#### 方式 3: 使用原始编辑器

```bash
# 启动原始的 PyQt5 编辑器
ros2 run simple_navigator waypoint_editor
```

## 🎮 使用方法

### 现代化界面操作

1. **添加航点**
   - 在左侧 2D 地图上点击任意位置
   - 航点自动命名为 WP_1, WP_2, ...
   - 蓝色圆点标记航点位置

2. **导航到航点**
   - 在右侧列表中选择目标航点
   - 点击 "Publish Selected Target" 发布目标
   - 点击 "START NAVIGATION" 开始导航

3. **紧急停止**
   - 点击 "EMERGENCY STOP" 立即停止机器人

4. **清除航点**
   - 点击 "Clear Waypoints" 清除所有航点

### 配置文件编辑

航点配置文件位于 `config/waypoints.yaml`：

```yaml
# 航点配置 (角度使用度数)
waypoints:
  home:
    x: 0.0
    y: 0.0
    yaw: 0.0  # 度数

  point_a:
    x: 1.0
    y: 0.0
    yaw: 90.0  # 度数

# 导航参数
navigation:
  position_tolerance: 0.05  # 米
  yaw_tolerance: 0.05       # 弧度
  control_frequency: 20.0   # Hz

# 控制器参数
controller:
  kp_x: 1.0
  kp_y: 1.0
  kp_yaw: 1.0
  max_linear_velocity: 0.3   # m/s
  max_angular_velocity: 0.5  # rad/s
```

## 📂 项目结构

```
simple_navigator/
├── config/
│   └── waypoints.yaml          # 航点配置文件
├── launch/
│   └── modern_editor.launch.py # 启动文件
├── simple_navigator/
│   ├── navigator_node.py       # 导航控制器节点
│   ├── mock_robot.py           # 模拟机器人
│   ├── waypoint_editor.py      # 原始编辑器 GUI
│   ├── modern_waypoint_editor.py # 现代化编辑器 GUI
│   ├── waypoint_manager.py     # 航点管理器
│   ├── controller.py           # 控制器基类
│   └── simple_controller.py    # 简单 PID 控制器
├── test/                       # 测试文件
├── package.xml                 # ROS2 包配置
├── setup.py                    # Python 包配置
└── README.md                   # 本文件
```

## 🔧 配置说明

### 控制器参数

在 `config/waypoints.yaml` 中调整控制器参数：

- **kp_x, kp_y**: 位置控制比例增益
- **kp_yaw**: 角度控制比例增益
- **max_linear_velocity**: 最大线速度 (m/s)
- **max_angular_velocity**: 最大角速度 (rad/s)
- **position_tolerance**: 位置容差 (米)
- **yaw_tolerance**: 角度容差 (弧度)

### 坐标系统

- **X 轴**: 向前为正
- **Y 轴**: 向左为正
- **Yaw**: 逆时针旋转为正（配置文件中使用度数）

## 🎯 工作原理

1. **航点管理** - 从 YAML 文件或 GUI 界面加载航点
2. **位置跟踪** - 通过 TF 或 odometry 获取机器人位置
3. **控制器** - PID 控制器计算速度命令
4. **速度输出** - 发布 `cmd_vel` 控制机器人运动

### ROS2 话题

- **`/cmd_vel`** (`geometry_msgs/Twist`) - 速度命令输出
- **`/target_pose`** (`geometry_msgs/PoseStamped`) - 目标位姿输入
- **`/go`** (`std_msgs/Bool`) - 导航启动/停止信号
- **`/odom`** (`nav_msgs/Odometry`) - 里程计输入

### TF 坐标变换

- **`odom` → `base_link`** - 机器人在里程计坐标系中的位置

## 📊 界面截图

### 现代化编辑器界面

- 左侧：2D 地图视图，显示机器人位置和航点
- 右侧：导航控制面板
- 实时机器人位置显示
- 点击添加航点功能

详细说明请参考 `screenshots/README.md`

## 🧪 测试

```bash
# 运行测试
colcon test --packages-select simple_navigator

# 查看测试结果
colcon test-result --all
```

## 🐛 故障排除

### GUI 无法显示

确保有 X11 显示服务器或 VNC：
```bash
# 检查 DISPLAY 环境变量
echo $DISPLAY

# 如果为空，设置显示服务器
export DISPLAY=:0
```

### ROS2 节点无法通信

确保 ROS2 环境 已加载：
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 依赖缺失

```bash
pip3 install PyQt5 transforms3d
```

## 📝 开发

### 添加新功能

1. 修改相应的源代码文件
2. 重新构建：`colcon build --symlink-install`
3. 测试新功能

### 代码规范

- 遵循 PEP 8 Python 代码规范
- 使用类型提示
- 添加文档字符串

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

## 📄 许可证

MIT License

## 👥 作者

Simple Navigator 开发团队

## 🙏 致谢

- ROS2 社区
- PyQt5 开发团队
- transforms3d 库

## 📞 联系方式

如有问题或建议，请提交 Issue 或 Pull Request。

---

**祝你使用愉快！🎉**
