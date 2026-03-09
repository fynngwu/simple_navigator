# Modern Waypoint Editor - 使用指南和截图说明

## 📸 界面预览

```
┌─────────────────────────────────────────────────────────────┐
│                  Modern Navigator Dashboard                  │
├───────────────────────────────┬─────────────────────────────┤
│                               │  Navigation Control         │
│       2D Map View             │  ┌─────────────────────┐   │
│   (600x600 pixels)            │  │ Waypoint List       │   │
│                               │  │ • WP_1: (1.0, 0.5)  │   │
│   ┌─────────────────────┐     │  │ • WP_2: (1.5, 1.0)  │   │
│   │  Grid (12x12m)      │     │  │ • WP_3: (0.8, 1.5)  │   │
│   │                     │     │  └─────────────────────┘   │
│   │    Y ↑              │     │                             │
│   │      │              │     │  [Clear Waypoints]         │
│   │      ├─── X         │     │                             │
│   │      │              │     │  [Publish Selected Target] │
│   │  ┌───┴───┐          │     │                             │
│   │  │  ▲   │  <- Robot  │     │  [START NAVIGATION]        │
│   │  │  ──  │  (green)   │     │     (Green Button)         │
│   │  └──────┘            │     │                             │
│   │                     │     │  [EMERGENCY STOP]          │
│   │  ● WP_1  ● WP_2     │     │     (Red Button)           │
│   │  (blue dots)         │     │                             │
│   └─────────────────────┘     │                             │
│                               │                             │
│  Robot Position: X: 1.2m     │                             │
│  Y: 0.8m | Yaw: 45.0°        │                             │
└───────────────────────────────┴─────────────────────────────┘
```

## 🎨 颜色方案

### 主题配色 (Catppuccin Mocha)

- **背景色**: `#1e1e2e` (深蓝灰)
- **文本色**: `#cdd6f4` (浅灰白)
- **地图背景**: `#181825` (深色)
- **边框色**: `#313244` (中灰)

### 功能元素

- **机器人**: `#a6e3a1` (绿色) - 三角形图标
- **航点**: `#89b4fa` (蓝色) - 圆点标记
- **启动按钮**: `#a6e3a1` (绿色)
- **停止按钮**: `#f38ba8` (红色)
- **标题**: `#89b4fa` (蓝色)

## 🖱️ 操作说明

### 添加航点
1. **点击地图** - 在地图任意位置点击左键
2. **自动编号** - 航点自动命名为 WP_1, WP_2, ...
3. **可视化** - 蓝色圆点显示在地图上
4. **列表更新** - 航点自动添加到右侧列表

### 选择和发布航点
1. **选择航点** - 在右侧列表中点击航点名称
2. **发布目标** - 点击 "Publish Selected Target" 按钮
3. **导航到点** - 航点位置发送到导航器

### 控制导航
- **启动导航** - 点击绿色 "START NAVIGATION" 按钮
- **紧急停止** - 点击红色 "EMERGENCY STOP" 按钮

### 清除航点
- 点击 "Clear Waypoints" 清除所有航点

## 📏 坐标系统

### ROS 坐标系
- **X 轴**: 向前为正
- **Y 轴**: 向左为正
- **Y 角**: 逆时针旋转为正 (弧度)

### 屏幕显示
- **地图尺寸**: 600x600 像素
- **比例尺**: 1 米 = 50 像素
- **覆盖范围**: 12x12 米
- **网格间隔**: 1 米

### 坐标转换
```python
# ROS 到屏幕
screen_x = ros_x * 50
screen_y = -ros_y * 50  # Y 轴翻转

# 屏幕到 ROS
ros_x = screen_x / 50
ros_y = -screen_y / 50
```

## 🔧 功能特性

### 实时更新
- **更新频率**: 20Hz (50ms 间隔)
- **TF 监听**: odom → base_link
- **位置显示**: 实时显示机器人位置和朝向

### 机器人显示
- **图标**: 绿色三角形
- **方向**: 三角形指向机器人朝向
- **大小**: 25x20 像素
- **中心**: 机器人实际位置

### 航点显示
- **图标**: 蓝色圆点
- **大小**: 10x10 像素
- **中心**: 航点实际位置

## 📊 状态指示

### 等待状态
```
Waiting for Odometry...
(橙色文本)
```

### 运行状态
```
Robot Position: X: 1.23m | Y: 0.56m | Yaw: 45.0°
(绿色文本)
```

## 🚀 启动方式

### 完整系统启动
```bash
# 启动所有节点
ros2 launch simple_navigator modern_editor.launch.py
```

### 分步启动
```bash
# 终端 1: 启动模拟机器人
ros2 run simple_navigator mock_robot

# 终端 2: 启动导航器
ros2 run simple_navigator navigator

# 终端 3: 启动现代化编辑器
ros2 run simple_navigator modern_waypoint_editor
```

## 💡 使用技巧

### 快速设置路径
1. 沿着 desired path 连续点击添加航点
2. 按顺序发布每个航点进行导航

### 精确定位
- 使用模拟机器人的实际位置作为参考
- 点击 "Use Current Position" 获取当前机器人位置

### 紧急情况
- 随时点击红色 "EMERGENCY STOP" 按钮立即停止导航

## 🎯 应用场景

1. **路径规划**: 可视化规划机器人路径
2. **实时监控**: 监控机器人位置和朝向
3. **远程控制**: 通过地图点击进行远程导航
4. **演示展示**: 现代化界面用于项目演示

## 📝 技术参数

- **GUI 框架**: PyQt5
- **绘图引擎**: QGraphicsScene/QGraphicsView
- **ROS 版本**: ROS2 Humble
- **Python 版本**: 3.10+
- **依赖包**: PyQt5, rclpy, tf2_ros

## 🔄 对比原始编辑器

| 特性 | 原始编辑器 | 现代化编辑器 |
|------|-----------|-------------|
| 界面风格 | 标准 Qt 风格 | 暗色现代主题 |
| 地图显示 | 无 | 2D 可视化 |
| 航点输入 | 手动输入坐标 | 点击地图添加 |
| 机器人显示 | 文本显示 | 实时图形显示 |
| 用户体验 | 传统 | 现代直观 |

## 🎓 学习资源

- **PyQt5 文档**: https://www.riverbankcomputing.com/static/Docs/PyQt5/
- **ROS2 教程**: https://docs.ros.org/en/humble/Tutorials.html
- **QGraphicsView**: https://doc.qt.io/qt-5/qgraphicsview.html
