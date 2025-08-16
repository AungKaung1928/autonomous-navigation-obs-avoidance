# Simple Navigation Project 🤖
A ROS2-based autonomous navigation system using Nav2 concepts for TurtleBot3 to continuously explore and avoid obstacles in Gazebo environments.

## Features ✨
- **Intelligent Obstacle Avoidance**: Real-time laser scan processing for dynamic obstacle detection
- **Nav2 Integration**: Implements Navigation2 patterns for goal-based navigation
- **Lane Width Analysis**: Evaluates available space before making turn decisions
- **Recovery Behaviors**: Automatic rotation when no clear path is available
- **Continuous Operation**: Endless exploration without human intervention

## Project Structure 📁
```
demo_robotics/
└── src/
    └── simple_navigation_project/
        ├── package.xml
        ├── setup.py
        ├── config/
        │   └── nav2_params.yaml
        ├── launch/
        │   └── nav2_simple.launch.py
        └── simple_navigation_project/
            ├── __init__.py
            └── obstacle_avoider.py
```

## How It Works 🧠
The navigation system implements a multi-layered decision process:

1. **Sensor Processing**: Analyzes 360° laser scan data in three sectors
   - Front sector (40°): Primary obstacle detection
   - Left/Right sectors (60° each): Alternative path evaluation
   - Wide scanning (90° each): Lane width measurement

2. **Decision Algorithm**:
   - Clear path → Move forward at configured speed
   - Obstacle detected → Evaluate left and right lanes
   - Choose wider lane when both available
   - No safe path → Execute recovery rotation

3. **Nav2 Goal Publishing**: Publishes navigation goals every 5 seconds for visualization

## Key Parameters 🔧

| Parameter | Default | Purpose | Tuning Guide |
|-----------|---------|---------|--------------|
| `min_distance` | 0.5m | Obstacle detection threshold | Decrease for tighter navigation |
| `normal_speed` | 0.2 m/s | Forward velocity | Increase for faster exploration |
| `turn_speed` | 0.5 rad/s | Rotation velocity | Adjust for smoother/sharper turns |
| `min_lane_width` | 0.8m | Minimum passage width | Lower for narrow spaces |

### Performance Tuning Examples:
```python
# Aggressive exploration mode
self.min_distance = 0.3
self.normal_speed = 0.4
self.turn_speed = 0.8

# Cautious navigation mode
self.min_distance = 0.8
self.normal_speed = 0.15
self.turn_speed = 0.3
```

## Getting Started 🚀

### Prerequisites
```bash
# Install required packages
sudo apt install ros-${ROS_DISTRO}-turtlebot3-gazebo
sudo apt install ros-${ROS_DISTRO}-nav2-msgs
```

### Build Instructions
```bash
cd ~/demo_robotics
colcon build --packages-select simple_navigation_project
source install/setup.bash
```

### Launch Commands

**Terminal 1** - Start Gazebo simulation:
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2** - Launch navigation system:
```bash
cd ~/demo_robotics
source install/setup.bash
ros2 launch simple_navigation_project nav2_simple.launch.py
```

## Navigation Behaviors 🎯

- **Forward Movement**: Maintains steady velocity when path is clear
- **Obstacle Response**: Immediate stop and evaluation when obstacles detected
- **Turn Selection**: Prioritizes wider lanes for safer navigation
- **Recovery Mode**: 360° rotation scan when trapped
- **Goal Updates**: Regular navigation goal publishing for path planning

## System Architecture 🏗️

### Core Components:
- **Obstacle Avoider Node**: Main decision-making and control logic
- **Laser Processor**: Real-time sensor data analysis
- **Velocity Publisher**: Smooth command generation at 10Hz
- **Goal Publisher**: Nav2-compatible goal generation

### Data Flow:
```
LaserScan (/scan) → Processing → Decision → Twist (/cmd_vel)
                                    ↓
                              Goal (/goal_pose)
```

## Safety Features 🔒
- Configurable safety distances for obstacle detection
- Lane width validation before committing to turns
- Graceful error handling for sensor failures
- Emergency stop on exception conditions

## Customization Options ⚙️

### Modify Detection Ranges:
```python
# Adjust scanning angles (in obstacle_avoider.py)
front_ranges = ranges[340:360] + ranges[0:20]  # Wider front view
left_ranges = ranges[45:135]                    # Broader left scan
right_ranges = ranges[225:315]                  # Broader right scan
```

### Change Update Frequencies:
```python
# In __init__ method
self.timer = self.create_timer(0.05, self.move_robot)  # 20Hz updates
self.goal_timer = self.create_timer(3.0, self.publish_nav_goal)  # Faster goals
```

## Dependencies 📦
- **ROS2**: Foxy or Humble
- **Python 3**: 3.8+
- **Core Packages**: rclpy, geometry_msgs, sensor_msgs, nav2_msgs
- **TurtleBot3**: Gazebo simulation packages

## Performance Metrics 📊
- Update Rate: 10Hz navigation decisions
- Goal Publishing: Every 5 seconds
- Reaction Time: <100ms obstacle response
- Turn Precision: ±5° accuracy

---
*Built with ROS2 and Navigation2 for reliable autonomous exploration*
