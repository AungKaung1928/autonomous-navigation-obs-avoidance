# 🤖 TurtleBot3 Wall Follower

Autonomous wall-following robot using ROS2 and laser-based navigation with precise distance control and obstacle avoidance.

## 📋 Project Overview

This project demonstrates autonomous wall-following capabilities using TurtleBot3 in a Gazebo simulation environment. The system showcases proficiency in:

- **Wall Following**: Maintains consistent distance from walls using PID control
- **Obstacle Avoidance**: Real-time front obstacle detection and emergency stop
- **Adaptive Navigation**: Seamless switching between left/right wall following
- **Sensor Processing**: Real-time laser scan data analysis for wall detection

### 🎯 Project Specifications

- **Primary Function**: Autonomous wall following with 0.6m target distance
- **Target Application**: Facility inspection, domestic robotics, educational demonstrations
- **Performance**: Stable navigation without oscillation or spinning

## ⚙️ Technical Features

### 🔧 Implementation Features

1. **Wall Detection Algorithm**
   - Multi-point laser scan analysis for wall identification
   - 270° laser range utilization for accurate distance measurement
   - Robust wall presence detection logic

2. **PID Control System**
   - Smooth angular velocity control for stable following
   - Tunable gains for different environments
   - Real-time error correction

3. **Safety Systems**
   - Front-facing obstacle detection
   - Emergency stop functionality
   - Configurable safety parameters

## 📋 System Requirements

- **ROS2 Humble** or higher
- **Ubuntu 22.04 LTS**
- **Gazebo Classic**
- **Python 3.8+**
- **TurtleBot3 packages**

## 🛠️ Installation

### Prerequisites
```bash
sudo apt update
sudo apt install ros-humble-desktop-full
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-gazebo-*
```

### Environment Setup
```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

## 📁 Project Structure

```
~/turtlebot3_wall_follower_ws/
├── src/
│   └── wall_following_project/
│       ├── package.xml 
│       ├── setup.py 
│       ├── config/                   
│       │   ├── nav2_params.yaml
│       │   └── wall_following_params.yaml       
│       ├── launch/
│       │   ├── wall_following.launch.py 
│       │   └── wall_follower_gazebo.launch.py
│       ├── rviz/
│       │   └── wall_follower_config.rviz
│       └── wall_following_project/
│           ├── __init__.py
│           ├── wall_follower_controller.py  # Main control logic
│           ├── wall_detector.py             # Wall detection algorithms
│           └── pid_controller.py            # PID control system
```

## 🚀 Build Instructions

```bash
# Create workspace
mkdir -p ~/turtlebot3_wall_follower_ws/src
cd ~/turtlebot3_wall_follower_ws/src

# Clone repository
git clone <repository-url> wall_following_project

# Build
cd ~/turtlebot3_wall_follower_ws
colcon build --packages-select wall_following_project
source install/setup.bash
```

## 🎮 Operation Instructions

### Running the System

**Terminal 1 - Launch Gazebo:**
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2 - Start Wall Follower:**
```bash
cd ~/turtlebot3_wall_follower_ws
source install/setup.bash
ros2 launch wall_following_project wall_following.launch.py
```

### Alternative: Complete System Launch
```bash
cd ~/turtlebot3_wall_follower_ws
source install/setup.bash
ros2 launch wall_following_project wall_follower_gazebo.launch.py
```

### Expected Behavior

The TurtleBot3 will:
1. **Initialize** wall detection and PID control systems
2. **Detect Wall** using laser scan analysis
3. **Follow Wall** maintaining 0.6m distance consistently
4. **Avoid Obstacles** with emergency stop when front blocked

## ✅ Verification

```bash
# Check active nodes
ros2 node list

# Monitor robot commands
ros2 topic echo /cmd_vel
ros2 topic echo /scan

# Check wall follower status
ros2 node info /wall_follower_controller
```

## 📊 Technical Specifications

### Configuration Parameters
- **Target Distance**: 0.6m from wall
- **Maximum Speed**: 0.25 m/s
- **Control Frequency**: 10Hz
- **Safety Range**: 0.5m front obstacle detection

### System Architecture
- **Robot Platform**: TurtleBot3 Burger
- **Sensor**: 360° LiDAR
- **Control Algorithm**: PID-based wall following
- **Safety System**: Front obstacle avoidance

## 🔧 Customization

Edit `config/wall_following_params.yaml`:
```yaml
target_distance: 0.6           # meters
max_linear_speed: 0.25         # m/s
max_angular_speed: 1.0         # rad/s
safety_distance: 0.5           # meters
control_frequency: 10          # Hz
```

## 🔍 Debugging

```bash
# System monitoring
ros2 topic hz /scan
ros2 topic hz /cmd_vel
ros2 node info /wall_follower_controller

# Visualization
rviz2 -d src/wall_following_project/rviz/wall_follower_config.rviz
```

## 🚨 Troubleshooting

### Common Issues

**Robot Not Following Wall:**
```bash
# Check laser scan data
ros2 topic echo /scan --once

# Verify wall detection
ros2 node info /wall_follower_controller
```

**Build Errors:**
```bash
# Clean and rebuild
rm -rf build/ install/ log/
colcon build --packages-select wall_following_project
source install/setup.bash
```

**Gazebo Launch Issues:**
```bash
# Check environment variables
echo $TURTLEBOT3_MODEL
source /opt/ros/humble/setup.bash
```
