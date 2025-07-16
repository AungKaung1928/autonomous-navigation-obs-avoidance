# 🤖 TurtleBot3 Autonomous Navigation with Obstacle Avoidance

Autonomous navigation system for TurtleBot3 with continuous movement and dynamic obstacle avoidance in Gazebo simulation.

## 📋 Project Overview

This project demonstrates advanced autonomous navigation capabilities using TurtleBot3 in a Gazebo simulation environment. The system showcases proficiency in:

- **Autonomous Navigation**: Continuous movement without human intervention
- **Obstacle Avoidance**: Real-time detection and avoidance of static and dynamic obstacles
- **Navigation 2 Integration**: Professional use of ROS2 Nav2 stack
- **Sensor Processing**: Real-time laser scan data processing and analysis

### 🎯 Project Specifications

- **Primary Function**: Autonomous navigation with obstacle avoidance in standard TurtleBot3 world
- **Target Application**: Mobile robotics, autonomous vehicles, service robots
- **Performance**: Continuous operation with real-time obstacle detection

## ⚙️ Technical Features

### 🔧 Implementation Features

1. **Continuous Movement Algorithm**
   - Smart path selection based on laser scan data
   - Dynamic speed adjustment for safety
   - Adaptive turning behavior in narrow spaces

2. **Obstacle Detection System**
   - Real-time laser scan processing
   - Configurable safe distance parameters
   - Lane width detection for navigation decisions

3. **Navigation Integration**
   - Navigation 2 stack integration
   - Custom parameter configuration
   - Robust error handling and recovery

## 📋 System Requirements

- **ROS2 Humble** or higher
- **Ubuntu 22.04 LTS**
- **Gazebo Classic**
- **Python**
- **Navigation 2 (Nav2) stack**

## 🛠️ Installation

### Prerequisites
```bash
sudo apt update
sudo apt install ros-humble-desktop-full
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

### Environment Setup
```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

## 📁 Project Structure

```
~/demo_robotics/
├── src/
│   └── simple_navigation_project/
│       ├── package.xml 
│       ├── setup.py 
│       ├── config/                   
│       │   └── nav2_params.yaml       
│       ├── launch/
│       │   └── nav2_simple.launch.py 
│       └── simple_navigation_project/
│           ├── __init__.py
│           └── obstacle_avoider.py
```

## 🚀 Build Instructions

```bash
# Create workspace
mkdir -p ~/demo_robotics/src
cd ~/demo_robotics/src

# Clone repository
git clone https://github.com/AungKaung1928/autonomous-navigation-obs-avoidance.git

# Build
cd ~/demo_robotics
colcon build --packages-select simple_navigation_project
source install/setup.bash
```

## 🎮 Operation Instructions

### Running the System

**Terminal 1 - Launch Gazebo:**(This is custom ws and world)
```bash
# Custom world workspace
cd ~/turtlebot3_ws
colcon build
source install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_my_world.launch.py
```

**Terminal 2 - Start Navigation:**
```bash
# Navigation project workspace
cd ~/demo_robotics
colcon build --packages-select simple_navigation_project
source install/setup.bash
ros2 launch simple_navigation_project nav2_simple.launch.py
```

### Expected Behavior

The TurtleBot3 will:
1. **Initialize** navigation system and sensor processing
2. **Scan Environment** using laser sensors
3. **Navigate Continuously** while avoiding obstacles
4. **Adapt Path** dynamically based on sensor feedback

## ✅ Verification

```bash
# Check active nodes
ros2 node list

# Monitor sensor data
ros2 topic echo /scan
ros2 topic echo /cmd_vel
ros2 topic echo /odom

# Verify navigation status
ros2 topic list | grep nav
```

## 📊 Technical Specifications

### Configuration Parameters
- **Minimum Safe Distance**: 0.5m
- **Minimum Lane Width**: 0.8m
- **Linear Speed**: 0.3 m/s
- **Angular Speed**: 0.3 rad/s

### System Architecture
- **Robot Platform**: TurtleBot3 Burger
- **Sensor**: 360° LiDAR
- **Navigation Stack**: Nav2
- **Control Algorithm**: Custom obstacle avoidance

## 🔧 Customization

Edit `config/nav2_params.yaml`:
```yaml
minimum_safe_distance: 0.5    # meters
minimum_lane_width: 0.8       # meters
linear_speed: 0.3              # m/s
angular_speed: 0.3             # rad/s
```

## 🔍 Debugging

```bash
# System monitoring
ros2 topic list
ros2 topic hz /scan
ros2 node info /simple_obstacle_avoider

# Visualization
rqt_graph
```

## 🚨 Troubleshooting

### Common Issues

**Gazebo Launch Issues:**
```bash
# Check environment variables
echo $TURTLEBOT3_MODEL
source /opt/ros/humble/setup.bash
```

**Navigation Errors:**
```bash
# Verify laser scan topics
ros2 topic echo /scan

# Check Nav2 services
ros2 service list | grep nav
```
