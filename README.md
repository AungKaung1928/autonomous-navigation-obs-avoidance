# 🤖 TurtleBot3 Simple Obstacle Avoider

Autonomous navigation robot using ROS2 and laser-based obstacle detection with intelligent lane selection and path planning.

## 📋 Project Overview

This project demonstrates autonomous obstacle avoidance capabilities using TurtleBot3 in a Gazebo simulation environment. The system showcases proficiency in:

- **Obstacle Detection**: Real-time front, left, and right obstacle detection using laser scan
- **Lane Analysis**: Intelligent measurement of available lane widths for safe navigation
- **Adaptive Navigation**: Smart path selection based on lane width and safety criteria
- **Smooth Movement**: Reduced speed settings for stable and controlled navigation

### 🎯 Project Specifications

- **Primary Function**: Autonomous obstacle avoidance with intelligent lane selection
- **Target Application**: Autonomous delivery, indoor navigation, educational robotics
- **Performance**: Smooth navigation with minimal oscillation and smart path planning

## ⚙️ Technical Features

### 🔧 Implementation Features

1. **Multi-Directional Obstacle Detection**
   - 40° front scanning for forward obstacle detection
   - 60° left and right side scanning for lane monitoring
   - 90° wide-angle scanning for accurate lane width measurement

2. **Intelligent Lane Selection Algorithm**
   - Minimum lane width requirement (0.8m) for safe passage
   - Comparative analysis to choose the wider available lane
   - Safety-first approach with continuous path evaluation

3. **Adaptive Movement Control**
   - Reduced speed settings (0.3 m/s) for smoother operation
   - Dynamic speed adjustment based on obstacle proximity
   - Emergency stop functionality when no safe path is available

4. **Robust Sensor Processing**
   - Invalid reading filtering (inf, nan, out-of-range values)
   - Multi-point distance analysis for reliable detection
   - Exception handling for sensor data processing

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
│           └── obstacle_avoider.py    # Main obstacle avoidance logic
```

## 🚀 Build Instructions

```bash
# Create workspace (if not already created)
mkdir -p ~/demo_robotics/src
cd ~/demo_robotics/src

# Navigate to project directory
cd ~/demo_robotics

# Build the project
colcon build --packages-select simple_navigation_project
source install/setup.bash
```

## 🎮 Operation Instructions

### Running the System

**Terminal 1 - Launch Gazebo Environment:**
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

(or) You can create custom world and make turtlebot3 run in that custom world.

```bash
cd ~/turtlebot3_ws
colcon build
source install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_my_world.launch.py
```

**Terminal 2 - Launch Obstacle Avoider:**
```bash
cd ~/demo_robotics
colcon build --packages-select simple_navigation_project
source install/setup.bash
ros2 launch simple_navigation_project nav2_simple.launch.py
```

### Expected Behavior

The TurtleBot3 will:
1. **Initialize** obstacle detection and lane analysis systems
2. **Move Forward** when path is clear at 0.3 m/s
3. **Analyze Lanes** when obstacle detected in front
4. **Select Best Path** based on lane width and safety criteria
5. **Navigate Smoothly** through available passages

## ✅ Verification

```bash
# Check active nodes
ros2 node list

# Monitor robot movement commands
ros2 topic echo /cmd_vel

# Check laser scan data
ros2 topic echo /scan

# View obstacle avoider logs
ros2 node info /simple_obstacle_avoider
```

## 📊 Technical Specifications

### Navigation Parameters
- **Normal Speed**: 0.3 m/s (reduced for smooth operation)
- **Turn Speed**: 0.3 rad/s
- **Minimum Safe Distance**: 0.5m from obstacles
- **Minimum Lane Width**: 0.8m for safe passage
- **Control Frequency**: 10Hz (0.1s timer)

### Sensor Configuration
- **Front Detection**: 40° arc (340°-360° + 0°-20°)
- **Side Detection**: 60° arcs (Left: 60°-120°, Right: 240°-300°)
- **Lane Measurement**: 90° arcs (Left: 45°-135°, Right: 225°-315°)
- **Valid Range**: 0.1m - 10.0m

### Decision Logic
- **Path Selection**: Prioritizes wider lanes when both sides are safe
- **Safety Check**: Ensures minimum lane width before turning
- **Fallback Behavior**: Continues rotating when no safe path is found

## 🔧 Customization

Modify parameters in `obstacle_avoider.py`:
```python
# Distance thresholds
self.min_distance = 0.5      # Obstacle detection range (meters)
self.min_lane_width = 0.8    # Minimum safe lane width (meters)

# Speed settings
self.normal_speed = 0.3      # Forward movement speed (m/s)
self.turn_speed = 0.3        # Turning speed (rad/s)
```

## 🔍 Debugging and Monitoring

```bash
# Real-time laser data monitoring
ros2 topic echo /scan --once

# Check command velocity output
ros2 topic echo /cmd_vel

# Monitor node status
ros2 node info /simple_obstacle_avoider

# View system topics
ros2 topic list
```

## 🚨 Troubleshooting

### Common Issues

**Robot Not Moving:**
```bash
# Check if cmd_vel topic is being published
ros2 topic hz /cmd_vel

# Verify laser scan data is available
ros2 topic hz /scan

# Check node status
ros2 node list | grep obstacle
```

**Build Errors:**
```bash
# Clean workspace and rebuild
cd ~/demo_robotics
rm -rf build/ install/ log/
colcon build --packages-select simple_navigation_project
source install/setup.bash
```
