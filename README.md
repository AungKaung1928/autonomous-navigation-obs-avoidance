# 🤖 TurtleBot3 Simple Navigation Project

Autonomous navigation robot using ROS2 Navigation 2 stack with laser-based obstacle detection and intelligent waypoint navigation.

## 🌟 Project Overview

This project demonstrates autonomous navigation capabilities using TurtleBot3 in a Gazebo simulation environment with ROS2 Navigation 2 (Nav2). The system showcases proficiency in:

- **Nav2 Integration**: Leverages the powerful Navigation 2 stack for robust path planning
- **Obstacle Detection**: Real-time front area obstacle detection using laser scan
- **Waypoint Navigation**: Intelligent navigation through predefined waypoints
- **Automatic Replanning**: Nav2 handles dynamic obstacle avoidance and path replanning
- **Continuous Exploration**: Robot keeps moving and exploring after completing waypoints

## 🎯 Project Specifications

- **Primary Function**: Autonomous navigation using Nav2 with obstacle avoidance
- **Target Application**: Autonomous delivery, indoor navigation, educational robotics
- **Performance**: Robust navigation with Nav2's advanced path planning algorithms

## 🚀 Technical Features

### ⚡ Implementation Features

1. **Nav2 Integration**
   - Uses `nav2_simple_commander` for easy navigation control
   - Leverages Nav2's built-in obstacle avoidance capabilities
   - Automatic path replanning when obstacles are detected
   - Professional-grade navigation stack

2. **Smart Obstacle Detection**
   - 60° front scanning for forward obstacle detection
   - Real-time laser scan monitoring
   - Maintains 0.6m minimum safe distance from obstacles
   - Integrates seamlessly with Nav2's planning algorithms

3. **Intelligent Waypoint System**
   - Predefined waypoint sequence for structured navigation
   - Random waypoint generation for continuous exploration
   - Goal completion tracking and automatic progression
   - Robust error handling and recovery

4. **Robust Navigation Control**
   - Nav2 handles complex path planning and obstacle avoidance
   - Automatic replanning when blocked paths are detected
   - Continuous movement with fallback to random exploration
   - Professional navigation behavior

## 💻 System Requirements

- **ROS2 Foxy** or higher (Humble recommended)
- **Ubuntu 20.04 LTS** (Foxy) or **Ubuntu 22.04 LTS** (Humble)
- **Gazebo Classic**
- **Python 3.8+**
- **TurtleBot3 packages**
- **Navigation 2 stack**

## 🛠️ Installation

### Prerequisites
```bash
# Install ROS2 and dependencies
sudo apt update
sudo apt install ros-$ROS_DISTRO-desktop-full
sudo apt install ros-$ROS_DISTRO-turtlebot3*
sudo apt install ros-$ROS_DISTRO-nav2-bringup
sudo apt install ros-$ROS_DISTRO-nav2-simple-commander

# Install tf_transformations
pip3 install transforms3d
```

### Environment Setup
```bash
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models" >> ~/.bashrc
source ~/.bashrc
```

## 📁 Project Structure

```
~/demo_robotics/
├── src/
│   └── simple_navigation_project/
│       ├── package.xml 
│       ├── setup.py      
│       ├── launch/
│       │   └── nav2_simple.launch.py  # Main launch file
│       └── simple_navigation_project/
│           ├── __init__.py
│           └── obstacle_avoider.py    # Main navigation logic with Nav2
```

## 🔥 Build Instructions

```bash
# Create workspace (if not already created)
mkdir -p ~/demo_robotics/src
cd ~/demo_robotics/src

# Copy project files to simple_navigation_project/
# Make sure __init__.py exists in simple_navigation_project/simple_navigation_project/
touch simple_navigation_project/simple_navigation_project/__init__.py

# Navigate to workspace and build
cd ~/demo_robotics
colcon build --packages-select simple_navigation_project
source install/setup.bash
```

## 🎮 Operation Instructions

### Method 1: Using Launch File (Recommended)

**Single Terminal - Launch Everything:**
```bash
cd ~/demo_robotics
source install/setup.bash
ros2 launch simple_navigation_project nav2_simple.launch.py
```

### Method 2: Manual Launch (Step by Step)

**Terminal 1 - Launch Gazebo Environment:**
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2 - Launch Nav2:**
```bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

**Terminal 3 - Launch Navigation Node:**
```bash
cd ~/demo_robotics
source install/setup.bash
ros2 run simple_navigation_project obstacle_avoider
```

### Expected Behavior

The TurtleBot3 will:
1. **Initialize** Nav2 and wait for activation
2. **Set Initial Pose** and start navigation system
3. **Navigate Waypoints** through predefined sequence
4. **Detect Obstacles** using laser scan in real-time
5. **Auto-Replan Paths** when Nav2 detects obstacles
6. **Explore Randomly** after completing all waypoints
7. **Continue Moving** indefinitely with intelligent navigation

## 💎 Verification

```bash
# Check active nodes (should see Nav2 and our node)
ros2 node list

# Monitor navigation goals
ros2 topic echo /goal_pose

# Check laser scan data
ros2 topic echo /scan

# View navigation logs
ros2 node info /simple_obstacle_avoider

# Check Nav2 status
ros2 topic list | grep nav2
```

## 📡 Technical Specifications

### Navigation Parameters
- **Minimum Safe Distance**: 0.6m from obstacles
- **Front Detection Area**: 60° arc (330°-30°)
- **Waypoint System**: 5 predefined + infinite random waypoints
- **Control Frequency**: 10Hz (0.1s spin rate)

### Nav2 Configuration
- **Path Planning**: Uses Nav2's built-in planners
- **Obstacle Avoidance**: Nav2's dynamic obstacle avoidance
- **Recovery Behaviors**: Automatic recovery when stuck
- **Replanning**: Real-time path replanning on obstacle detection

### Waypoint Sequence
1. **(2.0, 0.0, 0°)** - Move right
2. **(2.0, 2.0, 90°)** - Move up
3. **(0.0, 2.0, 180°)** - Move to center-top
4. **(-2.0, 0.0, -90°)** - Move left
5. **(0.0, 0.0, 0°)** - Return to origin
6. **Random exploration** continues...

## 🎨 Customization

Modify parameters in `obstacle_avoider.py`:
```python
# Obstacle detection
self.min_distance = 0.6      # Safe distance from obstacles (meters)

# Waypoint bounds for random generation
x = random.uniform(-3.0, 3.0)  # X-axis exploration range
y = random.uniform(-3.0, 3.0)  # Y-axis exploration range

# Front detection area
front_ranges = ranges[330:360] + ranges[0:30]  # 60° front area
```

## 🔍 Debugging and Monitoring

```bash
# Monitor Nav2 status
ros2 topic echo /behavior_tree_log

# Check navigation feedback
ros2 topic echo /navigate_to_pose/_action/feedback

# Real-time laser data monitoring
ros2 topic echo /scan --once

# Check map and localization
ros2 topic echo /map
ros2 topic echo /amcl_pose

# View all Nav2 topics
ros2 topic list | grep -E "(nav2|navigate|goal|path)"
```

## 🔧 Troubleshooting

### Common Issues

**Nav2 Not Ready:**
```bash
# Check if all Nav2 nodes are running
ros2 node list | grep nav2

# Wait longer for Nav2 initialization
# The node will log "Waiting for Nav2 to activate..."
```

**Robot Not Moving:**
```bash
# Check if navigation goals are being sent
ros2 topic hz /goal_pose

# Verify Nav2 is publishing cmd_vel
ros2 topic hz /cmd_vel

# Check node status
ros2 node list | grep obstacle
```

**TurtleBot3 Model Issues:**
```bash
# Ensure correct model is set
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models
```

**Build Errors:**
```bash
# Install missing dependencies
sudo apt install python3-transforms3d-pip
pip3 install transforms3d

# Clean workspace and rebuild
cd ~/demo_robotics
rm -rf build/ install/ log/
colcon build --packages-select simple_navigation_project
source install/setup.bash
```

**Launch File Not Found:**
```bash
# Make sure package is built and sourced
cd ~/demo_robotics
colcon build --packages-select simple_navigation_project
source install/setup.bash
```
