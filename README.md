# 🤖 TurtleBot3 Simple Navigation Project

Autonomous navigation robot using ROS2 Navigation 2 stack with waypoint-based obstacle avoidance navigation.

## 🌟 Project Overview

This project demonstrates autonomous navigation using TurtleBot3 in Gazebo with ROS2 Navigation 2 (Nav2). The robot follows predefined waypoints while automatically avoiding obstacles using Nav2's built-in planning algorithms.

### 🎯 Key Features
- **Nav2 Integration**: Uses `nav2_simple_commander` for professional navigation
- **Waypoint Navigation**: Follows predefined waypoints with automatic path planning
- **Obstacle Avoidance**: Nav2 handles collision avoidance automatically
- **Continuous Movement**: Robot moves without colliding with obstacles

### 💻 System Requirements
- **ROS2 Humble** or higher
- **Ubuntu 22.04 LTS**
- **Gazebo Classic**
- **Python 3.8+**

## 🛠️ Installation & Setup

### Prerequisites
```bash
# Install dependencies
sudo apt update
sudo apt install ros-humble-nav2-bringup ros-humble-nav2-simple-commander
sudo apt install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-cartographer
sudo apt install python3-tf-transformations

# Set TurtleBot3 model
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
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
│       │   └── nav2_params.yaml       # Navigation 2 parameters
│       ├── launch/
│       │   └── nav2_simple.launch.py  # Main launch file
│       └── simple_navigation_project/
│           ├── __init__.py
│           └── waypoint_navigator.py  # Nav2 waypoint navigation
```

## 🔥 Build Instructions

### Step 1: Create Project Structure
```bash
# Create workspace
mkdir -p ~/demo_robotics/src
cd ~/demo_robotics/src

# Create ROS2 package
ros2 pkg create --build-type ament_python simple_navigation_project

# Create directories and files
cd simple_navigation_project
mkdir config launch
touch config/nav2_params.yaml
touch launch/nav2_simple.launch.py
touch simple_navigation_project/waypoint_navigator.py
```

### Step 2: Build Project
```bash
# Navigate to workspace
cd ~/demo_robotics

# Build package
colcon build --packages-select simple_navigation_project

# Source workspace
source install/setup.bash
```

## 🎮 Operation Instructions

### Method 1: All-in-One Launch (Recommended)
```bash
cd ~/demo_robotics
source install/setup.bash
ros2 launch simple_navigation_project nav2_simple.launch.py
```

### Method 2: Step-by-Step Launch
```bash
# Terminal 1: Launch Gazebo + TurtleBot3
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Launch Navigation 2
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=/opt/ros/humble/share/turtlebot3_navigation2/map/map.yaml

# Terminal 3: Launch waypoint navigator
cd ~/demo_robotics
source install/setup.bash
ros2 run simple_navigation_project waypoint_navigator

# Terminal 4: RViz visualization (optional)
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

## 📡 Navigation 2 Concepts

### 🧠 Core Nav2 Features Demonstrated
1. **BasicNavigator**: Simple interface for Nav2 commands
2. **Waypoint Following**: Robot navigates through predefined points
3. **Automatic Obstacle Avoidance**: Nav2 handles collision avoidance
4. **Path Planning**: Global and local path planning algorithms
5. **Localization**: AMCL for robot pose estimation
6. **Recovery Behaviors**: Automatic recovery when blocked

### 🎯 How It Works
1. **Initialize**: Set robot's initial pose and wait for Nav2
2. **Create Waypoints**: Define target poses with coordinates
3. **Navigate**: Nav2 plans paths and avoids obstacles automatically
4. **Monitor**: Track progress and handle completion

### 📍 Waypoint Sequence
The robot follows these waypoints:
- **(2.0, 0.0, 0°)** → Move forward
- **(2.0, 1.0, 90°)** → Turn left  
- **(0.0, 1.0, 180°)** → Move back
- **(-0.0, -1.0, -90°)** → Turn right
- **(1.0, -1.0, 0°)** → Complete loop

## 💎 Verification & Testing

```bash
# Check active nodes
ros2 node list

# Monitor navigation goals
ros2 topic echo /navigate_to_pose/_action/goal

# Check waypoint progress
ros2 topic echo /navigate_to_pose/_action/feedback

# View Nav2 status
ros2 service call /is_navigator_ready std_srvs/srv/Empty
```

## 🔧 Troubleshooting

**Nav2 Not Ready:**
- Wait for "Nav2 is ready!" message in terminal
- Check all Nav2 nodes are running: `ros2 node list | grep nav2`

**Robot Not Moving:**
- Verify TurtleBot3 model: `echo $TURTLEBOT3_MODEL`
- Check navigation feedback: `ros2 topic hz /navigate_to_pose/_action/feedback`

**Build Errors:**
```bash
# Clean and rebuild
cd ~/demo_robotics
rm -rf build/ install/ log/
colcon build --packages-select simple_navigation_project
source install/setup.bash
```
