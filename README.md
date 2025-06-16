# A Simple TurtleBot3 Obstacle Avoidance Navigator Project

A ROS2 Python package that implements autonomous obstacle avoidance for TurtleBot3 using LiDAR sensor data in Gazebo classic simulation.

## 🚀 Features

- **Real-time obstacle detection** using 360° LiDAR scanning
- **Gazebo simulation integration** with TurtleBot3 models
- **Launch file automation** for streamlined deployment
- **RViz visualization** support for real-time monitoring

## 🛠️ Technologies Used

- **ROS2 Humble** (Python)
- **Gazebo Classic** simulation environment
- **TurtleBot3 Burger** robot platform
- **LiDAR sensor processing** with 360° coverage
- **Python 3** with modern ROS2 patterns

## 📁 Package Structure

```
simple_navigation_project/
├── simple_navigation_project/
│   ├── __init__.py
│   └── obstacle_avoider.py      # Main navigation node
├── launch/
│   └── simple_navigation.launch.py  # Complete system launcher
├── setup.py                     # Python package configuration
├── package.xml                  # ROS2 package metadata
└── resource/                    # Package resources
```

## 🏗️ How It Works

1. **LiDAR Processing**: Analyzes 360° laser scan data to detect obstacles in front, left, and right sectors
2. **Decision Logic**: 
   - **Front clear** → Move forward at 0.2 m/s(You can adjust the speed)
   - **Obstacle ahead + Right clear** → Turn right
   - **Obstacle ahead + Left clear** → Turn left  
   - **Both sides blocked** → Turn around
3. **Safety Features**: 50cm minimum distance threshold and error handling

### Build & Run
```bash
# Clone and build
git clone <your-repo-url>
cd demo_robotics
colcon build --packages-select simple_navigation_project
source install/setup.bash

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Terminal 1: Launch Gazebo world
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Run obstacle avoider
source install/setup.bash
ros2 run simple_navigation_project obstacle_avoider
```

### Alternative: Complete Launch 
```bash
# Single command launch
ros2 launch simple_navigation_project simple_navigation.launch.py
```

## 🔧 Debugging Commands

```bash
# Monitor laser scan data
ros2 topic echo /scan

# Monitor robot movement commands
ros2 topic echo /cmd_vel

# Check node status
ros2 node list
ros2 node info /simple_obstacle_avoider

# View topic connections
ros2 topic info /scan
ros2 topic info /cmd_vel
```
