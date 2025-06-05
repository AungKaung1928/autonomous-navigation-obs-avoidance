# Simple TurtleBot3 Obstacle Avoidance Navigator

A ROS2 Python package that implements autonomous obstacle avoidance for TurtleBot3 using LiDAR sensor data in Gazebo simulation.

## 🚀 Features

- **Real-time obstacle detection** using 360° LiDAR scanning
- **Intelligent navigation logic** with multi-directional obstacle avoidance
- **Python-based ROS2 implementation** for easy understanding and modification
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
   - **Front clear** → Move forward at 0.2 m/s
   - **Obstacle ahead + Right clear** → Turn right
   - **Obstacle ahead + Left clear** → Turn left  
   - **Both sides blocked** → Turn around
3. **Safety Features**: 50cm minimum distance threshold and error handling

## 🚀 Quick Start

### Prerequisites
```bash
# Install TurtleBot3 packages
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-gazebo-*
```

### Build & Run
```bash
# Clone and build
git clone <your-repo-url>
cd demo_robotics_ws
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

### Alternative: Complete Launch (Coming Soon)
```bash
# Single command launch (under development)
ros2 launch simple_navigation_project simple_navigation.launch.py
```

## 🎯 Key ROS2 Concepts Demonstrated

- **Node-based Architecture**: Single-node obstacle avoidance system
- **Publisher/Subscriber Pattern**: `/cmd_vel` publishing and `/scan` subscription
- **Timer-based Control**: 10Hz navigation loop for responsive control
- **Sensor Data Processing**: LiDAR range filtering and sectoral analysis
- **Python Package Structure**: Proper entry points and dependencies
- **Launch File Integration**: Automated system deployment

## 📊 System Architecture

```
[LiDAR /scan] → [Obstacle Avoider Node] → [/cmd_vel] → [TurtleBot3 Movement]
                        ↓
            [Gazebo Simulation Environment]
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

## ⚙️ Configuration Parameters

- **Minimum Distance**: 0.5m obstacle detection threshold
- **Forward Speed**: 0.2 m/s safe navigation speed
- **Angular Speed**: 0.5-1.0 rad/s turning rates
- **Scan Sectors**: Front (340°-20°), Left (60°-120°), Right (240°-300°)

## 🎓 Learning Outcomes

- ROS2 Python node development and lifecycle management
- LiDAR sensor data processing and interpretation
- Real-time robotics decision-making algorithms
- Gazebo simulation environment integration
- ROS2 launch system configuration
- Modern Python practices in robotics applications

## 🤖 Demo Behavior

The robot demonstrates autonomous navigation by:
- **Continuous forward movement** when path is clear
- **Right-preference turning** when obstacles detected ahead
- **Left turning** as fallback when right is blocked
- **180° rotation** when completely surrounded
- **Real-time logging** of navigation decisions

## 🚧 Known Issues & Future Improvements

- [ ] Complete launch file integration with Gazebo spawning
- [ ] Add dynamic reconfigure for parameters
- [ ] Implement wall-following behavior
- [ ] Add goal-oriented navigation
- [ ] Enhance multi-robot support

## 🔗 Dependencies

```xml
<!-- Key ROS2 dependencies -->
<depend>rclpy</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
<depend>gazebo_ros</depend>
<depend>turtlebot3_gazebo</depend>
```

---

*"A foundational project for learning ROS2 navigation concepts and Python-based robotics development with TurtleBot3 simulation."*
