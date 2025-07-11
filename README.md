# 🤖 TurtleBot3 Obstacle Avoidance Project

A simple obstacle avoidance system for TurtleBot3 in ROS2 Humble with custom Gazebo map integration.

## 📁 Project Structure

```
~/demo_robotics/
├── src/
│   └── simple_navigation_project/
│       ├── package.xml
│       ├── setup.py
│       ├── launch/
│       │   └── simple_navigation.launch.py
│       └── simple_navigation_project/
│           ├── __init__.py
│           └── obstacle_avoider.py
```

## 🚀 What This Does

- **🛡️ Smart Obstacle Detection**: Uses laser scan data to detect obstacles in front, left, and right
- **🧠 Intelligent Path Planning**: Measures lane widths before turning to avoid narrow passages
- **⚡ Smooth Movement**: Reduced speeds for stable navigation and collision avoidance
- **🔄 Adaptive Behavior**: Continuously searches for safe paths when blocked

## 🏗️ Custom Map Setup

This project uses a custom Gazebo world instead of the default TurtleBot3 world:

1. **🎨 Map Creation**: Built custom floor plan using Gazebo Building Editor
2. **🌍 World Integration**: Custom world file integrated into TurtleBot3 workspace
3. **🎯 Spawn Configuration**: TurtleBot3 spawns at custom position in your map

## 🎛️ Key Features

- **Safety First**: 50cm minimum distance from obstacles
- **Lane Width Check**: 80cm minimum lane width requirement
- **Real-time Processing**: 10Hz laser scan processing
- **Smooth Turning**: Optimized angular velocities for stable movement

## 🚦 How to Run

1. Launch your custom Gazebo world
2. Run the obstacle avoidance node
3. Watch TurtleBot3 navigate autonomously! 🎉

## 📡 Topics Used

- **Subscribe**: `/scan` (LaserScan data)
- **Publish**: `/cmd_vel` (Twist commands)

---

*Built with ROS2 Humble 🚀*
