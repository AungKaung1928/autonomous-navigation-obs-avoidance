🤖 Autonomous Navigation with Obstacle Avoidance
A ROS2-based autonomous navigation system for TurtleBot3 that enables continuous movement while avoiding obstacles in Gazebo simulation environments.
📋 Requirements
	• ROS Version: ROS2 Humble
	• OS: Ubuntu 22.04
	• Simulation: Gazebo Classic
	• Language: Python
	• Navigation: Navigation 2 (Nav2) stack
🚀 Features
	• ✅ Continuous autonomous movement
	• 🛡️ Dynamic obstacle avoidance
	• 🔍 Lane width analysis for safe navigation
	• 📊 Real-time laser scan processing
	• 🎯 Smart path selection algorithm
🛠️ Build Instructions
1. Prerequisites
# Install ROS2 and TurtleBot3 packages
sudo apt update
sudo apt install ros-humble-desktop-full
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
2. Environment Setup
# Set TurtleBot3 model
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
3. Build the Project
# Clone the repository
git clone https://github.com/AungKaung1928/autonomous-navigation-obs-avoidance.git
cd autonomous-navigation-obs-avoidance
# Build the workspace
colcon build --packages-select simple_navigation_project
source install/setup.bash
🎮 Usage Instructions
1. Launch Gazebo Classic World
# Terminal 1: Start TurtleBot3 in your custom world (This is my custom world and you can create your new custom world)
cd ~/turtlebot3_ws
colcon build
source install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_my_world.launch.py

2. Start Navigation Stack
# Terminal 2: Launch Navigation 2
ros2 launch simple_navigation_project nav2_simple.launch.py

🔧 Configuration
The system uses configurable parameters in config/nav2_params.yaml:
	• Minimum safe distance: 0.5m
	• Minimum lane width: 0.8m
	• Movement speed: 0.3 m/s
	• Turn speed: 0.3 rad/s
📊 How It Works
	1. Laser Scan Processing: Analyzes 360° laser data to detect obstacles
	2. Lane Width Analysis: Measures available space in left/right directions
	3. Smart Decision Making: Chooses the safest and widest path
	4. Continuous Movement: Maintains forward motion when path is clear
🎯 Algorithm Features
	• Front Detection: 40° scanning range for forward obstacles
	• Side Analysis: 60° scanning for left/right path evaluation
	• Safety Margins: Configurable minimum distances and lane widths
	• Error Handling: Robust error recovery and logging
📝 Project Structure
demo_robotics/
├── src/
│   └── simple_navigation_project/
│       ├── package.xml
│       ├── setup.py
│       ├── config/
│       │   └── nav2_params.yaml
│       ├── launch/
│       │   ├── simple_navigation.launch.py
│       │   └── nav2_simple.launch.py
│       └── simple_navigation_project/
│           ├── __init__.py
│           └── obstacle_avoider.py

🔍 Debugging & Monitoring
View Active Topics
# List all active ROS2 topics
ros2 topic list
# Monitor laser scan data
ros2 topic echo /scan
# Monitor velocity commands
ros2 topic echo /cmd_vel
# Check robot odometry
ros2 topic echo /odom
Useful Commands
# Check node information
ros2 node info /simple_obstacle_avoider
# View topic details
ros2 topic info /scan
ros2 topic info /cmd_vel
# Monitor topic frequency
ros2 topic hz /scan
ros2 topic hz /cmd_vel
🧪 Testing
The system has been tested in:
	• ✅ TurtleBot3 standard world (Gazebo Classic)
	• ✅ Various obstacle configurations
	• ✅ Narrow passage scenarios
	• ✅ Dynamic environment changes
![Uploading image.png…]()
