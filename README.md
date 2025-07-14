🤖 TurtleBot3 Autonomous Navigation with Obstacle Avoidance
A robust autonomous navigation system for TurtleBot3 that enables continuous movement while dynamically avoiding obstacles in Gazebo simulation environments.
📋 System Requirements
	• ROS Version: ROS2 Humble or higher
	• Operating System: Ubuntu 22.04 LTS
	• Simulation Environment: Gazebo Classic
	• Programming Language: Python
	• Navigation Framework: Navigation 2 (Nav2) stack
🚀 Features
	• Continuous Movement: Robot maintains forward motion whenever possible
	• Dynamic Obstacle Avoidance: Real-time detection and avoidance of obstacles
	• Intelligent Path Planning: Analyzes lane width for optimal navigation decisions
	• Real-time Laser Processing: 360-degree laser scan analysis for comprehensive environment awareness
	• Configurable Parameters: Adjustable safety margins and movement speeds
🛠️ Installation and Setup
Prerequisites
Install ROS2 and required packages:
# Update package manager
sudo apt update
# Install ROS2 Humble desktop full
sudo apt install ros-humble-desktop-full
# Install TurtleBot3 packages
sudo apt install ros-humble-turtlebot3*
# Install Navigation 2 stack
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
🔧 Environment Configuration
Set up TurtleBot3 environment variables:
# Configure TurtleBot3 model
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
📦 Build Process
	1. Clone the Repository
git clone https://github.com/AungKaung1928/autonomous-navigation-obs-avoidance.git
cd autonomous-navigation-obs-avoidance
	2. Build the Workspace
colcon build --packages-select simple_navigation_project
source install/setup.bash
🎮 Usage Instructions
🌍 Launch Simulation Environment
Open a new terminal and start the TurtleBot3 simulation:
# Terminal 1: Launch Gazebo with TurtleBot3
cd ~/turtlebot3_ws
colcon build
source install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
🚀 Start Navigation System
Open another terminal and launch the autonomous navigation:
# Terminal 2: Start obstacle avoidance navigation
ros2 launch simple_navigation_project nav2_simple.launch.py
✅ Verification Commands
Monitor the system operation with these commands:
# Check active nodes
ros2 node list
# Monitor laser scan data
ros2 topic echo /scan
# Monitor velocity commands
ros2 topic echo /cmd_vel
# Check robot position
ros2 topic echo /odom
🏗️ System Architecture
🔧 Core Components
	1. Laser Scan Processor: Analyzes 360° laser data for obstacle detection
	2. Lane Width Analyzer: Evaluates available space in multiple directions
	3. Decision Engine: Selects optimal movement strategy based on environmental analysis
	4. Motion Controller: Executes smooth movement commands
🧠 Algorithm Overview
The navigation system employs a multi-stage decision process:
	• Forward Scanning: 40° range analysis for immediate obstacles
	• Lateral Assessment: 60° scanning for left/right path evaluation
	• Safety Validation: Ensures minimum clearance distances
	• Path Selection: Chooses the safest available direction
⚙️ Configuration
System parameters can be adjusted in config/nav2_params.yaml:
# Safety parameters
minimum_safe_distance: 0.5    # meters
minimum_lane_width: 0.8       # meters
# Movement parameters (linear_speed/angular_speed can be adjusted)
linear_speed: 0.3              # m/s
angular_speed: 0.3             # rad/s

📁 Project Structure
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

🔍 Debugging and Monitoring
🩺 System Diagnostics
# View all active topics
ros2 topic list
# Monitor specific topics
ros2 topic echo /scan
ros2 topic echo /cmd_vel
ros2 topic echo /odom
# Check topic frequencies
ros2 topic hz /scan
ros2 topic hz /cmd_vel
# Node information
ros2 node info /simple_obstacle_avoider
📊 Performance Monitoring
# Check computation graph
rqt_graph
# View log messages
ros2 log
🧪 Testing and Validation
The system has been validated in multiple scenarios:
	• Standard World Navigation: Continuous movement in TurtleBot3 default world
	• Obstacle-rich Environments: Navigation through complex obstacle configurations
	• Narrow Passages: Safe traversal through constrained spaces
	• Dynamic Scenarios: Adaptation to changing environmental conditions
🔧 Troubleshooting
⚠️ Common Issues
	1. Gazebo Launch Issues
		○ Ensure proper workspace sourcing
		○ Verify TurtleBot3 model environment variable
	2. Navigation Errors
		○ Check laser scan topic availability
		○ Verify Nav2 parameters configuration
	3. Build Failures
		○ Ensure all dependencies are installed
		○ Check Python package requirements
🚀 Performance Optimization
	• Adjust laser scan frequency for better responsiveness
	• Tune safety margins based on environment requirements
	• Optimize angular velocity for smoother turns
<img width="925" height="3387" alt="image" src="https://github.com/user-attachments/assets/49bc061a-a7d3-43fa-9cb9-ad69c65b383a" />
