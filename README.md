# 🤖 TurtleBot3 Autonomous Navigation with Obstacle Avoidance
 
Autonomous navigation system for TurtleBot3 with continuous movement and dynamic obstacle avoidance in Gazebo simulation.
 
## 📋 System Requirements
 
- **ROS2 Humble** or higher
- **Ubuntu 22.04 LTS**
- **Gazebo Classic**
- **Python**
- **Navigation 2 (Nav2) stack**
 
## 🚀 Features
 
- ✅ Continuous autonomous movement
- 🛡️ Dynamic obstacle avoidance
- 🔍 Real-time laser scan processing
- 🧠 Smart path selection algorithm
 
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
 
### Build
```bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone https://github.com/AungKaung1928/autonomous-navigation-obs-avoidance.git
cd ~/turtlebot3_ws
colcon build --packages-select simple_navigation_project
source install/setup.bash
```
 
## 🎮 Usage
 
### 1. Launch Gazebo
```bash
# Terminal 1 (This is my custom world, you can change your custom world here)
cd ~/turtlebot3_ws
colcon build
source install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_my_world.launch.py
```
 
### 2. Start Navigation
```bash
# Terminal 2
ros2 launch simple_navigation_project nav2_simple.launch.py
```
 
## ✅ Verification
 
```bash
# Check nodes
ros2 node list
 
# Monitor topics
ros2 topic echo /scan
ros2 topic echo /cmd_vel
ros2 topic echo /odom
```
 
## ⚙️ Configuration
 
Edit `config/nav2_params.yaml`:
```yaml
minimum_safe_distance: 0.5    # meters
minimum_lane_width: 0.8       # meters
linear_speed: 0.3              # m/s
angular_speed: 0.3             # rad/s
```
 
## 📁 Project Structure
 
```
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
│           ├── **init**.py
│           └── obstacle_avoider.py
```
 
## 🔍 Debugging
 
```bash
# Monitor system
ros2 topic list
ros2 topic hz /scan
ros2 node info /simple_obstacle_avoider
rqt_graph
```
 
## 🧪 Testing
 
Validated in:
- ✅ TurtleBot3 standard world
- ✅ Obstacle-rich environments
- ✅ Narrow passages
- ✅ Dynamic scenarios
 
## ⚠️ Troubleshooting
 
- **Gazebo issues**: Check workspace sourcing and TURTLEBOT3_MODEL
- **Navigation errors**: Verify laser scan topics and Nav2 config
- **Build failures**: Install all dependencies
 
---
