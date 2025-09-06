# Simple Navigation Project

A simple ROS 2 robotics package providing autonomous navigation capabilities with obstacle avoidance and goal-based navigation using Nav2.

## 📦 Package Contents

### Folder Structure

```
simple_navigation_project/
├── config/
│   └── nav2_params.yaml           # Nav2 stack configuration
├── launch/
│   ├── nav2_bringup.launch.py     # Full navigation stack launcher
│   └── nav2_simple.launch.py      # Simple obstacle avoidance launcher
├── resource/                      # ROS 2 package resources
├── simple_navigation_project/
│   ├── __init__.py               # Python package initialization
│   ├── obstacle_avoider.py       # Reactive obstacle avoidance node
│   └── simple_navigator.py       # Random goal generation node
├── test/                         # Unit tests directory
│   ├── test_copyright.py         # Copyright compliance tests
│   ├── test_flake8.py            # Python code style tests
│   └── test_pep257.py            # Python docstring style tests
├── CMakeLists.txt               # CMake build configuration
├── package.xml                  # ROS 2 package manifest
├── setup.cfg                    # Python setup configuration
├── setup.py                     # Python package setup
└── README.md                    # This documentation
```

### Core Nodes

- **`obstacle_avoider.py`** - Reactive obstacle avoidance using laser scan data
- **`simple_navigator.py`** - Random goal generation for autonomous exploration

### Configuration Files

- **`nav2_params.yaml`** - Complete Nav2 stack configuration
- **`nav2_bringup.launch.py`** - Full navigation stack launcher
- **`nav2_simple.launch.py`** - Simple obstacle avoidance launcher

## 🚀 Features

### Obstacle Avoidance
- **Reactive Navigation**: Real-time obstacle detection and avoidance
- **Laser Scan Processing**: 360-degree or limited range laser support
- **Configurable Parameters**: Adjustable speed, turn rate, and safety distances
- **Smart Turning Logic**: Intelligent direction selection based on clearance
- **Graceful Shutdown**: Safe robot stopping on exit

### Navigation System
- **Random Goal Generation**: Autonomous exploration with random waypoints
- **Nav2 Integration**: Full navigation stack with path planning
- **Costmap Management**: Local and global costmaps for obstacle representation
- **Multiple Planners**: NavFn planner with A* support

## 🛠️ Installation

```bash
# Navigate to your ROS 2 workspace
cd ~/ros2_ws/src

# Clone or copy the package
git clone <repository_url> simple_navigation_project

# Build the package
cd ~/ros2_ws
colcon build --packages-select simple_navigation_project

# Source the workspace
source install/setup.bash
```

## 🏃 Usage

### Simple Obstacle Avoidance

Launch the basic obstacle avoidance behavior:

```bash
ros2 launch simple_navigation_project nav2_simple.launch.py
```

**Parameters:**
- `normal_speed` (0.1-0.5): Forward movement speed in m/s (default: 0.2)
- `turn_speed` (0.3-1.5): Angular turning speed in rad/s (default: 0.5)
- `min_distance` (0.3-1.0): Minimum obstacle distance in meters (default: 0.5)

**Custom parameters example:**
```bash
ros2 launch simple_navigation_project nav2_simple.launch.py normal_speed:=0.3 turn_speed:=0.8 min_distance:=0.6
```

### Full Navigation Stack

Launch the complete Nav2 navigation system:

```bash
ros2 launch simple_navigation_project nav2_bringup.launch.py
```

**Parameters:**
- `use_sim_time` (default: true): Use simulation time
- `autostart` (default: true): Auto-start navigation nodes

### Random Navigation

Start autonomous exploration with random goals:

```bash
ros2 run simple_navigation_project simple_navigator
```

## 🔧 Node Details

### SimpleObstacleAvoider

**Subscribed Topics:**
- `/scan` (sensor_msgs/LaserScan): Laser scan data

**Published Topics:**
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands

**Parameters:**
- `normal_speed`: Forward speed (validated: 0.1-0.5 m/s)
- `turn_speed`: Angular speed (validated: 0.3-1.5 rad/s)
- `min_distance`: Safety distance (validated: 0.3-1.0 m)

**Behavior Logic:**
1. **Clear Path**: Move forward with slight random variation
2. **Obstacle Detected**: Choose turn direction based on sensor clearance
   - Both sides clear → Random turn direction
   - Right clear → Turn right
   - Left clear → Turn left
   - Both blocked → Default left turn

### SimpleNavigator

**Published Topics:**
- `/goal_pose` (geometry_msgs/PoseStamped): Navigation goals

**Functionality:**
- Generates random goals every 10 seconds
- Goal range: ±2.0m in X and Y
- Random orientation: ±π radians
- Publishes in `map` frame for Nav2 compatibility

## 📋 Requirements

### ROS 2 Dependencies
- `rclpy`: ROS 2 Python client library
- `geometry_msgs`: Twist and PoseStamped message types
- `sensor_msgs`: LaserScan message type
- `nav2_bringup`: Nav2 navigation stack
- `tf2_ros`: Transform library

### Hardware Requirements
- **Laser Scanner**: 2D LIDAR (e.g., RPLidar, Hokuyo)
- **Mobile Robot**: Differential drive robot with cmd_vel interface
- **Computing**: ROS 2 compatible system

### Frame Requirements
- `base_link`: Robot base frame
- `odom`: Odometry frame  
- `map`: Global map frame (provided by static transform)

## ⚙️ Configuration

### Nav2 Stack Configuration

The `nav2_params.yaml` file configures:

- **Controller Server**: DWB local planner with velocity limits
- **Planner Server**: NavFn global planner
- **Costmaps**: Local (3x3m rolling) and global costmaps
- **Behavior Server**: Spin, backup, and wait behaviors

**Key Parameters:**
- Robot radius: 0.22m
- Max velocity: 0.26 m/s linear, 1.0 rad/s angular
- Costmap resolution: 0.05m
- Inflation radius: 0.55m

### Safety Features

- **Parameter Validation**: All speed and distance parameters are range-checked
- **Scan Validation**: Filters invalid laser readings (< 0.1m or > 10.0m)
- **Graceful Shutdown**: Sends stop command before node termination
- **Error Handling**: Robust error handling for sensor failures

## 🐛 Troubleshooting

**No laser data received:**
- Verify laser scanner is publishing to `/scan` topic
- Check laser scanner power and connections

**Robot doesn't move:**
- Confirm `/cmd_vel` topic is connected to robot base
- Check parameter values are within valid ranges

**Navigation fails:**
- Ensure `map` → `odom` → `base_link` transform chain exists
- Verify costmap configuration matches robot dimensions

**Build errors:**
- Ensure all ROS 2 dependencies are installed
- Source workspace: `source install/setup.bash`
