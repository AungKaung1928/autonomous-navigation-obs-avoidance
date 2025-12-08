# Test Edit 3
# Simple Autonomous Patrol Robot

Production-grade ROS2 robot that autonomously navigates environments with intelligent obstacle avoidance and narrow space detection.

## Features

- **Autonomous Navigation**: Continuous forward movement with intelligent path planning
- **Smart Obstacle Avoidance**: 0.8m safe clearance with real-time laser scan processing
- **Lane Selection**: Chooses wider paths and avoids narrow spaces automatically
- **Lifecycle Management**: Production-ready state management for reliable deployment
- **Never Gets Stuck**: Always moving - forward, turning, or searching for paths

## Quick Start

### Installation

```bash
cd ~/simple_nav_ws/src/simple_navigation_project
touch resource/simple_navigation_project

cd ~/simple_nav_ws
colcon build --packages-select simple_navigation_project
source install/setup.bash
```

### Run with TurtleBot3 Simulation

**Terminal 1 - Launch Gazebo:**
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2 - Start Patrol:**
```bash
cd ~/simple_nav_ws
source install/setup.bash
ros2 launch simple_navigation_project patrol_robot.launch.py
```

## Parameter Tuning

### Quick Parameter Changes (Temporary)

```bash
# Change parameters at launch (no file editing needed)
ros2 launch simple_navigation_project patrol_robot.launch.py \
    patrol_speed:=0.3 \
    turn_speed:=0.7 \
    safe_distance:=1.0 \
    min_lane_width:=0.6
```

### Parameter Reference

| Parameter | Default | Range | What It Does |
|-----------|---------|-------|--------------|
| `patrol_speed` | 0.2 | 0.1-0.5 | Forward speed (m/s) |
| `turn_speed` | 0.5 | 0.3-1.0 | Rotation speed (rad/s) |
| `safe_distance` | 0.8 | 0.5-1.5 | Stop distance from obstacles (m) |
| `min_lane_width` | 0.8 | 0.5-1.2 | Minimum gap to enter (m) |

### Common Tuning Scenarios

**Robot moving too fast:**
```bash
ros2 launch simple_navigation_project patrol_robot.launch.py patrol_speed:=0.15
```

**Robot too cautious:**
```bash
ros2 launch simple_navigation_project patrol_robot.launch.py safe_distance:=0.6
```

**Robot touching obstacles:**
```bash
ros2 launch simple_navigation_project patrol_robot.launch.py safe_distance:=1.0
```

**Getting stuck in narrow spaces:**
```bash
ros2 launch simple_navigation_project patrol_robot.launch.py min_lane_width:=1.0
```

**Slow turning:**
```bash
ros2 launch simple_navigation_project patrol_robot.launch.py turn_speed:=0.8
```

**For tight environments:**
```bash
ros2 launch simple_navigation_project patrol_robot.launch.py \
    patrol_speed:=0.15 \
    min_lane_width:=0.6
```

**For open spaces:**
```bash
ros2 launch simple_navigation_project patrol_robot.launch.py \
    patrol_speed:=0.35 \
    safe_distance:=0.7
```

### Change Parameters While Running

```bash
# List all parameters
ros2 param list /patrol_controller

# Change speed on-the-fly
ros2 param set /patrol_controller patrol_speed 0.25

# Check current value
ros2 param get /patrol_controller patrol_speed
```

## Monitoring

```bash
# Watch robot velocity
ros2 topic echo /cmd_vel

# Monitor laser scan
ros2 topic echo /scan

# Check lifecycle state
ros2 lifecycle get /patrol_controller
```

## How It Works

1. **Laser Processing**: Divides 360° scan into front, left, right sectors
2. **Path Clear?** → Move forward at patrol speed
3. **Obstacle Detected?** → Stop forward motion, analyze lanes
4. **Lane Selection**: Measures left/right clearance, picks wider safe path
5. **Narrow Space Check**: Refuses to enter gaps < min_lane_width
6. **Always Moving**: Either forward OR rotating, never stuck

## Troubleshooting

**Robot doesn't move:**
```bash
ros2 topic hz /scan  # Check laser is working
```

**Robot keeps backing up or spinning:**
```bash
# Reduce turn speed, increase safe distance
ros2 launch simple_navigation_project patrol_robot.launch.py \
    turn_speed:=0.4 \
    safe_distance:=1.0
```

**Robot enters too-narrow spaces:**
```bash
# Increase minimum lane width
ros2 launch simple_navigation_project patrol_robot.launch.py min_lane_width:=1.0
```

**Gazebo crashes on startup:**
```bash
killall -9 gzserver gzclient
rm -rf /tmp/.gazebo-*
```

## Make Changes Permanent

Edit config file for permanent changes:
```bash
nano ~/simple_nav_ws/src/simple_navigation_project/config/patrol_params.yaml
```

Then rebuild:
```bash
cd ~/simple_nav_ws
colcon build --packages-select simple_navigation_project
source install/setup.bash
```

## Project Structure

```
simple_navigation_project/
├── simple_navigation_project/
│   ├── __init__.py
│   └── patrol_controller.py    # Main logic with lifecycle
├── launch/
│   └── patrol_robot.launch.py  # Launch file
├── config/
│   └── patrol_params.yaml       # Default parameters
├── resource/
│   └── simple_navigation_project
├── package.xml
├── setup.py
└── README.md
```

## License

Apache-2.0

---

**TL;DR**: Launch with `ros2 launch simple_navigation_project patrol_robot.launch.py`. Add `patrol_speed:=0.3` to change speed. Robot avoids obstacles at 0.8m, won't enter narrow gaps, never gets stuck.


