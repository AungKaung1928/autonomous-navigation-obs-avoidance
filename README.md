# Simple Autonomous Patrol Robot

ROS 2 Humble LifecycleNode that wanders a TurtleBot3 through Gazebo Harmonic: drive while the
front sector is clear, otherwise stop and turn toward the wider clear lane, refusing gaps
narrower than `min_lane_width`. Reactive, map-free, 10 Hz.

Stack: ROS 2 Humble · Gazebo Harmonic (gz-sim 8) via `ros-humble-ros-gzharmonic` · Python.

## Features

- **Autonomous Navigation**: Continuous forward movement with intelligent path planning
- **Smart Obstacle Avoidance**: 0.8m safe clearance with real-time laser scan processing
- **Lane Selection**: Chooses wider paths and avoids narrow spaces automatically
- **Lifecycle Management**: Production-ready state management for reliable deployment
- **Never Gets Stuck**: Always moving - forward, turning, or searching for paths

## Quick Start

```bash
sudo apt install gz-harmonic ros-humble-ros-gzharmonic ros-humble-turtlebot3-description \
                 ros-humble-xacro ros-humble-robot-state-publisher
mkdir -p ~/patrol_ws/src && cd ~/patrol_ws/src
git clone https://github.com/AungKaung1928/autonomous-navigation-obs-avoidance.git
cd .. && colcon build --symlink-install && source install/setup.bash

pkill -9 -f "gz sim"      # stale gz servers share the bus and corrupt /clock and /odom
ros2 launch simple_navigation_project patrol_gazebo.launch.py
```
`patrol_gazebo.launch.py` starts Gazebo Harmonic with `worlds/turtlebot3_world.sdf` (hexagon +
9 pillars), spawns the burger from `description/turtlebot3_burger.urdf.xacro`, bridges
`/scan /cmd_vel /odom /tf /clock` with `config/gz_bridge.yaml`, and starts the controller.
Arguments: `world`, `headless:=true`, `x_pose y_pose yaw`, `params_file`. A 6 x 6 m room with
a partition is available as `world:=.../worlds/wall_follow_world.sdf`.

Controller only (a simulator or robot already publishes `/scan`):
```bash
ros2 launch simple_navigation_project patrol_robot.launch.py
```

## Measured (Gazebo Harmonic, headless, 120 s, turtlebot3_world, spawn (-2.0, -0.5))
Independent 10 Hz recorder on `/odom` and `/scan`:

| Metric | Value |
|---|---|
| Path length | 19.3 m |
| Longest stall (net move < 5 cm) | 2.9 s |
| Closest scan return | 0.31 m (no contact; contact = < 0.16 m) |
| Decisions logged | 62 forward · 23 turn · 22 rotate-for-opening · 6 lane-too-narrow |

The trajectory circulates the whole arena (every 10 s sample lands in a different sector).

## What was fixed (2026-09-05)
- Scan sectors were hard-coded index ranges assuming the Classic LDS convention (index 0 =
  front, one index per degree). Under Harmonic's gpu_lidar `angle_min = -pi`, so "front" read the
  rear and left/right swapped. Sectors are now derived from `angle_min` / `angle_increment`.
- `config/patrol_params.yaml` contained pasted Markdown after line 5 and could not be parsed;
  the node died at start-up whenever the file was used.
- `/scan` subscription is BEST_EFFORT (sensor QoS), matching both the bridge and the real LDS.
- Shutdown deactivates and stops the robot while the rclpy context is still valid.

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

**Gazebo shows no robot or /clock jumps:**
```bash
pkill -9 -f "gz sim"   # a stale server from a previous run is still on the gz bus
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
Apache License 2.0 - See [LICENSE](LICENSE) for details.

This project demonstrates production-grade ROS2 obstacle avoidance with lifecycle management.
Free to use for commercial and educational purposes.
---
