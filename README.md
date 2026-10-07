# Mecanum Nav Bot · ROS 2 Jazzy

An autonomous mecanum-wheel robot built on **ROS 2 Jazzy** and **Nav2**. It maps its surroundings with SLAM from an RPLIDAR A1M8 and moves in any direction (forward, sideways, rotate) on four mecanum wheels. A Raspberry Pi runs the ROS stack, and an STM32 drives the motors over serial. The same robot model runs in Gazebo for simulation.

## Hardware

| Part | Role |
|---|---|
| Raspberry Pi (Ubuntu + ROS 2 Jazzy) | Runs Nav2, SLAM and the lidar driver |
| STM32F103 Blue Pill | Motor controller, connected to the Pi over USB serial |
| 4× RMCS-3001 BLDC + NEMA32 | Mecanum wheel drive |
| RPLIDAR A1M8 | 2D laser scans for mapping and obstacle avoidance |

Odometry is computed from the commanded velocity and published on `/odom` at 20 Hz.

## How it fits together

```
Nav2 / teleop ──/cmd_vel──▶ avatar_bridge.py ──serial──▶ STM32 ──▶ 4 mecanum wheels
                                   │
                                   └──▶ /odom + odom→base_footprint TF
RPLIDAR A1M8 ──/scan──▶ SLAM Toolbox ──/map──▶ Nav2
```

## Repo layout

```
launch/     bringup.launch.py (real robot), sim.launch.py (Gazebo), display.launch.py (model viewer)
config/     Nav2 parameters, controllers, Gazebo bridge, RViz views
urdf/       mecanum_bot_v3.urdf.xacro (robot model)
worlds/     Gazebo test world
scripts/    avatar_bridge.py (ROS ↔ STM32 bridge), stm32_bridge.py, twist_stamper.py, lidar_probe.py
firmware/   STM32 sketch
```

## Build

```bash
# inside your ROS 2 workspace, with this repo at src/my_robot_nav
colcon build --packages-select my_robot_nav
source install/setup.bash
```

## Run

**Simulation**
```bash
ros2 launch my_robot_nav sim.launch.py
```

**Real robot** (SLAM mode; set the serial ports for your machine)
```bash
ros2 launch my_robot_nav bringup.launch.py serial_port:=/dev/ttyUSB0 stm32_port:=/dev/ttyACM0 state_live:=True
```

`state_live` defaults to `False`, which keeps the motors at zero. Set it to `True` to let the robot move.

**Navigate with a saved map** (instead of SLAM)
```bash
ros2 launch my_robot_nav bringup.launch.py slam:=False map:=/path/to/map.yaml
```
