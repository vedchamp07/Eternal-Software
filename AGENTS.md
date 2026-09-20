# AGENTS.md — my_robot_nav

ROS 2 Jazzy package. Workspace root is `/home/vedant/ros2_ws`; this package is `src/my_robot_nav`.

## Build / run
- Build from workspace root: `colcon build --packages-select my_robot_nav`, then `source install/setup.bash`.
- `CMakeLists.txt` only installs `launch config urdf maps` — no C++ targets (`src/`, `include/my_robot_nav/` are empty, ignore them).
- Entrypoints: `launch/bringup.launch.py` (Nav2 + RSP + lidar), `launch/display.launch.py` (URDF viewer only).
- `bringup.launch.py` defaults: `slam:=True`, `map:=''`, `use_sim_time:=False` (real hardware). `maps/` is empty, so AMCL mode (`slam:=False map:=<file>.yaml`) has no map yet.
- No tests/CI/lint beyond `ament_lint_auto`. No README.

## Gotchas (verified)
- `rplidar_ros` is in `package.xml` and `bringup.launch.py` but NOT installed (`ros2 pkg executables rplidar_ros` fails). Install via `sudo apt install ros-jazzy-rplidar-ros`, then confirm executable name (`rplidar_composition` is currently assumed, unverified).
- Lidar port is a launch arg (`serial_port:=...`, default `/dev/ttyUSB1` for A1M8 @115200, `frame_id: laser_link` matches URDF). Enumeration vs STM32 (`/dev/ttyUSB0`, same CP210x chip) is unstable; prefer a `/dev/rplidar` udev symlink. Lidar needs a powered USB hub (undervoltage/timeouts when direct to RPi).
- Motors: 4x RMCS-3001 (BLDC, RS485 RTU Modbus) + NEMA32, driven GPIO-only by STM32F103 Blue Pill (`ENABLE` PWM chop + `DIR` pin, no Modbus yet). `DIR` needs power reset per datasheet — mecanum strafe/rotate unproven until `t` test passes. No MAX485 modules yet, so no live direction + no RPM feedback.
- No odometry anywhere: no `/odom`, no `odom->base_link`. STM32 is RPi-USB powered. Nav2 plans but cannot navigate until Modbus Mode-1 bridge provides odom + consumes `cmd_vel`.
- `bringup.launch.py` launches no RViz; `config/nav2_default_view.rviz` is dead unless run manually.
