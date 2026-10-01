# AGENTS.md — my_robot_nav

ROS 2 Jazzy package. Workspace root is `/home/vedant/ros2_ws`; this package is `src/my_robot_nav`.

> Hardware deploy handoff: read `DEPLOY_STATUS.md` first (SSH, USB map, verified
> state, next steps). It is synced to the Pi alongside this file.

## Build / run
- Build from workspace root: `colcon build --packages-select my_robot_nav`, then `source install/setup.bash`.
- `CMakeLists.txt` only installs `launch config urdf maps` — no C++ targets (`src/`, `include/my_robot_nav/` are empty, ignore them).
- Entrypoints: `launch/bringup.launch.py` (Nav2 + RSP + lidar), `launch/display.launch.py` (URDF viewer only).
- `bringup.launch.py` defaults: `slam:=True`, `map:=''`, `use_sim_time:=False` (real hardware). `maps/` is empty, so AMCL mode (`slam:=False map:=<file>.yaml`) has no map yet.
- No tests/CI/lint beyond `ament_lint_auto`. No README.

## Gotchas (verified 2026-09-27, see DEPLOY_STATUS.md)
- `rplidar_ros` IS installed on the Pi (`rplidar_composition` verified). Only fresh machines need `sudo apt install ros-jazzy-rplidar-ros`.
- Lidar port is a launch arg (`serial_port:=...`, default `/dev/ttyUSB1` is WRONG on the Pi — use `/dev/ttyUSB0` for A1M8 @115200, `frame_id: laser_link` matches URDF). Enumeration is unstable; prefer a `/dev/rplidar` udev symlink. Lidar needs a powered USB hub (undervoltage/timeouts when direct to RPi). Full bringup always starts the lidar node, so lidar-less motor tests must run `avatar_bridge.py` standalone via `ros2 run`.
- Motors: 4x RMCS-3001 (BLDC) + NEMA32, driven by STM32F103 Blue Pill over AVATAR binary serial (`scripts/avatar_bridge.py`, CDC `/dev/ttyACM0`). Fwd/rev/strafe wheel patterns verified live 2026-09-27; fwd↔rev DIR flips work without power reset. No encoders/MAX485 — odom is open-loop only.
- Odom exists: `avatar_bridge.py` publishes `/odom` @ 20 Hz + `odom->base_footprint` TF from commanded velocity (high covariance, drifts). `bringup.launch.py` args: `bridge:=True`, `stm32_port`, `state_live:=False` (SAFE default — MCU holds zero; `True` only for wheels-on-blocks tests, 400 ms MCU watchdog stops motion on topic end).
- Rotate-sign suspect: `angular.z=+` yields left-fwd/right-bwd wheel pattern (reads as CW, expected CCW) — verify physically before Nav2 runs; fix is a one-line WZ negation in the bridge.
- `bringup.launch.py` launches no RViz; `config/nav2_default_view.rviz` is dead unless run manually.
- SSH trap: `pkill -f '<name>'` over ssh self-matches when the same remote command contains the literal name — never combine pkill with the target's command line in one `ssh` call; use separate calls with quoted bracket patterns (`pkill -f '[a]vatar_bridge'`).
- Network moves: dev laptop and Pi roam hotspots; if `enrguild.local` fails, find the Pi by port-22 sweep and use `-o HostKeyAlias=enrguild.local govil@<ip>`. ModemManager is masked on the Pi — if serial breaks, check `fuser` holders first.
- Lidar drill: `python3 ~/lidar_probe.py` (repo: `scripts/lidar_probe.py`) must show non-empty `pkt:` lines; valid handshake + zero data = dead ranging core (bench the unit, don't debug drivers).
