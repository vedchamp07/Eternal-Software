# DEPLOY_STATUS — hardware bringup handoff (read this first next session)

Last verified: 2026-09-27 (motor motion tests passed on all 3 axes; Pi test stack killed/parked).
Dev box: `Sadhguru`, x86_64, Ubuntu 24.04, ROS Jazzy. Repo: `/home/vedant/ros2_ws/src/my_robot_nav` (git).
Robot: hostname `enrguild`, aarch64 (RPi), ROS Jazzy, user `govil`. Repo: `~/ros2_ws/src/my_robot_nav` (NOT a git repo — plain copy; sync via rsync below).

## SSH (key auth works, no password needed)

```bash
ssh govil@enrguild.local
# all one-liners in this doc run as: ssh ... "source /opt/ros/jazzy/setup.bash; ..."
# WARNING: `pkill -f '<name>'` over ssh self-matches (remote bash -c contains
# the pattern and kills its own session). Always use bracket patterns:
pkill -f '[b]ringup'; pkill -f '[a]vatar_bridge'; pkill -f '[r]plidar'
```

## USB map (stable, verified via /dev/serial/by-id)

- Lidar (RPLIDAR A1M8, S/N 9754FA89C7E19EC8BCE499F01C835670, FW 1.29, HW rev 7):
  `Silicon_Labs_CP2102... -> /dev/ttyUSB0`, 115200. Launch with `serial_port:=/dev/ttyUSB0`
  (repo default `/dev/ttyUSB1` is WRONG on this Pi — enumeration flipped vs old notes).
  2026-09-27: lidar physically disconnected (only STM32 enumerated); motor tests run without it
  via standalone `ros2 run my_robot_nav avatar_bridge.py` (bringup always starts rplidar node,
  so full bringup is NOT usable lidar-less).
- STM32 Blue Pill (motor controller): `STMicroelectronics_BLUEPILL... -> /dev/ttyACM0`, 115200 CDC.
- No udev symlinks yet (`/dev/rplidar`, `/dev/stm32` don't exist). Recommended next: add them.
- Pi user `govil` is in `dialout`. LiDAR USB opens fine; lidar previously suspected of
  undervoltage on direct Pi USB — powered hub still recommended, not yet confirmed in use.

## What works (verified with logs this session)

1. **Lidar**: `ros2 run rplidar_ros rplidar_composition` with `serial_port:=/dev/ttyUSB0`,
   `frame_id:=laser_link` → `/scan` @ ~5.8 Hz. Both `rplidar_ros` and `sllidar_ros2`
   are installed on the Pi; bringup uses `rplidar_ros`. Oddity (non-blocking):
   some ranges report ~17.8 m despite `range_max: 12.0` — driver not clamping.
2. **Full bringup**: `ros2 launch my_robot_nav bringup.launch.py slam:=True
   serial_port:=/dev/ttyUSB0 stm32_port:=/dev/ttyACM0 state_live:=False`
   → RSP + lidar + avatar_bridge + slam_toolbox all start; slam activates, lidar healthy.
3. **Bridge duplex**: `/odom` @ 20 Hz, `odom->base_footprint` TF resolves,
    `/avatar_bridge/echo` counts MCU-parsed packets (seen 1560), `/avatar_bridge/wheels`
    echoes `[0,0,0,0]` at rest. Zero tracebacks. Costmap `Invalid frame ID "odom"`
    errors went 51 → ~3 startup transients.
4. **Motor motion 2026-09-27** (standalone bridge, `state_live:=True`, 2 s
   `ros2 topic pub --rate 10 --times 20` pulses, wheels visually confirmed spinning):
   fwd `linear.x=0.1` → wheels `[0.2,0.2,0.2,0.2]`; fwd `0.2` → `[0.4]*4`;
   reverse `-0.2` → `[-0.4]*4` (DIR flip fwd→rev works, NO power reset needed);
   strafe-left `linear.y=0.2` → `[FL,FR,RL,RR]=[-0.4,+0.4,+0.4,-0.4]` (textbook);
   rotate `angular.z=0.6` → `[+0.316,-0.316,+0.316,-0.316]` (0.6/1.9 exact).
   Stop verified every pulse (zeros after pub end via 400 ms watchdog + 0.5 s stale timeout).
   Odom integrated to x=0.72 (open-loop, expected drift).
   **Speed control 2026-09-27**: sweep `linear.x` = 0.1..0.5, 0.6, 1.0, 6.6 →
   wheels 0.2, 0.4, 0.6, 0.8, 1.0, then 1.0 flat (bridge `max_linear: 0.5` clamps;
   `cmd_vel` ≥ 0.5 m/s all give full speed; 6.6 m/s is not a real speed for this base).
   Proportional zone is ~0.06..0.5 m/s (below 0.06 snaps to zero via ON_THRESHOLD).

## Architecture (why it looks like this)

- `scripts/avatar_bridge.py` (installed via CMakeLists `PROGRAMS`) is the ONLY correct
  bridge. `scripts/stm32_bridge.py` speaks ASCII `V/O` — the flashed firmware does NOT;
  leave it alone (or delete later).
- Firmware on STM32: `stm32_avatar_mecanum_speed.ino` (user pasted in chat 2026-09-25;
  NOT yet saved under `firmware/` — only obsolete `stm32_modbus_bridge.ino` is there).
  Protocol: AVATAR binary (COBS, host lib `AVATAR`, pip 0.1.0), beacon BOARD_ID=1, 115200.
  Fields host→mcu: VX=2 strafe +right, VY=3 forward, VZ? no — WZ=4 +CCW, STATE=5 live;
  all -1..1. mcu→host: ECHO=12, W_FL=13..W_RR=16. MCU safety: 400 ms watchdog,
   DIR settle 250 ms, slew limit, ON_THRESHOLD 0.12, SPEED_FLOOR 20 / CEIL 85.
   2026-09-27: fwd↔rev DIR flips work live (no power reset needed in this sequence);
   strafe/rotate wheel patterns verified at `/avatar_bridge/wheels` level.
   **Rotate-sign suspect**: `angular.z=+0.6` (CCW) yields left-fwd/right-bwd
   (`[+,-,+,-]`), which reads as CW — firmware WZ convention or wiring may be
   inverted. Physically verify CCW-vs-CW before trusting Nav2 rotations; fix is a
   one-line WZ negation in the bridge if confirmed.
- Coordinate mapping in bridge: ROS `linear.x` fwd → VY, ROS `linear.y` left → VX negated,
  `angular.z` → WZ same sign. Scale params `max_linear: 0.5`, `max_angular: 1.9`
  (match MPPI limits). Demands under ON_THRESHOLD snap to zero so odom doesn't drift
  while wheels are physically stopped.
- Odom is OPEN-LOOP dead reckoning (no encoders/MAX485 on this base) with deliberately
  high covariance. Nav2 runs; autonomous goals WILL drift with slip. Laser odometry
  (`rf2o_laser_odometry`, not installed) is the recommended next accuracy upgrade.
- `bringup.launch.py` args: `bridge:=True` (default), `stm32_port` (default
  `/dev/ttyACM0`), `state_live:=False` (default SAFE — MCU holds zero).

## Known issues / traps

- **ModemManager grabs serial ports (ROOT CAUSE found 2026-09-28)**: Ubuntu's
  ModemManager holds `/dev/ttyACM0` (bridge FATAL: exclusive-lock denied) and probes
  the lidar's CP210x (AT commands wedge its parser → `80008000` timeouts). Fix applied
  on Pi: `sudo systemctl stop/disable/mask ModemManager` (mask survives reboot).
  If serial ever misbehaves again, check `fuser /dev/ttyUSB0 /dev/ttyACM0` first.
- **Lidar unit verdict 2026-09-28 (hardware, not software)**: raw pyserial probe
  (`~/lidar_probe.py` on Pi) shows full handshake — health `00 00 00`, valid scan
  descriptor — but ZERO measurement bytes, while the head spins. Ranging core (laser /
  detector / head data path) produces nothing. Bench checks: laser glow via phone
  camera, head flat-cable/slip-ring, swap USB cable, test on a laptop. No driver,
   baud, or power change can fix zero-byte data with a valid handshake.
  Without `/scan`, slam publishes no `/map`, and `planner_server` fails to activate
  (60 s bond timeout → lifecycle_manager_navigation aborts; controller+smoother stay
  active, bt_navigator and downstream never activate). Recovery after lidar fixed =
  full bringup relaunch (manager does not retry).
  2026-09-28 ~00:04: Pi REBOOTED mid-session (uptime reset to 3 min; mDNS flaked during
  reboot) and `throttled=0x50000` was already back within 3 min of boot — supply is
  marginal, treat powered hub / stronger supply as required, not optional.
- **Stale survivors on relaunch**: `pkill -f '[b]ringup'` kills only the launch wrapper;
  children (`robot_state_publisher`, `lifecycle_manager_slam`, `map_saver`, containers)
  survive and poison the next launch with duplicate node names (Nav2 components then
  silently never load). After killing a launch, `ps`+`kill <pid>` the leftovers and
  verify `ALL_CLEAN` before relaunching.

- **Firmware typo** (unconfirmed on flashed binary): pasted sketch has
  `ENABLE_PIN[N] = {PA0?...}` second entry `PBA9` — should be `PB9`. It would not
  compile as pasted; flashed binary was likely built from an older source. Verify in
  Arduino IDE before reflashing.
- **AVATAR lib skew (resolved)**: site-packages copy was older than `~/AVATAR` source
  (no `discover`/beacon/`exclusive`). Reinstalled via
  `pip install --user --break-system-packages --upgrade ~/AVATAR/python`. If packets
  stop parsing after an apt/pip change, re-check `inspect.signature(AVATARlink.__init__)`.
- **Bridge init-order race (fixed)**: AVATAR reader thread can fire `on_packet` during
  open/settle; publishers must be created BEFORE opening the link. Don't reorder.
- **Old Pi bringup had**: hardcoded `/dev/ttyUSB1`, `frame_id: laser`, no `ParameterValue`,
  ESP32 comments. All fixed by rsync from dev; if Pi ever looks stale again, re-sync.
- Pi rebooted mid-session (uptime reset, /tmp wiped). Keep test logs in `~` not `/tmp`.
- `maps/` empty → always `slam:=True` until a map is saved. No `sim.launch.py` usage on Pi.

## Sync + build (dev → Pi)

```bash
rsync -avz --exclude='__pycache__' --exclude='.git' --exclude='.claude' \
  /home/vedant/ros2_ws/src/my_robot_nav/ govil@enrguild.local:~/ros2_ws/src/my_robot_nav/
ssh govil@enrguild.local "cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash \
  && colcon build --packages-select my_robot_nav"
```

## Hardware state at wrap (2026-09-28 ~01:10)

- Network is a moving target (dev laptop roamed 4 subnets tonight; Pi on phone hotspot
  as `10.229.225.168`, mDNS `enrguild.local` flaky — if SSH fails, re-scan port 22
  for `.168` or check `ip neigh` for the Pi MAC).
- Pi power-cycled twice tonight (brownouts; `throttled=0x50000` returns within minutes
  of boot). ModemManager masked — survives reboot.
- **Lidar DOWN (hardware)**: enumerates, valid handshake, zero data bytes (see probe).
  Raw probe saved at `scripts/lidar_probe.py` (also `~/lidar_probe.py` on Pi).
- **Stack left RUNNING + LIVE**: `bringup_nav2c` launch, `state_live:=True`, bridge echo
  ~1226+, Nav2 loaded except planner/BT (need `/map`). Motors respond; drive battery
  proven (bot moved). Kill with separate `pkill -f '[b]ringup'` / `'[a]vatar_bridge'` /
  `'[r]plidar'` calls + PID sweep of survivors before any relaunch.

## Next session checklist

1. SSH (see network note above); confirm `/dev/ttyUSB0` (lidar) + `/dev/ttyACM0` present,
   `fuser` shows no holders, ModemManager still masked.
2. Lidar first: `python3 ~/lidar_probe.py` must show non-empty `pkt:` lines. If not,
   bench the unit (laser-glow phone-cam test, cables, laptop test) — do not burn time
   on drivers.
3. With scan flowing: clean relaunch bringup (`state_live:=True`), confirm planner +
   BT active, send `/goal_pose` 1 m ahead → autonomous run with avoidance.
4. Rotate-sign check still open: pulse `angular.z=+0.6`, confirm CCW; else negate WZ.
5. After a good map: save it (`slam:=False map:=...` only after).
6. Optional upgrades: udev symlinks, `rf2o_laser_odometry`, save sketch to `firmware/`,
   fix `PBA9`, retire `stm32_bridge.py` + `stm32_modbus_bridge.ino`.
