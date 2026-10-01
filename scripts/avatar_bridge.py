#!/usr/bin/env python3
"""AVATAR <-> ROS 2 bridge for stm32_avatar_mecanum_speed.ino.

Firmware (STM32F103 Blue Pill, AVATAR beacon BOARD_ID=1, 115200 CDC):
  host -> mcu : VX=2 (strafe +right, -1..1), VY=3 (forward +fwd),
                WZ=4 (rotate +CCW), STATE=5 (bool, motors live)
  mcu -> host : ECHO=12 (packets parsed), W_FL=13..W_RR=16 (computed wheels)
  Safety on MCU: 400 ms watchdog -> stop, DIR settle, slew limit,
  ON_THRESHOLD 0.12, SPEED_FLOOR/CEIL. DIR flip needs power reset per datasheet.

ROS side:
  Subscribes cmd_vel (m/s, rad/s), scales to normalized -1..1 with
  max_linear / max_angular params, sends at cmd_rate Hz (watchdog margin).
  Publishes OPEN-LOOP /odom + odom->base_footprint TF by integrating the
  commanded velocity (drifts with slip; no encoders on this base).
  Publishes ~/echo (Int32) + ~/wheels (Float32MultiArray FL FR RL RR).

Safety:
  STATE (motors live) defaults False -> MCU holds wheels at zero even with
  valid packets. Set state_live:=True only for wheels-on-blocks tests.
  Stale cmd_vel (> cmd_timeout s) sends zeros. Shutdown sends STATE=False.
"""
import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster

# Wire IDs must match stm32_avatar_mecanum_speed.ino Wire_ namespace.
VX, VY, WZ, STATE = 2, 3, 4, 5
ECHO, W_FL, W_FR, W_RL, W_RR = 12, 13, 14, 15, 16


class AvatarBridge(Node):
    def __init__(self):
        super().__init__('avatar_bridge')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('board_id', 1)
        self.declare_parameter('use_discover', False)
        self.declare_parameter('max_linear', 0.5)   # m/s that maps to 1.0
        self.declare_parameter('max_angular', 1.9)  # rad/s that maps to 1.0
        self.declare_parameter('cmd_rate', 20.0)    # Hz, MCU watchdog is 400 ms
        self.declare_parameter('cmd_timeout', 0.5)  # s without cmd_vel -> zeros
        self.declare_parameter('state_live', False)  # STATE=False is safe default
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.max_lin = self.get_parameter('max_linear').value
        self.max_ang = self.get_parameter('max_angular').value
        self.live = self.get_parameter('state_live').value

        self.cmd = (0.0, 0.0, 0.0)  # ros: fwd_x, left_y, ccw_wz (m/s, rad/s)
        self.cmd_time = self.get_clock().now()
        self.echo_count = 0
        self.wheels = [0.0, 0.0, 0.0, 0.0]
        self.lock = threading.Lock()
        self.x = self.y = self.th = 0.0
        self.last = self.get_clock().now()
        self.link = None

        # Publishers FIRST: the AVATAR reader thread can fire on_packet
        # during the open/settle window, before __init__ would otherwise
        # reach these lines (init-order race -> AttributeError spam).
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.echo_pub = self.create_publisher(Int32, '~/echo', 10)
        self.wheels_pub = self.create_publisher(Float32MultiArray, '~/wheels', 10)
        self.tf = TransformBroadcaster(self)

        try:
            from AVATAR import AVATARlink
        except ImportError as e:
            self.get_logger().fatal(f'AVATAR host lib missing: {e} (pip install ~/AVATAR/python)')
            raise

        layout = {'VX': VX, 'VY': VY, 'WZ': WZ, 'STATE': STATE,
                  'ECHO': ECHO, 'W_FL': W_FL, 'W_FR': W_FR,
                  'W_RL': W_RL, 'W_RR': W_RR}
        try:
            if self.get_parameter('use_discover').value:
                self.link = AVATARlink.discover(
                    self.get_parameter('board_id').value, baud=self.baud,
                    layout=layout, auto_reset=True, timeout=10.0)
            else:
                # Firmware header: host must open with auto_reset=True.
                self.link = AVATARlink(port=self.port, baud=self.baud,
                                       layout=layout, auto_reset=True)
            self.link.packet_callback = self.on_packet
            self.get_logger().info(f'AVATAR open: {self.port}@{self.baud} live={self.live}')
        except Exception as e:
            self.get_logger().fatal(f'AVATAR open failed: {e}')
            raise

        period = 1.0 / self.get_parameter('cmd_rate').value
        self.create_timer(period, self.tick)

    def on_cmd(self, msg):
        with self.lock:
            self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
            self.cmd_time = self.get_clock().now()

    def on_packet(self, fields):
        with self.lock:
            if 'ECHO' in fields:
                self.echo_count = int(fields['ECHO'])
                m = Int32()
                m.data = self.echo_count
                self.echo_pub.publish(m)
            w = [fields.get(k, None) for k in ('W_FL', 'W_FR', 'W_RL', 'W_RR')]
            if all(v is not None for v in w):
                self.wheels = [float(v) for v in w]
                m = Float32MultiArray()
                m.data = self.wheels
                self.wheels_pub.publish(m)

    @staticmethod
    def clamp(v, lo=-1.0, hi=1.0):
        return max(lo, min(hi, v))

    def tick(self):
        now = self.get_clock().now()
        dt = max((now - self.last).nanoseconds * 1e-9, 1e-3)
        self.last = now
        with self.lock:
            fx, ly, wz = self.cmd
            stale = (now - self.cmd_time).nanoseconds * 1e-9 > self.get_parameter('cmd_timeout').value
        if stale:
            fx, ly, wz = 0.0, 0.0, 0.0
        # ROS (x=fwd, y=left) -> firmware (VY=+fwd, VX=+right, WZ=+CCW).
        vy_n = self.clamp(fx / self.max_lin)
        vx_n = self.clamp(-ly / self.max_lin)
        wz_n = self.clamp(wz / self.max_ang)
        # Sub-threshold demands never move the wheels (firmware ON_THRESHOLD
        # 0.12) but would still drift odom if integrated: snap them to zero.
        if abs(vy_n) < 0.12 and abs(vx_n) < 0.12 and abs(wz_n) < 0.12:
            fx, ly, wz = 0.0, 0.0, 0.0
            vy_n, vx_n, wz_n = 0.0, 0.0, 0.0
        try:
            self.link.send_fields({'VY': float(vy_n), 'VX': float(vx_n),
                                   'WZ': float(wz_n), 'STATE': bool(self.live)})
        except Exception as e:
            self.get_logger().warn(f'AVATAR send failed: {e}', throttle_duration_sec=5.0)

        # Open-loop dead reckoning from commanded (not measured) velocity.
        self.x += (fx * math.cos(self.th) - ly * math.sin(self.th)) * dt
        self.y += (fx * math.sin(self.th) + ly * math.cos(self.th)) * dt
        self.th += wz * dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.get_parameter('odom_frame').value
        odom.child_frame_id = self.get_parameter('base_frame').value
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)
        # High covariance: open-loop, no encoders. Signals unreliability.
        odom.pose.covariance[0] = odom.pose.covariance[7] = 0.1
        odom.pose.covariance[35] = 0.2
        odom.twist.twist.linear.x = fx
        odom.twist.twist.linear.y = ly
        odom.twist.twist.angular.z = wz
        odom.twist.covariance[0] = odom.twist.covariance[7] = 0.1
        odom.twist.covariance[35] = 0.2
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.get_parameter('odom_frame').value
        t.child_frame_id = self.get_parameter('base_frame').value
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.th / 2.0)
        t.transform.rotation.w = math.cos(self.th / 2.0)
        self.tf.sendTransform(t)

    def shutdown(self):
        try:
            self.link.send_fields({'VY': 0.0, 'VX': 0.0, 'WZ': 0.0, 'STATE': False})
        except Exception:
            pass
        try:
            self.link.close()
        except Exception:
            pass


def main():
    rclpy.init()
    node = AvatarBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
