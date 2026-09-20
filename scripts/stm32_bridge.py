#!/usr/bin/env python3
"""STM32 <-> ROS 2 bridge (FUTURE protocol, not the current wasd sketch).

Wire-up assumed after MAX485 rewire: RPi USB serial <-> STM32 Blue Pill
<-> RS485 bus <-> 4x RMCS-3001 (Mode 1 digital closed-loop, IDs 1-4).

Serial protocol (115200 8N1, \\n-terminated ASCII):
  ROS -> STM32:  V <vx> <vy> <wz>\\n     (m/s, m/s, rad/s, FL FR RL RR order
                                          is handled on the STM32 side)
  STM32 -> ROS:  O <h0> <h1> <h2> <h3>\\n (wheel feedback in Hz, FL FR RL RR)

Wheel linear speed: v = hz * 2*pi*r / pole_pairs.
Mecanum FK (X-config, matches firmware mix tables):
  vy = (v0+v1+v2+v3)/4
  vx = (v0-v1-v2+v3)/4
  wz = (v0-v1+v2-v3)/(4*(lx+ly))

Publishes /odom + odom->base_footprint TF. Without hardware (default),
holds zero velocity and warns — safe to run for clean-bringup testing.
"""
import math
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class Stm32Bridge(Node):
    def __init__(self):
        super().__init__('stm32_bridge')
        self.declare_parameter('port', '/dev/stm32')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_radius', 0.048)
        self.declare_parameter('lx', 0.220)
        self.declare_parameter('ly', 0.1196)
        self.declare_parameter('pole_pairs', 4)  # TODO: confirm NEMA32 datasheet
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.r = self.get_parameter('wheel_radius').value
        self.lx = self.get_parameter('lx').value
        self.ly = self.get_parameter('ly').value
        self.pp = self.get_parameter('pole_pairs').value

        self.cmd = (0.0, 0.0, 0.0)
        self.x = self.y = self.th = 0.0
        self.last = self.get_clock().now()
        self.ser = None
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self.get_logger().info(f'serial open: {self.port}@{self.baud}')
        except Exception as e:
            self.get_logger().warn(f'no serial ({e}); holding zero odom')

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf = TransformBroadcaster(self)
        self.create_timer(0.05, self.tick)  # 20 Hz

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        if self.ser:
            try:
                self.ser.write(f"V {self.cmd[0]:.3f} {self.cmd[1]:.3f} {self.cmd[2]:.3f}\n".encode())
            except Exception as e:
                self.get_logger().warn(f'serial write failed: {e}')

    def read_wheels(self):
        """Return 4 wheel linear speeds (m/s) or None."""
        if not self.ser:
            return None
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
        except Exception:
            return None
        if not line.startswith('O'):
            return None
        try:
            hz = [float(v) for v in line[1:].split()]
            assert len(hz) == 4
            k = 2 * math.pi * self.r / self.pp
            return [h * k for h in hz]
        except Exception:
            return None

    def tick(self):
        now = self.get_clock().now()
        dt = max((now - self.last).nanoseconds * 1e-9, 1e-3)
        self.last = now

        w = self.read_wheels()
        if w is None:
            vx, vy, wz = 0.0, 0.0, 0.0
        else:
            v0, v1, v2, v3 = w
            vy = (v0 + v1 + v2 + v3) / 4.0
            vx = (v0 - v1 - v2 + v3) / 4.0
            wz = (v0 - v1 + v2 - v3) / (4.0 * (self.lx + self.ly))

        self.x += (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        self.y += (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
        self.th += wz * dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.get_parameter('odom_frame').value
        odom.child_frame_id = self.get_parameter('base_frame').value
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
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


def main():
    rclpy.init()
    rclpy.spin(Stm32Bridge())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
