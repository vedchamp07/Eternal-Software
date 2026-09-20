#!/usr/bin/env python3
"""Twist -> TwistStamped relay.

Nav2 (controller_server, velocity_smoother, collision_monitor, teleop) speaks
unstamped geometry_msgs/Twist on /cmd_vel, but mecanum_drive_controller 4.42
only subscribes to TwistStamped on its ~/reference input (the old
`use_stamped_vel: false` option is no longer honored). This node stamps every
incoming Twist (sim-time clock when use_sim_time is set) and republishes it,
so the whole Nav2 Twist pipeline works unchanged with the stamped controller.

Wired in launch/sim.launch.py: /cmd_vel -> twist_stamper -> /cmd_vel_stamped
(URDF remaps ~/reference to /cmd_vel_stamped).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistStamper(Node):
    def __init__(self):
        super().__init__('twist_stamper')
        self.declare_parameter('in_topic', 'cmd_vel')
        self.declare_parameter('out_topic', 'cmd_vel_stamped')
        self.declare_parameter('frame_id', 'base_link')
        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.pub = self.create_publisher(TwistStamped, out_topic, 10)
        self.sub = self.create_subscription(Twist, in_topic, self.on_cmd, 10)

    def on_cmd(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.twist = msg
        self.pub.publish(out)


def main():
    rclpy.init()
    node = TwistStamper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
