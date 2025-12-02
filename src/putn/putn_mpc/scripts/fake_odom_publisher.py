#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math


class FakeOdomPublisher(Node):
    def __init__(self):
        super().__init__('fake_odom_publisher')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('rate', 10.0)

        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.z = float(self.get_parameter('z').value)
        self.yaw = float(self.get_parameter('yaw').value)
        rate = float(self.get_parameter('rate').value)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(1.0/max(rate, 0.1), self._tick)

    def _tick(self):
        half = 0.5 * self.yaw
        qx = 0.0
        qy = 0.0
        qz = math.sin(half)
        qw = math.cos(half)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = t.header.stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = FakeOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
