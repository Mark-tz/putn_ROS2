#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
import time

class SimpleSim(Node):
    def __init__(self):
        super().__init__('simple_sim')
        
        # Parameters
        self.declare_parameter('frame_id', 'world') # Usually 'odom' or 'world'
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('rate', 50.0) # 50Hz simulation rate

        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        
        # Initial State
        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.z = float(self.get_parameter('z').value)
        self.yaw = float(self.get_parameter('yaw').value)
        
        # Velocities
        self.v = 0.0 # Linear velocity
        self.w = 0.0 # Angular velocity
        
        # ROS Infrastructure
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        
        rate = float(self.get_parameter('rate').value)
        self.dt = 1.0 / max(rate, 1.0)
        self.timer = self.create_timer(self.dt, self._tick)
        
        self.last_time = self.get_clock().now()
        self.get_logger().info(f"SimpleSim initialized at ({self.x}, {self.y}, {self.yaw})")

    def cmd_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def _tick(self):
        current_time = self.get_clock().now()
        # dt = (current_time - self.last_time).nanoseconds / 1e9
        # For simplicity and stability, we can use fixed dt from rate, 
        # or use real dt. Real dt is better for wall-clock sync.
        # But if simulation is lagging, fixed dt might be smoother for logic?
        # Let's use fixed dt for predictable kinematic integration steps.
        dt = self.dt 
        
        # Kinematic Integration (Euler)
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt
        
        # Normalize yaw
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        
        # Calculate Quaternion
        half = 0.5 * self.yaw
        qx = 0.0
        qy = 0.0
        qz = math.sin(half)
        qw = math.cos(half)
        
        # Publish TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
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

        # Publish Odom
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        
        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        # Twist (feedback actual velocity)
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        
        self.odom_pub.publish(odom)
        
        self.last_time = current_time


def main():
    rclpy.init()
    node = SimpleSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
