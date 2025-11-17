#!/usr/bin/env python3
import numpy as np
import sys
import math
import select
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, Float32, Int16
import math
from tf2_ros import Buffer, TransformListener


class Controller(Node):
    def __init__(self):
        super().__init__('control')
        self.N = 10
        self.rate_hz = 50
        self.curr_state = np.zeros(4)
        self.sub1 = self.create_subscription(Float32MultiArray, '/local_plan', self.local_planner_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub2 = self.create_publisher(Float32MultiArray, '/curr_state', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.have_plan = 0
        self.curr_time = 0
        self.time_sol = 0
        self.local_plan = np.zeros([self.N, 2])
        self.control_cmd = Twist()
        self.timer_localization = self.create_timer(0.01, self.get_current_state)
        self.timer_control = self.create_timer(1.0/self.rate_hz, self.control_tick)
        self.timer_health = self.create_timer(1.0, self.health_tick)
        self.declare_parameter('control_mode', 'auto')
        mode_value = self.get_parameter('control_mode').value
        self.is_manual = (str(mode_value).lower() == 'manual')

    def quart_to_rpy(self, x, y, z, w):
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def get_current_state(self):
        try:
            t = self.tf_buffer.lookup_transform('world', 'base_link', rclpy.time.Time())
            self.curr_state[0] = t.transform.translation.x
            self.curr_state[1] = t.transform.translation.y
            self.curr_state[2] = t.transform.translation.z
            roll, pitch, yaw = self.quart_to_rpy(
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w
            )
            self.curr_state[3] = yaw
            c = Float32MultiArray()
            c.data = [self.curr_state[0], self.curr_state[1], self.curr_state[2],
                      (self.curr_state[3]+np.pi) % (2*np.pi)-np.pi, roll, pitch]
            self.pub2.publish(c)
        except Exception:
            pass

    def cmd(self, data):
        self.control_cmd.linear.x = float(data[0])
        self.control_cmd.angular.z = float(data[1])
        self.pub.publish(self.control_cmd)

    def getKey(self):
        try:
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            if rlist:
                return sys.stdin.read(1)
        except Exception:
            return ''
        return ''

    def control_tick(self):
        if self.is_manual:
            self.manual_step()
        else:
            self.auto_step()

    def health_tick(self):
        self.health_log()

    def auto_step(self):
        if self.have_plan:
            ref_inputs = self.local_plan[0]
            self.cmd(ref_inputs)
        else:
            self.cmd(np.array([0.0, 0.0]))

    def manual_step(self):
        data = np.array([0.0, 0.0])
        key = self.getKey()
        if key == 'w':
            if(data[0] < 0.6):
                data[0] += 0.2
            else:
                data = data
        elif key == 'x':
            if(data[0] > -0.6):
                data[0] -= 0.2
            else:
                data = data
        elif key == 'a':
            if(data[1] < 0.6):
                data[1] += 0.2
            else:
                data = data
        elif key == 'd':
            if(data[1] > -0.6):
                data[1] -= 0.2
            else:
                data = data
        elif key == 'q':
            if(data[0] < 0.6):
                data[0] += 0.2
            else:
                data = data
            if(data[1] < 0.6):
                data[1] += 0.2
            else:
                data = data
        elif key == 'e':
            if(data[0] < 0.6):
                data[0] += 0.2
            else:
                data = data
            if(data[1] > -0.6):
                data[1] -= 0.2
            else:
                data = data
        elif key == 'c':
            if(data[0] > -0.6):
                data[0] -= 0.2
            else:
                data = data
            if(data[1] > -0.6):
                data[1] -= 0.2
            else:
                data = data
        elif key == 'z':
            if(data[0] > -0.6):
                data[0] -= 0.2
            else:
                data = data
            if(data[1] < 0.6):
                data[1] += 0.2
            else:
                data = data      
        elif key == 's':
            data = np.array([0.0, 0.0])
        elif key == 'i':
            self.is_manual = False
            self.get_logger().info('controller: switch to auto mode')
            return True
        elif key == 'm':
            self.is_manual = True
            self.get_logger().info('controller: switch to manual mode')
            return True
        elif (key == '\x03'):
            return False
        else:
            data = data
        self.cmd(data)

    def local_planner_cb(self, msg):
        for i in range(self.N):
            self.local_plan[i, 0] = msg.data[0+2*i]
            self.local_plan[i, 1] = msg.data[1+2*i]
        self.have_plan = 1
        try:
            self.get_logger().info(f"controller: received local_plan first=({self.local_plan[0,0]:.3f},{self.local_plan[0,1]:.3f})")
        except Exception:
            pass

        
    def health_log(self):
        try:
            if not hasattr(self, '_health_count'):
                self._health_count = 0
            self._health_count += 1
            if self._health_count % 50 == 0:
                self.get_logger().info(f"controller: curr_state=({self.curr_state[0]:.3f},{self.curr_state[1]:.3f},{self.curr_state[3]:.3f}), cmd=({self.control_cmd.linear.x:.3f},{self.control_cmd.angular.z:.3f})")
        except Exception:
            pass


def main():
    rclpy.init()
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
