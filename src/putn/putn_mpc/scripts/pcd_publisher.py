#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
import os
import importlib

pypcd = None


class PCDPublisher(Node):
    def __init__(self):
        super().__init__('pcd_publisher')
        self.declare_parameter('pcd_path', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic', '/test_map')
        self.declare_parameter('repeat_rate', 0.0)
        self.declare_parameter('crop/enabled', False)
        self.declare_parameter('crop/min_x', -1e9)
        self.declare_parameter('crop/max_x', 1e9)
        self.declare_parameter('crop/min_y', -1e9)
        self.declare_parameter('crop/max_y', 1e9)

        param_path = self.get_parameter('pcd_path').value
        self.pcd_path = param_path if param_path else os.path.join('/home/mark/ws/oasis/ros_ws/src/putn_ROS2/src/putn/putn_mpc/scripts/map.pcd')
        self.frame_id = self.get_parameter('frame_id').value
        topic = self.get_parameter('topic').value
        self.topic = topic
        self.repeat_rate = float(self.get_parameter('repeat_rate').value)
        self.crop_enabled = bool(self.get_parameter('crop/enabled').value)
        self.crop_min_x = float(self.get_parameter('crop/min_x').value)
        self.crop_max_x = float(self.get_parameter('crop/max_x').value)
        self.crop_min_y = float(self.get_parameter('crop/min_y').value)
        self.crop_max_y = float(self.get_parameter('crop/max_y').value)

        

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(PointCloud2, topic, qos)

        self.points = None
        if self.pcd_path and os.path.isfile(self.pcd_path):
            try:
                arr = self._load_pcd(self.pcd_path)
                if arr is not None:
                    if self.crop_enabled:
                        mask = (
                            (arr[:,0] >= self.crop_min_x) & (arr[:,0] <= self.crop_max_x) &
                            (arr[:,1] >= self.crop_min_y) & (arr[:,1] <= self.crop_max_y)
                        )
                        arr = arr[mask]
                        self.get_logger().info(
                            f'Applied XY crop: x=[{self.crop_min_x},{self.crop_max_x}] y=[{self.crop_min_y},{self.crop_max_y}]')
                    self.points = arr
                    self.get_logger().info(f'Loaded PCD: {self.pcd_path}, points={arr.shape[0]}')
                else:
                    self.get_logger().error('Unsupported PCD format')
            except Exception as e:
                self.get_logger().error(f'Failed to parse PCD: {e}')
        else:
            self.get_logger().error(f'Invalid pcd_path: {self.pcd_path}')

        if self.points is not None and self.points.size > 0:
            if self.repeat_rate <= 0.0:
                self.repeat_rate = 1.0
            self.timer = self.create_timer(self.repeat_rate, self.publish_once)
        else:
            self.get_logger().warn('No points to publish')

    def publish_once(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        msg = pc2.create_cloud_xyz32(header, self.points.tolist())
        self.pub.publish(msg)

    def _load_pcd(self, path):
        with open(path, 'rb') as f:
            raw = f.read()
        lines = raw.split(b'\n')
        header_end = 0
        fields = []
        sizes = []
        types = []
        counts = []
        width = None
        height = None
        points = None
        data_mode = None
        for i, line in enumerate(lines):
            s = line.decode('ascii', errors='ignore')
            if s.startswith('FIELDS'):
                fields = s.split()[1:]
            elif s.startswith('SIZE'):
                sizes = [int(v) for v in s.split()[1:]]
            elif s.startswith('TYPE'):
                types = s.split()[1:]
            elif s.startswith('COUNT'):
                counts = [int(v) for v in s.split()[1:]]
            elif s.startswith('WIDTH'):
                width = int(s.split()[1])
            elif s.startswith('HEIGHT'):
                height = int(s.split()[1])
            elif s.startswith('POINTS'):
                points = int(s.split()[1])
            elif s.startswith('DATA'):
                token = s.split()[1]
                data_mode = token
                header_end = i + 1
                break
        if not fields:
            return None
        if points is None and width is not None and height is not None:
            points = width * height
        if points is None:
            return None
        if data_mode == 'ascii':
            pts = []
            for line in lines[header_end:]:
                if not line:
                    continue
                parts = line.decode('ascii', errors='ignore').split()
                try:
                    x = float(parts[fields.index('x')])
                    y = float(parts[fields.index('y')])
                    z = float(parts[fields.index('z')])
                    pts.append((x, y, z))
                except Exception:
                    continue
            if len(pts) == 0:
                return None
            return np.asarray(pts, dtype=np.float32)
        elif data_mode == 'binary':
            offset = sum(len(l) + 1 for l in lines[:header_end])
            data = raw[offset:]
            if not sizes or not types:
                return None
            if not counts:
                counts = [1] * len(fields)
            dtype_fields = []
            total_size = 0
            for name, sz, tp, cnt in zip(fields, sizes, types, counts):
                for c in range(cnt):
                    if tp == 'F':
                        if sz == 4:
                            dt = '<f4'
                        elif sz == 8:
                            dt = '<f8'
                        else:
                            return None
                    elif tp == 'I':
                        if sz == 1:
                            dt = '<i1'
                        elif sz == 2:
                            dt = '<i2'
                        elif sz == 4:
                            dt = '<i4'
                        elif sz == 8:
                            dt = '<i8'
                        else:
                            return None
                    elif tp == 'U':
                        if sz == 1:
                            dt = '<u1'
                        elif sz == 2:
                            dt = '<u2'
                        elif sz == 4:
                            dt = '<u4'
                        elif sz == 8:
                            dt = '<u8'
                        else:
                            return None
                    else:
                        return None
                    dtype_fields.append((f'{name}_{c}' if cnt > 1 else name, dt))
                    total_size += sz
            try:
                arr = np.frombuffer(data[:points*total_size], dtype=np.dtype(dtype_fields))
            except Exception:
                return None
            xs = None; ys = None; zs = None
            if 'x' in arr.dtype.names:
                xs = arr['x'].astype(np.float32)
            if 'y' in arr.dtype.names:
                ys = arr['y'].astype(np.float32)
            if 'z' in arr.dtype.names:
                zs = arr['z'].astype(np.float32)
            if xs is None or ys is None or zs is None:
                return None
            return np.stack([xs, ys, zs], axis=1)
        else:
            return None


def main():
    rclpy.init()
    node = PCDPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
