#!/usr/bin/env python3
"""Publish a KITTI x,y,z,intensity .bin scan as ROS 2 PointCloud2."""
import argparse
import time
from pathlib import Path

import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bin', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'examples/data/000095.bin')
    parser.add_argument('--topic', default='/points_raw')
    parser.add_argument('--frame-id', default='lidar')
    parser.add_argument('--stride', type=int, default=10,
                        help='Use every Nth point; 10 reproduces the included CSVs')
    parser.add_argument('--wait-subscribers', type=int, default=3)
    parser.add_argument('--repeat', type=int, default=1)
    args = parser.parse_args()
    if args.stride < 1 or args.repeat < 1 or args.wait_subscribers < 1:
        parser.error('stride, repeat and wait-subscribers must be positive')

    raw = np.fromfile(args.bin, dtype=np.float32)
    if raw.size == 0 or raw.size % 4:
        parser.error('Input must contain float32 x,y,z,intensity tuples')
    points = [tuple(map(float, row)) for row in raw.reshape(-1, 4)[::args.stride]]
    fields = [PointField(name=name, offset=4 * index, datatype=PointField.FLOAT32, count=1)
              for index, name in enumerate(('x', 'y', 'z', 'intensity'))]

    rclpy.init()
    node = rclpy.create_node('descriptor_example_publisher')
    publisher = node.create_publisher(PointCloud2, args.topic, 10)
    deadline = time.monotonic() + 15
    while publisher.get_subscription_count() < args.wait_subscribers:
        if time.monotonic() >= deadline:
            raise RuntimeError(f'Expected {args.wait_subscribers} subscribers on {args.topic}; '
                               f'found {publisher.get_subscription_count()}')
        rclpy.spin_once(node, timeout_sec=0.2)
    for index in range(args.repeat):
        header = Header()
        header.frame_id = args.frame_id
        header.stamp = node.get_clock().now().to_msg()
        cloud = point_cloud2.create_cloud(header, fields, points)
        publisher.publish(cloud)
        print(f'Published scan {index + 1}/{args.repeat}: {len(points)} points on {args.topic}')
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
