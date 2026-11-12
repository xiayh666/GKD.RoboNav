#!/usr/bin/env python3
"""Bounded hardware receive check; does not run odometry or modify the lidar."""
import argparse
import json
import math
import statistics
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import Imu, PointCloud2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=10.)
    parser.add_argument('--format', choices=['custom', 'pointcloud2'], default='custom')
    parser.add_argument('--lidar-topic', default='/livox/lidar')
    parser.add_argument('--imu-topic', default='/livox/imu')
    args = parser.parse_args()
    assert args.seconds > 0
    rclpy.init()
    node = rclpy.create_node('mid360_receive_probe')
    lidar_times, imu_times, point_counts, offset_spans, acceleration_norms = [], [], [], [], []
    invalid = []
    previous = {}
    sensor_gaps = {"lidar": [], "imu": []}
    qos = QoSProfile(depth=256, reliability=ReliabilityPolicy.BEST_EFFORT)
    fields = []

    def stamp(msg):
        return msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec

    def monotonic(kind, value):
        if kind in previous and value <= previous[kind]:
            invalid.append(kind + '_non_increasing_time')
        if kind in previous:
            sensor_gaps[kind].append((value-previous[kind])/1e9)
        previous[kind] = value

    def lidar(msg):
        nonlocal fields
        lidar_times.append(time.monotonic())
        monotonic('lidar', stamp(msg))
        if args.format == 'custom':
            point_counts.append(msg.point_num)
            if msg.point_num != len(msg.points):
                invalid.append('point_count_mismatch')
            if stamp(msg) != msg.timebase:
                invalid.append('header_timebase_mismatch')
            if msg.points:
                offsets = [p.offset_time for p in msg.points]
                offset_spans.append((max(offsets)-min(offsets))/1e9)
                if any(not all(map(math.isfinite, (p.x, p.y, p.z))) for p in msg.points):
                    invalid.append('nonfinite_point')
        else:
            point_counts.append(msg.width*msg.height)
            fields = [f.name for f in msg.fields]
            if not {'x', 'y', 'z', 'timestamp'}.issubset(fields):
                invalid.append('missing_pointcloud2_fields')
            if len(msg.data) != msg.row_step*msg.height:
                invalid.append('pointcloud2_data_size_mismatch')

    def imu(msg):
        imu_times.append(time.monotonic())
        monotonic('imu', stamp(msg))
        a = msg.linear_acceleration
        w = msg.angular_velocity
        values = [a.x, a.y, a.z, w.x, w.y, w.z]
        if not all(map(math.isfinite, values)):
            invalid.append('nonfinite_imu')
        acceleration_norms.append(math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z))

    node.create_subscription(CustomMsg if args.format == 'custom' else PointCloud2,
                             args.lidar_topic, lidar, qos)
    node.create_subscription(Imu, args.imu_topic, imu, qos)
    begin = time.monotonic()
    try:
        while time.monotonic()-begin < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    def frequency(times):
        return (len(times)-1)/(times[-1]-times[0]) if len(times)>1 else None
    result = {'duration_s': time.monotonic()-begin, 'format': args.format,
              'lidar_topic': args.lidar_topic, 'imu_topic': args.imu_topic,
              'lidar_messages': len(lidar_times), 'lidar_receive_hz': frequency(lidar_times),
              'imu_messages': len(imu_times), 'imu_receive_hz': frequency(imu_times),
              'imu_stamp_gap_s_max': max(sensor_gaps['imu'], default=None),
              'imu_stamp_gap_s_p50': statistics.median(sensor_gaps['imu']) if sensor_gaps['imu'] else None,
              'points_per_message_min': min(point_counts, default=0),
              'points_per_message_max': max(point_counts, default=0),
              'points_received': sum(point_counts), 'pointcloud2_fields': fields,
              'offset_span_s_min': min(offset_spans, default=None),
              'offset_span_s_max': max(offset_spans, default=None),
              'imu_acceleration_norm_raw_p50': statistics.median(acceleration_norms) if acceleration_norms else None,
              'errors': sorted(set(invalid)), 'error_count': len(invalid)}
    print(json.dumps(result, indent=2))
    if not lidar_times or not imu_times or invalid:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
