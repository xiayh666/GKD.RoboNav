#!/usr/bin/env python3
"""Synthetic ROS integration check; publishes only isolated test sensor topics."""
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Imu


def main():
    rclpy.init()
    node = rclpy.create_node('point_lio_synthetic_check')
    lidar = node.create_publisher(CustomMsg, '/lio_test/lidar', 10)
    imu = node.create_publisher(Imu, '/lio_test/imu', 1000)
    odometry, diagnostics, paths = [], [], []
    node.create_subscription(Odometry, '/lio_test/odometry', odometry.append, 1000)
    node.create_subscription(DiagnosticArray, '/lio_test/diagnostics', diagnostics.append, 100)
    node.create_subscription(PathMsg, '/lio_test/path', paths.append,
                             QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    # Three orthogonal surfaces, deterministic points and independent acquisition times.
    positions = []
    for i in range(999):
        j = i//3
        a, b = j % 37*.15-2.7, j//37*.15-.6
        positions.append((a, b, -1.5) if i % 3 == 0 else
                         ((3., a, b) if i % 3 == 1 else (a, 3., b)))
    command = ['ros2', 'run', 'point_lio_bridge', 'point_lio_node', '--ros-args',
               '-r', 'livox/lidar:=/lio_test/lidar', '-r', 'livox/imu:=/lio_test/imu',
               '-r', 'point_lio/odometry:=/lio_test/odometry',
               '-r', 'point_lio/diagnostics:=/lio_test/diagnostics',
               '-p', 'odom_frame:=test_odom', '-p', 'imu_frame:=test_imu',
               '-p', 'lidar_frame:=test_lidar', '-p', 'path_capacity:=8',
               '-r', 'point_lio/path:=/lio_test/path']
    log = Path('doc/point-lio-integration/results/synthetic-node.log').open('w')
    process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)

    def spin_for(seconds):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.005)

    def stamp(header, ns):
        header.stamp.sec, header.stamp.nanosec = divmod(ns, 1000000000)

    def phase(base_ns):
        begin = time.monotonic()
        for tick in range(600):
            ns = base_ns+tick*5000000
            m = Imu()
            stamp(m.header, ns)
            m.linear_acceleration.z = 1.  # Driver units g; adapter must convert.
            imu.publish(m)
            if tick % 20 == 19:
                start_ns = base_ns+(tick//20)*100000000
                cloud = CustomMsg()
                stamp(cloud.header, start_ns)
                cloud.timebase = start_ns
                cloud.point_num = len(positions)
                cloud.points = [CustomPoint(x=float(p[0]), y=float(p[1]), z=float(p[2]),
                                            offset_time=int(.099*i/999*1e9),
                                            reflectivity=1, tag=0x10, line=0)
                                for i, p in enumerate(positions)]
                lidar.publish(cloud)
            deadline = begin+(tick+1)*.005
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=max(0., min(.001, deadline-time.monotonic())))
        spin_for(.3)

    try:
        spin_for(1.)
        assert process.poll() is None, 'node failed at startup'
        phase(100000000000)
        first = list(odometry)
        assert len(first) >= 15, f'only {len(first)} initial poses'
        first_count = len(first)
        phase(1000000000)  # Sensor restart: clock rolls back, map must reinitialize.
        spin_for(.5)
        second = odometry[first_count:]
        assert len(second) >= 15, f'only {len(second)} poses after clock rollback'
        for poses in (first, second):
            times = [m.header.stamp.sec*1e9+m.header.stamp.nanosec for m in poses]
            assert all(b > a for a, b in zip(times, times[1:])), 'nonmonotonic frame outputs'
        max_position, max_angle = 0., 0.
        for m in odometry:
            p, q = m.pose.pose.position, m.pose.pose.orientation
            values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w, *m.pose.covariance, *m.twist.covariance]
            assert all(map(math.isfinite, values))
            assert m.header.frame_id == 'test_odom' and m.child_frame_id == 'test_imu'
            assert abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) < 1e-6
            max_position = max(max_position, math.sqrt(p.x*p.x+p.y*p.y+p.z*p.z))
            max_angle = max(max_angle, 2*math.acos(min(1., abs(q.w))))
        resets = max((int(v.value) for d in diagnostics for s in d.status for v in s.values if v.key == 'resets'), default=0)
        assert resets >= 1, 'clock rollback not reported'
        assert any(p.poses for p in paths), 'trajectory missing'
        assert any(not p.poses for p in paths), 'reset did not clear trajectory'
        for p in paths:
            assert len(p.poses) <= 8
            stamps = [x.header.stamp.sec*1e9+x.header.stamp.nanosec for x in p.poses]
            assert all(b > a for a, b in zip(stamps, stamps[1:]))
            if stamps:
                assert stamps[-1]-stamps[0] < 1e9, 'trajectory joined across reset'
        assert max_position < .02 and max_angle < .02, 'synthetic stationary drift'
        result = {'first_epoch_poses': len(first), 'second_epoch_poses': len(second),
                  'max_position_m': max_position, 'max_angle_rad': max_angle,
                  'reset_counter_max': resets, 'finite_covariance': True,
                  'monotonic_within_epochs': True, 'sensor_topics_isolated': True, 'bounded_trajectory': True,
                  'trajectory_reset_verified': True}
        print(json.dumps(result, indent=2))
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
