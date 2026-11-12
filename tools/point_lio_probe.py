#!/usr/bin/env python3
"""Observe real Point-LIO outputs; static validation requires a stationary sensor."""
import argparse
import csv
import json
import math
import statistics
import time
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=30.)
    parser.add_argument('--out', default='doc/point-lio-integration/results/live')
    parser.add_argument('--stationary', action='store_true',
                        help='Enforce drift limits only with a stationary sensor and scene')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('point_lio_output_probe')
    rows, arrivals, states, transforms, extrinsics, paths = [], [], [], {}, {}, []

    def odometry(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        ns = m.header.stamp.sec*1000000000+m.header.stamp.nanosec
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w, *m.pose.covariance, *m.twist.covariance]
        assert all(map(math.isfinite, values)), 'nonfinite odometry'
        assert abs(sum(v*v for v in [q.x, q.y, q.z, q.w])-1) < 1e-6
        assert m.header.frame_id == 'odom' and m.child_frame_id == 'lio_imu'
        rows.append([ns, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        arrivals.append(time.monotonic())

    def diagnostic(m):
        for s in m.status:
            if s.name == 'point_lio/estimator':
                states.append({'level': int.from_bytes(s.level, 'little') if isinstance(s.level, bytes) else int(s.level), 'message': s.message, **{v.key: v.value for v in s.values}})

    def dynamic(m):
        for t in m.transforms:
            if t.header.frame_id == 'odom' and t.child_frame_id == 'lio_imu':
                ns = t.header.stamp.sec*1000000000+t.header.stamp.nanosec
                p, q = t.transform.translation, t.transform.rotation
                transforms[ns] = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]

    def static(m):
        for t in m.transforms:
            extrinsics[(t.header.frame_id, t.child_frame_id)] = t

    node.create_subscription(PathMsg, '/point_lio/path', paths.append,
                             QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Odometry, '/point_lio/odometry', odometry, 1000)
    node.create_subscription(DiagnosticArray, '/point_lio/diagnostics', diagnostic, 100)
    node.create_subscription(TFMessage, '/tf', dynamic, 1000)
    node.create_subscription(TFMessage, '/tf_static', static,
                             QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    begin = time.monotonic()
    try:
        while time.monotonic()-begin < args.seconds:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    prefix = Path(args.out)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    with prefix.with_suffix('.csv').open('w') as f:
        writer = csv.writer(f, lineterminator='\n')
        writer.writerow(['stamp_ns', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        writer.writerows(rows)
    assert len(rows) >= 10, f'only {len(rows)} poses; diagnostics={states[-3:]}'
    assert all(b[0] > a[0] for a, b in zip(rows, rows[1:])), 'nonmonotonic output'
    initial = rows[0]
    position_error = [math.sqrt(sum((r[i]-initial[i])**2 for i in (1, 2, 3))) for r in rows]
    angle_error = [2*math.acos(min(1., abs(sum(r[i]*initial[i] for i in (4, 5, 6, 7))))) for r in rows]
    common = [r for r in rows if r[0] in transforms]
    assert len(common) >= 10, 'dynamic TF missing'
    assert all(max(abs(a-b) for a, b in zip(r[1:], transforms[r[0]])) < 1e-12 for r in common)
    t = extrinsics.get(('lio_imu', 'livox_frame'))
    assert t is not None, 'extrinsic static TF missing'
    actual = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    assert max(abs(a-b) for a, b in zip(actual, [-.011, -.02329, .04412])) < 1e-12
    assert any(p.poses for p in paths), 'trajectory path missing'
    assert all(len(p.poses) <= 3000 for p in paths), 'unbounded trajectory'
    assert all(p.header.frame_id == 'odom' for p in paths), 'wrong trajectory frame'
    result = {'validation_mode': 'stationary' if args.stationary else 'motion',
              'trajectory_messages': len(paths),
              'max_trajectory_poses': max(len(p.poses) for p in paths),
              'sensor_elapsed_s': (rows[-1][0]-rows[0][0])/1e9,
              'receive_elapsed_s': arrivals[-1]-arrivals[0],
              'observed_s': time.monotonic()-begin, 'poses': len(rows),
              'receive_hz': (len(rows)-1)/(arrivals[-1]-arrivals[0]),
              'max_relative_position_m': max(position_error), 'max_relative_angle_rad': max(angle_error),
              'dynamic_tf_matched': len(common), 'static_extrinsic_translation': actual,
              'finite_and_unit_quaternion': True, 'monotonic': True,
              'diagnostics': states,
              'static_budget_pass': max(position_error)<.15 and max(angle_error)<.05}
    prefix.with_suffix('.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'diagnostics'}, indent=2))
    if args.stationary and not result['static_budget_pass']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
