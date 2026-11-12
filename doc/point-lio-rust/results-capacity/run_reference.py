#!/usr/bin/env python3
"""Run the unmodified official ROS node on the shared CSV event stream.
Wall timing includes ROS delivery/ack; do not equate it to compute-only timing.
"""
import argparse
import csv
import json
import math
import os
import signal
import subprocess
import threading
import time
from pathlib import Path
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header


def params(gyro_limit):
    # Align physical units, extrinsics, gate/noise values and point selection.
    return {
        'common/lid_topic': '/comparison/cloud', 'common/imu_topic': '/comparison/imu',
        'common/con_frame': False, 'common/cut_frame': False,
        'preprocess/lidar_type': 2, 'preprocess/scan_line': 16,
        'preprocess/timestamp_unit': 0, 'preprocess/blind': .1,
        'mapping/det_range': 100., 'mapping/imu_en': True,
        'mapping/extrinsic_est_en': False, 'mapping/imu_time_inte': .005,
        'mapping/lidar_time_inte': .1, 'mapping/satu_acc': 100.,
        'mapping/satu_gyro': gyro_limit, 'mapping/acc_norm': 9.81,
        'mapping/lidar_meas_cov': .01, 'mapping/vel_cov': 20.,
        'mapping/gyr_cov_output': .1, 'mapping/acc_cov_output': .1,
        'mapping/b_gyr_cov': .0001, 'mapping/b_acc_cov': .0001,
        'mapping/imu_meas_omg_cov': .01, 'mapping/imu_meas_acc_cov': .1,
        'mapping/plane_thr': .1, 'mapping/match_s': 81.,
        'mapping/gravity': [0.,0.,-9.81], 'mapping/gravity_init': [0.,0.,-9.81],
        'mapping/extrinsic_T': [0.,0.,0.], 'mapping/extrinsic_R': [1.,0.,0.,0.,1.,0.,0.,0.,1.],
        'mapping/ivox_grid_resolution': .3, 'ivox_nearby_type': 26,
        'use_imu_as_input': False, 'prop_at_freq_of_imu': True, 'check_satu': True,
        'init_map_size': 10, 'point_filter_num': 1, 'space_down_sample': False,
        'filter_size_surf': .3, 'filter_size_map': .3,
        'odometry/publish_odometry_without_downsample': False,
        'publish/path_en': False, 'publish/scan_publish_en': False,
        'publish/scan_bodyframe_pub_en': False, 'runtime_pos_log_enable': False,
        'pcd_save/pcd_save_en': False,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--gyro-limit', type=float, default=35.)
    parser.add_argument('--timing', action='store_true')
    parser.add_argument('--capacity', type=int, default=1000000)
    args = parser.parse_args()
    rospy.init_node('comparison_driver', anonymous=True, disable_signals=True)
    settings = params(args.gyro_limit)
    settings['comparison_capacity'] = args.capacity
    settings['runtime_pos_log_enable'] = args.timing
    for key, value in settings.items(): rospy.set_param('/laserMapping/'+key, value)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix('.params.json').write_text(json.dumps(settings, indent=2))
    condition = threading.Condition()
    observed = [-1.]
    with args.output.open('w') as out, args.output.with_suffix('.node.log').open('w') as log:
        writer = csv.writer(out)
        writer.writerow(['time','x','y','z','qx','qy','qz','qw'])
        def callback(msg):
            p = msg.pose.pose.position; q = msg.pose.pose.orientation
            with condition:
                writer.writerow([msg.header.stamp.to_sec(),p.x,p.y,p.z,q.x,q.y,q.z,q.w])
                out.flush(); observed[0] = msg.header.stamp.to_sec(); condition.notify_all()
        def map_callback(msg):
            with condition:
                observed[0] = msg.header.stamp.to_sec(); condition.notify_all()
        rospy.Subscriber('/aft_mapped_to_init', Odometry, callback, queue_size=1000)
        rospy.Subscriber('/Laser_map', PointCloud2, map_callback, queue_size=10)
        imu_pub = rospy.Publisher('/comparison/imu', Imu, queue_size=10000)
        cloud_pub = rospy.Publisher('/comparison/cloud', PointCloud2, queue_size=1)
        node = subprocess.Popen(['rosrun','point_lio','pointlio_mapping','__name:=laserMapping'], stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic()+30
            while not (imu_pub.get_num_connections() and cloud_pub.get_num_connections()):
                if node.poll() is not None: raise RuntimeError('official node exited at startup; see node.log')
                if time.monotonic()>deadline: raise RuntimeError('timeout waiting for official subscriptions')
                time.sleep(.05)
            fields = [PointField('x',0,PointField.FLOAT32,1),PointField('y',4,PointField.FLOAT32,1),
                PointField('z',8,PointField.FLOAT32,1),PointField('intensity',12,PointField.FLOAT32,1),
                PointField('time',16,PointField.FLOAT32,1),PointField('ring',20,PointField.UINT16,1)]
            points=[]; pending_imus=[]; frames=0
            with args.input.open() as src:
                for row in csv.reader(src):
                    if row[0]=='I': pending_imus.append([float(v) for v in row[1:]])
                    elif row[0]=='P': points.append([float(v) for v in row[1:]])
                    elif row[0]=='S':
                        start,end=map(float,row[1:])
                        # Deliver all IMU first; allow callbacks to drain before cloud.
                        for t,gx,gy,gz,ax,ay,az in pending_imus:
                            m=Imu();m.header.stamp=rospy.Time.from_sec(t)
                            m.angular_velocity.x=gx;m.angular_velocity.y=gy;m.angular_velocity.z=gz
                            m.linear_acceleration.x=ax;m.linear_acceleration.y=ay;m.linear_acceleration.z=az
                            imu_pub.publish(m)
                        pending_imus.clear(); time.sleep(.01)
                        cloud=point_cloud2.create_cloud(Header(stamp=rospy.Time.from_sec(start),frame_id='lidar'), fields,
                            [(x,y,z,intensity,t-start,0) for t,x,y,z,intensity in points])
                        if not points:
                            cloud_pub.publish(cloud)
                            frames += 1
                            continue
                        expected=max(p[0] for p in points)
                        cloud_pub.publish(cloud); points.clear()
                        deadline=time.monotonic()+15
                        with condition:
                            while observed[0]<expected-.00001:
                                if node.poll() is not None: raise RuntimeError('official node exited during replay')
                                remaining=deadline-time.monotonic()
                                if remaining<=0: raise RuntimeError(f'no official output for frame {frames}, expected {expected}, observed {observed[0]}')
                                condition.wait(min(remaining,.1))
                        frames+=1
                        if frames%100==0: print(f'official frames={frames}',flush=True)
            print(f'official frames={frames}; trajectory={args.output}',flush=True)
        finally:
            node.send_signal(signal.SIGINT)
            try:node.wait(timeout=10)
            except subprocess.TimeoutExpired:node.kill();node.wait()

if __name__=='__main__':main()
