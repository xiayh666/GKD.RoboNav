#!/usr/bin/env python3
"""Build a native oracle using the author's actual filter header and functions.
Only ROS/PCL types and driver code are excluded. No filter algebra is rewritten.
The laser callback supplies two known plane constraints without map matching.
"""
from pathlib import Path
import argparse
import subprocess


def function(source, signature):
    begin=source.index(signature)
    brace=source.index('{',begin)
    depth=1; end=brace+1
    while depth:
        depth += (source[end]=='{') - (source[end]=='}'); end+=1
    return source[begin:end]

p=argparse.ArgumentParser()
p.add_argument('output', type=Path)
p.add_argument('--upstream', type=Path, required=True,
               help='External checkout of hku-mars/Point-LIO at the documented commit')
args=p.parse_args()
upstream=args.upstream.resolve()
if not (upstream/'include/common_lib.h').is_file():
    p.error('--upstream must point to the official C++ source checkout')
common=(upstream/'include/common_lib.h').read_text()
types=common[common.index('typedef MTK::vect'):common.index('#define PBWIDTH')]
estimator=(upstream/'src/Estimator.cpp').read_text()
functions='\n'.join(function(estimator, name) for name in [
    'Eigen::Matrix<double, 30, 30> process_noise_cov_output()',
    'Eigen::Matrix<double, 30, 1> get_f_output(',
    'Eigen::Matrix<double, 30, 30> df_dx_output(',
    'void h_model_IMU_output('])
code=r'''
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <cstring>
#include "IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp"
using namespace Eigen;
using namespace std;
using V3D = Eigen::Vector3d;
TYPES
esekfom::esekf<state_input,24,input_ikfom> kf_input;
esekfom::esekf<state_output,30,input_ikfom> kf_output;
double vel_cov=20., gyr_cov_output=.1, acc_cov_output=.1, b_gyr_cov=.0001, b_acc_cov=.0001;
double G_m_s2=9.81, acc_norm=9.81, imu_meas_omg_cov=.01, imu_meas_acc_cov=.1;
bool check_satu=true;
double satu_gyro=35., satu_acc=100.;
V3D angvel_avr,acc_avr;
FUNCTIONS
void planes(state_output &s, Matrix3d, Matrix3d, esekfom::dyn_share_modified<double>& out) {
    const V3D point(1.,.5,.4);
    const V3D world=s.rot*point+s.pos;
    out.h_x=MatrixXd::Zero(2,12); out.z=VectorXd::Zero(2);
    for(int row=0;row<2;++row) {
        V3D n= row==0 ? V3D(0,0,1) : V3D(1,0,0);
        V3D c=s.rot.transpose()*n;
        V3D a=point.cross(c);
        out.h_x.block<1,3>(row,0)=n.transpose();
        out.h_x.block<1,3>(row,3)=a.transpose();
        out.z[row]=(row==0?.4:1.)-n.dot(world);
    }
    out.M_Noise=.01;out.valid=true;
}
void write(double time) {
    auto &s=kf_output.x_;
    Eigen::Quaterniond q(s.rot), e(s.offset_R_L_I);
    cout<<setprecision(17)<<time;
    auto vec=[](auto x){for(int i=0;i<x.size();++i)cout<<","<<x[i];};
    vec(s.pos);vec(q.coeffs());vec(e.coeffs());vec(s.offset_T_L_I);vec(s.vel);vec(s.omg);vec(s.acc);vec(s.gravity);vec(s.bg);vec(s.ba);
    auto cov=kf_output.get_P();for(int j=0;j<30;++j)for(int i=0;i<30;++i)cout<<","<<cov(i,j);
    cout<<"\n";
}
int main() {
    kf_output.init_dyn_share_modified_3h(get_f_output,df_dx_output,planes,h_model_IMU_output);
    kf_output.x_.acc=V3D(0,0,9.81);kf_output.x_.gravity=V3D(0,0,-9.81);
    Matrix<double,30,30> p=Matrix<double,30,30>::Identity()*.01;
    for(int i=6;i<12;++i)p(i,i)=0.;
    for(int i=21;i<24;++i)p(i,i)=.0001;
    for(int i=24;i<30;++i)p(i,i)=.001;
    kf_output.change_P(p);
    auto noise=process_noise_cov_output();input_ikfom input;
    double previous=0.;
    for(int tick=1;tick<=100;++tick) {
        double time=tick*.005;
        // Laser-time nominal propagation does not subdivide the covariance clock.
        double midpoint=time-.0025, dt=midpoint-previous;
        kf_output.predict(dt,noise,input,true,false);
        dt=time-midpoint;kf_output.predict(dt,noise,input,true,false);
        double cov_dt=.005;kf_output.predict(cov_dt,noise,input,false,true);
        angvel_avr=V3D(.02*sin(time*7),-.01*cos(time*3),.3*time);
        acc_avr=V3D(.1*sin(time*5),.04*cos(time*11),9.81);
        if(tick>=60 && tick<80)angvel_avr.z()=40.;
        kf_output.update_iterated_dyn_share_IMU();
        if(tick%4==0)kf_output.update_iterated_dyn_share_modified();
        if(tick==1 || tick==4 || tick==25 || tick==60 || tick==79 || tick==80 || tick==100)write(time);
        previous=time;
    }
}
'''.replace('TYPES',types).replace('FUNCTIONS',functions)
args.output.parent.mkdir(parents=True,exist_ok=True)
cpp=args.output.with_suffix('.cpp');binary=args.output.with_suffix('.bin')
cpp.write_text(code)
subprocess.run(['g++','-O2','-std=c++17','-I/usr/include/eigen3','-I'+str(upstream/'include'),str(cpp),'-o',str(binary)],check=True)
with args.output.open('w') as out:subprocess.run([str(binary)],check=True,stdout=out)
print(args.output)
