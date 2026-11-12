#!/usr/bin/env python3
"""Report errors against analytic truth at each implementation's output timestamp."""
import argparse
import csv
import json
import math
from pathlib import Path


def metrics(path, rotating, frames=None):
    count=0; max_p=0.; max_a=0.; sum_p2=0.; sum_a2=0.; first=None; last=None
    with path.open() as src:
        for row in csv.DictReader(src):
            t=float(row['time'])
            if frames is not None and round((t-.5)/.1)-1 not in frames:continue
            p=[float(row[k]) for k in ['x','y','z']]
            q=[float(row[k]) for k in ['qx','qy','qz','qw']]
            if not all(math.isfinite(v) for v in [t]+p+q):raise ValueError('non-finite trajectory')
            yaw=0.
            if rotating:
                tau=max(0.,t-.5)
                yaw=.6*(tau-(1.-math.exp(-2.*tau))/2.)
            norm=math.sqrt(sum(v*v for v in q))
            if norm<1e-12:raise ValueError('invalid quaternion')
            dot=abs((q[2]*math.sin(yaw/2.)+q[3]*math.cos(yaw/2.))/norm)
            a=2.*math.acos(min(1.,dot)); position=math.sqrt(sum(v*v for v in p))
            max_p=max(max_p,position);max_a=max(max_a,a)
            sum_p2+=position*position;sum_a2+=a*a;count+=1
            first=t if first is None else first;last=t
    if not count:raise ValueError('empty trajectory')
    return dict(samples=count, first_time=first,last_time=last,max_position_m=max_p,
        rmse_position_m=math.sqrt(sum_p2/count),max_angle_rad=max_a,rmse_angle_rad=math.sqrt(sum_a2/count))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('paths',nargs='+',type=Path);p.add_argument('--rotating',action='store_true');args=p.parse_args()
    for path in args.paths:print(json.dumps(dict(path=str(path),**metrics(path,args.rotating)),ensure_ascii=False))
