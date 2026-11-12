#!/usr/bin/env python3
"""Export the exact formulas used by tests/usability.rs into a shared CSV.
Coordinates/timestamps are f64 round-trip text. ROS PointCloud2 narrows xyz/time
into the author's float32 input fields; the Rust comparator can also quantize.
"""
import argparse
import math
from pathlib import Path


def points(count=999, floor_only=False):
    for i in range(count):
        a = ((i // 3) % 37) * .15 - 2.7
        b = ((i // 111) % 37) * .15 - 2.7
        yield ((a, b, -1.5), (3., a, b), (a, 3., b))[0 if floor_only else i % 3]


def export(path, scenario):
    rotating = scenario.startswith('rotation')
    timing = scenario.startswith('timing_')
    frames = 6 if timing else (25 if scenario == 'recovery' else (30 if rotating or scenario == 'floor' else 600))
    count = int(scenario.split('_')[1]) if timing else (999 if scenario == 'stationary' else 1665)
    with path.open('w') as out:
        def row(*items):
            out.write(','.join(format(x, '.17g') if isinstance(x, float) else str(x) for x in items) + '\n')
        for tick in range(100):
            row('I', tick * .005, 0., 0., 0., 0., 0., 9.81)
        for frame in range(frames):
            first = 100 if frame == 0 else 101 + frame * 20
            for tick in range(first, 121 + frame * 20):
                t = tick * .005
                if timing:
                    row('I', t, 0., 0., 0., 0., 0., 9.81)
                elif scenario in ('floor', 'recovery'):
                    ax = .2 if scenario == 'floor' and frame > 0 else (.1 if scenario == 'recovery' and frame >= 5 else 0.)
                    row('I', t, 0., 0., 0., ax, 0., 9.81)
                elif rotating:
                    wz = .6 * (1. - math.exp(-2. * (t - .5)))
                    if scenario == 'rotation_clipped': wz = min(wz, .3)
                    row('I', t, 0., 0., wz, 0., 0., 9.81)
                else:
                    row('I', t, math.sin(t*13.)*.001, math.cos(t*17.)*.001,
                        math.sin(t*19.)*.001, math.sin(t*11.)*.02,
                        math.cos(t*7.)*.02, 9.81+math.sin(t*23.)*.02)
            start = .5 + frame * .1
            for i, (x,y,z) in enumerate(points(count, scenario == 'floor')):
                if scenario == 'recovery' and 5 <= frame < 10: break
                offset = .099 * i / count
                if rotating:
                    t = max(0., start+offset-.5)
                    yaw = .6 * (t - (1.-math.exp(-2.*t))/2.)
                    x,y = math.cos(yaw)*x + math.sin(yaw)*y, -math.sin(yaw)*x + math.cos(yaw)*y
                elif scenario == 'stationary':
                    t = frame*.31 + i*1.73
                    x += math.sin(t)*.01
                    y += math.sin(t*1.3)*.01
                    z += math.cos(t*1.7)*.01
                row('P', start+offset, x,y,z,1.)
            row('S', start, start+.1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for name in ['stationary', 'rotation_control', 'rotation_clipped', 'floor', 'recovery', 'timing_1000', 'timing_10000', 'timing_20000']:
        dest = args.output / (name+'.csv')
        export(dest, name)
        print(dest)
