#!/usr/bin/env python3
"""Compare capacity replay trajectories on the intersection of output scan IDs."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

from compare_metrics import metrics


def frames(path):
    with path.open() as source:
        return {round((float(row['time']) - .5) / .1) - 1
                for row in csv.DictReader(source)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('results', type=Path)
    parser.add_argument('input', type=Path)
    args = parser.parse_args()
    names = ['official_100000', 'official_300', 'official_301',
             'rust_100000', 'rust_300']
    common = set.intersection(*(frames(args.results / (n + '.csv')) for n in names))
    if not common:
        raise ValueError('no common output frames')
    report = dict(input_sha256=hashlib.sha256(args.input.read_bytes()).hexdigest(),
                  common_frames=len(common), position_limit_m=.15, angle_limit_rad=.05,
                  trajectories={})
    for name in names:
        result = metrics(args.results / (name + '.csv'), False, common)
        result['passes_pose_limits'] = (result['max_position_m'] < .15
                                      and result['max_angle_rad'] < .05)
        report['trajectories'][name] = result
    (args.results / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
