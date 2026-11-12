#!/usr/bin/env python3
import argparse
import csv
import hashlib
import json
import re
import statistics
from pathlib import Path
from compare_metrics import metrics

p=argparse.ArgumentParser();p.add_argument('directory',type=Path);args=p.parse_args()
result={'inputs':{},'trajectories':{},'official_compute_timing_ms':{},'common_frame_trajectories':{},'common_frame_counts':{}}
for name in ['stationary','rotation_control','rotation_clipped','floor','recovery']:
    inp=args.directory/(name+'.csv')
    if inp.exists():result['inputs'][name]=hashlib.sha256(inp.read_bytes()).hexdigest()
    for implementation in ['rust','official']:
        path=args.directory/(implementation+'_'+name+'.csv')
        if path.exists():result['trajectories'][implementation+'_'+name]=metrics(path,name.startswith('rotation'))
for name in ['stationary','rotation_control','rotation_clipped','floor','recovery']:
    paths=[args.directory/(prefix+'_'+name+'.csv') for prefix in ['rust','official']]
    if all(path.exists() for path in paths):
        frames=[]
        for path in paths:
            with path.open() as source:
                frames.append({round((float(row['time'])-.5)/.1)-1 for row in csv.DictReader(source)})
        common=frames[0]&frames[1]
        result['common_frame_counts'][name]=len(common)
        for prefix,path in zip(['rust','official'],paths):
            result['common_frame_trajectories'][prefix+'_'+name]=metrics(path,name.startswith('rotation'),common)

for count in [1000,10000,20000]:
    path=args.directory/f'official_timing_{count}.node.log'
    if not path.exists():continue
    # Upstream prints cumulative mean (seconds) as aver_time_consu, so recover
    # individual per-frame durations. Frame 1 in this series is post-bootstrap.
    means=[float(v) for v in re.findall(r'ave total: ([0-9.eE+-]+)',path.read_text())]
    durations=[(v*(i+1)-(means[i-1]*i if i else 0.))*1000 for i,v in enumerate(means)]
    if durations:result['official_compute_timing_ms'][str(count)]={
        'samples':len(durations),'median':statistics.median(durations),
        'min':min(durations),'max':max(durations),'durations':durations}
text=json.dumps(result,indent=2)
(args.directory/'summary.json').write_text(text+'\n')
print(text)
