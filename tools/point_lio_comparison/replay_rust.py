#!/usr/bin/env python3
"""Replay shared fixtures with current Rust and compare saved official outputs.
The official baseline remains unchanged; check input hashes before comparing.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

root=Path(__file__).resolve().parents[2]
p=argparse.ArgumentParser()
p.add_argument('inputs',type=Path)
p.add_argument('output',type=Path)
p.add_argument('--baseline',type=Path,default=root/'doc/point-lio-rust/results')
args=p.parse_args()
inputs=args.inputs.resolve();output=args.output.resolve();output.mkdir(parents=True,exist_ok=True)
expected=json.loads((args.baseline/'summary.json').read_text())['inputs']
subprocess.run(['cargo','build','--release','-p','point-lio','--example','replay'],cwd=root,check=True)
for name,digest in expected.items():
    data=inputs/(name+'.csv')
    if hashlib.sha256(data.read_bytes()).hexdigest()!=digest:raise ValueError('input changed: '+name)
    command=[str(root/'target/release/examples/replay'),str(data),'--ros-f32']
    if name=='rotation_clipped':command.append('--gyro-saturation=0.3')
    with (output/('rust_'+name+'.csv')).open('w') as dest:subprocess.run(command,stdout=dest,check=True)
    shutil.copyfile(args.baseline/('official_'+name+'.csv'),output/('official_'+name+'.csv'))
    print('replayed',name,flush=True)
with (output/'report.log').open('w') as log:
    subprocess.run(['python3',str(root/'tools/point_lio_comparison/report.py'),str(output)],stdout=log,check=True)
path=output/'summary.json';summary=json.loads(path.read_text());summary['inputs']=expected
path.write_text(json.dumps(summary,indent=2)+'\n')
