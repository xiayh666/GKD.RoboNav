#!/usr/bin/env python3
"""Run cases sequentially to avoid benchmark contention inside this suite."""
import json
import pathlib
import platform
import subprocess
import sys
import time

root = pathlib.Path('doc/terrain-map/results/chain-stress')
root.mkdir(parents=True, exist_ok=True)
binary = '/tmp/terrain-map-stress-target/release/examples/chain_stress'
metadata = {'utc_started': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            'platform': platform.platform(), 'cpuinfo': pathlib.Path('/proc/cpuinfo').read_text().split('\n\n')[0],
            'meminfo': pathlib.Path('/proc/meminfo').read_text(),
            'cpu_max': pathlib.Path('/sys/fs/cgroup/cpu.max').read_text(),
            'memory_max': pathlib.Path('/sys/fs/cgroup/memory.max').read_text(),
            'rustc': subprocess.check_output(['/home/ubuntu/.cargo/bin/rustc', '-Vv'], text=True),
            'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
            'working_tree': subprocess.check_output(['git', 'status', '--short'], text=True)}
(root/'environment.json').write_text(json.dumps(metadata, indent=2)+'\n')
cases = [
    ('chain-1000-10hz', ['chain', '1000', '600', '10', '2']),
    ('chain-10000-10hz', ['chain', '10000', '600', '10', '2']),
    ('chain-20000-10hz', ['chain', '20000', '600', '10', '2']),
    ('soak-10000-moving-300s', ['chain', '10000', '3000', '10', '2', 'moving']),
    ('direct-10000-1thread', ['direct', '10000', '1000', '0', '1']),
    ('direct-10000-2threads', ['direct', '10000', '1000', '0', '2']),
    ('direct-10000-4threads', ['direct', '10000', '1000', '0', '4']),
    ('direct-20000-2threads', ['direct', '20000', '1000', '0', '2']),
    ('burst-20000', ['burst', '20000', '1000', '0', '2']),
    ('lifecycle-10000-moving-window', ['lifecycle', '10000', '3000', '0', '2']),
    ('capacity-200000', ['capacity', '10000', '120', '0', '2']),
]
for name, args in cases:
    print(f'START {name}', flush=True)
    case_binary = binary if args[0] != 'capacity' else '/tmp/terrain-map-stress-target/release/examples/capacity_stress'
    with (root/(name+'.runner.log')).open('w') as output:
        completed = subprocess.run([sys.executable, 'tools/terrain_map_stress/run.py', '--binary', case_binary,
                                    '--out', str(root/name), *args], stdout=output, stderr=subprocess.STDOUT)
    if completed.returncode:
        print(f'FAILED {name}; see {root/name}.runner.log', flush=True)
        raise SystemExit(completed.returncode)
    result = json.loads((root/(name+'.json')).read_text())
    print(f"DONE {name} wall={result['wall_s']:.1f}s e2e_p99={result['metrics']['e2e_ms']['p99']:.3f}ms scheduled_max={result['metrics']['scheduled_ms']['max']:.3f}ms rss_peak={result['resources']['peak_hwm_mib']:.1f}MiB", flush=True)
print('ALL_CASES_FINISHED', flush=True)
