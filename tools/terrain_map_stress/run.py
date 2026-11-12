#!/usr/bin/env python3
"""Run release harness without compilation; sample child-process resources on Linux."""
import argparse
import csv
import json
import math
import os
import pathlib
import subprocess
import time


def percentile(values, q):
    values = sorted(values)
    return values[max(0, math.ceil(len(values) * q) - 1)]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--binary', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('args', nargs='+')
    a = p.parse_args()
    out = pathlib.Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    samples = []
    ticks = os.sysconf('SC_CLK_TCK')
    start = time.monotonic()
    with out.with_suffix('.csv').open('w') as stdout, out.with_suffix('.log').open('w') as stderr:
        proc = subprocess.Popen([a.binary, *a.args], stdout=stdout, stderr=stderr)
        while proc.poll() is None:
            try:
                status = dict(line.split(':', 1) for line in pathlib.Path(f'/proc/{proc.pid}/status').read_text().splitlines() if ':' in line)
                # Fields after comm: state(3), ... utime(14), stime(15).
                stat = pathlib.Path(f'/proc/{proc.pid}/stat').read_text().rsplit(')', 1)[1].split()
                cpu_s = (int(stat[11]) + int(stat[12])) / ticks
                samples.append({'wall_s': time.monotonic() - start, 'cpu_s': cpu_s,
                                'rss_kib': int(status.get('VmRSS', '0 kB').split()[0]),
                                'hwm_kib': int(status.get('VmHWM', '0 kB').split()[0]),
                                'threads': int(status.get('Threads', '0'))})
            except (FileNotFoundError, ProcessLookupError):
                pass
            time.sleep(0.2)
    elapsed = time.monotonic() - start
    with out.with_suffix('.resources.csv').open('w') as f:
        w = csv.DictWriter(f, fieldnames=['wall_s', 'cpu_s', 'rss_kib', 'hwm_kib', 'threads'])
        w.writeheader()
        w.writerows(samples)
    rows = list(csv.DictReader(out.with_suffix('.csv').open()))
    rows = [{k: float(v) for k, v in row.items()} for row in rows]
    frames = int(a.args[2])
    frame_ids = sorted(int(r['frame']) for r in rows)
    complete = frame_ids == list(range(frames))
    warm = min(50, max(0, frames // 10))
    steady = [r for r in rows if r['frame'] >= warm]
    metrics = {}
    for key in ['lio_ms', 'export_rebuild_ms', 'admission_ms', 'ack_ms', 'e2e_ms', 'scheduled_ms']:
        values = [r[key] for r in steady]
        metrics[key] = {name: percentile(values, q) for name, q in [('p50', .5), ('p95', .95), ('p99', .99), ('max', 1)]} if values else {}
    usable = [s for s in samples if s['rss_kib'] > 0]
    last = samples[-1] if samples else {'cpu_s': 0, 'wall_s': 1}
    resource = {'sample_period_s': .2, 'peak_rss_mib': max((s['rss_kib'] for s in samples), default=0)/1024,
                'peak_hwm_mib': max((s['hwm_kib'] for s in samples), default=0)/1024,
                'cpu_percent_one_core': last['cpu_s']/max(last['wall_s'], .001)*100,
                'cpu_seconds_observed': last['cpu_s'], 'max_threads': max((s['threads'] for s in samples), default=0)}
    # Compare the final quarter against the preceding quarter, after startup.
    for name, low, high in [('previous_quarter', .5, .75), ('last_quarter', .75, 1.01)]:
        selected = [s['rss_kib']/1024 for s in usable if low*elapsed <= s['wall_s'] < high*elapsed]
        resource[name + '_rss_mib_p50'] = percentile(selected, .5) if selected else None
    result = {'command': [a.binary, *a.args], 'returncode': proc.returncode, 'complete_unique_frames': complete,
              'frames': len(rows), 'warmup_frames_excluded': warm, 'wall_s': elapsed,
              'effective_hz_including_pacing': len(rows)/elapsed, 'metrics': metrics, 'resources': resource,
              'max_pending_receipts': max((r['pending'] for r in rows), default=0),
              'over_100ms_work_frames_steady': sum(r['e2e_ms'] > 100 for r in steady),
              'over_100ms_scheduled_frames_steady': sum(r['scheduled_ms'] > 100 for r in steady),
              'max_position_error_m': max((r['position_error_m'] for r in rows), default=0),
              'max_angle_error_rad': max((r['angle_error_rad'] for r in rows), default=0),
              'final_frame': max(rows, key=lambda r: r['frame']) if rows else None,
              'stderr': out.with_suffix('.log').read_text()}
    out.with_suffix('.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2), flush=True)
    if proc.returncode or not complete:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
