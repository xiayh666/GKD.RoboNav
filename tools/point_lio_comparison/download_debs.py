#!/usr/bin/env python3
"""Fetch apt's signed-index-selected packages concurrently; apt verifies again."""
import concurrent.futures
import hashlib
from pathlib import Path
import shlex
import sys
import time
import urllib.request

jobs=[]
for line in Path(sys.argv[1]).read_text().splitlines():
    if not line.startswith("'"):continue
    url,name,size,digest=shlex.split(line)
    if Path(name).name != name:raise ValueError('invalid apt filename')
    jobs.append((url,name,int(size),digest))

def fetch(job):
    url,name,size,digest=job
    path=Path('/var/cache/apt/archives')/name
    for attempt in range(3):
        try:
            with urllib.request.urlopen(url,timeout=120) as response:
                data=response.read()
            if len(data)!=size:raise ValueError('package size mismatch')
            algorithm,expected=digest.split(':',1)
            algorithm={'SHA256':'sha256','MD5Sum':'md5','SHA512':'sha512'}[algorithm]
            if hashlib.new(algorithm,data).hexdigest()!=expected:raise ValueError('package hash mismatch')
            path.write_bytes(data)
            return name
        except Exception:
            if attempt==2:raise
            time.sleep(1+attempt)

with concurrent.futures.ThreadPoolExecutor(max_workers=12) as pool:
    for n,_ in enumerate(pool.map(fetch,jobs),1):
        if n%40==0 or n==len(jobs):print(f'verified packages {n}/{len(jobs)}',flush=True)
