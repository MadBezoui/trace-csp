"""Provenance block stored with every SUMO result: code, inputs, versions and machine load."""
import hashlib
import os
import platform
import sys
from importlib import metadata
from pathlib import Path

import trace_csp

PACKAGES=('ortools','numpy','eclipse-sumo','traci','sumolib')


def sha256(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for chunk in iter(lambda:f.read(1<<20),b''): h.update(chunk)
    return h.hexdigest()


def tree_sha256(root,pattern='*.py'):
    """One digest over the relative paths and contents of every matching file."""
    root=Path(root); h=hashlib.sha256()
    for path in sorted(root.rglob(pattern)):
        if '__pycache__' in path.parts: continue
        h.update(str(path.relative_to(root)).encode()); h.update(sha256(path).encode())
    return h.hexdigest()


def provenance(inputs=(),argv=None):
    versions={}
    for name in PACKAGES:
        try: versions[name]=metadata.version(name)
        except metadata.PackageNotFoundError: versions[name]=None
    return {'trace_csp_version':trace_csp.__version__,'package_sha256':tree_sha256(Path(trace_csp.__file__).parent),
            'python':sys.version.split()[0],'platform':platform.platform(),'packages':versions,
            'inputs':{str(p):sha256(p) for p in inputs if p and Path(p).is_file()},
            'argv':list(sys.argv if argv is None else argv),'cpu_count':os.cpu_count(),'load_average_start':list(os.getloadavg())}
