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


def git_state(repo):
    """Commit of the repository read from .git without the git program, and its dirty state
    when git is available (None when it cannot be determined)."""
    repo=Path(repo); head=repo/'.git'/'HEAD'
    if not head.is_file(): return {'revision':None,'dirty':None}
    ref=head.read_text().strip(); revision=ref
    if ref.startswith('ref: '):
        name=ref[5:]; loose=repo/'.git'/name; revision=None
        if loose.is_file(): revision=loose.read_text().strip()
        elif (repo/'.git'/'packed-refs').is_file():
            for line in (repo/'.git'/'packed-refs').read_text().splitlines():
                if line.endswith(' '+name): revision=line.split()[0]
    try:
        import subprocess
        out=subprocess.run(['git','-C',str(repo),'status','--porcelain'],capture_output=True,text=True,timeout=60)
        dirty=bool(out.stdout.strip()) if out.returncode==0 else None
    except Exception: dirty=None
    return {'revision':revision,'dirty':dirty}


def manifest(paths,relative_to):
    """SHA-256 of every file under the given files or folders, keyed by path relative to relative_to."""
    out={}
    for path in paths:
        path=Path(path)
        files=[path] if path.is_file() else sorted(f for f in path.rglob('*') if f.is_file()) if path.is_dir() else []
        for f in files: out[str(f.relative_to(relative_to))]=sha256(f)
    return out


def provenance(inputs=(),argv=None,benchmark=(),benchmark_root=None):
    """inputs: files hashed by path. benchmark: files or folders of the external benchmark the run
    reads, hashed into a manifest relative to benchmark_root."""
    versions={}
    for name in PACKAGES:
        try: versions[name]=metadata.version(name)
        except metadata.PackageNotFoundError: versions[name]=None
    package=Path(trace_csp.__file__).parent; repo=package.parents[1]
    return {'trace_csp_version':trace_csp.__version__,'package_sha256':tree_sha256(package),
            'scripts_sha256':tree_sha256(repo/'scripts') if (repo/'scripts').is_dir() else None,'repository':git_state(repo),
            'python':sys.version.split()[0],'platform':platform.platform(),'packages':versions,
            'inputs':{str(p):sha256(p) for p in inputs if p and Path(p).is_file()},
            'benchmark_manifest':manifest(benchmark,benchmark_root) if benchmark else {},
            'argv':list(sys.argv if argv is None else argv),'cpu_count':os.cpu_count(),'load_average_start':list(os.getloadavg())}
