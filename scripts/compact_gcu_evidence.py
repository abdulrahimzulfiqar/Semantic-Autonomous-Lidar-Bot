#!/usr/bin/env python3
"""Bundle development evidence while keeping final acceptance reports directly readable.

Archive members retain their original relative paths and bytes. Sources are
removed only after every archived member has passed its SHA-256 check.
"""
import argparse
import gzip
import hashlib
import io
import json
from pathlib import Path
import tarfile

ARCHIVE = 'development_history.tar.gz'
MANIFEST = 'development_history_manifest.json'


class EvidenceFiles:
    def __init__(self, root):
        self.root = Path(root)
        self.archive = None

    def read(self, relative):
        path = self.root / relative
        if path.is_file():
            return path.read_bytes()
        if self.archive is None:
            self.archive = tarfile.open(self.root / ARCHIVE, 'r:gz')
        member = self.archive.getmember(relative)
        if not member.isfile():
            raise ValueError('Evidence member is not a regular file: ' + relative)
        return self.archive.extractfile(member).read()

    def close(self):
        if self.archive is not None:
            self.archive.close()


def verify_archive(root):
    root = Path(root)
    manifest = json.loads((root / MANIFEST).read_text())
    assert hashlib.sha256((root / ARCHIVE).read_bytes()).hexdigest() == manifest['sha256']
    with tarfile.open(root / ARCHIVE, 'r:gz') as archive:
        assert set(archive.getnames()) == set(manifest['members'])
        for name, sha in manifest['members'].items():
            assert hashlib.sha256(archive.extractfile(name).read()).hexdigest() == sha, name
    return manifest


def compact(root):
    root = Path(root)
    if (root / ARCHIVE).exists():
        return verify_archive(root)
    paths = sorted(p for p in root.rglob('*') if p.is_file() and (
        p.relative_to(root).parts[0] == 'interrupted_suites' or
        (p.parent == root / 'trials' and p.name.startswith(('development_', 'interrupted_')))))
    if not paths:
        raise ValueError('No development history to compact')
    members = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    target = root / ARCHIVE
    temporary = target.with_suffix('.tmp')
    with temporary.open('xb') as output:
        with gzip.GzipFile(filename='', fileobj=output, mode='wb', mtime=0) as compressed:
            with tarfile.open(fileobj=compressed, mode='w') as archive:
                for path in paths:
                    data = path.read_bytes()
                    info = tarfile.TarInfo(str(path.relative_to(root)))
                    info.size = len(data)
                    info.mode = 0o644
                    archive.addfile(info, io.BytesIO(data))
    temporary.rename(target)
    manifest = {'archive': ARCHIVE, 'sha256': hashlib.sha256(target.read_bytes()).hexdigest(),
                'scope': 'Development reports/logs and earlier suite metadata; final acceptance files remain unpacked',
                'members': members}
    (root / MANIFEST).write_text(json.dumps(manifest, indent=2) + '\n')
    verify_archive(root)
    results_path = root / 'results.json'
    results = json.loads(results_path.read_text())
    results['development_history_archive'] = {'path': ARCHIVE, 'manifest': MANIFEST,
        'member_paths': 'Original relative paths, including historical full_report references'}
    results_path.write_text(json.dumps(results, indent=2) + '\n')
    for path in paths:
        path.unlink()
    previous = root / 'interrupted_suites'
    if previous.exists():
        for path in sorted(previous.rglob('*'), reverse=True):
            if path.is_dir():
                path.rmdir()
        previous.rmdir()
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', required=True, type=Path)
    parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args()
    manifest = verify_archive(args.evidence) if args.verify_only else compact(args.evidence)
    print('Verified history archive:', len(manifest['members']), 'files')
