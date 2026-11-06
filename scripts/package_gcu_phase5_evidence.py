#!/usr/bin/env python3
"""Losslessly package Phase 05 runtime evidence, retaining failed attempts.

Run after tests finish. Packaging an incomplete run is allowed and cannot turn
its status into a pass. Every archived file is checked before reporting success.
"""
import argparse
import gzip
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile


def package(metric, preview, workspace, output, final_group):
    output.mkdir(parents=True, exist_ok=False)
    members = {}
    for prefix, source in [('metric', metric), ('preview', preview)]:
        for path in sorted(source.rglob('*')):
            if path.is_file() and '__pycache__' not in path.parts:
                members[prefix+'/'+str(path.relative_to(source))] = path.read_bytes()
    preview_workspace=preview.parent
    preview_sources=list(preview_workspace.glob('*.py'))
    preview_sources += [p for p in (preview_workspace/'probe_src').rglob('*') if p.is_file()]
    preview_sources += [p for p in (preview_workspace/'assets').rglob('*') if p.is_file()]
    preview_hashes={}
    for path in preview_sources:
        name=str(path.relative_to(preview_workspace));data=path.read_bytes()
        members['preview_runtime/'+name]=data
        preview_hashes[name]=hashlib.sha256(data).hexdigest()
    members['preview_runtime/source_hashes.json']=json.dumps(preview_hashes,indent=2).encode()
    command=['dpkg-query','-W','ros-humble-ros-gz-bridge','ros-humble-nav2-rotation-shim-controller',
             'libignition-gazebo6','libignition-transport11','libignition-msgs8',
             'python3-shapely','python3-pyproj']
    environment=subprocess.run(command,capture_output=True,text=True)
    members['runtime_final/packages.txt']=(environment.stdout+environment.stderr).encode()
    for name in ['gcu_phase5_geometry_install.log','gcu_phase5_probe_build.log',
                 'gcu_phase5_ordered_build.log','gcu_phase5_runner_v2.py',
                 'gcu_phase5_audit_v1.py','gcu_phase5_audit_wait_v1.py']:
        path=Path('/tmp')/name
        if path.is_file():members['history_support/'+name]=path.read_bytes()
    final = metric/final_group
    frozen = json.loads((final/'frozen_inputs.json').read_text())
    mismatches = []
    for name, digest in frozen.items():
        path = workspace/name
        data = path.read_bytes()
        if hashlib.sha256(data).hexdigest() != digest:
            mismatches.append(name)
        # Source/config snapshots suffice; binary hashes are retained in frozen.
        if path.suffix in ['.py', '.cpp', '.hpp', '.yaml', '.xml', '.sdf', '.pgm'] or path.name=='CMakeLists.txt':
            members['runtime_final/'+name] = data
    members['runtime_final/current_input_verification.json'] = json.dumps({
        'passed': not mismatches, 'checked_files': len(frozen), 'mismatches': mismatches
    }, indent=2).encode()
    # Earlier runtime variants retain source bytes and hashes of omitted binaries.
    source_archives=list(Path('/tmp').glob('gcu_phase5_runtime_variant*.tar.gz'))
    source_archives += [p for p in [Path('/tmp/gcu_phase5_deploy.tar.gz'),
                                    Path('/tmp/gcu_phase5_bridge_fix.tar.gz')] if p.is_file()]
    for source in sorted(source_archives):
        omitted = {}
        with tarfile.open(source) as archive:
            for item in archive:
                if not item.isfile():
                    continue
                data = archive.extractfile(item).read()
                if Path(item.name).suffix in ['.py', '.cpp', '.hpp', '.yaml', '.xml'] or Path(item.name).name=='CMakeLists.txt':
                    members['history_sources/'+source.stem+'/'+item.name] = data
                else:
                    omitted[item.name] = hashlib.sha256(data).hexdigest()
        members['history_sources/'+source.stem+'/omitted_member_hashes.json'] = json.dumps(omitted, indent=2).encode()
    offline=Path('/tmp/gcu_phase5_offline_development.tar.gz')
    if offline.is_file():
        with tarfile.open(offline) as archive:
            for item in archive:
                if item.isfile():members['offline_development/'+item.name]=archive.extractfile(item).read()
    target = output/'runtime_evidence.tar.gz'
    with target.open('xb') as raw:
        with gzip.GzipFile(filename='', fileobj=raw, mode='wb', mtime=0) as compressed:
            with tarfile.open(fileobj=compressed, mode='w') as archive:
                for name, data in sorted(members.items()):
                    item = tarfile.TarInfo(name)
                    item.mode = 0o644
                    item.size = len(data)
                    archive.addfile(item, io.BytesIO(data))
    manifest = {'archive': target.name,
                'sha256': hashlib.sha256(target.read_bytes()).hexdigest(),
                'members': {name: hashlib.sha256(data).hexdigest() for name, data in members.items()}}
    with tarfile.open(target) as archive:
        assert set(archive.getnames()) == set(members)
        for item in archive:
            assert hashlib.sha256(archive.extractfile(item).read()).hexdigest() == manifest['members'][item.name]
    (output/'runtime_evidence_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    groups = json.loads((final/'reliability_gates.json').read_text())
    suite_path = final/'matrix_01/suite.json'
    suite = json.loads(suite_path.read_text()) if suite_path.exists() else {'passed': False, 'trials': []}
    rows = []
    for trial in suite['trials']:
        path = final/'matrix_01'/trial['name']
        run = json.loads((path/'run.json').read_text())
        raw_path = path/'report.json'
        report = json.loads(raw_path.read_text()) if raw_path.exists() else {}
        row = {k: v for k, v in report.items() if k not in [
            'truth', 'localization', 'commands', 'paths', 'contacts', 'display_paths', 'bridge_statuses', 'preflight']}
        row.update(name=trial['name'], runner=run,
                   raw_report='metric/'+str(raw_path.relative_to(metric)))
        rows.append(row)
    result = {'passed': bool(groups['passed'] and suite['passed'] and not mismatches),
              'final_group': final_group, 'reliability': groups,
              'matrix_passed': suite['passed'], 'matrix': rows,
              'scope': 'Metric reliability gates only; offline and live preview have separate reports.'}
    (output/'metric_results.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({'passed': result['passed'], 'archived_files': len(members),
                      'archive_bytes': target.stat().st_size}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--metric-results', type=Path, required=True)
    parser.add_argument('--preview-results', type=Path, required=True)
    parser.add_argument('--runtime-workspace', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--final-group', required=True)
    args = parser.parse_args()
    package(args.metric_results, args.preview_results, args.runtime_workspace,
            args.output, args.final_group)
