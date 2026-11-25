#!/usr/bin/env python3
"""Extract only estimator inputs from a pinned Phase 08 capture for local replay."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile
import numpy as np
from replay_timing import validate_recording


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture',type=Path,required=True)
    p.add_argument('--destination',type=Path,required=True)
    a=p.parse_args()
    if a.destination.exists():p.error('fresh_destination_required')
    source=Path(__file__).parent
    with tarfile.open(a.capture) as t:
        prefix='results/paired_capture_clean_01/'
        capture=json.load(t.extractfile(prefix+'capture.json'))
        raw=t.extractfile(prefix+'observations.npz').read()
        if hashlib.sha256(raw).hexdigest()!=capture['files_sha256']['observations.npz']:
            raise ValueError('recording_checksum_mismatch')
        import io
        with np.load(io.BytesIO(raw),allow_pickle=False) as data:validate_recording(data)
        a.destination.mkdir(parents=True)
        (a.destination/'observations.npz').write_bytes(raw)
        for name in ['amcl.yaml','gcu_ground_floor.yaml','gcu_ground_floor.pgm']:
            (a.destination/name).write_bytes(t.extractfile('assets/'+name).read())
    for name in ['local_replay.py','replay_timing.py','resources.py']:
        shutil.copyfile(source/name,a.destination/name)
    (a.destination/'dds.xml').write_text('''<CycloneDDS><Domain id="any"><General>
<AllowMulticast>false</AllowMulticast>
</General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>32</MaxAutoParticipantIndex>
<Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>\n''')
    manifest=dict(scope='Known-start AMCL-only replay of recorded scans and interpolated SE(2) odometry; no evaluator data extracted',
        source_capture_archive_sha256=hashlib.sha256(a.capture.read_bytes()).hexdigest(),
        original_capture_manifest=capture['files_sha256'],
        recording_sha256=hashlib.sha256(raw).hexdigest(),
        files_sha256={v.name:hashlib.sha256(v.read_bytes()).hexdigest() for v in a.destination.iterdir() if v.is_file()})
    (a.destination/'replay_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(dict(destination=str(a.destination),recording_sha256=manifest['recording_sha256'])))


if __name__=='__main__':main()
