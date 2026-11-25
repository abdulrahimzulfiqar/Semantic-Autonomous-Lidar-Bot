"""Stage an isolated Pi full-stack replay from a successful simulator capture."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

if __package__:
    from .replay_stack import load_clip
else:
    from replay_stack import load_clip


def prepare(template, destination, remote_root, recording, begin_ns, end_ns, *, trace_actor=False):
    _, _, counts = load_clip(recording, begin_ns, end_ns)
    import yaml
    shutil.copytree(template, destination)
    path = destination/'assets/nav2_sensor_constraints.yaml'
    config = yaml.safe_load(path.read_text())
    navigator = config['bt_navigator']['ros__parameters']
    for key in ['default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml']:
        filename = Path(navigator[key]).name
        navigator[key] = str(remote_root/'assets'/filename)
    path.write_text(yaml.safe_dump(config, sort_keys=False))
    source = Path(__file__).parent
    for name in ['run_role.py', 'simulation_profile.py', 'worker.py', 'dispatch_admission.py',
                 'executor_groups.py', 'sensor_recording.py', 'replay_stack.py', 'callback_trace.py']:
        target = {'simulation_profile.py':'profile_phase9.py','worker.py':'worker_phase9.py'}.get(name,name)
        shutil.copyfile(source/name, destination/target)
    shutil.copyfile(source.parents[1]/'scripts/wait_gcu_localization.py',
                    destination/'wait_gcu_localization.py')
    shutil.copyfile(source/'resources.py', destination/'resources_phase9.py')
    shutil.copyfile(recording,destination/'sensor_recording.json.gz')
    dds = destination/'dds.xml'
    dds.write_text('<CycloneDDS><Domain id="any"><General><AllowMulticast>false</AllowMulticast>'
                   '</General><Discovery><ParticipantIndex>auto</ParticipantIndex>'
                   '<MaxAutoParticipantIndex>32</MaxAutoParticipantIndex><Peers>'
                   '<Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>\n')
    profile_path = destination/'profile.json'
    profile = json.loads(profile_path.read_text())
    profile.update(scope='gcu_phase9_local_stack_replay', command_topic='/phase9/replay_commands',
                   partition='gcu_phase9_'+remote_root.name, local_stack_replay=True,
                   dedicated_tf=True, capture_inputs=False, reliable_scan_link=False,
                   stationary_clip_ns=[begin_ns,end_ns], dds_sha256=digest(dds))
    if trace_actor:
        profile.update(timing_trace=True, independent_arrival_probe=False)
    profile_path.write_text(json.dumps(profile, indent=2)+'\n')
    manifest_path = destination/'phase9_manifest.json'
    hashes = {str(path.relative_to(destination)):digest(path) for path in destination.rglob('*')
              if path.is_file() and path != manifest_path}
    manifest_path.write_text(json.dumps(dict(files_sha256=hashes, clip_inputs=counts),indent=2)+'\n')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['template','destination','remote-root','recording']:
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--start-ns',type=int,required=True)
    parser.add_argument('--end-ns',type=int,required=True)
    parser.add_argument('--trace-actor',action='store_true')
    args = parser.parse_args()
    prepare(args.template,args.destination,args.remote_root,args.recording,args.start_ns,args.end_ns,
            trace_actor=args.trace_actor)
