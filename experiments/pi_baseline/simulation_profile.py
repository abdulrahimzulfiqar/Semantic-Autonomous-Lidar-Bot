"""Explicit two-host simulation profile; accepted loopback guard is unchanged."""
import hashlib
import ipaddress
import json
from pathlib import Path


def simulation_guard(execute, environment):
    if not execute or environment.get('ROS_DOMAIN_ID') != '218':
        raise ValueError('phase9_split_simulation_required')
    path = environment.get('EDGEOSMAG_PHASE9_PROFILE')
    if not path:
        raise ValueError('phase9_profile_required')
    profile = json.loads(Path(path).read_text())
    replay = profile.get('local_stack_replay') is True
    expected_scope = 'gcu_phase9_local_stack_replay' if replay else 'gcu_phase9_processor_in_loop'
    expected_command = '/phase9/replay_commands' if replay else '/phase9/sim_cmd_vel'
    if environment.get('ROS_LOCALHOST_ONLY') != ('1' if replay else '0'):
        raise ValueError('phase9_wrong_network_scope')
    if (profile.get('scope') != expected_scope or
            profile.get('command_topic') != expected_command or
            profile.get('domain') != 218 or profile.get('role') != 'navigation_pi' or
            not profile.get('partition', '').startswith('gcu_phase9_') or
            profile.get('partition') != environment.get('IGN_PARTITION') or
            environment.get('RMW_IMPLEMENTATION') != 'rmw_cyclonedds_cpp'):
        raise ValueError('phase9_profile_mismatch')
    if not replay:
        addresses = [ipaddress.ip_address(profile[k]) for k in ['pi_ip', 'simulator_ip']]
        if addresses[0] == addresses[1] or any(v.version != 4 or not v.is_private or v.is_loopback for v in addresses):
            raise ValueError('phase9_private_lan_required')
    uri = environment.get('CYCLONEDDS_URI', '')
    if not uri.startswith('file://'):
        raise ValueError('phase9_pinned_dds_profile_required')
    if hashlib.sha256(Path(uri[7:]).read_bytes()).hexdigest() != profile.get('dds_sha256'):
        raise ValueError('phase9_dds_profile_changed')
    return profile
