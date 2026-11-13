"""Read only explicitly admitted, versioned simulation mission fixtures."""
import hashlib,json,math
from pathlib import Path


def load_fixture(path):
    path=Path(path);data=json.loads(path.read_text())
    if not isinstance(data.get('name'),str) or not data['name']:raise ValueError('missing_fixture_name')
    for name in ['spawn','goal']:
        if len(data.get(name,[]))!=3 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in data[name]):raise ValueError('invalid_fixture_pose')
    passages=data.get('expected_passages',[]);areas=data.get('expected_areas',[])
    if not passages or len(set(passages))!=len(passages) or not all(isinstance(v,str) for v in passages) or len(areas)!=len(passages)+1:raise ValueError('invalid_fixture_topology')
    if not data.get('mask_validation',{}).get('valid'):raise ValueError('fixture_padding_not_validated')
    if not 0<data.get('simulation_timeout_s',0)<=180 or not 0<data.get('checker_wall_timeout_s',0)<=850:raise ValueError('invalid_fixture_deadlines')
    if not data.get('input_sha256'):raise ValueError('missing_fixture_inputs')
    if data.get('local_costmap_frame','odom') not in ['map','odom']:raise ValueError('unsupported_local_costmap_frame')
    if 'local_costmap_publish_hz' in data and (not isinstance(data['local_costmap_publish_hz'],(int,float)) or not math.isfinite(data['local_costmap_publish_hz']) or not 0<data['local_costmap_publish_hz']<=20):raise ValueError('invalid_costmap_publish_rate')
    if 'amcl_sigma_hit' in data and (not isinstance(data['amcl_sigma_hit'],(int,float)) or not math.isfinite(data['amcl_sigma_hit']) or not .01<=data['amcl_sigma_hit']<=1.):raise ValueError('invalid_amcl_sigma_hit')
    if 'next_mission' in data:
        later=data['next_mission']
        if not isinstance(later,dict) or 'next_mission' in later:raise ValueError('only_two_consecutive_missions_supported')
        validate_sequence_leg(data,later)
    return data,hashlib.sha256(path.read_bytes()).hexdigest()


def verify_runtime_sources(fixture,directory):
    for name,digest in fixture.get('execution_source_sha256',{}).items():
        if Path(name).name!=name or hashlib.sha256((Path(directory)/name).read_bytes()).hexdigest()!=digest:
            raise ValueError('fixture_execution_source_changed:'+name)
    if 'next_mission' in fixture:verify_runtime_sources(fixture['next_mission'],directory)


def validate_sequence_leg(first,later):
    for key in ['name','spawn','goal','expected_passages','expected_areas','mask_validation','simulation_timeout_s','checker_wall_timeout_s','input_sha256']:
        if key not in later:raise ValueError('incomplete_next_mission:'+key)
    if not isinstance(later['name'],str) or not later['name'] or later['name']==first['name'] or later['input_sha256']!=first['input_sha256'] or later.get('local_costmap_frame','odom')!=first.get('local_costmap_frame','odom') or later.get('amcl_sigma_hit')!=first.get('amcl_sigma_hit'):
        raise ValueError('consecutive_identity_map_or_frame_changed')
    if later['spawn']!=first['goal'] or not later['mask_validation'].get('valid'):
        raise ValueError('next_mission_offline_admission_invalid')
    if len(later['goal'])!=3 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in later['goal']):raise ValueError('invalid_next_goal')
    if not 0<later['simulation_timeout_s']<=180 or not 0<later['checker_wall_timeout_s']<=850:raise ValueError('invalid_next_deadlines')
    if not later['expected_passages'] or len(set(later['expected_passages']))!=len(later['expected_passages']) or len(later['expected_areas'])!=len(later['expected_passages'])+1:raise ValueError('invalid_next_topology')


def validate_current_contract(contract,snapshot):
    """Recheck the freshly localized route; a fixture proves only its own start."""
    from gcu_plan_validation import validate_path
    result=validate_path(snapshot.combined,snapshot.resolution,snapshot.origin,contract.path,snapshot.radius+snapshot.padding)
    if not result['valid']:raise ValueError('current_contract_padding_not_validated:'+json.dumps(result,sort_keys=True))
    return result
