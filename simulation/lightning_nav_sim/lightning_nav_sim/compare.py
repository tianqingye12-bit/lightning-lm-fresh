"""Reject A/B comparisons that changed truth world, scenario, Nav2 config or goal criteria."""
import argparse
import json
from pathlib import Path


def compare(first,second):
    a,b=[json.loads((Path(p)/'summary.json').read_text()) for p in (first,second)]
    for report in (a,b):
        if not report.get('complete') or report.get('scope')!='2d_navigation_map_evaluation_amcl':
            raise ValueError('Only completed navigation trials can be compared')
    for key in ('world_hash','protocol_hash','navigation_config_hash','goal_timeout_s','wall_timeout_s',
                'truth_tolerance_m','truth_yaw_tolerance_rad','goals_planned'):
        if a.get(key) is None or a[key] != b.get(key):
            raise ValueError(f'Incompatible trials: {key} differs or is missing')
    keys=('success','success_rate','goals_attempted','elapsed_s','path_length_m','collision_contacts',
          'position_rmse_m','yaw_rmse_rad','localization_samples','localization_coverage')
    return dict(scope=a['scope'],same_navigation_map=a['navigation_map_hash']==b['navigation_map_hash'],
                first={k:a.get(k) for k in keys},second={k:b.get(k) for k in keys},
                caveat='A shorter failed run is not a better path. Repeat trials; AMCL/planning are not deterministic.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--first',required=True); parser.add_argument('--second',required=True)
    args=parser.parse_args()
    try: print(json.dumps(compare(args.first,args.second),indent=2))
    except (OSError,ValueError,KeyError) as exc: parser.exit(1,str(exc)+'\n')
