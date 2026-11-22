"""Isolated floor-pick tests; full-task slip recovery is in full_task.py."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation

from diagnostics import write_contact_diagnostics
from floor_recovery import FloorRecovery
from full_task import FullTaskExperiment, TaskFailure
from pick_nav_reach_env import PickNavReachEnv


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--objects', nargs='+', type=int, default=[2, 4, 5, 6, 14])
    parser.add_argument('--seeds', nargs='+', type=int, default=[0])
    parser.add_argument('--fallen', action='store_true')
    parser.add_argument('--record', action='store_true')
    parser.add_argument('--output', type=Path, default=Path('results/floor_pick'))
    args = parser.parse_args()
    results = []
    for obj in args.objects:
        for seed in args.seeds:
            env = PickNavReachEnv(seed=seed, object_idx=obj, gui=False)
            try:
                # This sets the isolated test's initial condition before its
                # first physics step. Full-task recovery never resets the object.
                orientation = Rotation.from_euler('xyz', [np.pi/2, 0., seed*.37]).as_quat() if args.fallen else [0., 0., 0., 1.]
                p.resetBasePositionAndOrientation(env.object_id, [-1., 0., .16], orientation)
                output = args.output/f'object_{obj:02d}_seed_{seed}'
                experiment = FullTaskExperiment(env, output, record=args.record)
                recovery = FloorRecovery(experiment)
                try:
                    result = recovery.run()
                except TaskFailure as error:
                    result = recovery.finish(False, str(error))
                result.update(object_idx=obj, seed=seed, test_mode='isolated floor initial condition',
                              fallen_initial_orientation=args.fallen,
                              max_arm_floor_penetration_m=max((r['arm_floor_penetration_m'] for r in experiment.trace), default=0.),
                              max_robot_obstacle_penetration_m=max((r['obstacle_penetration_m'] for r in experiment.trace), default=0.))
                results.append(result)
                (output/'floor_result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
                write_contact_diagnostics(output, experiment.trace)
                if experiment.trace:
                    with (output/'trace.csv').open('w', newline='') as stream:
                        writer = csv.DictWriter(stream, fieldnames=experiment.trace[0])
                        writer.writeheader()
                        writer.writerows(experiment.trace)
                if args.record:
                    experiment.capture().save(output/'floor_pick.png')
                    if experiment.frames:
                        experiment.frames[0].save(output/'floor_pick.gif', save_all=True,
                            append_images=experiment.frames[1:], duration=165, loop=0)
                print(json.dumps(result), flush=True)
            finally:
                env.close()
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'summary.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
    with (args.output/'summary.csv').open('w', newline='') as stream:
        fields = ['object_idx', 'seed', 'success', 'reason', 'duration_physics_steps',
                  'stable_hold_control_steps', 'final_floor_clearance_m', 'max_hold_drift_m']
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(results)


if __name__ == '__main__':
    main()
