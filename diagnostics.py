"""Contact evidence collected without altering simulator dynamics."""
import json
from pathlib import Path

import numpy as np


def contact_diagnostics(rows):
    phases = {}
    for row in rows:
        phases.setdefault(row['phase'], []).append(row)
    summary = []
    for phase, samples in phases.items():
        utilization = []
        for row in samples:
            for finger in (1, 2):
                force = row.get(f'finger_{finger}_force_n', 0.)
                friction = row.get(f'finger_{finger}_friction', 0.)
                if force > .1 and friction > 0:
                    utilization.append(row[f'finger_{finger}_tangent_n']/(friction*force))
        summary.append(dict(phase=phase, observations=len(samples),
            first_physics_step=samples[0]['physics_steps'], last_physics_step=samples[-1]['physics_steps'],
            bilateral_fraction=float(np.mean([r['bilateral_contact'] for r in samples])),
            median_normal_force_n=[float(np.median([r.get(f'finger_{i}_force_n', 0.) for r in samples]))
                                   for i in (1, 2)],
            peak_friction_utilization=max(utilization, default=None),
            max_relative_translation_m=max(r['relative_drift'] for r in samples),
            max_relative_rotation_rad=max(r.get('relative_rotation_rad', 0.) for r in samples),
            max_gripper_rotation_rad=max(r.get('gripper_rotation_rad', 0.) for r in samples)))
    carrying = [r for r in rows if r['phase'] in ('tuck', 'retreat', 'navigate', 'reach', 'goal hold')]
    warning = None
    lost = None
    streak = 0
    for row in carrying:
        if warning is None and (row['relative_drift'] > .01 or row.get('relative_rotation_rad', 0.) > np.deg2rad(15)):
            warning = dict(physics_steps=row['physics_steps'], phase=row['phase'],
                           translation_m=row['relative_drift'], rotation_rad=row.get('relative_rotation_rad', 0.))
        streak = streak+1 if not row['bilateral_contact'] else 0
        if lost is None and streak >= 3:
            lost = dict(physics_steps=row['physics_steps'], phase=row['phase'])
    return dict(phases=summary, first_pose_warning=warning, first_sustained_contact_loss=lost,
                warning_definition='Observed relative translation > 10 mm or rotation > 15 degrees; warning alone is not a drop.',
                contact_loss_definition='Three consecutive control observations without positive force on both fingers.',
                last_observation=rows[-1] if rows else None)


def write_contact_diagnostics(directory, rows):
    result = contact_diagnostics(rows)
    (Path(directory)/'contact_diagnostics.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    return result
