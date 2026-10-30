"""Reproducible pick-only demo: python controller.py --headless --objects 5."""
import argparse
import csv
import json
from pathlib import Path
import time

import numpy as np
import pybullet as p
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation

from grasping import MotionPlanner, generate_grasps
from pick_nav_reach_env import PickNavReachEnv


class HoldController:
    def __init__(self, initial_obs):
        self.target_qpos = initial_obs['qpos'].copy()

    def get_action(self, obs):
        return self.target_qpos.copy()


class GraspExperiment:
    def __init__(self, env, output, record=False, mu=0.5):
        self.env, self.output, self.record = env, Path(output), record
        self.output.mkdir(parents=True, exist_ok=True)
        self.planner = MotionPlanner(env, env.seed)
        self.mu = mu
        self.frames, self.trace = [], []
        self.phase = 'settle'
        self.initial_relative = None
        self.selected = None
        self.start_time = time.perf_counter()
        self.table_height = p.getAABB(env.table_id)[1][2]

    def capture(self):
        view = p.computeViewMatrixFromYawPitchRoll([-.25, 0, .85], 2.3, 135, -24, 0, 2)
        proj = p.computeProjectionMatrixFOV(48, 1.5, .05, 10)
        rgba = p.getCameraImage(720, 480, view, proj, renderer=p.ER_TINY_RENDERER)[2]
        pixels = np.frombuffer(rgba, dtype=np.uint8) if isinstance(rgba, bytes) else np.asarray(rgba, dtype=np.uint8)
        im = Image.fromarray(pixels.reshape(480, 720, 4)[:, :, :3])
        draw = ImageDraw.Draw(im)
        draw.rectangle((0, 0, 720, 33), fill=(20, 29, 47))
        draw.text((14, 10), f'Antipodal grasp | object {self.env.object_idx} | seed {self.env.seed} | {self.phase}', fill='white')
        return im

    def step(self, q):
        q = np.asarray(q)
        if q.shape != (self.env.action_size,) or not np.isfinite(q).all():
            raise ValueError('Controller must return finite absolute joint targets')
        obs, _ = self.env.step(q)
        touching = {c[3] for c in p.getContactPoints(self.env.robot_id, self.env.object_id) if c[9] > .01}
        bottom = p.getAABB(self.env.object_id)[0][2]
        ee = p.getLinkState(self.env.robot_id, self.planner.ee, computeForwardKinematics=True)
        inv = p.invertTransform(ee[4], ee[5])
        relative = np.array(p.multiplyTransforms(*inv, obs['object_pos'], obs['object_xyzw'])[0])
        if self.phase == 'hold' and self.initial_relative is None:
            self.initial_relative = relative.copy()
        drift = np.linalg.norm(relative-self.initial_relative) if self.initial_relative is not None else 0.
        obstacle_contacts = [c for body in self.planner.obstacles
                             for c in p.getContactPoints(self.env.robot_id, body)]
        self.trace.append(dict(step=self.env.step_count, physics_steps=self.env.step_count*self.env.substeps,
            phase=self.phase, object_z=float(obs['object_pos'][2]), bottom_z=float(bottom),
            clearance=float(bottom-self.table_height), bilateral_contact=self.planner.fingers <= touching,
            relative_drift=float(drift), obstacle_penetration_m=max((-c[8] for c in obstacle_contacts), default=0.)))
        if self.record and self.env.step_count % 8 == 0:
            self.frames.append(self.capture())
        return obs

    def move(self, path, phase, increment=.020):
        self.phase = phase
        for goal in path[1:]:
            start = self.env.agent_qpos.astype(float)
            count = max(1, int(np.ceil(np.max(np.abs(goal-start))/increment)))
            for t in range(1, count+1):
                self.step(start+(goal-start)*t/count)
        for _ in range(15):
            self.step(path[-1])

    def query(self, fn):
        saved = p.saveState()
        try:
            return fn()
        finally:
            p.restoreState(saved)
            p.removeState(saved)

    def run(self):
        q = self.env.agent_qpos.astype(float)
        q[-2:] = .05
        for _ in range(60):
            self.step(q)
        ready = q.copy()
        ready[3] = .3
        ready[7], ready[9], ready[11] = -.65, 1.5, .72
        path = self.query(lambda: self.planner.plan(self.env.agent_qpos.astype(float), ready))
        if path is None:
            return self.finish('no collision-free ready posture')
        self.move(path, 'ready')
        near = ready.copy()
        near[0] = 1.25
        path = self.query(lambda: [self.env.agent_qpos.astype(float), near]
                          if self.planner.edge(self.env.agent_qpos, near) else None)
        if path is None:
            return self.finish('base approach blocked')
        self.move(path, 'base approach', increment=.015)
        obs = self.env._get_obs()
        grasps = generate_grasps(obs['object_pc'], obs['object_normals'], self.table_height, self.mu)
        self.proposals = len(grasps)
        print(f'object={self.env.object_idx} seed={self.env.seed}: {len(grasps)} proposals', flush=True)
        for index, grasp in enumerate(grasps):
            for flip in (False, True):
                if flip:
                    grasp.rotation[:, 1:] *= -1
                pre = grasp.center-grasp.rotation[:, 0]*.15
                # Allow the object's furthest point to swing below the pads.
                # Include half a finger length because contact need not be at
                # the gripper frame origin. This matters for the long banana.
                radius = np.linalg.norm(obs['object_pc']-grasp.center, axis=1).max()
                lift_height = max(.19, self.table_height+.12+radius+.0315-grasp.center[2])
                start = self.env.agent_qpos.astype(float)
                def build():
                    target = self.planner.ik(pre, grasp.quaternion, start)
                    if target is None:
                        return None
                    transit = self.planner.plan(start, target, iterations=350)
                    if transit is None:
                        return None
                    approach = self.planner.cartesian(target, np.linspace(pre, grasp.center, 16)[1:], grasp.quaternion)
                    if approach is None:
                        return None
                    positions = grasp.center+np.linspace(0, lift_height, 24)[1:, None]*[0, 0, 1]
                    lift = self.planner.cartesian(approach[-1], positions, grasp.quaternion)
                    return (transit, approach, lift) if lift is not None else None
                plan = self.query(build)
                if plan is None:
                    continue
                self.selected = grasp
                self.lift_height = float(lift_height)
                print(f'  selected proposal {index}, width={grasp.width:.3f}m, margin={grasp.cone_margin:.3f}', flush=True)
                self.move(plan[0], 'pregrasp')
                self.move(plan[1], 'approach', increment=.007)
                target = plan[1][-1].copy()
                self.phase = 'close'
                for opening in np.linspace(.05, 0., 70):
                    target[-2:] = opening
                    self.step(target)
                for _ in range(40):
                    self.step(target)
                lift = [q.copy() for q in plan[2]]
                for qlift in lift:
                    qlift[-2:] = 0.
                self.move(lift, 'lift', increment=.008)
                self.phase = 'hold'
                for _ in range(100):
                    self.step(lift[-1])
                return self.finish('executed')
        return self.finish('no feasible grasp trajectory')

    def finish(self, reason):
        hold = [r for r in self.trace if r['phase'] == 'hold']
        longest, consecutive = 0, 0
        for row in hold:
            stable = row['clearance'] >= .1 and row['bilateral_contact'] and row['relative_drift'] < .02
            consecutive = consecutive+1 if stable else 0
            longest = max(longest, consecutive)
        lifted = [r for r in self.trace if r['clearance'] >= .1 and r['bilateral_contact']]
        first_lift = lifted[0]['physics_steps'] if lifted else None
        result = dict(object_idx=self.env.object_idx, seed=self.env.seed, reason=reason,
            proposals=getattr(self, 'proposals', 0), selected=self.selected is not None,
            success=bool(longest >= 50 and first_lift is not None and first_lift <= 3000
                         and self.trace[-1]['clearance'] >= .1),
            first_lift_physics_steps=first_lift,
            max_clearance_m=max((r['clearance'] for r in self.trace if r['phase'] in ('lift', 'hold')), default=0.), final_clearance_m=self.trace[-1]['clearance'],
            stable_hold_control_steps=longest, stable_hold_physics_steps=longest*self.env.substeps,
            control_steps=self.env.step_count, physics_steps=self.env.step_count*self.env.substeps,
            collision_checks=self.planner.checks, wall_seconds=time.perf_counter()-self.start_time,
            max_hold_relative_drift_m=max((r['relative_drift'] for r in hold), default=None),
            mu_filter=self.mu, position_error_m=None, orientation_error_rad=None)
        result['max_robot_obstacle_penetration_m'] = max(r['obstacle_penetration_m'] for r in self.trace)
        result['sustained_hold'] = bool(longest == 100 and self.trace[-1]['clearance'] >= .1)
        result['planned_lift_m'] = getattr(self, 'lift_height', None)
        if self.selected is not None:
            ee = p.getLinkState(self.env.robot_id, self.planner.ee, computeForwardKinematics=True)
            result['position_error_m'] = float(np.linalg.norm(np.array(ee[4])-(self.selected.center+[0, 0, self.lift_height])))
            result['orientation_error_rad'] = float((Rotation.from_quat(self.selected.quaternion)*Rotation.from_quat(ee[5]).inv()).magnitude())
            result['grasp'] = dict(contacts=self.selected.contacts.tolist(), center=self.selected.center.tolist(),
                quaternion=self.selected.quaternion.tolist(), width=self.selected.width, cone_margin=self.selected.cone_margin)
        (self.output/'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        with (self.output/'trace.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=self.trace[0].keys())
            writer.writeheader()
            writer.writerows(self.trace)
        if self.record:
            self.capture().save(self.output/'final.png')
            if self.frames:
                self.frames[0].save(self.output/'demo.gif', save_all=True, append_images=self.frames[1:], duration=165, loop=0)
        print(json.dumps({k:v for k,v in result.items() if k != 'grasp'}, indent=2), flush=True)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--record', action='store_true')
    parser.add_argument('--objects', nargs='+', type=int, default=[5])
    parser.add_argument('--seeds', nargs='+', type=int, default=[42])
    parser.add_argument('--output', type=Path, default=Path('results/grasping'))
    parser.add_argument('--mu', type=float, default=.5)
    args = parser.parse_args()
    results = []
    for obj in args.objects:
        for seed in args.seeds:
            env = PickNavReachEnv(seed=seed, object_idx=obj, gui=not args.headless)
            try:
                results.append(GraspExperiment(env, args.output/f'object_{obj:02d}_seed_{seed}', args.record, args.mu).run())
            finally:
                env.close()
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'summary.json').write_text(json.dumps(results, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
