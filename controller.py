"""Reproducible pick-only demo: python controller.py --headless --objects 5."""
import argparse
import csv
import json
from pathlib import Path
import time
from dataclasses import replace

import numpy as np
import pybullet as p
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation

from grasping import MotionPlanner, generate_grasps, wrench_grasps, spatial_grasps
from diagnostics import write_contact_diagnostics
from pick_nav_reach_env import PickNavReachEnv
from utils import closest_joint_values


class HoldController:
    def __init__(self, initial_obs):
        self.target_qpos = initial_obs['qpos'].copy()

    def get_action(self, obs):
        return self.target_qpos.copy()


class GraspExperiment:
    def __init__(self, env, output, record=False, mu=0.5, grasp_mode='geometry', fast_pick=True,
                 candidate_rank=0, high_rate=True):
        self.env, self.output, self.record = env, Path(output), record
        self.output.mkdir(parents=True, exist_ok=True)
        self.planner = MotionPlanner(env, env.seed)
        self.mu = mu
        if grasp_mode not in ('geometry', 'wrench', 'adaptive', 'spatial', 'palm'):
            raise ValueError('unknown grasp scoring mode')
        self.grasp_mode, self.fast_pick = grasp_mode, fast_pick
        self.candidate_rank = candidate_rank
        self.control_substeps = getattr(env, 'control_substeps', env.substeps)
        env.control_substeps = self.control_substeps
        self.control_dt = self.control_substeps/240.
        self.high_rate = high_rate
        physics_elapsed = env.step_count*env.substeps
        target_substeps = 1 if high_rate else self.control_substeps
        if physics_elapsed % target_substeps:
            raise ValueError('cannot change control grouping at a partial reference interval')
        env.step_count = physics_elapsed//target_substeps
        env.substeps = target_substeps
        self.micro_steps = self.control_substeps//env.substeps
        self.control_steps = 0
        self.previous_target = env.agent_qpos.astype(float)
        object_mu = p.getDynamicsInfo(env.object_id, -1)[1]
        self.effective_frictions = [float(np.clip(object_mu*p.getDynamicsInfo(env.robot_id, link)[1], 0, 10))
                                    for link in sorted(self.planner.fingers)]
        self.scoring_used = 'geometry'
        self.frames, self.trace = [], []
        self.phase = 'settle'
        self.initial_relative = None
        self.initial_relative_rotation = None
        self.initial_ee_rotation = None
        self.selected = None
        self.start_time = time.perf_counter()
        self.table_height = p.getAABB(env.table_id)[1][2]
        self.respect_joint_limits = False
        self.joint_speeds = np.array([p.getJointInfo(env.robot_id, j)[11] for j in env.joint_indices])
        self.joint_speeds[self.joint_speeds <= 0] = 1.

    def capture(self):
        view = p.computeViewMatrixFromYawPitchRoll(getattr(self, 'camera_focus', [-.25, 0, .85]), 2.3, 135, -24, 0, 2)
        proj = p.computeProjectionMatrixFOV(48, 1.5, .05, 10)
        rgba = p.getCameraImage(720, 480, view, proj, renderer=p.ER_TINY_RENDERER)[2]
        pixels = np.frombuffer(rgba, dtype=np.uint8) if isinstance(rgba, bytes) else np.asarray(rgba, dtype=np.uint8)
        im = Image.fromarray(pixels.reshape(480, 720, 4)[:, :, :3])
        draw = ImageDraw.Draw(im)
        draw.rectangle((0, 0, 720, 33), fill=(20, 29, 47))
        draw.text((14, 10), f'Pick-Nav-Reach | object {self.env.object_idx} | seed {self.env.seed} | {self.phase}', fill='white')
        return im

    def step(self, q):
        q = np.asarray(q)
        if q.shape != (self.env.action_size,) or not np.isfinite(q).all():
            raise ValueError('Controller must return finite absolute joint targets')
        if self.micro_steps > 1:
            q = closest_joint_values(q, self.previous_target, self.env.joint_upper == 314)
            for micro in range(1, self.micro_steps+1):
                target = self.previous_target+(q-self.previous_target)*micro/self.micro_steps
                obs, _ = self.env.step(target)
        else:
            obs, _ = self.env.step(q)
        self.previous_target = q.copy()
        self.control_steps += 1
        contacts = p.getContactPoints(self.env.robot_id, self.env.object_id)
        touching = {c[3] for c in contacts if c[9] > .01}
        bottom = p.getAABB(self.env.object_id)[0][2]
        ee = p.getLinkState(self.env.robot_id, self.planner.ee, computeForwardKinematics=True)
        inv = p.invertTransform(ee[4], ee[5])
        relative_pose = p.multiplyTransforms(*inv, obs['object_pos'], obs['object_xyzw'])
        relative = np.array(relative_pose[0])
        relative_rotation = Rotation.from_quat(relative_pose[1])
        ee_rotation = Rotation.from_quat(ee[5])
        if self.phase == 'hold' and self.initial_relative is None:
            self.initial_relative = relative.copy()
            self.initial_relative_rotation = relative_rotation
            self.initial_ee_rotation = ee_rotation
        drift = np.linalg.norm(relative-self.initial_relative) if self.initial_relative is not None else 0.
        rotation_drift = ((relative_rotation*self.initial_relative_rotation.inv()).magnitude()
                          if self.initial_relative_rotation is not None else 0.)
        fingers = sorted(self.planner.fingers)
        forces = [sum(c[9] for c in contacts if c[3] == finger) for finger in fingers]
        tangential = [sum(np.hypot(c[10], c[12]) for c in contacts if c[3] == finger)
                      for finger in fingers]
        object_mu = p.getDynamicsInfo(self.env.object_id, -1)[1]
        friction = [object_mu*p.getDynamicsInfo(self.env.robot_id, finger)[1] for finger in fingers]
        obstacle_contacts = [c for body in self.planner.obstacles
                             for c in p.getContactPoints(self.env.robot_id, body)]
        floor_contacts = [c for c in p.getContactPoints(self.env.robot_id, self.env.plane_id)
                          if c[3] in self.planner.floor_links]
        self.trace.append(dict(step=self.control_steps, physics_steps=self.env.step_count*self.env.substeps,
            phase=self.phase, object_z=float(obs['object_pos'][2]), bottom_z=float(bottom),
            clearance=float(bottom-self.table_height), bilateral_contact=self.planner.fingers <= touching,
            relative_drift=float(drift), relative_rotation_rad=float(rotation_drift),
            gripper_rotation_rad=float((ee_rotation*self.initial_ee_rotation.inv()).magnitude()
                if self.initial_ee_rotation is not None else 0.),
            gripper_x_m=float(ee[4][0]), gripper_y_m=float(ee[4][1]), gripper_z_m=float(ee[4][2]),
            relative_x_m=float(relative[0]), relative_y_m=float(relative[1]), relative_z_m=float(relative[2]),
            finger_1_force_n=float(forces[0]), finger_2_force_n=float(forces[1]),
            finger_1_tangent_n=float(tangential[0]), finger_2_tangent_n=float(tangential[1]),
            finger_1_friction=friction[0], finger_2_friction=friction[1],
            finger_1_position_m=float(obs['qpos'][-2]), finger_2_position_m=float(obs['qpos'][-1]),
            base_speed_m_s=float(np.linalg.norm(obs['qvel'][:2])),
            base_x_m=float(obs['qpos'][0]-2), base_y_m=float(obs['qpos'][1]),
            torso_height_m=float(obs['qpos'][3]), torso_target_m=float(q[3]),
            max_arm_tracking_error_rad=float(np.max(np.abs((obs['qpos'][6:13]-q[6:13]+np.pi)%(2*np.pi)-np.pi))),
            arm_floor_penetration_m=max((max(0., -c[8]) for c in floor_contacts), default=0.),
            obstacle_penetration_m=max((-c[8] for c in obstacle_contacts), default=0.)))
        if self.record and self.env.step_count % 8 == 0:
            self.frames.append(self.capture())
        return obs

    def move(self, path, phase, increment=.020):
        self.phase = phase
        for goal in path[1:]:
            start = self.env.agent_qpos.astype(float)
            count = max(1, int(np.ceil(np.max(np.abs(goal-start))/increment)))
            if self.respect_joint_limits:
                per_step = .8*self.joint_speeds[:-2]*self.control_dt
                count = max(count, int(np.ceil(np.max(np.abs(goal[:-2]-start[:-2])/per_step))))
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
        self.move(path, 'ready', increment=.03 if self.fast_pick else .02)
        near = ready.copy()
        near[0] = 1.25
        path = self.query(lambda: [self.env.agent_qpos.astype(float), near]
                          if self.planner.edge(self.env.agent_qpos, near) else None)
        if path is None:
            return self.finish('base approach blocked')
        self.move(path, 'base approach', increment=.015)
        obs = self.env._get_obs()
        grasps = generate_grasps(obs['object_pc'], obs['object_normals'], self.table_height, self.mu)
        if self.grasp_mode == 'spatial':
            grasps = spatial_grasps(obs['object_pc'], obs['object_normals'], self.table_height,
                                   obs['object_pos'], obs['qpos'][:2]+[-2., 0.], self.mu)
            self.scoring_used = 'spatial'
        extent = np.ptp(obs['object_pc'], axis=0)
        self.shape_ratio = float(extent.max()/max(extent.min(), 1e-6))
        # The patch model helped long objects in development, but degraded
        # compact fruit and boxes. Route by observed geometry, never object ID.
        if self.grasp_mode == 'wrench' or (self.grasp_mode == 'adaptive' and self.shape_ratio > 2.5):
            measured_mu = min(self.effective_frictions)
            if measured_mu <= 0:
                return self.finish('no friction available for a pinch grasp')
            grasps = wrench_grasps(obs['object_pc'], obs['object_normals'], self.table_height,
                                  measured_mu, carry_mode=getattr(self, 'carry_mode', 'rotate'))
            self.scoring_used = 'wrench'
        self.proposals = len(grasps)
        print(f'object={self.env.object_idx} seed={self.env.seed}: {len(grasps)} proposals', flush=True)
        feasible = 0
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
                contact_center = grasp.center.copy()
                def build():
                    nonlocal contact_center
                    if self.grasp_mode == 'spatial':
                        contact_q = self.planner.ik(grasp.center, grasp.quaternion, start, attempts=3)
                        if contact_q is None or not self.planner.clear(contact_q, allow_fingers=True):
                            return None
                    else:
                        contact_q = start
                    target = self.planner.ik(pre, grasp.quaternion, contact_q)
                    if target is None:
                        return None
                    transit = self.planner.plan(start, target, iterations=350)
                    if transit is None:
                        return None
                    if self.grasp_mode == 'palm':
                        approach = self.planner.contact_approach(target, np.linspace(pre, grasp.center, 55)[1:], grasp.quaternion)
                    else:
                        approach = self.planner.cartesian(target, np.linspace(pre, grasp.center, 16)[1:], grasp.quaternion)
                    if approach is None:
                        return None
                    contact_center = self.planner.pose(approach[-1])[0] if self.grasp_mode == 'palm' else grasp.center
                    positions = contact_center+np.linspace(0, lift_height, 24)[1:, None]*[0, 0, 1]
                    self.planner.palm_contact = self.grasp_mode == 'palm'
                    try:
                        lift = self.planner.cartesian(approach[-1], positions, grasp.quaternion)
                    finally:
                        self.planner.palm_contact = False
                    return (transit, approach, lift) if lift is not None else None
                plan = self.query(build)
                if plan is None:
                    continue
                if feasible < self.candidate_rank:
                    feasible += 1
                    continue
                if self.grasp_mode == 'palm':
                    grasp = replace(grasp, center=contact_center)
                self.selected = grasp
                self.lift_height = float(lift_height)
                print(f'  selected proposal {index}, width={grasp.width:.3f}m, margin={grasp.cone_margin:.3f}', flush=True)
                self.move(plan[0], 'pregrasp', increment=.035 if self.fast_pick else .02)
                self.move(plan[1], 'approach', increment=.007)
                target = plan[1][-1].copy()
                self.phase = 'close'
                loaded = 0
                for opening in np.linspace(.05, 0., 70):
                    target[-2:] = opening
                    self.step(target)
                    row = self.trace[-1]
                    loaded = loaded+1 if min(row['finger_1_force_n'], row['finger_2_force_n']) > 45. else 0
                    if self.fast_pick and loaded >= 10:
                        break
                target[-2:] = 0.
                for _ in range(20 if self.fast_pick else 40):
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

    def align_on_table(self):
        """Unload internal shear while the table supports the object's weight."""
        self.phase = 'tactile alignment'
        ee = p.getLinkState(self.env.robot_id, self.planner.ee, computeForwardKinematics=True)
        origin = Rotation.from_quat(ee[5])
        opening = float(self.env.agent_qpos[-2:].mean()-.001)
        center = float((self.env.agent_qpos[-2]-self.env.agent_qpos[-1])/2)
        for index in range(45):
            ee = p.getLinkState(self.env.robot_id, self.planner.ee, computeForwardKinematics=True)
            rotation = Rotation.from_quat(ee[5])
            contacts = p.getContactPoints(self.env.robot_id, self.env.object_id)
            normal_torque = np.zeros(3)
            forces = []
            for finger in sorted(self.planner.fingers):
                forces.append(sum(c[9] for c in contacts if c[3] == finger))
                for c in contacts:
                    if c[3] == finger:
                        local = rotation.inv().apply(np.array(c[5])-ee[4])
                        normal = rotation.inv().apply(c[9]*np.array(c[7]))
                        normal_torque += np.cross(local, normal)
            desired_force = 5. if index < 25 else 5.+(index-25)*2.5
            opening += np.clip(5e-5*(np.mean(forces)-desired_force), -.0005, .0005)
            opening = float(np.clip(opening, abs(center), .05-abs(center)))
            delta = np.clip(.04*normal_torque, -.004, .004)
            delta[1] = 0.
            desired = rotation*Rotation.from_rotvec(delta)
            if (desired*origin.inv()).magnitude() > .12:
                desired = rotation
            start = self.env.agent_qpos.astype(float)
            def build():
                q = self.planner.ik(np.array(ee[4]), desired.as_quat(), start, attempts=1)
                return q if q is not None and self.planner.edge(start, q, True) else None
            target = self.query(build)
            if target is None:
                target = start
            target[-2:] = [opening+center, opening-center]
            self.step(target)
        self.phase = 'alignment preload'
        target = self.env.agent_qpos.astype(float)
        target[-2:] = 0.
        for _ in range(15):
            self.step(target)
        ee = p.getLinkState(self.env.robot_id, self.planner.ee, computeForwardKinematics=True)
        self.alignment = dict(rotation_change_rad=float((Rotation.from_quat(ee[5])*origin.inv()).magnitude()))

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
            scoring=self.scoring_used, fast_pick=self.fast_pick,
            high_rate_targets=self.high_rate,
            candidate_rank=self.candidate_rank,
            shape_ratio=getattr(self, 'shape_ratio', None), effective_friction=self.effective_frictions,
            proposals=getattr(self, 'proposals', 0), selected=self.selected is not None,
            success=bool(longest >= 50 and first_lift is not None and first_lift <= 3000
                         and self.trace[-1]['clearance'] >= .1),
            first_lift_physics_steps=first_lift,
            max_clearance_m=max((r['clearance'] for r in self.trace if r['phase'] in ('lift', 'hold')), default=0.), final_clearance_m=self.trace[-1]['clearance'],
            stable_hold_control_steps=longest, stable_hold_physics_steps=longest*self.control_substeps,
            control_steps=self.control_steps, physics_steps=self.env.step_count*self.env.substeps,
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
                quaternion=self.selected.quaternion.tolist(), width=self.selected.width, cone_margin=self.selected.cone_margin,
                transport_capacity_kg=self.selected.transport_capacity_kg)
        write_contact_diagnostics(self.output, self.trace)
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
    parser.add_argument('--grasp-mode', choices=['geometry', 'wrench', 'adaptive', 'spatial', 'palm'], default='geometry')
    parser.add_argument('--legacy-pick', action='store_true')
    parser.add_argument('--legacy-rate', action='store_true')
    args = parser.parse_args()
    results = []
    for obj in args.objects:
        for seed in args.seeds:
            env = PickNavReachEnv(seed=seed, object_idx=obj, gui=not args.headless)
            try:
                results.append(GraspExperiment(env, args.output/f'object_{obj:02d}_seed_{seed}', args.record, args.mu,
                    grasp_mode=args.grasp_mode, fast_pick=not args.legacy_pick,
                    high_rate=not args.legacy_rate).run())
            finally:
                env.close()
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'summary.json').write_text(json.dumps(results, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
