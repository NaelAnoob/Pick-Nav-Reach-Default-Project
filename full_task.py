"""Pick, carry through the maze, and reach: python full_task.py --headless."""
import argparse
import csv
import json
import time
from pathlib import Path
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation

from controller import GraspExperiment
from navigation import GridNavigator
from pick_nav_reach_env import PickNavReachEnv
from diagnostics import write_contact_diagnostics
from floor_recovery import FloorRecovery
from table_regrasp import regrasp_at_edge, TableRegraspError


class TaskFailure(RuntimeError):
    pass


class GraspSlip(TaskFailure):
    pass


class FullTaskExperiment(GraspExperiment):
    def __init__(self, *args, carry_mode='rotate', recover_slips=False, max_recoveries=1,
                 speed=1.8, acceleration=.8, **kwargs):
        kwargs.setdefault('grasp_mode', 'geometry')
        super().__init__(*args, **kwargs)
        if not np.isfinite([speed, acceleration]).all() or min(speed, acceleration) <= 0:
            raise ValueError('speed and acceleration must be finite and positive')
        if not isinstance(max_recoveries, int) or max_recoveries < 0:
            raise ValueError('max_recoveries must be a nonnegative integer')
        self.carry_mode = carry_mode
        self.transport_started = False
        self.carry_reference = None
        self.max_carry_drift = 0.
        self.task_trace = []
        self.route = []
        self.recover_slips, self.max_recoveries = recover_slips, max_recoveries
        self.speed, self.acceleration = speed, acceleration
        self.recoveries = []
        self.drops = 0
        self.edge_regrasp = None

    def run(self):
        try:
            return super().run()
        except TaskFailure as error:
            return self.finish(str(error))

    def step(self, q):
        if self.transport_started:
            q = np.array(q, dtype=float, copy=True)
            q[-2:] = 0.
            self.camera_focus = np.r_[self.env.agent_qpos[:2]+[-2, 0], .85].tolist()
        if self.env.step_count*self.env.substeps+self.control_substeps > 15000:
            raise TaskFailure('15000 physics-step budget exhausted')
        obs = super().step(q)
        distance = float(np.linalg.norm(obs['object_pos']-obs['goal_pos']))
        if self.transport_started:
            ee = p.getLinkState(self.env.robot_id, self.planner.ee, computeForwardKinematics=True)
            relative = np.array(p.multiplyTransforms(*p.invertTransform(ee[4], ee[5]),
                                                     obs['object_pos'], obs['object_xyzw'])[0])
            drift = float(np.linalg.norm(relative-self.carry_reference))
            self.max_carry_drift = max(self.max_carry_drift, drift)
            if obs['object_pos'][2] < .45 or drift > .09:
                raise GraspSlip('object slipped from the gripper')
        self.task_trace.append(dict(physics_steps=self.env.step_count*self.env.substeps,
                                   phase=self.phase, distance_to_goal_m=distance,
                                   carry_drift_m=self.max_carry_drift))
        return obs

    def update_payload(self):
        ee = p.getLinkState(self.env.robot_id, self.planner.ee, computeForwardKinematics=True)
        obs = self.env._get_obs()
        self.planner.payload = p.multiplyTransforms(*p.invertTransform(ee[4], ee[5]),
                                                    obs['object_pos'], obs['object_xyzw'])

    def footprint(self):
        """Conservative union of all robot collision AABBs and the object."""
        boxes = [p.getAABB(self.env.robot_id, link) for link in range(-1, p.getNumJoints(self.env.robot_id))
                 if p.getCollisionShapeData(self.env.robot_id, link)]
        boxes.append(p.getAABB(self.env.object_id))
        origin = self.env.agent_qpos[:2]+[-2, 0]
        return np.min(np.array(boxes)[:, 0, :2], axis=0)-origin, np.max(np.array(boxes)[:, 1, :2], axis=0)-origin

    def drive(self, path, speed=.9, acceleration=.8, phase='navigate'):
        """Stop at corners using bounded-acceleration base targets."""
        self.phase = phase
        dt = self.control_dt
        for target in path[1:]:
            start = self.env.agent_qpos.astype(float)
            delta = target[:2]-start[:2]
            length = np.linalg.norm(delta)
            if length < 1e-6:
                continue
            peak = min(speed, np.sqrt(length*acceleration))
            ramp = peak/acceleration
            cruise = max(0., (length-peak*ramp)/peak)
            duration = 2*ramp+cruise
            for t in np.arange(dt, duration+dt, dt):
                t = min(t, duration)
                if t < ramp:
                    distance = .5*acceleration*t*t
                elif t < ramp+cruise:
                    distance = .5*peak*ramp+peak*(t-ramp)
                else:
                    distance = length-.5*acceleration*(duration-t)**2
                q = target.copy()
                q[:2] = start[:2]+delta*distance/length
                self.step(q)
            for _ in range(8):
                self.step(target)

    def carry_and_reach(self, after_recovery=False):
        if self.carry_mode == 'edge' and self.edge_regrasp is None and not after_recovery:
            try:self.edge_regrasp = regrasp_at_edge(self)
            except TableRegraspError as error:raise TaskFailure(str(error)) from error
        self.transport_started = True
        self.update_payload()
        self.carry_reference = np.array(self.planner.payload[0])
        # Retract above the tabletop, with the heading fixed throughout transport.
        start = self.env.agent_qpos.astype(float)
        ee = p.getLinkState(self.env.robot_id, self.planner.ee, computeForwardKinematics=True)
        base = start[:2]+[-2, 0]
        # Alternate orientations are exposed for reproducible carry experiments.
        mode = 'topdown' if after_recovery or self.carry_mode == 'edge' else self.carry_mode
        support = mode == 'support'
        carry_rotation = Rotation.from_quat(ee[5])
        if support:
            carry_rotation = carry_rotation*Rotation.from_euler('z', -np.pi/2)
        elif mode == 'rotate':
            carry_rotation = Rotation.from_euler('y', -np.pi/2)*carry_rotation
        carry_quat = carry_rotation.as_quat()
        direction = carry_rotation.as_matrix()[:2, 0] if support else np.array([1., 0])
        if after_recovery:
            direction = np.array([np.cos(start[2]), np.sin(start[2])])
        direction = direction/np.linalg.norm(direction)
        carry = [start.copy(), start.copy()] if after_recovery else None
        rotated = mode != 'topdown'
        for x in (() if after_recovery else ((.5, .45, .42, .55) if rotated else (.45, .42, .5, .55))):
            for z in ((.9, 1.05, 1.15, 1.25) if rotated else (1.05, .9, 1.15, 1.25)):
                def build():
                    target = self.planner.ik(np.r_[base+direction*x, z], carry_quat, start,
                                             attempts=30, collision_free=True)
                    if target is None:
                        return None
                    target[-2:] = 0
                    return self.planner.plan(start, target)
                carry = self.query(build)
                if carry is not None:
                    break
            if carry is not None:
                break
        if carry is None:
            raise TaskFailure('no collision-free carrying posture')
        old_limits = self.respect_joint_limits
        self.respect_joint_limits = old_limits or after_recovery
        try:
            self.move(carry, 'tuck', increment=.020)
        finally:
            self.respect_joint_limits = old_limits
        self.update_payload()
        # Back away before applying the conservative tabletop footprint.
        start = self.env.agent_qpos.astype(float)
        retreat = start.copy()
        retreat[0] = start[0] if after_recovery else min(start[0], .6)
        if not self.query(lambda: self.planner.edge(start, retreat)):
            raise TaskFailure(f'retreat blocked: {self.planner.last_collision}')
        self.drive([start, retreat], speed=.4, acceleration=.3, phase='retreat')
        # Retreat can shift the object inside the grasp. Validate the maze
        # route using the observed pose, rather than the pre-retreat pose.
        self.update_payload()
        obs = self.env._get_obs()
        base = obs['qpos'][:2]+[-2, 0]
        offset = obs['object_pos'][:2]-base
        # Stage just inside the exit; the object itself must reach the marker.
        goal_base = obs['goal_pos'][:2]-offset
        table_box = p.getAABB(self.env.table_id)
        nav = GridNavigator(obs['cube_positions'], self.footprint(),
                            extra_boxes=[(np.array(table_box[0])[:2], np.array(table_box[1])[:2])])
        print('Carrying footprint:', self.footprint(), flush=True)
        route = nav.plan(base, goal_base)
        if route is None:
            offsets = [np.array([dx, dy]) for dx in np.arange(-.3,.301,.05)
                       for dy in np.arange(-.3,.301,.05)]
            offsets.sort(key=np.linalg.norm)
            for offset in offsets:
                staging = goal_base+offset
                if not nav.clear_edge(staging, staging):continue
                def reachable():
                    target = self.env.agent_qpos.astype(float)
                    target[:2] = staging-[-2.,0.]
                    pos, rotation = self.planner.pose(target)
                    carried = pos+rotation.apply(self.planner.payload[0])
                    positions = pos+np.linspace(0,1,24)[1:,None]*(obs['goal_pos']-carried)
                    return self.planner.cartesian(target,positions,rotation.as_quat()) is not None
                if not self.query(reachable):continue
                route = nav.plan(base,staging)
                if route is not None:break
        if route is None:
            raise TaskFailure('no route for the carrying footprint')
        self.route = [point.tolist() for point in route]
        print(f'Navigation: {len(route)} waypoints, {nav.expanded} A* expansions', flush=True)
        q = self.env.agent_qpos.astype(float)
        path = [q.copy()]
        for point in route[1:]:
            target = q.copy()
            target[:2] = point-[-2, 0]
            path.append(target)
        # Check actual link and carried-object geometry before motor execution.
        if not self.query(lambda: all(self.planner.edge(a, b) for a, b in zip(path, path[1:]))):
            raise TaskFailure('route failed full-geometry collision validation')
        navigation_speed = self.speed
        navigation_acceleration = self.acceleration
        def expected_steps(speed):
            duration = 0.
            for a, b in zip(path, path[1:]):
                length = np.linalg.norm(b[:2]-a[:2])
                peak = min(speed, np.sqrt(length*navigation_acceleration))
                if peak > 0:
                    duration += (length/peak+peak/navigation_acceleration)/self.control_dt+9
            return duration*self.control_substeps
        remaining = 15000-self.env.step_count*self.env.substeps
        while navigation_speed < 2.2 and expected_steps(navigation_speed)+500 > remaining:
            navigation_speed = min(2.2, navigation_speed+.1)
        while navigation_acceleration < 1.1 and expected_steps(navigation_speed)+500 > remaining:
            navigation_acceleration = min(1.1, navigation_acceleration+.05)
        self.navigation_speed = navigation_speed
        self.navigation_acceleration = navigation_acceleration
        self.drive(path, speed=navigation_speed, acceleration=navigation_acceleration)
        # Recompute object-to-gripper offset after transport to correct any slip.
        for _ in range(3):
            self.update_payload()
            start = self.env.agent_qpos.astype(float)
            ee = p.getLinkState(self.env.robot_id, self.planner.ee, computeForwardKinematics=True)
            obs = self.env._get_obs()
            correction = obs['goal_pos']-obs['object_pos']
            if np.linalg.norm(correction) < .075:
                break
            positions = np.array(ee[4])+np.linspace(0, 1, 20)[1:, None]*correction
            path = self.query(lambda: self.planner.cartesian(start, positions, ee[5]))
            if path is None:
                def whole_body_reach():
                    active = self.planner.active.copy()
                    self.planner.active = np.r_[[0, 1], active]
                    try:
                        candidate = self.planner.cartesian(start, positions, ee[5])
                        if candidate is not None and all(np.linalg.norm(q[:2]-start[:2]) <= .6 for q in candidate):
                            return candidate
                        return None
                    finally:
                        self.planner.active = active
                path = self.query(whole_body_reach)
            if path is None:
                # A straight Cartesian descent can reach a singularity even
                # when another collision-free arm configuration reaches the
                # same object pose. Search that configuration and its path.
                def alternate_reach():
                    rotation = Rotation.from_quat(ee[5])
                    adjustments = [Rotation.identity()]
                    adjustments += [Rotation.from_rotvec(np.eye(3)[axis]*angle)
                                    for angle in (.25, -.25, .5, -.5, np.pi/2, -np.pi/2, np.pi) for axis in range(3)]
                    for adjustment in adjustments:
                        desired = adjustment*rotation
                        position = obs['goal_pos']-desired.apply(self.planner.payload[0])
                        target = self.planner.ik(position, desired.as_quat(), start,
                                                 attempts=20, collision_free=True)
                        if target is None:
                            continue
                        target[-2:] = 0.
                        candidate = self.planner.plan(start, target)
                        if candidate is not None:
                            return candidate
                    return None
                path = self.query(alternate_reach)
            if path is None:
                state = dict(qpos=start.tolist(), object_pos=obs['object_pos'].tolist(),
                             object_xyzw=obs['object_xyzw'].tolist(), payload=self.planner.payload,
                             collision=self.query(lambda: (self.planner.clear(start),self.planner.last_collision)))
                (self.output/'reach_failure.json').write_text(json.dumps(state))
                raise TaskFailure('final object-position correction blocked')
            self.move(path, 'reach', increment=.006)
        self.phase = 'goal hold'
        q = self.env.agent_qpos.astype(float)
        for _ in range(60):
            self.step(q)
        if not all(row['distance_to_goal_m'] < .1 for row in self.task_trace[-60:]):
            raise TaskFailure('final object position outside 10 cm tolerance')

    def finish(self, reason):
        pick = super().finish(reason)
        task_reason = reason if reason != 'executed' else 'grasp did not remain stable'
        reached = False
        acquired = pick['stable_hold_control_steps'] >= 50 and pick['final_clearance_m'] >= .1
        while acquired and self.env.step_count*self.env.substeps < 15000:
            try:
                self.carry_and_reach(after_recovery=bool(self.recoveries))
                reached, task_reason = True, 'delivered and held at goal'
                break
            except GraspSlip as error:
                self.drops += 1
                self.transport_started = False
                self.planner.payload = None
                if not self.recover_slips or len(self.recoveries) >= self.max_recoveries:
                    task_reason = str(error)
                    break
                recovery = FloorRecovery(self)
                try:
                    outcome = recovery.run()
                except TaskFailure as recovery_error:
                    outcome = recovery.finish(False, str(recovery_error))
                self.recoveries.append(outcome)
                print('Floor recovery:', json.dumps(outcome), flush=True)
                if not outcome['success']:
                    task_reason = outcome['reason']
                    break
            except TaskFailure as error:
                task_reason = str(error)
                break
        result = dict(object_idx=self.env.object_idx, seed=self.env.seed, success=reached,
                      reason=task_reason, pick_success=pick['success'],
                      distance_to_goal_m=float(self.env.evaluate()['dist_to_goal']),
                      physics_steps=self.env.step_count*self.env.substeps,
                      max_carry_drift_m=self.max_carry_drift,
                      carry_stable=bool(reached and self.drops == 0 and self.max_carry_drift < .09),
                      max_robot_obstacle_penetration_m=max(r['obstacle_penetration_m'] for r in self.trace),
                      wall_seconds=time.perf_counter()-self.start_time,
                      collision_checks=self.planner.checks,
                      cube_positions=self.env.cube_positions[:, :2].tolist(),
                      goal_pos=self.env.goal_pos.tolist(),
                      carry_mode=self.carry_mode,
                      edge_regrasp=self.edge_regrasp,
                      drops=self.drops, recoveries=self.recoveries,
                      recovery_attempts=len(self.recoveries),
                      successful_recoveries=sum(r['success'] for r in self.recoveries),
                      recovery_physics_steps=sum(r['duration_physics_steps'] for r in self.recoveries),
                      recovery_enabled=self.recover_slips, max_recoveries=self.max_recoveries,
                      speed_m_s=self.speed, acceleration_m_s2=self.acceleration,
                      navigation_speed_m_s=getattr(self, 'navigation_speed', None),
                      navigation_acceleration_m_s2=getattr(self, 'navigation_acceleration', None),
                      route=self.route, pick=pick)
        (self.output/'task_result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        write_contact_diagnostics(self.output, self.trace)
        for name, rows in [('trace.csv', self.trace), ('task_trace.csv', self.task_trace)]:
            if rows:
                with (self.output/name).open('w', newline='') as stream:
                    writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
                    writer.writeheader()
                    writer.writerows(rows)
        if self.record:
            self.capture().save(self.output/'final.png')
            if self.frames:
                self.frames[0].save(self.output/'demo.gif', save_all=True, append_images=self.frames[1:], duration=165, loop=0)
        print(json.dumps({k: v for k, v in result.items() if k not in ('pick', 'route', 'cube_positions')}, indent=2), flush=True)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--record', action='store_true')
    parser.add_argument('--carry-mode', choices=['rotate', 'topdown', 'support', 'edge'], default='rotate')
    parser.add_argument('--grasp-mode', choices=['geometry', 'wrench', 'adaptive', 'spatial', 'palm'], default='geometry')
    parser.add_argument('--legacy-pick', action='store_true')
    parser.add_argument('--high-rate', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--legacy-rate', action='store_true')
    parser.add_argument('--candidate-rank', type=int, default=0)
    parser.add_argument('--recover-slips', action='store_true')
    parser.add_argument('--max-recoveries', type=int, default=1)
    parser.add_argument('--speed', type=float, default=1.8)
    parser.add_argument('--acceleration', type=float, default=.8)
    parser.add_argument('--objects', nargs='+', type=int, default=[5])
    parser.add_argument('--seeds', nargs='+', type=int, default=[7])
    parser.add_argument('--output', type=Path, default=Path('results/full_task'))
    args = parser.parse_args()
    results = []
    for obj in args.objects:
        for seed in args.seeds:
            env = PickNavReachEnv(seed=seed, object_idx=obj, gui=not args.headless)
            try:
                results.append(FullTaskExperiment(env, args.output/f'object_{obj:02d}_seed_{seed}', args.record,
                    carry_mode=args.carry_mode, grasp_mode=args.grasp_mode, fast_pick=not args.legacy_pick,
                    candidate_rank=args.candidate_rank,
                    high_rate=not args.legacy_rate,
                    recover_slips=args.recover_slips,
                    max_recoveries=args.max_recoveries, speed=args.speed, acceleration=args.acceleration).run())
            finally:
                env.close()
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'summary.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
    with (args.output/'summary.csv').open('w', newline='') as stream:
        fields = ['object_idx', 'seed', 'success', 'reason', 'pick_success', 'distance_to_goal_m',
                  'physics_steps', 'max_carry_drift_m', 'carry_stable', 'drops', 'recovery_attempts',
                  'successful_recoveries', 'recovery_physics_steps', 'speed_m_s', 'acceleration_m_s2']
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(results)


if __name__ == '__main__':
    main()
