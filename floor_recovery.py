"""Plan and execute a contact-based floor regrasp with the supplied robot."""
from dataclasses import replace

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation

from grasping import generate_grasps


class FloorRecovery:
    def __init__(self, experiment):
        self.exp = experiment
        self.env = experiment.env
        self.planner = experiment.planner
        self.start_steps = self.env.step_count*self.env.substeps
        self.result = dict(success=False, reason='not attempted', start_physics_steps=self.start_steps,
                           attempts=0, stable_hold_control_steps=0)

    def finish(self, success, reason):
        self.result.update(success=bool(success), reason=reason,
                           end_physics_steps=self.env.step_count*self.env.substeps,
                           duration_physics_steps=self.env.step_count*self.env.substeps-self.start_steps)
        return self.result

    def stop_and_settle(self):
        exp, env = self.exp, self.env
        exp.phase = 'recovery brake'
        obs = env._get_obs()
        q = obs['qpos'].astype(float)
        velocity = np.array(obs['qvel'][:2], float)
        dt = exp.control_dt
        speed = float(np.linalg.norm(velocity))
        if speed > .01:
            direction = velocity/speed
            duration = speed/.8
            for t in np.arange(dt, duration+dt, dt):
                t = min(t, duration)
                target = q.copy()
                target[:2] += direction*(speed*t-.4*t*t)
                target[-2:] = .05
                if not exp.query(lambda: self.planner.edge(env.agent_qpos.astype(float), target, True)):
                    target = env.agent_qpos.astype(float)
                    target[-2:] = .05
                    exp.step(target)
                    self.result['brake_stopped_for_obstacle'] = True
                    break
                exp.step(target)
        exp.phase = 'recovery settle'
        target = env.agent_qpos.astype(float)
        target[-2:] = .05
        quiet = 0
        for _ in range(300):
            exp.step(target)
            linear, angular = p.getBaseVelocity(env.object_id)
            floor_contact = any(c[9] > .01 for c in p.getContactPoints(env.object_id, env.plane_id))
            quiet = quiet+1 if floor_contact and np.linalg.norm(linear) < .04 and np.linalg.norm(angular) < 1. else 0
            if quiet >= 12:
                return True
        return False

    def plan_pick(self):
        exp, env, planner = self.exp, self.env, self.planner
        obs = env._get_obs()
        ground = p.getAABB(env.plane_id)[1][2]
        grasps = generate_grasps(obs['object_pc'], obs['object_normals'], ground, exp.mu, max_grasps=200)
        top = float(obs['object_pc'][:, 2].max())
        # Leave travel for jaw centering; avoid low wrists that hit the palm.
        grasps = [g for g in grasps if g.width < .085 and g.center[2] >= top-.032]
        start = env.agent_qpos.astype(float)
        stances = []
        for angle in (0., -np.pi/4, np.pi/4, -np.pi/2, np.pi/2, np.pi):
            for radius in (.55, .65, .45):
                base = start.copy()
                base[:2] = obs['object_pos'][:2]-radius*np.array([np.cos(angle), np.sin(angle)])+[2., 0.]
                base[2], base[3], base[-2:] = angle, 0., .05
                stances.append(base)
        stances.sort(key=lambda q: np.linalg.norm(q[:2]-start[:2])+.2*abs(q[2]-start[2]))
        original_active = planner.active.copy()
        configurations = 0
        try:
            planner.active = original_active[original_active != 3]
            for base in stances:
                if not exp.query(lambda: planner.edge(start, base)):
                    continue
                for grasp in grasps:
                    for flip in (False, True):
                        configurations += 1
                        self.result['planning_configurations'] = configurations
                        if configurations > 128:
                            self.result['planning_search_limit_reached'] = True
                            return None
                        rotation = grasp.rotation.copy()
                        if flip:
                            rotation[:, 1:] *= -1
                        grasp = replace(grasp, rotation=rotation)
                        pre = grasp.center-rotation[:, 0]*.12
                        def build():
                            for shape in ((.6, 1.2, -.23), (1., -1., 1.57), (1.4, -.5, .67)):
                                seed = base.copy()
                                seed[6], seed[[7, 9, 11]] = 0., shape
                                contact = planner.ik(grasp.center, grasp.quaternion, seed,
                                                     attempts=3, collision_free=True)
                                if contact is None:
                                    continue
                                ready = planner.ik(pre, grasp.quaternion, contact, attempts=1, collision_free=True)
                                if ready is None:
                                    continue
                                transit = planner.plan(base, ready, iterations=500)
                                if transit is None:
                                    continue
                                approach = planner.cartesian(ready, np.linspace(pre, grasp.center, 20)[1:], grasp.quaternion)
                                if approach is None:
                                    continue
                                previous_payload, previous_support = planner.payload, planner.payload_support
                                try:
                                    planner.payload = p.multiplyTransforms(*p.invertTransform(grasp.center, grasp.quaternion),
                                                                           obs['object_pos'], obs['object_xyzw'])
                                    planner.payload_support = env.plane_id
                                    saved_active = planner.active
                                    planner.active = original_active
                                    positions = grasp.center+np.linspace(0, max(.65, .72-grasp.center[2]), 60)[1:, None]*[0., 0., 1.]
                                    lift = planner.cartesian(approach[-1], positions, grasp.quaternion)
                                    if lift is not None:
                                        return transit, approach
                                finally:
                                    planner.active = saved_active
                                    planner.payload, planner.payload_support = previous_payload, previous_support
                            return None
                        plan = exp.query(build)
                        if plan is not None:
                            return base, grasp, plan
            return None
        finally:
            planner.active = original_active

    def run(self):
        exp, env, planner = self.exp, self.env, self.planner
        old_limits = exp.respect_joint_limits
        old_tolerances = planner.position_tolerance, planner.orientation_tolerance
        # Near the floor, the folded arm is close to its workspace boundary.
        # Permit the original IK accuracy; verify the achieved contact pose
        # and floor clearance during execution before closing the fingers.
        planner.position_tolerance, planner.orientation_tolerance = .003, .025
        planner.payload = None
        planner.payload_support = None
        exp.respect_joint_limits = True
        try:
            if not self.stop_and_settle():
                return self.finish(False, 'object did not settle on the floor')
            self.result['floor_pose'] = list(p.getBasePositionAndOrientation(env.object_id)[0])
            exp.camera_focus = [*self.result['floor_pose'][:2], .45]
            plan = self.plan_pick()
            if plan is None:
                return self.finish(False, 'no collision-free floor grasp and stance')
            base, grasp, (transit, approach) = plan
            self.result['attempts'] += 1
            self.result['grasp'] = dict(center=grasp.center.tolist(), quaternion=grasp.quaternion.tolist(), width=grasp.width)
            exp.move([env.agent_qpos.astype(float), base], 'recovery position', increment=.015)
            exp.move(transit, 'floor transit', increment=.03)
            exp.move(approach, 'floor approach', increment=.012)
            ee = p.getLinkState(env.robot_id, planner.ee, computeForwardKinematics=True)
            error = float(np.linalg.norm(np.array(ee[4])-grasp.center))
            self.result['approach_position_error_m'] = error
            if error > .008:
                return self.finish(False, 'arm did not reach the floor grasp')
            exp.phase = 'floor close'
            target = approach[-1].copy()
            for opening in np.linspace(.05, 0., 55):
                target[-2:] = opening
                exp.step(target)
            for _ in range(20):
                exp.step(target)
            if not exp.trace[-1]['bilateral_contact']:
                return self.finish(False, 'floor closure did not contact both fingers')
            exp.update_payload()
            planner.payload_support = env.plane_id
            ee = p.getLinkState(env.robot_id, planner.ee, computeForwardKinematics=True)
            # Return to a useful transport height, rather than merely clearing
            # the floor. All movement remains ordinary env.step motor targets.
            height = max(.65, .72-ee[4][2])
            positions = np.array(ee[4])+np.linspace(0, height, 60)[1:, None]*[0., 0., 1.]
            positions[:, :2] = grasp.center[:2]
            # Keep the orientation whose entire lift was checked. A tiny
            # measured tilt at full arm extension can make a different fixed
            # orientation unreachable even though the planned lift is feasible.
            lift = exp.query(lambda: planner.cartesian(env.agent_qpos.astype(float), positions, grasp.quaternion))
            if lift is None:
                self.result['planning_failure'] = planner.last_collision or 'lift IK unreachable'
                return self.finish(False, 'floor lift path blocked')
            for q in lift:
                q[-2:] = 0.
            exp.move(lift, 'floor lift', increment=.015)
            planner.payload_support = None
            exp.phase = 'floor hold'
            relative_start = None
            longest = consecutive = 0
            max_drift = 0.
            for _ in range(50):
                obs = exp.step(lift[-1])
                ee = p.getLinkState(env.robot_id, planner.ee, computeForwardKinematics=True)
                relative = np.array(p.multiplyTransforms(*p.invertTransform(ee[4], ee[5]),
                                                         obs['object_pos'], obs['object_xyzw'])[0])
                if relative_start is None:
                    relative_start = relative.copy()
                drift = float(np.linalg.norm(relative-relative_start))
                max_drift = max(max_drift, drift)
                stable = exp.trace[-1]['bilateral_contact'] and p.getAABB(env.object_id)[0][2] > .1 and drift < .02
                consecutive = consecutive+1 if stable else 0
                longest = max(longest, consecutive)
            self.result.update(stable_hold_control_steps=longest, max_hold_drift_m=max_drift,
                               final_floor_clearance_m=float(p.getAABB(env.object_id)[0][2]))
            if longest < 50:
                return self.finish(False, 'floor grasp did not remain stable')
            exp.update_payload()
            return self.finish(True, 'regrasped from floor and held')
        finally:
            exp.respect_joint_limits = old_limits
            planner.position_tolerance, planner.orientation_tolerance = old_tolerances
            planner.payload_support = None
            if not self.result['success']:
                planner.payload = None
