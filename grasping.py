"""Antipodal point-cloud grasps and collision-checked motion planning.

Lecture connection: Week 3&4 Grasping, pp. 13, 49; Week 5 Motion
Planning, pp. 49, 62-65. Normals supplied by the environment point OUTWARD.
"""
from dataclasses import dataclass

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation
from scipy.optimize import linprog


@dataclass
class Grasp:
    contacts: np.ndarray
    normals: np.ndarray
    center: np.ndarray
    rotation: np.ndarray
    width: float
    cone_margin: float
    score: float
    transport_capacity_kg: float | None = None

    @property
    def quaternion(self):
        return Rotation.from_matrix(self.rotation).as_quat()


def spatial_grasps(points, normals, support_height, center_of_mass, base_xy, mu=.5):
    """Seat central contacts using inclined approaches and real pad dimensions.

    Palm clearance is checked against the simulator geometry by the planner,
    rather than pushing the wrist above the object's entire bounding box.
    """
    points, center_of_mass = np.asarray(points), np.asarray(center_of_mass)
    pairs = generate_grasps(points, normals, support_height, mu, max_grasps=200)
    pairs.sort(key=lambda g: np.linalg.norm(g.contacts.mean(axis=0)-center_of_mass))
    facing = np.r_[center_of_mass[:2]-base_xy, 0.]
    facing /= max(np.linalg.norm(facing), 1e-9)
    candidates = []
    for g in pairs[:60]:
        y = g.rotation[:, 1]
        horizontal = facing-(facing@y)*y
        if np.linalg.norm(horizontal) < .1:
            horizontal = np.cross(y, [0., 0., 1.])
        horizontal /= np.linalg.norm(horizontal)
        for tilt in np.deg2rad([15., 20., 30., 45., 60., 75.]):
            x = np.cos(tilt)*horizontal+np.sin(tilt)*np.array([0., 0., -1.])
            x -= (x@y)*y
            x /= np.linalg.norm(x)
            z = np.cross(x, y)
            rotation = np.column_stack((x, y, z))
            for depth in [.024, .028, .016, .032, .008, 0.]:
                center = g.contacts.mean(axis=0)-depth*x
                pad_floor = .0315*abs(x[2])+.0575*abs(y[2])+.012*abs(z[2])
                center[2] = max(center[2], support_height+.003+pad_floor)
                offset = (center_of_mass-center)@rotation
                if abs(offset[2]) > .013 or abs(offset[0]) > .036:
                    continue
                score = (3*g.cone_margin-20*np.linalg.norm(g.contacts.mean(axis=0)-center_of_mass)
                         -4*abs(depth-.012)-12*abs(offset[2]))
                candidates.append(Grasp(g.contacts.copy(), g.normals.copy(), center,
                                        rotation, g.width, g.cone_margin, float(score)))
    return sorted(candidates, key=lambda g: g.score, reverse=True)


def antipodal_margin(points, normals, mu=0.5):
    """Positive iff the contact segment is strictly inside BOTH inward cones."""
    if not np.isfinite(mu) or mu <= 0:
        raise ValueError("mu must be positive and finite")
    points, normals = np.asarray(points), np.asarray(normals)
    delta = points[1] - points[0]
    width = np.linalg.norm(delta)
    lengths = np.linalg.norm(normals, axis=1)
    if width < 1e-9 or np.any(lengths < 1e-9):
        return -np.inf
    direction = delta / width
    n = normals / lengths[:, None]
    return min(-n[0] @ direction, n[1] @ direction) - 1 / np.sqrt(1 + mu * mu)


def generate_grasps(points, normals, table_height, mu=0.5, max_grasps=80):
    """Enumerate contact pairs; filter aperture, cone, and tabletop geometry.

    The top-down restriction keeps the approach clear of the tabletop. The
    score is a geometric heuristic, NOT the 6-D wrench-hull epsilon metric.
    """
    points, normals = np.asarray(points, float), np.asarray(normals, float)
    if points.ndim != 2 or points.shape[1] != 3 or normals.shape != points.shape:
        raise ValueError("points and normals must have matching (N, 3) shapes")
    if not np.isfinite(points).all() or not np.isfinite(normals).all():
        raise ValueError("point cloud and normals must be finite")
    if not np.isfinite(mu) or mu <= 0:
        raise ValueError("mu must be positive and finite")
    nlength = np.linalg.norm(normals, axis=1)
    valid = nlength > 1e-8
    points, normals = points[valid], normals[valid] / nlength[valid, None]
    if len(points) < 2:
        return []
    i, j = np.triu_indices(len(points), 1)
    delta = points[j] - points[i]
    width = np.linalg.norm(delta, axis=1)
    direction = delta / np.maximum(width[:, None], 1e-9)
    margin = np.minimum(np.einsum('ij,ij->i', -normals[i], direction),
                        np.einsum('ij,ij->i', normals[j], direction)) - 1 / np.sqrt(1 + mu * mu)
    centers = (points[i] + points[j]) / 2
    keep = ((width > 0.008) & (width < 0.092) & (margin > 0.015)
            & (np.abs(direction[:, 2]) < 0.12)
            & (centers[:, 2] > table_height + 0.008))
    indices = np.flatnonzero(keep)
    middle = (points.min(axis=0) + points.max(axis=0)) / 2
    score = (3 * margin - 3 * np.linalg.norm(centers - middle, axis=1)
             - 2 * np.abs(direction[:, 2]))
    grasps = []
    for k in indices[np.argsort(score[indices])[::-1]]:
        # Fetch local +x points toward the fingertips; +y is the closing axis.
        y = direction[k]
        x = np.array([0., 0., -1.])
        x -= (x @ y) * y
        x /= np.linalg.norm(x)
        z = np.cross(x, y)
        rot = np.column_stack((x, y, z))
        if any(np.linalg.norm(centers[k] - g.center) < 0.012
               and abs(y @ g.rotation[:, 1]) > 0.97 for g in grasps):
            continue
        center = centers[k].copy()
        # Fingers extend 31.5 mm along local x on either side of gripper_link.
        # A low contact can sit below the link origin, along the finger pad.
        center[2] = max(center[2], table_height + 0.038)
        grasps.append(Grasp(points[[i[k], j[k]]], normals[[i[k], j[k]]],
                            center, rot, float(width[k]),
                            float(margin[k]), float(score[k])))
        if len(grasps) >= max_grasps:
            break
    return grasps


def wrench_grasps(points, normals, table_height, mu=.25, max_grasps=80, carry_mode='rotate'):
    """Rank finite pad contacts by gravity/acceleration wrench feasibility.

    Contact patches come from the observed cloud inside the 63 x 24 mm pads.
    An eight-sided friction pyramid and the existing 60 N per-jaw limit give
    an approximate mass capacity. This is a ranking model, not a guarantee.
    """
    points, normals = np.asarray(points, float), np.asarray(normals, float)
    grasps = generate_grasps(points, normals, table_height, mu, max_grasps)
    if not grasps:
        return []
    com = (points.min(axis=0)+points.max(axis=0))/2
    angles = np.arange(8)*np.pi/4
    for grasp in grasps:
        grasp.transport_capacity_kg = 0.
        local = (points-grasp.center)@grasp.rotation
        inside = (np.abs(local[:, 0]) < .030) & (np.abs(local[:, 2]) < .011)
        patch = local[inside]
        if len(patch) < 4:
            continue
        locations, forces, sides = [], [], []
        for side in (-1, 1):
            extreme = np.max(side*patch[:, 1])
            contacts = patch[side*patch[:, 1] > extreme-.0015]
            if len(contacts) > 16:
                contacts = contacts[np.linspace(0, len(contacts)-1, 16).astype(int)]
            for contact in contacts:
                for angle in angles:
                    locations.append(contact-(com-grasp.center)@grasp.rotation)
                    forces.append([mu*np.cos(angle), -side, mu*np.sin(angle)])
                    sides.append(side)
        forces, locations, sides = np.array(forces), np.array(locations), np.array(sides)
        wrench = np.column_stack((forces, np.cross(locations, forces))).T
        budget = np.array([sides == -1, sides == 1], float)
        carry = Rotation.from_matrix(grasp.rotation)
        if carry_mode == 'rotate':
            carry = Rotation.from_euler('y', -np.pi/2)*carry
        elif carry_mode == 'support':
            carry = carry*Rotation.from_euler('z', -np.pi/2)
        capacities = []
        for acceleration in ([0, 0, -9.8], [.8, 0, -9.8], [-.8, 0, -9.8],
                             [0, .8, -9.8], [0, -.8, -9.8]):
            external = np.r_[carry.inv().apply(acceleration), [0, 0, 0]]
            eq = np.column_stack((wrench, external))
            cost = np.r_[np.zeros(wrench.shape[1]), -1.]
            result = linprog(cost, A_ub=np.column_stack((budget, [0, 0])),
                b_ub=[60., 60.], A_eq=eq, b_eq=np.zeros(6), bounds=(0, None), method='highs')
            capacities.append(float(result.x[-1]) if result.success else 0.)
        grasp.transport_capacity_kg = min(capacities)
        grasp.score += 10*np.log1p(grasp.transport_capacity_kg)
    return sorted(grasps, key=lambda g: g.score, reverse=True)



class MotionPlanner:
    """Damped Jacobian IK with fixed base, plus RRT-Connect and local checks.

    Uses temporary kinematic states for queries. Callers restore the simulation
    state before executing the returned joint targets with env.step().
    """
    def __init__(self, env, seed=0):
        self.env = env
        self.robot = env.robot_id
        self.rng = np.random.default_rng(seed)
        self.active = np.array([3, 6, 7, 8, 9, 10, 11, 12])
        self.links = {p.getJointInfo(self.robot, j)[12].decode(): j
                      for j in range(p.getNumJoints(self.robot))}
        self.ee = self.links['gripper_link']
        self.fingers = {self.links['r_gripper_finger_link'], self.links['l_gripper_finger_link']}
        self.obstacles = [env.table_id, env.maze_id]
        self.checks = 0
        self.payload = None
        self.payload_support = None
        self.palm_contact = False
        self.position_tolerance = .0005
        self.orientation_tolerance = .002
        # Exclude adjacent/fixed assemblies from geometric self-collision tests.
        parents = {j: p.getJointInfo(self.robot, j)[16] for j in range(p.getNumJoints(self.robot))}
        self.self_pairs = set()
        arm = {self.links[n] for n in ['shoulder_lift_link', 'upperarm_roll_link',
               'elbow_flex_link', 'forearm_roll_link', 'wrist_flex_link',
               'wrist_roll_link', 'gripper_link', 'r_gripper_finger_link', 'l_gripper_finger_link']}
        self.floor_links = arm
        for a in arm:
            for b in range(-1, p.getNumJoints(self.robot)):
                if a == b:
                    continue
                ancestors_a, ancestors_b = [a], [b]
                for _ in range(4):
                    ancestors_a.append(parents.get(ancestors_a[-1], -1))
                    ancestors_b.append(parents.get(ancestors_b[-1], -1))
                if any(ancestors_a[u] == ancestors_b[v] for u in range(4) for v in range(4-u)):
                    continue
                self.self_pairs.add(tuple(sorted((a, b))))

    def pose(self, q):
        self.env._set_qpos(q)
        state = p.getLinkState(self.robot, self.ee, computeForwardKinematics=True)
        return np.array(state[4]), Rotation.from_quat(state[5])

    def ik(self, position, quaternion, seed, attempts=5, collision_free=False, position_tolerance=None):
        if position_tolerance is None:
            position_tolerance = self.position_tolerance
        desired = Rotation.from_quat(quaternion)
        for attempt in range(attempts):
            q = seed.copy().astype(float)
            if attempt:
                q[self.active] += self.rng.normal(0, 0.45, len(self.active))
            q = np.clip(q, self.env.joint_lower, self.env.joint_upper)
            for _ in range(180):
                pos, rot = self.pose(q)
                error = np.r_[position - pos, (desired * rot.inv()).as_rotvec()]
                if np.linalg.norm(error[:3]) < position_tolerance and np.linalg.norm(error[3:]) < self.orientation_tolerance:
                    if collision_free and not self.clear(q):
                        break
                    return q
                linear, angular = p.calculateJacobian(self.robot, self.ee, [0, 0, 0],
                    q.tolist(), [0.] * len(q), [0.] * len(q))
                jac = np.vstack((linear, angular))[:, self.active]
                step = jac.T @ np.linalg.solve(jac @ jac.T + 0.003 * np.eye(6), error)
                q[self.active] += np.clip(step, -0.12, 0.12)
                q = np.clip(q, self.env.joint_lower, self.env.joint_upper)
        return None

    def clear(self, q, allow_fingers=False):
        self.checks += 1
        self.last_collision = None
        self.env._set_qpos(q)
        if self.payload is not None:
            ee = p.getLinkState(self.robot, self.ee, computeForwardKinematics=True)
            pos, orn = p.multiplyTransforms(ee[4], ee[5], *self.payload)
            p.resetBasePositionAndOrientation(self.env.object_id, pos, orn)
            for body in self.obstacles + [self.env.plane_id]:
                contacts = p.getClosestPoints(self.env.object_id, body, distance=.003)
                if body == self.payload_support:
                    contacts = [c for c in contacts if c[8] < -.005]
                if contacts:
                    self.last_collision = f'payload versus body {body}'
                    return False
            allow_fingers = True
        for body in self.obstacles:
            if p.getClosestPoints(self.robot, body, distance=0.002):
                self.last_collision = f'robot versus body {body}'
                return False
        for link in self.floor_links:
            if p.getClosestPoints(self.robot, self.env.plane_id, distance=.002, linkIndexA=link):
                self.last_collision = f'robot arm link {link} versus floor'
                return False
        for contact in p.getClosestPoints(self.robot, self.env.object_id, distance=0.001):
            permitted = self.fingers | ({self.ee} if self.payload is not None else set())
            if self.palm_contact and allow_fingers and contact[3] == self.ee and contact[8] >= -.0001:
                continue
            if not (allow_fingers and contact[3] in permitted):
                self.last_collision = f'object versus robot link {contact[3]}'
                return False
        for a, b in self.self_pairs:
            if p.getClosestPoints(self.robot, self.robot, distance=-0.003, linkIndexA=a, linkIndexB=b):
                self.last_collision = f'robot links {a} and {b}'
                return False
        return True

    def edge(self, start, goal, allow_fingers=False, resolution=0.035):
        count = max(1, int(np.ceil(np.max(np.abs(goal-start)) / resolution)))
        return all(self.clear(start + (goal-start)*t/count, allow_fingers)
                   for t in range(count+1))

    def plan(self, start, goal, iterations=900):
        if not self.clear(start) or not self.clear(goal):
            return None
        if self.edge(start, goal):
            return [start.copy(), goal.copy()]
        trees = [([start.copy()], [-1]), ([goal.copy()], [-1])]
        for iteration in range(iterations):
            a = iteration % 2
            nodes, parents = trees[a]
            other, other_parents = trees[1-a]
            sample = start.copy()
            lo = np.maximum(self.env.joint_lower[self.active], -np.pi)
            hi = np.minimum(self.env.joint_upper[self.active], np.pi)
            sample[self.active] = self.rng.uniform(lo, hi)
            if self.rng.random() < 0.2:
                sample = other[-1]
            nearest = np.argmin([np.linalg.norm((q-sample)[self.active]) for q in nodes])
            d = sample - nodes[nearest]
            qnew = nodes[nearest] + d * min(1., 0.25 / max(np.linalg.norm(d[self.active]), 1e-9))
            if not self.edge(nodes[nearest], qnew):
                continue
            nodes.append(qnew)
            parents.append(int(nearest))
            nearest_b = int(np.argmin([np.linalg.norm((q-qnew)[self.active]) for q in other]))
            if not self.edge(other[nearest_b], qnew):
                continue
            def trace(ns, ps, idx):
                path = []
                while idx != -1:
                    path.append(ns[idx])
                    idx = ps[idx]
                return path[::-1]
            path_a = trace(nodes, parents, len(nodes)-1)
            path_b = trace(other, other_parents, nearest_b)
            path = path_a + path_b[::-1] if a == 0 else path_b + path_a[::-1]
            for _ in range(40):
                i, j = sorted(self.rng.integers(0, len(path), 2))
                if j > i+1 and self.edge(path[i], path[j]):
                    path = path[:i+1] + path[j:]
            return path
        return None

    def cartesian(self, start, positions, quaternion):
        path, q = [start.copy()], start.copy()
        for pos in positions:
            next_q = self.ik(pos, quaternion, q, attempts=1)
            if next_q is None or not self.edge(q, next_q, allow_fingers=True, resolution=0.02):
                return None
            path.append(next_q)
            q = next_q
        return path

    def contact_approach(self, start, positions, quaternion):
        """Stop at first palm contact without permitting deep penetration."""
        path, q = [start.copy()], start.copy()
        self.palm_contact = True
        try:
            for pos in positions:
                nxt = self.ik(pos, quaternion, q, attempts=1, position_tolerance=.0005)
                if nxt is None:
                    return None
                if not self.edge(q, nxt, True, resolution=.015):
                    if self.last_collision != f'object versus robot link {self.ee}':
                        return None
                    low, high = q.copy(), nxt.copy()
                    for _ in range(10):
                        mid = (low+high)/2
                        if self.clear(mid, True):
                            low = mid
                        else:
                            high = mid
                    path.append(low)
                    return path
                path.append(nxt)
                q = nxt
            return path
        finally:
            self.palm_contact = False
