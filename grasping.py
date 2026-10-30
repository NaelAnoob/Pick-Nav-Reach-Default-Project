"""Antipodal point-cloud grasps and collision-checked motion planning.

Lecture connection: Week 3&4 Grasping, pp. 13, 49; Week 5 Motion
Planning, pp. 49, 62-65. Normals supplied by the environment point OUTWARD.
"""
from dataclasses import dataclass

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation


@dataclass
class Grasp:
    contacts: np.ndarray
    normals: np.ndarray
    center: np.ndarray
    rotation: np.ndarray
    width: float
    cone_margin: float
    score: float

    @property
    def quaternion(self):
        return Rotation.from_matrix(self.rotation).as_quat()


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
        # Exclude adjacent/fixed assemblies from geometric self-collision tests.
        parents = {j: p.getJointInfo(self.robot, j)[16] for j in range(p.getNumJoints(self.robot))}
        self.self_pairs = set()
        arm = {self.links[n] for n in ['shoulder_lift_link', 'upperarm_roll_link',
               'elbow_flex_link', 'forearm_roll_link', 'wrist_flex_link',
               'wrist_roll_link', 'gripper_link', 'r_gripper_finger_link', 'l_gripper_finger_link']}
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

    def ik(self, position, quaternion, seed, attempts=5):
        desired = Rotation.from_quat(quaternion)
        for attempt in range(attempts):
            q = seed.copy().astype(float)
            if attempt:
                q[self.active] += self.rng.normal(0, 0.45, len(self.active))
            q = np.clip(q, self.env.joint_lower, self.env.joint_upper)
            for _ in range(180):
                pos, rot = self.pose(q)
                error = np.r_[position - pos, (desired * rot.inv()).as_rotvec()]
                if np.linalg.norm(error[:3]) < 0.003 and np.linalg.norm(error[3:]) < 0.025:
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
        self.env._set_qpos(q)
        for body in self.obstacles:
            if p.getClosestPoints(self.robot, body, distance=0.002):
                return False
        for contact in p.getClosestPoints(self.robot, self.env.object_id, distance=0.001):
            if not (allow_fingers and contact[3] in self.fingers):
                return False
        for a, b in self.self_pairs:
            if p.getClosestPoints(self.robot, self.robot, distance=-0.003, linkIndexA=a, linkIndexB=b):
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
