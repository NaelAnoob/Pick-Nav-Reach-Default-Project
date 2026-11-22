"""Use the existing table edge to obtain a central lateral transport grasp."""
from dataclasses import replace

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation

from grasping import generate_grasps


class TableRegraspError(RuntimeError):
    pass


def side_grasps(obs, table_height):
    pairs = generate_grasps(obs['object_pc'], obs['object_normals'], table_height, max_grasps=200)
    com = obs['object_pos']
    candidates = []
    for pair in pairs:
        y = pair.normals[1]-pair.normals[0]
        y /= np.linalg.norm(y)
        x = np.array([1., 0., 0.])
        x -= (x@y)*y
        if np.linalg.norm(x) < .4:
            continue
        x /= np.linalg.norm(x)
        rotation = np.column_stack((x, y, np.cross(x, y)))
        midpoint = pair.contacts.mean(axis=0)
        for depth in [.008, .016, .024, .032]:
            center = midpoint-x*depth
            offset = (com-center)@rotation
            if abs(offset[2]) > .012:
                continue
            score = 3*pair.cone_margin-30*np.linalg.norm(midpoint-com)-3*depth
            candidates.append(replace(pair, center=center, rotation=rotation.copy(), score=float(score)))
    return sorted(candidates, key=lambda g:g.score, reverse=True)


def regrasp_at_edge(exp):
    env, planner = exp.env, exp.planner
    started = env.step_count*env.substeps
    exp.transport_started = True
    exp.update_payload()
    exp.carry_reference = np.array(planner.payload[0])
    obs = env._get_obs()
    table_low = np.array(p.getAABB(env.table_id)[0])
    box = np.array(p.getAABB(env.object_id))
    radius = max(abs(box[:, 0]-obs['object_pos'][0]))
    destination = table_low[0]+radius+.008
    start = env.agent_qpos.astype(float)
    retreat = start.copy()
    retreat[0] += destination-obs['object_pos'][0]
    if not exp.query(lambda:planner.edge(start, retreat)):
        raise TableRegraspError('table-edge positioning blocked')
    exp.drive([start, retreat], speed=.18, acceleration=.08, phase='edge positioning')
    ee = p.getLinkState(env.robot_id, planner.ee, computeForwardKinematics=True)
    clearance = p.getAABB(env.object_id)[0][2]-exp.table_height
    positions = np.array(ee[4])-np.linspace(0, max(0., clearance-.008), 45)[1:, None]*[0., 0., 1.]
    planner.payload_support = env.table_id
    try:
        place = exp.query(lambda:planner.cartesian(env.agent_qpos.astype(float), positions, ee[5]))
        if place is None:
            raise TableRegraspError('vertical table placement blocked')
        for q in place:q[-2:]=0.
        old_limits = exp.respect_joint_limits
        exp.respect_joint_limits = True
        try:exp.move(place, 'edge placement', increment=.012)
        finally:exp.respect_joint_limits = old_limits
        exp.transport_started = False
        planner.payload = None
        exp.phase = 'edge release'
        q = env.agent_qpos.astype(float)
        for opening in np.linspace(q[-2:].mean(), .05, 30):
            q[-2:] = opening
            exp.step(q)
        for _ in range(15):exp.step(q)
        ee = p.getLinkState(env.robot_id, planner.ee, computeForwardKinematics=True)
        positions = np.array(ee[4])+np.linspace(0, .15, 25)[1:, None]*[0., 0., 1.]
        path = exp.query(lambda:planner.cartesian(env.agent_qpos.astype(float), positions, ee[5]))
        if path is None:raise TableRegraspError('table hand retreat blocked')
        exp.move(path, 'edge hand retreat', increment=.025)
        obs = env._get_obs()
        for index, grasp in enumerate(side_grasps(obs, exp.table_height)):
            for flip in (False, True):
                rotation = grasp.rotation.copy()
                if flip:rotation[:,1:] *= -1
                grasp = replace(grasp, rotation=rotation)
                pre = grasp.center-grasp.rotation[:,0]*.12
                start = env.agent_qpos.astype(float)
                def build():
                    contact = planner.ik(grasp.center, grasp.quaternion, start, attempts=4)
                    if contact is None or not planner.clear(contact, True):return None
                    ready = planner.ik(pre, grasp.quaternion, contact, attempts=2)
                    if ready is None:return None
                    transit = planner.plan(start, ready)
                    if transit is None:return None
                    approach = planner.cartesian(ready,np.linspace(pre,grasp.center,25)[1:],grasp.quaternion)
                    if approach is None:return None
                    return transit, approach
                plan = exp.query(build)
                if plan is None:continue
                print('SIDE_GRASP',index,grasp.center.tolist(),flush=True)
                exp.move(plan[0], 'side pregrasp', increment=.035)
                exp.move(plan[1], 'side approach', increment=.008)
                q = plan[1][-1].copy()
                exp.phase = 'side close'
                for opening in np.linspace(.05,0.,55):
                    q[-2:] = opening
                    exp.step(q)
                for _ in range(20):exp.step(q)
                if not exp.trace[-1]['bilateral_contact']:
                    raise TableRegraspError('side grasp did not close on both sides')
                exp.update_payload()
                ee = p.getLinkState(env.robot_id,planner.ee,computeForwardKinematics=True)
                positions = np.array(ee[4])+np.linspace(0,.24,35)[1:,None]*[0.,0.,1.]
                lift = exp.query(lambda:planner.cartesian(env.agent_qpos.astype(float),positions,ee[5]))
                if lift is None:raise TableRegraspError('side lift blocked')
                for q in lift:q[-2:]=0.
                exp.move(lift,'side lift',increment=.012)
                planner.payload_support = None
                exp.phase='side hold'
                for _ in range(50):exp.step(lift[-1])
                if not exp.trace[-1]['bilateral_contact']:
                    raise TableRegraspError('side grasp lost contact during lift')
                return dict(success=True, grasp=grasp.center.tolist(),
                            physics_steps=env.step_count*env.substeps-started)
        raise TableRegraspError('no collision-free side grasp')
    finally:
        planner.payload_support = None
