import unittest
from unittest.mock import patch
import numpy as np
import pybullet as p

from controller import GraspExperiment, HoldController
from grasping import antipodal_margin, generate_grasps
from pick_nav_reach_env import PickNavReachEnv
from full_task import FullTaskExperiment


class GeometryTests(unittest.TestCase):
    def test_inward_cone_signs_and_strict_boundary(self):
        points = np.array([[-.025, 0, .7], [.025, 0, .7]])
        normals = np.array([[-1., 0, 0], [1., 0, 0]])
        self.assertGreater(antipodal_margin(points, normals), 0)
        self.assertLess(antipodal_margin(points, -normals), 0)
        boundary = np.array([[-1., .5, 0], [1., .5, 0]])
        self.assertAlmostEqual(antipodal_margin(points, boundary), 0, places=12)
        self.assertLess(antipodal_margin(points[[0, 0]], normals), 0)

    def test_proposals_respect_aperture_cones_and_right_handed_frame(self):
        yz = np.array([(y, z) for y in np.linspace(-.03, .03, 9) for z in np.linspace(.65, .70, 9)])
        points = np.vstack([np.column_stack((np.full(len(yz), x), yz)) for x in [-.025, .025]])
        normals = np.repeat([[-1., 0, 0], [1., 0, 0]], len(yz), axis=0)
        grasps = generate_grasps(points, normals, .626)
        self.assertTrue(grasps)
        for g in grasps:
            self.assertGreater(antipodal_margin(g.contacts, g.normals), 0)
            self.assertLess(g.width, .092)
            np.testing.assert_allclose(g.rotation.T @ g.rotation, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(g.rotation), 1.)

    def test_bad_inputs_and_ungraspable_cloud(self):
        with self.assertRaises(ValueError):
            generate_grasps(np.zeros((2, 3)), np.zeros((3, 3)), .6)
        with self.assertRaises(ValueError):
            antipodal_margin(np.zeros((2, 3)), np.ones((2, 3)), mu=0)
        self.assertEqual(generate_grasps(np.zeros((2, 3)), np.zeros((2, 3)), .6), [])

    def test_hold_copies_input_and_output(self):
        obs = dict(qpos=np.zeros(15))
        hold = HoldController(obs)
        obs['qpos'][0] = 2
        action = hold.get_action(obs)
        action[0] = 3
        self.assertEqual(hold.get_action(obs)[0], 0)


class SimulatorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.env = PickNavReachEnv(seed=42, gui=False)
        cls.experiment = GraspExperiment(cls.env, 'tmp/test-output')

    @classmethod
    def tearDownClass(cls):
        cls.env.close()

    def test_ik_fixes_base_and_planning_restores_state(self):
        env, exp = self.env, self.experiment
        start = env.agent_qpos.astype(float)
        q = start.copy()
        q[0], q[3], q[7], q[9], q[11] = 1.25, .3, -.65, 1.5, .72
        def query():
            position, rotation = exp.planner.pose(q)
            result = exp.planner.ik(position+[0, 0, .03], rotation.as_quat(), q, attempts=1)
            self.assertIsNotNone(result)
            np.testing.assert_allclose(result[:3], q[:3])
            actual, actual_rot = exp.planner.pose(result)
            self.assertLess(np.linalg.norm(actual-(position+[0, 0, .03])), .003)
            self.assertLess((rotation*actual_rot.inv()).magnitude(), .025)
            return result
        exp.query(query)
        np.testing.assert_array_equal(env.agent_qpos, start)
        def tabletop_collision():
            buried = exp.planner.ik(np.array([0, 0, .63]),
                p.getQuaternionFromEuler([0, np.pi/2, 0]), q)
            self.assertIsNotNone(buried)
            return exp.planner.clear(buried)
        self.assertFalse(exp.query(tabletop_collision))

    def test_action_validation(self):
        with self.assertRaises(ValueError):
            self.env.step(np.full(15, np.nan))
        with self.assertRaises(ValueError):
            self.env.step(np.zeros(3))

    def test_payload_queries_restore_object_and_check_obstacles(self):
        env, exp = self.env, self.experiment
        q = env.agent_qpos.astype(float)
        before = p.getBasePositionAndOrientation(env.object_id)
        ee = p.getLinkState(env.robot_id, exp.planner.ee, computeForwardKinematics=True)
        # Place a hypothetical payload in the tabletop; collision must reject it.
        exp.planner.payload = p.multiplyTransforms(*p.invertTransform(ee[4], ee[5]),
                                                   [0, 0, .6], before[1])
        try:
            self.assertFalse(exp.query(lambda: exp.planner.clear(q)))
            after = p.getBasePositionAndOrientation(env.object_id)
            np.testing.assert_allclose(after[0], before[0])
            np.testing.assert_allclose(after[1], before[1])
            np.testing.assert_array_equal(env.agent_qpos, q)
        finally:
            exp.planner.payload = None

    def test_transport_never_relaxes_finger_targets(self):
        task = FullTaskExperiment(self.env, 'tmp/test-full-output')
        task.update_payload()
        task.carry_reference = np.array(task.planner.payload[0])
        task.transport_started = True
        action = self.env.agent_qpos.astype(float)
        action[-2:] = .05
        with patch.object(GraspExperiment, 'step', return_value=self.env._get_obs()) as step:
            task.step(action)
        np.testing.assert_array_equal(step.call_args.args[0][-2:], [0, 0])
        np.testing.assert_array_equal(action[-2:], [.05, .05])


if __name__ == '__main__':
    unittest.main()
