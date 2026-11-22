import unittest
from unittest.mock import patch

import numpy as np
import pybullet as p

from controller import GraspExperiment
from full_task import FullTaskExperiment, TaskFailure
from pick_nav_reach_env import PickNavReachEnv


class TargetInterpolationTests(unittest.TestCase):
    def sample_motion(self, high_rate):
        env = PickNavReachEnv(seed=0, object_idx=2, gui=False)
        try:
            experiment = GraspExperiment(env, 'tmp/test-target-interpolation', high_rate=high_rate)
            dynamics = p.getDynamicsInfo(env.object_id, -1)
            engine = p.getPhysicsEngineParameters()
            q = env.agent_qpos.astype(float)
            samples = []
            simulate = p.stepSimulation
            def tick():
                simulate()
                samples.append(p.getJointState(env.robot_id, env.joint_indices[0])[1])
            with patch.object(p, 'stepSimulation', side_effect=tick):
                for i in range(1, 49):
                    q[0] = .3*i*experiment.control_dt
                    experiment.step(q)
            self.assertEqual(env.step_count*env.substeps, 240)
            self.assertEqual(experiment.control_steps, 48)
            self.assertEqual(len(samples), 240)
            self.assertEqual(p.getDynamicsInfo(env.object_id, -1), dynamics)
            self.assertEqual(p.getPhysicsEngineParameters(), engine)
            self.assertLess(abs(env.agent_qpos[0]-.3), .006)
            return np.array(samples[-100:])
        finally:
            env.close()

    def test_per_tick_targets_remove_the_hold_and_brake_velocity_pulses(self):
        coarse = self.sample_motion(False)
        smooth = self.sample_motion(True)
        self.assertGreater(np.ptp(coarse), .2)
        self.assertLess(np.ptp(smooth), .02)
        self.assertLess(abs(smooth.mean()-.3), .01)

    def test_counter_unit_conversion_preserves_elapsed_physics(self):
        env = PickNavReachEnv(seed=0, object_idx=2, gui=False)
        try:
            env.step_count = 17
            before = env.step_count*env.substeps
            experiment = GraspExperiment(env, 'tmp/test-clock-conversion')
            self.assertEqual(env.step_count*env.substeps, before)
            experiment.step(env.agent_qpos.astype(float))
            self.assertEqual(env.step_count*env.substeps, before+5)
        finally:
            env.close()

    def test_partial_remaining_budget_is_never_overrun(self):
        env = PickNavReachEnv(seed=0, object_idx=2, gui=False)
        try:
            experiment = FullTaskExperiment(env, 'tmp/test-partial-budget')
            env.step_count = 14999
            self.assertEqual(env.substeps, 1)
            with self.assertRaises(TaskFailure):
                experiment.step(env.agent_qpos.astype(float))
            self.assertEqual(env.step_count, 14999)
        finally:
            env.close()


if __name__ == '__main__':
    unittest.main()
