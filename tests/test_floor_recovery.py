import unittest

import pybullet as p

from floor_recovery import FloorRecovery
from full_task import FullTaskExperiment, TaskFailure
from pick_nav_reach_env import PickNavReachEnv


class FloorRecoveryTests(unittest.TestCase):
    def test_recovery_cannot_reset_or_exceed_the_task_step_budget(self):
        env = PickNavReachEnv(seed=0, object_idx=2, gui=False)
        try:
            p.resetBasePositionAndOrientation(env.object_id, [-1., 0., .12], [0., 0., 0., 1.])
            env.step_count = 2999
            experiment = FullTaskExperiment(env, 'tmp/test-recovery-budget')
            recovery = FloorRecovery(experiment)
            with self.assertRaisesRegex(TaskFailure, '15000'):
                recovery.run()
            self.assertEqual(env.step_count*env.substeps, 15000)
            outcome = recovery.finish(False, 'time limit')
            self.assertEqual(outcome['start_physics_steps'], 14995)
            self.assertEqual(outcome['duration_physics_steps'], 5)
            self.assertFalse(experiment.respect_joint_limits)
            self.assertIsNone(experiment.planner.payload_support)
        finally:
            env.close()

    def test_physical_floor_pick_preserves_robot_and_object_properties(self):
        env = PickNavReachEnv(seed=0, object_idx=2, gui=False)
        try:
            # Isolated initial condition, before execution starts.
            p.resetBasePositionAndOrientation(env.object_id, [-1., 0., .12], [0., 0., 0., 1.])
            experiment = FullTaskExperiment(env, 'tmp/test-floor-recovery')
            before = p.getDynamicsInfo(env.object_id, -1)
            finger_properties = [p.getDynamicsInfo(env.robot_id, link) for link in sorted(experiment.planner.fingers)]
            constraints = p.getNumConstraints()
            result = FloorRecovery(experiment).run()
            self.assertTrue(result['success'], result)
            self.assertGreaterEqual(result['stable_hold_control_steps'], 50)
            self.assertGreater(result['final_floor_clearance_m'], .1)
            self.assertEqual(result['duration_physics_steps'], env.step_count*env.substeps)
            self.assertEqual(p.getNumConstraints(), constraints)
            self.assertEqual(p.getDynamicsInfo(env.object_id, -1), before)
            self.assertEqual([p.getDynamicsInfo(env.robot_id, link) for link in sorted(experiment.planner.fingers)], finger_properties)
            self.assertLess(max(r['arm_floor_penetration_m'] for r in experiment.trace), .005)
            self.assertFalse(experiment.respect_joint_limits)
            self.assertIsNone(experiment.planner.payload_support)
        finally:
            env.close()


if __name__ == '__main__':
    unittest.main()
