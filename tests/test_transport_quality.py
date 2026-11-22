import unittest

import numpy as np

from grasping import wrench_grasps


class TransportWrenchTests(unittest.TestCase):
    @staticmethod
    def box_cloud():
        yz = np.array([(y, z) for y in np.linspace(-.01, .01, 7)
                       for z in np.linspace(.665, .685, 7)])
        points = np.vstack([np.column_stack((np.full(len(yz), x), yz)) for x in [-.025, .025]])
        normals = np.repeat([[-1., 0., 0.], [1., 0., 0.]], len(yz), axis=0)
        return points, normals

    def test_capacity_matches_force_balance_with_opposing_flat_pads(self):
        points, normals = self.box_cloud()
        grasps = wrench_grasps(points, normals, .626, mu=.25, max_grasps=8, carry_mode='topdown')
        self.assertTrue(grasps)
        # Check both analytic limits: unequal normal loading, and the facet
        # between the 0 and 45 degree rays of the inscribed friction octagon.
        normal_limit = 120*.25/(9.8+.25*.8)
        tangent_limit = 120*.25/(9.8+np.tan(np.pi/8)*.8)
        expected = min(normal_limit, tangent_limit)
        best = max(g.transport_capacity_kg for g in grasps)
        self.assertAlmostEqual(best, expected, places=6)

    def test_friction_changes_capacity_without_changing_the_cloud(self):
        points, normals = self.box_cloud()
        before = points.copy()
        low = wrench_grasps(points, normals, .626, mu=.25, max_grasps=8, carry_mode='topdown')
        high = wrench_grasps(points, normals, .626, mu=.5, max_grasps=8, carry_mode='topdown')
        self.assertGreater(max(g.transport_capacity_kg for g in high),
                           max(g.transport_capacity_kg for g in low))
        np.testing.assert_array_equal(points, before)

    def test_no_contacts_does_not_produce_a_capacity_claim(self):
        self.assertEqual(wrench_grasps(np.empty((0, 3)), np.empty((0, 3)), .626), [])


if __name__ == '__main__':
    unittest.main()
