import unittest
import numpy as np

from navigation import GridNavigator


class NavigationTests(unittest.TestCase):
    def test_routes_around_wall_with_payload_footprint(self):
        nav = GridNavigator([[0, 0]], ([-.3, -.3], [.45, .3]))
        path = nav.plan([-2, 0], [2, 0])
        self.assertIsNotNone(path)
        self.assertGreater(len(path), 2)
        np.testing.assert_allclose(path[0], [-2, 0])
        np.testing.assert_allclose(path[-1], [2, 0])
        self.assertTrue(all(nav.clear_edge(a, b) for a, b in zip(path, path[1:])))

    def test_thin_segment_intersection_and_boundary(self):
        nav = GridNavigator([], ([0, 0], [0, 0]),
                            extra_boxes=[([0, -.001], [.001, .001])], margin=0)
        self.assertFalse(nav.clear_edge([-1, 0], [1, 0]))
        self.assertFalse(nav.clear_edge([0, -.01], [0, .01]))
        self.assertTrue(nav.clear_edge([-1, .01], [1, .01]))

    def test_no_diagonal_corner_cutting(self):
        nav = GridNavigator([[0, 1], [1, 0]], ([-.25, -.25], [.25, .25]), margin=0)
        self.assertFalse(nav.clear_edge([0, 0], [1, 1]))

    def test_blocked_endpoint_and_sealed_room(self):
        nav = GridNavigator([[0, 0]], ([-.3, -.3], [.3, .3]))
        self.assertIsNone(nav.plan([0, 0], [2, 2]))
        walls = [(x, y) for x in range(-2, 3) for y in range(-2, 3)
                 if abs(x) == 2 or abs(y) == 2]
        nav = GridNavigator(walls, ([-.3, -.3], [.3, .3]))
        self.assertIsNone(nav.plan([0, 0], [4, 0]))

    def test_asymmetric_payload_and_empty_scene(self):
        nav = GridNavigator([[0, 0]], ([-.2, -.2], [.6, .2]), margin=0)
        self.assertFalse(nav.clear_edge([-1, 0], [-1, 0]))
        self.assertTrue(nav.clear_edge([1, 0], [1, 0]))
        empty = GridNavigator([], ([-.3, -.3], [.3, .3]))
        self.assertEqual(len(empty.plan([0, 0], [4, 3])), 2)

    def test_grid_alignment_finds_narrow_asymmetric_corridor(self):
        free = {(1.5, y) for y in (-1.5, -.5, .5, 1.5)} | {(2.5, 1.5), (3.5, 1.5)}
        walls = [(x, y) for x in np.arange(.5, 5, 1) for y in np.arange(-2.5, 3, 1)
                 if (x, y) not in free]
        nav = GridNavigator(walls, ([-.29, -.3], [.62, .3]))
        # Safe base x lies in [1.315, 1.355], narrower than the 0.1 m grid.
        path = nav.plan([1.33, -1.5], [3.33, 1.5])
        self.assertIsNotNone(path)
        self.assertGreater(len(path), 2)
        self.assertTrue(all(nav.clear_edge(a, b) for a, b in zip(path, path[1:])))


if __name__ == '__main__':
    unittest.main()
