"""Holonomic, fixed-heading A* navigation for the supplied Fetch simulator.

Walls are inflated by the measured carrying footprint. Every grid edge and
shortcut is checked against the inflated rectangles, including its endpoints.
"""
import heapq
import itertools
import numpy as np


class GridNavigator:
    def __init__(self, cube_positions, footprint, extra_boxes=(), resolution=.1, margin=.025):
        cubes = np.asarray(cube_positions, float).reshape(-1, 2)
        self.resolution = float(resolution)
        if not np.isfinite(self.resolution) or self.resolution <= 0:
            raise ValueError('resolution must be positive and finite')
        low, high = np.asarray(footprint, float)
        if low.shape != (2,) or high.shape != (2,) or np.any(low > high):
            raise ValueError('footprint must be ordered 2D lower/upper offsets')
        boxes = [(c-.5, c+.5) for c in cubes] + list(extra_boxes)
        # Align grid samples with corridor centers after asymmetric footprint
        # inflation; anchoring at the start can miss a narrow but valid lane.
        self.lattice_origin = cubes[0]-(low+high)/2 if len(cubes) else np.zeros(2)
        self.boxes = np.array([(np.asarray(a)-high-margin, np.asarray(b)-low+margin)
                               for a, b in boxes]).reshape(-1, 2, 2)
        self.expanded = 0

    def clear_edge(self, start, end):
        """Exact segment versus closed axis-aligned rectangles (slab test)."""
        start, end = np.asarray(start, float), np.asarray(end, float)
        delta = end-start
        for low, high in self.boxes:
            enter, leave = 0., 1.
            for axis in range(2):
                if abs(delta[axis]) < 1e-12:
                    if start[axis] < low[axis] or start[axis] > high[axis]:
                        break
                else:
                    a, b = sorted(((low[axis]-start[axis])/delta[axis],
                                   (high[axis]-start[axis])/delta[axis]))
                    enter, leave = max(enter, a), min(leave, b)
                if enter > leave:
                    break
            else:
                return False
        return True

    def plan(self, start, goal):
        start, goal = np.asarray(start, float), np.asarray(goal, float)
        if not np.isfinite(np.r_[start, goal]).all():
            raise ValueError('start and goal must be finite')
        if not self.clear_edge(start, start) or not self.clear_edge(goal, goal):
            return None
        if self.clear_edge(start, goal):
            return [start, goal]
        # Bound the search to the observed scene and endpoints.
        points = np.vstack((start, goal, self.boxes.reshape(-1, 2)))
        lower = np.floor((points.min(axis=0)-1)/self.resolution).astype(int)
        upper = np.ceil((points.max(axis=0)+1)/self.resolution).astype(int)
        # Connect the exact start to a nearby lattice sample with a checked edge.
        def world(node):
            return self.lattice_origin+np.array(node)*self.resolution
        nearest = np.rint((start-self.lattice_origin)/self.resolution).astype(int)
        candidates = [tuple(nearest+[dx, dy]) for dx in (-1, 0, 1) for dy in (-1, 0, 1)]
        candidates.sort(key=lambda node: np.linalg.norm(world(node)-start))
        origin = next((node for node in candidates if self.clear_edge(start, world(node))), None)
        if origin is None:
            return None
        initial_cost = np.linalg.norm(world(origin)-start)
        costs, parents = {origin: initial_cost}, {origin: None}
        queue, serial = [], itertools.count()
        heapq.heappush(queue, (initial_cost+np.linalg.norm(goal-world(origin)), next(serial), origin, initial_cost))
        directions = [(x, y) for x in (-1, 0, 1) for y in (-1, 0, 1) if x or y]
        free_cache = {}
        self.expanded = 0
        while queue:
            _, _, node, cost = heapq.heappop(queue)
            if cost > costs[node]+1e-10:
                continue
            pos = world(node)
            self.expanded += 1
            if np.linalg.norm(pos-goal) <= self.resolution*1.5 and self.clear_edge(pos, goal):
                path = [goal]
                while node is not None:
                    path.append(world(node))
                    node = parents[node]
                path.reverse()
                if np.linalg.norm(path[0]-start) > 1e-9:
                    path.insert(0, start)
                simplified, i = [path[0]], 0
                while i < len(path)-1:
                    j = len(path)-1
                    while j > i+1 and not self.clear_edge(path[i], path[j]):
                        j -= 1
                    simplified.append(path[j])
                    i = j
                return simplified
            for dx, dy in directions:
                neighbor = (node[0]+dx, node[1]+dy)
                nxt = world(neighbor)
                if np.any(nxt < lower*self.resolution) or np.any(nxt > upper*self.resolution):
                    continue
                new_cost = cost+self.resolution*np.hypot(dx, dy)
                if new_cost >= costs.get(neighbor, np.inf):
                    continue
                key = tuple(sorted((node, neighbor)))
                if key not in free_cache:
                    free_cache[key] = self.clear_edge(pos, nxt)
                if not free_cache[key]:
                    continue
                costs[neighbor], parents[neighbor] = new_cost, node
                heapq.heappush(queue, (new_cost+np.linalg.norm(nxt-goal), next(serial), neighbor, new_cost))
        return None
