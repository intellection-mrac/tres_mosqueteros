import numpy as np
from gymnasium import Env

class ToolpathEnvV2_GR(Env):
    def __init__(self, points_uv, resolution=100):
        super().__init__()
        self.points = np.array(points_uv)
        self.resolution = resolution

        self.min_bounds = self.points.min(axis=0)
        self.max_bounds = self.points.max(axis=0)
        self.scale = (self.max_bounds - self.min_bounds) / resolution

        self.valid_points = set(tuple(p) for p in self.points)
        self.total_to_cover = set(tuple(p) for p in self.points)

        self.visited_grid = np.zeros((resolution, resolution), dtype=bool)
        self.visit_counts = np.zeros((resolution, resolution), dtype=int)

        self.max_steps = int(len(self.total_to_cover) * 2)
        self.reset()

    def reset(self):
        self.agent_pos = self.points[np.random.randint(len(self.points))]
        self.covered = set()
        self.steps = 0
        self.done = False
        self.path = [tuple(self.agent_pos)]
        self.visited_grid[:, :] = False
        self.visit_counts[:, :] = 0
        return self.agent_pos

    def _grid_index(self, point):
        normalized = (np.array(point) - self.min_bounds) / self.scale
        return tuple(np.clip(normalized.astype(int), 0, self.resolution - 1))

    def step_to(self, next_pos):
        if self.done:
            return self.agent_pos, 0, True

        next_pos = tuple(next_pos)
        if next_pos not in self.total_to_cover:
            return self.agent_pos, -1, False

        self.agent_pos = next_pos
        self.path.append(next_pos)
        self.steps += 1

        gx, gy = self._grid_index(next_pos)
        self.visit_counts[gx, gy] += 1
        reward = -0.1

        if not self.visited_grid[gx, gy]:
            self.visited_grid[gx, gy] = True
            self.covered.add(next_pos)
            reward += 1.0
        else:
            reward -= 0.3

        if self.covered == self.total_to_cover:
            reward += 10.0
            self.done = True
        elif self.steps >= self.max_steps:
            self.done = True

        return self.agent_pos, reward, self.done

    def get_path(self):
        return self.path

    def get_bounds(self):
        return self.min_bounds, self.max_bounds

    def get_resolution(self):
        return self.resolution