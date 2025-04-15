import numpy as np
import random
from gymnasium import Env, spaces
from utils.helpers import extract_local_mask

class ToolpathEnv(Env):
    def __init__(self, mask, grid_size, vision_radius, zone_count):
        super().__init__()
        self.grid_size = grid_size
        self.vision_radius = vision_radius
        self.zone_size = grid_size // zone_count
        self.visitable = (mask == 1)
        self.visited = np.zeros_like(mask, dtype=bool)
        self.visit_counts = np.zeros_like(mask, dtype=int)
        self.total_to_cover = set(zip(*np.where(self.visitable)))
        self.max_steps = int(len(self.total_to_cover) * 2)
        self.action_space = spaces.Discrete(8)
        self.reset()

    def reset(self, seed=None, options=None):
        valid = list(zip(*np.where(self.visitable)))
        self.agent_pos = random.choice(valid)
        self.visited[:, :] = False
        self.visit_counts[:, :] = 0
        self.covered = set()
        self.path = [self.agent_pos]
        self.revisits = 0
        self.steps = 0
        self.done = False
        return self.get_obs(), {}

    def get_obs(self):
        x, y = self.agent_pos
        mask = extract_local_mask(self.visited, x, y, self.vision_radius)
        zone = (x // self.zone_size, y // self.zone_size)
        return (x, y, zone, mask)

    def step(self, action):
        if self.done:
            return self.get_obs(), 0, True, False, {}

        ACTIONS = [(-1, 0), (1, 0), (0, 1), (0, -1), (-1, 1), (-1, -1), (1, 1), (1, -1)]
        dx, dy = ACTIONS[action]
        nx, ny = self.agent_pos[0] + dx, self.agent_pos[1] + dy

        if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size and self.visitable[nx, ny]:
            self.agent_pos = (nx, ny)

        self.path.append(self.agent_pos)
        self.steps += 1
        self.visit_counts[self.agent_pos] += 1

        REWARD_MOVE = -0.1
        REWARD_NEW = 1.0
        REWARD_REVISIT = -0.3
        REWARD_COMPLETE = 10.0
        reward = REWARD_MOVE

        if self.agent_pos in self.total_to_cover:
            if not self.visited[self.agent_pos]:
                self.visited[self.agent_pos] = True
                self.covered.add(self.agent_pos)
                reward += REWARD_NEW
            else:
                self.revisits += 1
                reward += REWARD_REVISIT

        if self.covered == self.total_to_cover:
            reward += REWARD_COMPLETE
            self.done = True
        elif self.steps >= self.max_steps:
            self.done = True

        return self.get_obs(), reward, self.done, False, {}
