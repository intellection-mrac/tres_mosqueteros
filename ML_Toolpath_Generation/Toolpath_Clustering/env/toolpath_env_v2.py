import numpy as np

class ToolpathEnvV2:
    def __init__(self, mask):
        self.mask = mask
        self.grid_size = mask.shape[0]
        self.visitable = (mask == 1)
        self.visited = np.zeros_like(mask, dtype=bool)
        self.visit_counts = np.zeros_like(mask, dtype=int)
        self.total_to_cover = set(zip(*np.where(self.visitable)))
        self.max_steps = int(len(self.total_to_cover) * 2)
        self.reset()

    def reset(self):
        valid = list(zip(*np.where(self.visitable)))
        self.agent_pos = valid[np.random.randint(len(valid))]
        self.visited[:, :] = False
        self.visit_counts[:, :] = 0
        self.covered = set()
        self.path = [self.agent_pos]
        self.steps = 0
        self.done = False
        return self.agent_pos

    def step_to(self, next_pos):
        if self.done:
            return self.agent_pos, 0, True

        if next_pos not in self.total_to_cover:
            return self.agent_pos, -1, False

        self.agent_pos = next_pos
        self.path.append(next_pos)
        self.steps += 1
        self.visit_counts[next_pos] += 1

        reward = -0.1
        if not self.visited[next_pos]:
            self.visited[next_pos] = True
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
