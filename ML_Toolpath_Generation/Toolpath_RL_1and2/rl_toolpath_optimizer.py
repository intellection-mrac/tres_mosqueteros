import os
import shutil
import numpy as np
import matplotlib.pyplot as plt
import random
import csv
from gymnasium import Env, spaces

# ----------------------- CONFIG -----------------------
GRID_SIZE = 20
EPISODES = 5000
EXPORTS = 10
ALPHA = 0.1
GAMMA = 0.99
EPSILON = 1.0
EPSILON_DECAY = 0.995
MIN_EPSILON = 0.01
REWARD_NEW = 1.0
REWARD_REVISIT = -0.3
REWARD_MOVE = -0.1
REWARD_COMPLETE = 10.0
EXPORT_PATH = "outputs/toolpath_rl"
maxsteps = 4

ACTIONS = [
    (-1, 0),  # N
    (1, 0),   # S
    (0, 1),   # E
    (0, -1),  # W
    (-1, 1),  # NE
    (-1, -1), # NW
    (1, 1),   # SE
    (1, -1),  # SW
]

# ------------------- UTILITY FUNCTIONS -------------------
def generate_blob(grid_size, num_points=100, radius=15, noise_scale=1.0, seed=None):
    if seed is not None:
        np.random.seed(seed)

    center = (grid_size // 2, grid_size // 2)
    angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    noise = np.random.normal(0, 1, num_points) * noise_scale
    radii = radius + noise

    boundary_points = []
    for angle, r in zip(angles, radii):
        x = int(center[0] + r * np.cos(angle))
        y = int(center[1] + r * np.sin(angle))
        boundary_points.append((x, y))

    blob_mask = np.zeros((grid_size, grid_size), dtype=int)
    for x in range(grid_size):
        for y in range(grid_size):
            dx, dy = x - center[0], y - center[1]
            angle = (np.arctan2(dy, dx) + 2 * np.pi) % (2 * np.pi)
            index = int(angle / (2 * np.pi) * num_points)
            if np.hypot(dx, dy) <= radii[index]:
                blob_mask[x, y] = 1

    valid_points = set(zip(*np.where(blob_mask == 1)))
    return valid_points

def find_random_valid_position(valid_points):
    return random.choice(list(valid_points))

def get_export_folder(base_path):
    os.makedirs("outputs", exist_ok=True)
    count = 1
    path = f"{base_path}_{count:02d}"
    while os.path.exists(path):
        count += 1
        path = f"{base_path}_{count:02d}"
    os.makedirs(path)
    return path

def save_path_plot(valid_points, path, export_file):
    fig, ax = plt.subplots()
    for x, y in valid_points:
        ax.add_patch(plt.Rectangle((y, x), 1, 1, edgecolor='lightgray', facecolor='white'))
        ax.plot(y + 0.5, x + 0.5, 'ko', markersize=2)
    if path:
        px, py = zip(*path)
        ax.plot([y + 0.5 for y in py], [x + 0.5 for x in px], 'r-', linewidth=1)
        ax.plot(py[0] + 0.5, px[0] + 0.5, 'go', label='Start')
    ax.set_xlim(0, GRID_SIZE)
    ax.set_ylim(0, GRID_SIZE)
    ax.set_aspect('equal')
    ax.invert_yaxis()
    ax.axis('off')
    plt.savefig(export_file, bbox_inches='tight', dpi=300)
    plt.close()

def save_path_csv(path, file, episode, total_reward, norm_reward):
    with open(file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['episode', episode])
        writer.writerow(['total_reward', total_reward])
        writer.writerow(['normalized_reward', norm_reward])
        writer.writerow(['step', 'x', 'y'])
        for idx, (x, y) in enumerate(path):
            writer.writerow([idx, x, y])

def save_progress_log(log, file):
    with open(file, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=log[0].keys())
        writer.writeheader()
        for row in log:
            writer.writerow(row)

# ------------------- ENVIRONMENT -------------------
class ToolpathEnv(Env):
    def __init__(self, valid_points, grid_size):
        super().__init__()
        self.valid_points = set(valid_points)
        self.grid_size = grid_size
        self.action_space = spaces.Discrete(8)
        self.observation_space = spaces.Box(low=0, high=self.grid_size, shape=(2,), dtype=np.int32)
        self.total_to_cover = set(valid_points)
        self.max_steps = int(len(self.total_to_cover) * maxsteps)
        self.reset()

    def reset(self, seed=None, options=None):
        self.agent_pos = find_random_valid_position(self.valid_points)
        self.covered = set()
        self.path = [self.agent_pos]
        self.revisits = 0
        self.steps = 0
        self.done = False
        return np.array(self.agent_pos), {}

    def step(self, action):
        if self.done:
            return np.array(self.agent_pos), 0, True, False, {}

        dx, dy = ACTIONS[action]
        nx, ny = self.agent_pos[0] + dx, self.agent_pos[1] + dy

        if (0 <= nx < self.grid_size and 0 <= ny < self.grid_size and (nx, ny) in self.valid_points):
            self.agent_pos = (nx, ny)

        self.path.append(self.agent_pos)
        self.steps += 1
        reward = REWARD_MOVE

        if self.agent_pos in self.total_to_cover:
            if self.agent_pos not in self.covered:
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

        return np.array(self.agent_pos), reward, self.done, False, {}

# ------------------- MAIN TRAINING LOOP -------------------
folder = get_export_folder(EXPORT_PATH)
progress_log = []
export_every = EPISODES // EXPORTS
q_table = np.zeros((GRID_SIZE, GRID_SIZE, 8))
epsilon = EPSILON

for episode in range(EPISODES):
    valid_points = generate_blob(GRID_SIZE, radius=15, noise_scale=2.0, seed=episode)
    env = ToolpathEnv(valid_points, GRID_SIZE)
    state, _ = env.reset()
    done = False
    total_reward = 0
    start_pos = tuple(state)
    step_counter = 0
    reevaluation_interval = 10

    while not done:
        x, y = state

        if step_counter % reevaluation_interval == 0:
            remaining = list(env.total_to_cover - env.covered)
            if remaining:
                closest = min(remaining, key=lambda pt: abs(pt[0] - x) + abs(pt[1] - y))
                dx = closest[0] - x
                dy = closest[1] - y
                preferred_direction = np.argmax([
                    -abs(dx + ax) - abs(dy + ay) for (ax, ay) in ACTIONS
                ])
            else:
                preferred_direction = None
        else:
            preferred_direction = None

        if random.uniform(0, 1) < epsilon:
            action = random.randint(0, 7)
        elif preferred_direction is not None:
            action = preferred_direction
        else:
            action = np.argmax(q_table[x, y])

        next_state, reward, done, _, _ = env.step(action)
        nx, ny = next_state
        q_table[x, y, action] = (1 - ALPHA) * q_table[x, y, action] + ALPHA * (reward + GAMMA * np.max(q_table[nx, ny]))
        state = next_state
        total_reward += reward
        step_counter += 1

    epsilon = max(MIN_EPSILON, epsilon * EPSILON_DECAY)
    norm_reward = total_reward / len(env.total_to_cover)
    max_steps_hit = env.steps >= env.max_steps

    progress_log.append({
        'episode': episode,
        'total_reward': total_reward,
        'normalized_reward': norm_reward,
        'milling_points': len(env.total_to_cover),
        'path_length': len(env.path),
        'revisit_count': env.revisits,
        'steps_taken': env.steps,
        'max_steps_reached': max_steps_hit,
        'start_position': start_pos,
        'completion_status': env.covered == env.total_to_cover
    })

    if (episode + 1) % export_every == 0 or (episode == EPISODES - 1):
        img_path = os.path.join(folder, f"ep_{episode:03d}.png")
        csv_path = os.path.join(folder, f"ep_{episode:03d}.csv")
        save_path_plot(env.valid_points, env.path, img_path)
        save_path_csv(env.path, csv_path, episode, total_reward, norm_reward)

    if (episode + 1) % (EPISODES // 10) == 0 or episode == EPISODES - 1:
        percent_done = int(((episode + 1) / EPISODES) * 100)
        print(f"Progress: {percent_done}% ({episode + 1}/{EPISODES} episodes completed)")

save_progress_log(progress_log, os.path.join(folder, "progress_log.csv"))
print(f"Training complete. Outputs saved in: {folder}")
