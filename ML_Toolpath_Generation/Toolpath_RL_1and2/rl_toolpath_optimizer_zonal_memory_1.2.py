import os
import numpy as np
import matplotlib.pyplot as plt
import random
import csv
import pandas as pd
from gymnasium import Env, spaces
from collections import defaultdict

# ----------------------- CONFIG -----------------------
GRID_SIZE = 50
ZONE_COUNT = 5
VISION_RADIUS = max(1, GRID_SIZE // 25)
EPISODES = 500
EXPORTS = 10
ALPHA = 0.1
GAMMA = 0.99
EPSILON = 1.0
EPSILON_DECAY = 0.999
MIN_EPSILON = 0.01
REWARD_NEW = 1.0
REWARD_REVISIT = -0.3
REWARD_MOVE = -0.1
REWARD_COMPLETE = 10.0
EXPORT_PATH = "outputs/toolpath_rl_zonal"

ACTIONS = [
    (-1, 0), (1, 0), (0, 1), (0, -1),
    (-1, 1), (-1, -1), (1, 1), (1, -1),
]

# ------------------- FILE & FOLDER HANDLING -------------------
def get_script_name():
    return os.path.splitext(os.path.basename(__file__))[0]

def get_versioned_output_folder():
    base_folder = os.path.join(EXPORT_PATH, get_script_name())
    os.makedirs(base_folder, exist_ok=True)
    count = 1
    while os.path.exists(os.path.join(base_folder, f"run_{count:02d}")):
        count += 1
    versioned_folder = os.path.join(base_folder, f"run_{count:02d}")
    os.makedirs(versioned_folder)
    return versioned_folder

def save_episode_csv(log, output_path):
    keys = log[0].keys()
    with open(output_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(log)

# ------------------- ENV HELPERS -------------------
def extract_local_mask(visited, x, y, radius):
    r = radius
    mask = []
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            nx, ny = x + dx, y + dy
            if 0 <= nx < visited.shape[0] and 0 <= ny < visited.shape[1]:
                mask.append(int(visited[nx, ny]))
            else:
                mask.append(1)
    return tuple(mask)

def generate_blob(grid_size, num_points=100, radius=15, noise_scale=1.0, seed=None):
    if seed is not None:
        np.random.seed(seed)

    center = (grid_size // 2, grid_size // 2)
    angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    noise = np.random.normal(0, 1, num_points) * noise_scale
    radii = radius + noise

    blob_mask = np.zeros((grid_size, grid_size), dtype=int)
    for x in range(grid_size):
        for y in range(grid_size):
            dx, dy = x - center[0], y - center[1]
            angle = (np.arctan2(dy, dx) + 2 * np.pi) % (2 * np.pi)
            index = int(angle / (2 * np.pi) * num_points)
            if np.hypot(dx, dy) <= radii[index]:
                blob_mask[x, y] = 1

    return blob_mask

def plot_agent_path_with_heatmap(mask, visit_counts, path, zone_size, output_path):
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_aspect('equal')

    for x in range(mask.shape[0]):
        for y in range(mask.shape[1]):
            if mask[x, y]:
                visits = visit_counts[x, y]
                color = plt.cm.Reds(min(1.0, visits / 5)) if visits > 0 else 'white'
                ax.add_patch(plt.Rectangle((y, x), 1, 1, facecolor=color, edgecolor='lightgray'))
                ax.plot(y + 0.5, x + 0.5, 'k.', markersize=1.5)

    if path:
        px, py = zip(*path)
        ax.plot([y + 0.5 for y in py], [x + 0.5 for x in px], 'b-', linewidth=1)
        ax.plot(py[0] + 0.5, px[0] + 0.5, 'go')
        ax.plot(py[-1] + 0.5, px[-1] + 0.5, 'ro')

    for i in range(1, mask.shape[0] // zone_size):
        ax.axhline(i * zone_size, color='black', linewidth=0.5)
        ax.axvline(i * zone_size, color='black', linewidth=0.5)

    ax.set_xlim(0, mask.shape[1])
    ax.set_ylim(0, mask.shape[0])
    ax.invert_yaxis()
    ax.axis('off')
    plt.title("Agent Path and Zone Heatmap")
    plt.savefig(output_path, bbox_inches='tight', dpi=300)
    plt.close()

# ------------------- ZONE MANAGER -------------------
class ZoneManager:
    def __init__(self, grid_size, zone_count):
        self.grid_size = grid_size
        self.zone_count = zone_count
        self.zone_size = grid_size // zone_count
        self.zone_coverage = {}
        self.zone_totals = {}
        self.completed_zones = set()
        self.current_zone = None
        self.next_zone = None
        self.exit_point = None

    def zone_index(self, x, y):
        return (x // self.zone_size, y // self.zone_size)

    def register_valid_points(self, valid_points):
        from collections import defaultdict
        self.zone_coverage = defaultdict(int)
        self.zone_totals = defaultdict(int)
        for x, y in valid_points:
            zone = self.zone_index(x, y)
            self.zone_totals[zone] += 1

    def update_coverage(self, x, y):
        z = self.zone_index(x, y)
        self.zone_coverage[z] += 1
        if self.is_zone_complete(z):
            self.completed_zones.add(z)

    def is_zone_complete(self, zone):
        covered = self.zone_coverage.get(zone, 0)
        total = self.zone_totals.get(zone, 1)
        return covered / total >= 0.98

    def select_next_zone(self):
        candidates = [z for z in self.zone_totals if z not in self.completed_zones]
        if not candidates:
            return None
        return min(candidates, key=lambda z: self.zone_coverage.get(z, 0))

    def find_exit_point(self, valid_points, current_zone, target_zone):
        cx, cy = current_zone
        tx, ty = target_zone
        border = []
        for x, y in valid_points:
            z = self.zone_index(x, y)
            if z == current_zone:
                if abs(x - tx * self.zone_size) <= 1 or abs(y - ty * self.zone_size) <= 1:
                    border.append((x, y))
        if not border:
            return None
        return min(border, key=lambda p: abs(p[0] - tx * self.zone_size) + abs(p[1] - ty * self.zone_size))

    def enter_new_zone(self, current_position, valid_points):
        self.current_zone = self.zone_index(*current_position)
        self.next_zone = self.select_next_zone()
        if self.next_zone:
            self.exit_point = self.find_exit_point(valid_points, self.current_zone, self.next_zone)

# ------------------- ENVIRONMENT -------------------
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
        self.action_space = spaces.Discrete(len(ACTIONS))
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

        dx, dy = ACTIONS[action]
        nx, ny = self.agent_pos[0] + dx, self.agent_pos[1] + dy

        if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size and self.visitable[nx, ny]:
            self.agent_pos = (nx, ny)

        self.path.append(self.agent_pos)
        self.steps += 1
        self.visit_counts[self.agent_pos] += 1

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

# ------------------- Q-LEARNING LOOP -------------------
print("Starting Q-learning...")
q_table = defaultdict(lambda: np.zeros(len(ACTIONS)))
folder = get_versioned_output_folder()
progress_log = []

for episode in range(EPISODES):
    mask = generate_blob(GRID_SIZE, seed=episode)
    env = ToolpathEnv(mask, GRID_SIZE, VISION_RADIUS, ZONE_COUNT)
    obs, _ = env.reset()
    zone_mgr = ZoneManager(GRID_SIZE, ZONE_COUNT)
    zone_mgr.register_valid_points(env.total_to_cover)
    zone_mgr.enter_new_zone(env.agent_pos, env.total_to_cover)

    done = False
    total_reward = 0

    while not done:
        zone_mgr.update_coverage(*env.agent_pos)

        if zone_mgr.is_zone_complete(zone_mgr.current_zone):
            if env.agent_pos == zone_mgr.exit_point:
                zone_mgr.enter_new_zone(env.agent_pos, env.total_to_cover)

        if random.random() < EPSILON:
            action = random.randint(0, len(ACTIONS) - 1)
        else:
            action = np.argmax(q_table[obs])

        next_obs, reward, done, _, _ = env.step(action)
        total_reward += reward
        q_table[obs][action] = (1 - ALPHA) * q_table[obs][action] + ALPHA * (reward + GAMMA * np.max(q_table[next_obs]))
        obs = next_obs

    EPSILON = max(MIN_EPSILON, EPSILON * EPSILON_DECAY)

    progress_log.append({
        'episode': episode + 1,
        'total_reward': total_reward,
        'steps_taken': env.steps,
        'unique_visited': len(env.covered),
        'revisits': env.revisits,
        'current_zone': zone_mgr.current_zone,
        'next_zone': zone_mgr.next_zone,
        'exit_point': zone_mgr.exit_point
    })

    if (episode + 1) % (EPISODES // EXPORTS) == 0 or episode == EPISODES - 1:
        img_path = os.path.join(folder, f"path_ep_{episode+1:03}.png")
        plot_agent_path_with_heatmap(env.visitable, env.visit_counts, env.path, env.zone_size, img_path)
        print(f"Episode {episode+1}/{EPISODES} - Total Reward: {total_reward:.2f}")

# Save progress log
csv_path = os.path.join(folder, "progress_log.csv")
save_episode_csv(progress_log, csv_path)
print("Training complete. Outputs saved in:", folder)

# Plot summary after training
df = pd.read_csv(csv_path)
fig, ax1 = plt.subplots(figsize=(10, 6))
ax1.plot(df['episode'], df['total_reward'], label='Total Reward', color='tab:blue')
ax1.set_xlabel('Episode')
ax1.set_ylabel('Total Reward', color='tab:blue')
ax1.tick_params(axis='y', labelcolor='tab:blue')

ax2 = ax1.twinx()
ax2.plot(df['episode'], df['unique_visited'], label='Unique Visited Points', color='tab:green', linestyle='--')
ax2.plot(df['episode'], df['revisits'], label='Revisits', color='tab:red', linestyle=':')
ax2.set_ylabel('Visited / Revisits', color='gray')
ax2.tick_params(axis='y', labelcolor='gray')

lines, labels = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(lines + lines2, labels + labels2, loc='upper left')

plt.title('RL Training Progress')
plt.tight_layout()
plt.savefig(os.path.join(folder, 'training_progress_plot.png'), dpi=300)
plt.show()
