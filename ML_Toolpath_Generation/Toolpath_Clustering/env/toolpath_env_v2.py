# File: env/toolpath_env_v2.py

import numpy as np
from gymnasium import Env, spaces

class ToolpathEnvV2(Env):
    def __init__(self, mask):
        super().__init__()
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


# File: env/clustered_zone_manager_v2.py

import numpy as np
from collections import defaultdict
import joblib

class ClusteredZoneManagerV2:
    def __init__(self, model_path):
        self.model = joblib.load(model_path)
        self.point_to_cluster = {}
        self.cluster_to_points = defaultdict(set)

    def assign_clusters(self, valid_points):
        coords = np.array(list(valid_points))
        labels = self.model.predict(coords)
        for pt, label in zip(coords, labels):
            self.point_to_cluster[tuple(pt)] = label
            self.cluster_to_points[label].add(tuple(pt))

    def get_clusters(self):
        return self.cluster_to_points


# File: planners_v2/zone_order_v2.py

import numpy as np

def compute_centroids(cluster_to_points):
    return {label: np.mean(list(pts), axis=0) for label, pts in cluster_to_points.items()}

def plan_zone_order(cluster_to_points, start_point):
    centroids = compute_centroids(cluster_to_points)
    order = []
    unvisited = set(centroids.keys())
    current = start_point

    while unvisited:
        next_label = min(unvisited, key=lambda k: np.linalg.norm(centroids[k] - current))
        order.append(next_label)
        current = centroids[next_label]
        unvisited.remove(next_label)

    return order


# File: planners_v2/zone_solver_v2.py

import numpy as np

def solve_zone_path(points, entry):
    points = set(points)
    path = [entry]
    points.remove(entry)

    while points:
        last = path[-1]
        next_pt = min(points, key=lambda p: np.linalg.norm(np.array(p) - np.array(last)))
        path.append(next_pt)
        points.remove(next_pt)

    return path


# File: SSL_v2/train_cluster_model.py

import numpy as np
from sklearn.cluster import KMeans
import joblib

# Dummy data: list of (x, y) points (replace with real training examples)
all_points = np.vstack([np.array([[i, j] for j in range(50)]) for i in range(50)])
valid_points = all_points[np.random.rand(len(all_points)) < 0.1]

model = KMeans(n_clusters=5, random_state=42)
model.fit(valid_points)
joblib.dump(model, "SSL_v2/clustering_model.pkl")
print("Model saved to SSL_v2/clustering_model.pkl")


# File: main_plan_v2.py

import numpy as np
from utils.helpers import generate_blob
from env.toolpath_env_v2 import ToolpathEnvV2
from env.clustered_zone_manager_v2 import ClusteredZoneManagerV2
from planners_v2.zone_order_v2 import plan_zone_order
from planners_v2.zone_solver_v2 import solve_zone_path
from utils.visualizer import plot_agent_path_with_heatmap
import os

# --- Config ---
GRID_SIZE = 50
N_CLUSTERS = 5
MODEL_PATH = "SSL_v2/clustering_model.pkl"

# --- Generate Data ---
mask = generate_blob(GRID_SIZE)
valid_points = set(zip(*np.where(mask == 1)))

# --- Setup ---
env = ToolpathEnvV2(mask)
manager = ClusteredZoneManagerV2(MODEL_PATH)
manager.assign_clusters(valid_points)
clusters = manager.get_clusters()

# --- Plan Traversal ---
start_point = env.agent_pos
zone_order = plan_zone_order(clusters, np.array(start_point))

# --- Full Path ---
full_path = []
current_pos = start_point
for zone in zone_order:
    points = clusters[zone]
    entry = min(points, key=lambda p: np.linalg.norm(np.array(p) - np.array(current_pos)))
    path = solve_zone_path(points, entry)
    full_path.extend(path)
    current_pos = path[-1]

# --- Step through env ---
total_reward = 0
for pos in full_path:
    _, reward, done = env.step_to(pos)
    total_reward += reward
    if done:
        break

# --- Visualize ---
plot_agent_path_with_heatmap(mask, env.visit_counts, env.get_path(), zone_size=GRID_SIZE // N_CLUSTERS, output_path="outputs/plan_v2_path.png")
print(f"Finished deterministic pathing. Total reward: {total_reward:.2f}")
