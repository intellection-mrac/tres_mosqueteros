import numpy as np
from utils.helpers import generate_blob
from env.toolpath_env_v2 import ToolpathEnvV2
from env.clustered_zone_manager_v2 import ClusteredZoneManagerV2
from planners_v2.zone_order_v2_1 import plan_zone_order_sweep
from planners_v2.zone_solver_v2 import solve_zone_path
from utils.visualizer import plot_agent_path_with_heatmap
from utils.io_utils import get_version_folder, save_episode_csv

import os

# --- Config ---
GRID_SIZE = 50
N_CLUSTERS = 5
LETTER = "A"
MODEL_PATH = f"Toolpath_Clustering/SSL/SSL_v2/clustering_model_{LETTER}.pkl"
SWEEP_MODE = "left-right"  # Options: left-right, top-bottom, etc.

# --- Generate Data ---
mask = generate_blob(GRID_SIZE)
valid_points = set(zip(*np.where(mask == 1)))

# --- Setup ---
env = ToolpathEnvV2(mask)
manager = ClusteredZoneManagerV2(MODEL_PATH)
manager.assign_clusters(valid_points)
clusters = manager.get_clusters()

# --- Plan Traversal (sweep-based) ---
zone_order = plan_zone_order_sweep(clusters, mode=SWEEP_MODE)

# --- Full Path ---
full_path = []
current_pos = env.agent_pos
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

# --- io hash sequence ---
# Define the modules used to generate version hash
run_folder = get_version_folder([
    "toolpath_env_v2",
    "clustered_zone_manager_v2",
    "zone_order_v2_1",
    "zone_solver_v2"
])

# --- CSV logging ---

def compute_path_data(path):
    log = []
    total_length = 0.0
    for i in range(1, len(path)):
        prev = np.array(path[i - 1])
        curr = np.array(path[i])
        dist = np.linalg.norm(curr - prev)
        total_length += dist
        log.append({
            "step": i,
            "x": curr[0],
            "y": curr[1],
            "segment_length": dist,
            "cumulative_length": total_length
        })
    return log

# --- Visualize ---
output_path = os.path.join(run_folder, f"plan_v2_path_{LETTER}.png")

plot_agent_path_with_heatmap(mask, env.visit_counts, env.get_path(), zone_size=GRID_SIZE // N_CLUSTERS, output_path=output_path)

path_log = compute_path_data(env.get_path())
save_episode_csv(path_log, os.path.join(run_folder, "path_data.csv"))


print(f"Finished deterministic pathing for letter '{LETTER}'. Total reward: {total_reward:.2f}")

