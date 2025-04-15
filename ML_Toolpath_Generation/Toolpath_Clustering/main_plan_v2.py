import numpy as np
import os
from utils.helpers import generate_blob
from utils.visualizer import plot_agent_path_with_heatmap
from env.toolpath_env_v2 import ToolpathEnvV2
from env.clustered_zone_manager_v2 import ClusteredZoneManagerV2
from planners_v2.zone_order_v2 import plan_zone_order
from planners_v2.zone_solver_v2 import solve_zone_path

# --- Config ---
GRID_SIZE = 50
N_CLUSTERS = 5
MODEL_PATH = "SSL_v2/clustering_model.pkl"
OUTPUT_IMG_PATH = "outputs/plan_v2_path.png"

# --- Generate synthetic mask ---
mask = generate_blob(GRID_SIZE)
valid_points = set(zip(*np.where(mask == 1)))

# --- Environment and zone manager setup ---
env = ToolpathEnvV2(mask)
zone_mgr = ClusteredZoneManagerV2(MODEL_PATH)
zone_mgr.assign_clusters(valid_points)
clusters = zone_mgr.get_clusters()

# --- Plan traversal order of zones ---
start_pos = env.agent_pos
zone_order = plan_zone_order(clusters, np.array(start_pos))

# --- Build complete path ---
full_path = []
current_pos = start_pos

for zone_id in zone_order:
    zone_points = clusters[zone_id]
    entry = zone_mgr.get_nearest_point(current_pos, zone_id)
    zone_path = solve_zone_path(zone_points, entry)
    full_path.extend(zone_path)
    current_pos = zone_path[-1]

# --- Simulate toolpath traversal ---
total_reward = 0
for pos in full_path:
    _, reward, done = env.step_to(pos)
    total_reward += reward
    if done:
        break

# --- Visualize ---
os.makedirs("outputs", exist_ok=True)
plot_agent_path_with_heatmap(mask, env.visit_counts, env.get_path(), zone_size=GRID_SIZE // N_CLUSTERS, output_path=OUTPUT_IMG_PATH)
print(f"Finished deterministic pathing. Total reward: {total_reward:.2f}")