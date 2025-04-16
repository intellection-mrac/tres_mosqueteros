import numpy as np
import os
import csv

from utils.geometry_utils_gr import compute_local_frame, project_to_2d, reproject_to_3d
from env.toolpath_env_v2_gr import ToolpathEnvV2_GR
from env.clustered_zone_manager_v2 import ClusteredZoneManagerV2
from planners_v2.zone_order_v2 import plan_zone_order
from planners_v2.zone_solver_v2 import solve_zone_path
from utils.visualizer_gr import plot_agent_path_with_heatmap_gr
from utils.io_utils_gr import save_path_csv, load_points_csv

# --- Config ---
INPUT_CSV = "inputs/milling_points.csv"
MODEL_PATH = "SSL_v2/clustering_model.pkl"
OUTPUT_FOLDER = "outputs_gr"
RESOLUTION = 100

# --- Setup ---
os.makedirs(OUTPUT_FOLDER, exist_ok=True)
points_3d = load_points_csv(INPUT_CSV)
T, T_inv = compute_local_frame(points_3d)
points_2d = project_to_2d(points_3d, T_inv)

# --- Environment & Clustering ---
env = ToolpathEnvV2_GR(points_2d, resolution=RESOLUTION)
manager = ClusteredZoneManagerV2(MODEL_PATH)
manager.assign_clusters(points_2d)
clusters = manager.get_clusters()

# --- Zone Order + Path Planning ---
start_pos = env.agent_pos
zone_order = plan_zone_order(clusters, np.array(start_pos))

full_path = []
current_pos = start_pos
for zone_id in zone_order:
    zone_points = clusters[zone_id]
    entry = manager.get_nearest_point(current_pos, zone_id)
    zone_path = solve_zone_path(zone_points, entry)
    full_path.extend(zone_path)
    current_pos = zone_path[-1]

# --- Traverse ---
total_reward = 0
for pos in full_path:
    _, reward, done = env.step_to(pos)
    total_reward += reward
    if done:
        break

# --- Export + Visualize ---
plot_agent_path_with_heatmap_gr(points_2d, env.visit_counts, env.get_path(),
                                resolution=RESOLUTION,
                                output_path=os.path.join(OUTPUT_FOLDER, "path_uv_heatmap.png"))

save_path_csv(env.get_path(), os.path.join(OUTPUT_FOLDER, "path_uv.csv"), headers=("u", "v"))
path_3d = reproject_to_3d(env.get_path(), T)
save_path_csv(path_3d, os.path.join(OUTPUT_FOLDER, "path_xyz.csv"), headers=("x", "y", "z"))

print(f"Finished GR path planning. Total reward: {total_reward:.2f}")
