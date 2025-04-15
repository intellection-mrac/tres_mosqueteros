import numpy as np
import os
import sys
from PIL import Image
from utils.visualizer import plot_agent_path_with_heatmap
from utils.io_utils import save_episode_csv, get_version_folder
from env.toolpath_env_v2 import ToolpathEnvV2
from env.clustered_zone_manager_v2 import ClusteredZoneManagerV2
from planners_v2.zone_order_v2_1 import plan_zone_order_sweep
from planners_v2.zone_solver_v2 import solve_zone_path
from SSL.letter_dataset_utils import generate_letter_dataset, mask_to_valid_points
from SSL.train_cluster_model_ABC import train_clustering_model_for_letter, save_cluster_visualization

# --- Input letter ---
VALID_LETTERS = set(chr(c) for c in range(65, 91))  # A-Z
letter = input("Choose a letter (A–Z): ").strip().upper()
while letter not in VALID_LETTERS:
    print("Invalid letter. Please choose a single letter from A to Z.")
    letter = input("Choose a letter (A–Z): ").strip().upper()

# --- Input grid and clustering resolution ---
def input_int(prompt, default, min_val=1, max_val=300):
    try:
        val = input(f"{prompt} (default {default}): ").strip()
        if val == "":
            return default
        val = int(val)
        if min_val <= val <= max_val:
            return val
    except ValueError:
        pass
    print(f"Invalid input. Using default {default}.")
    return default

grid_size = input_int("Grid size", 300)
cluster_size = input_int("Toolpath cluster size", 25)

# --- Define paths ---
here = os.path.dirname(__file__)
dataset_dir = os.path.join(here, "SSL", "letter_dataset")
model_dir = os.path.join(here, "SSL", "SSL_v2")

# --- Generate dataset if needed ---
points_path = os.path.join(dataset_dir, f"{letter}_points.npy")
mask_path = os.path.join(dataset_dir, f"{letter}_mask.png")
if not os.path.exists(points_path) or not os.path.exists(mask_path):
    print(f"[!] Generating dataset for letter '{letter}'...")
    generate_letter_dataset([letter], output_folder=dataset_dir, image_size=grid_size, font_size=int(grid_size * 0.8), max_points=1000)

# --- Train clustering model ---
print(f"[!] Training clustering model for '{letter}'...")
train_clustering_model_for_letter(letter, dataset_dir, model_dir)

# --- Load data ---
mask = np.array(Image.open(mask_path).convert("L")) > 128
mask = mask.astype(np.uint8)
valid_points = np.load(points_path)

# --- Setup environment and clustering ---
env = ToolpathEnvV2(mask)
model_path = os.path.join(model_dir, f"clustering_model_{letter}.pkl")
manager = ClusteredZoneManagerV2(model_path)
manager.assign_clusters(valid_points)
clusters = manager.get_clusters()

# --- Plan toolpath ---
zone_order = plan_zone_order_sweep(clusters, mode="left-right")
full_path = []
current_pos = env.agent_pos
for zone_id in zone_order:
    zone_points = clusters[zone_id]
    entry = manager.get_nearest_point(current_pos, zone_id)
    path = solve_zone_path(zone_points, entry)
    full_path.extend(path)
    current_pos = path[-1]

# --- Run agent ---
total_reward = 0
for pos in full_path:
    _, reward, done = env.step_to(pos)
    total_reward += reward
    if done:
        break

# --- Export ---
run_folder = get_version_folder([
    f"ToolpathEnvV2", f"ClusteredZoneManagerV2({letter})", "zone_order_v2_1", "zone_solver_v2"
])

# Safe zone size calculation
n_segments = max(1, len(valid_points) // cluster_size)
zone_size = max(1, grid_size // n_segments)

plot_agent_path_with_heatmap(
    mask,
    env.visit_counts,
    env.get_path(),
    zone_size=zone_size,
    output_path=os.path.join(run_folder, f"heatmap_{letter}.png")
)

# --- Save original mask and cluster visualization ---
Image.fromarray((mask * 255).astype(np.uint8)).save(os.path.join(run_folder, f"letter_mask_{letter}.png"))
save_cluster_visualization(valid_points, manager.point_to_cluster, os.path.join(run_folder, f"clusters_{letter}.png"))

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

save_episode_csv(
    compute_path_data(env.get_path()),
    os.path.join(run_folder, f"path_data_{letter}.csv")
)

print(f"\n✅ Completed toolpath for letter '{letter}' with total reward: {total_reward:.2f}")
