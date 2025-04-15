import numpy as np
import os
from sklearn.cluster import KMeans
import joblib
import matplotlib.pyplot as plt

# --- Per-letter cluster settings ---
LETTER_CLUSTER_COUNTS = {
    'A': 3, 'B': 3, 'C': 2, 'D': 3, 'E': 3, 'F': 3, 'G': 3, 'H': 3, 'I': 2, 'J': 2, 'K': 3, 'L': 2, 'M': 4,
    'N': 3, 'O': 3, 'P': 3, 'Q': 3, 'R': 3, 'S': 3, 'T': 2, 'U': 3, 'V': 2, 'W': 4, 'X': 3, 'Y': 3, 'Z': 3
}


def train_clustering_model_for_letter(letter, dataset_folder, model_folder):
    os.makedirs(model_folder, exist_ok=True)

    points_path = os.path.join(dataset_folder, f"{letter}_points.npy")
    if not os.path.exists(points_path):
        raise FileNotFoundError(f"No dataset found for letter '{letter}' at {points_path}")

    valid_points = np.load(points_path)
    print(f"Loaded {len(valid_points)} points from {points_path}")

    n_clusters = LETTER_CLUSTER_COUNTS.get(letter.upper(), 3)
    if len(valid_points) < n_clusters:
        raise ValueError(f"Too few points ({len(valid_points)}) for {n_clusters} clusters.")

    model = KMeans(n_clusters=n_clusters, random_state=42)
    model.fit(valid_points)

    model_path = os.path.join(model_folder, f"clustering_model_{letter}.pkl")
    joblib.dump(model, model_path)
    print(f"Model saved to {model_path}")


def save_cluster_visualization(points, labels_dict, output_path):
    points = np.array(list(labels_dict.keys()))
    labels = np.array([labels_dict[tuple(p)] for p in points])
    plt.figure(figsize=(6, 6))
    plt.scatter(points[:, 1], -points[:, 0], c=labels, cmap='viridis', s=10)
    plt.axis('equal')
    plt.title("Clustered Zones")
    plt.savefig(output_path, bbox_inches='tight')
    plt.close()