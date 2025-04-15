# File: SSL/train_cluster_model.py

import numpy as np
from sklearn.cluster import KMeans
import joblib
import os

def train_clustering_model_for_letter(letter, dataset_folder, model_folder, n_clusters):
    """
    Loads points for a given letter and trains a clustering model.
    Saves it to the model_folder.
    """
    os.makedirs(model_folder, exist_ok=True)

    points_path = os.path.join(dataset_folder, f"{letter}_points.npy")
    if not os.path.exists(points_path):
        raise FileNotFoundError(f"No dataset found for letter '{letter}' at {points_path}")

    valid_points = np.load(points_path)
    print(f"Loaded {len(valid_points)} points from {points_path}")

    if len(valid_points) < n_clusters:
        raise ValueError(f"Too few points ({len(valid_points)}) for {n_clusters} clusters.")

    model = KMeans(n_clusters=n_clusters, random_state=42)
    model.fit(valid_points)

    model_path = os.path.join(model_folder, f"clustering_model_{letter}.pkl")
    joblib.dump(model, model_path)
    print(f"Model saved to {model_path}")
