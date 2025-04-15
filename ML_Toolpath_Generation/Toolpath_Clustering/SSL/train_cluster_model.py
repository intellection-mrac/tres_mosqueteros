import numpy as np
from sklearn.cluster import KMeans
import joblib
import os

# --- Config ---
N_CLUSTERS = 3
LETTER = "H"

here = os.path.dirname(__file__)
DATASET_FOLDER = os.path.join(here, "letter_dataset")
OUTPUT_PATH = os.path.join(here, f"SSL_v2/clustering_model_{LETTER}.pkl")

# --- Load selected .npy point set ---
points_path = os.path.join(DATASET_FOLDER, f"{LETTER}_points.npy")
if not os.path.exists(points_path):
    raise FileNotFoundError(f"No dataset found for letter '{LETTER}' at {points_path}")

valid_points = np.load(points_path)
print(f"Loaded {len(valid_points)} points from {points_path}")

# --- Train clustering model ---
model = KMeans(n_clusters=N_CLUSTERS, random_state=42)
model.fit(valid_points)

# --- Save model ---
LETTER = "A"
os.makedirs("SSL_v2", exist_ok=True)
joblib.dump(model, f"SSL_v2/clustering_model_{LETTER}.pkl")

print(f"Model saved to {OUTPUT_PATH}")
