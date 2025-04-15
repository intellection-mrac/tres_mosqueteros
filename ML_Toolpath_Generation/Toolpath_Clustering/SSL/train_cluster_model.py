import numpy as np
from sklearn.cluster import KMeans
import joblib
import os

# --- Config ---
GRID_SIZE = 50
N_CLUSTERS = 5
OUTPUT_PATH = "SSL_v2/clustering_model.pkl"

# --- Generate dummy training data (replace with real examples as needed) ---
np.random.seed(42)
all_points = np.array([(i, j) for i in range(GRID_SIZE) for j in range(GRID_SIZE)])
mask = np.random.rand(GRID_SIZE, GRID_SIZE) < 0.1
valid_points = all_points[mask.flatten()]

# --- Train clustering model ---
model = KMeans(n_clusters=N_CLUSTERS, random_state=42)
model.fit(valid_points)

# --- Save model ---
os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
joblib.dump(model, OUTPUT_PATH)
print(f"Model saved to {OUTPUT_PATH}")
