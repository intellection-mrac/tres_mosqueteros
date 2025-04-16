import numpy as np
import matplotlib.pyplot as plt
import joblib
import os

LETTER = "H"
here = os.path.dirname(__file__)
DATASET_FOLDER = os.path.join(here, "letter_dataset")
MODEL_PATH = os.path.join(here, f"SSL_v2/clustering_model_{LETTER}.pkl")
OUTPUT_PATH = os.path.join(here, f"SSL_v2/{LETTER}_clusters_visualized.png")


# --- Load Data ---
points = np.load(os.path.join(DATASET_FOLDER, f"{LETTER}_points.npy"))
model = joblib.load(MODEL_PATH)
labels = model.predict(points)

# --- Plot ---
plt.figure(figsize=(6, 6))
plt.axis('equal')
scatter = plt.scatter(points[:, 1], points[:, 0], c=labels, cmap='tab10', s=20, marker='s')
plt.gca().invert_yaxis()
plt.title(f"Clusters for letter '{LETTER}'")
plt.xticks([])
plt.yticks([])
plt.savefig(OUTPUT_PATH, bbox_inches='tight', dpi=300)
plt.close()

print(f"Cluster visualization saved to {OUTPUT_PATH}")
