import numpy as np

def compute_centroids(cluster_to_points):
    return {label: np.mean(list(pts), axis=0) for label, pts in cluster_to_points.items()}

def plan_zone_order_sweep(cluster_to_points, mode="left-right"):
    """
    Plan zone traversal by sweeping across centroid coordinates.
    mode: one of ['left-right', 'right-left', 'top-bottom', 'bottom-top', 'diagonal']
    """
    centroids = compute_centroids(cluster_to_points)

    if mode == "left-right":
        sorted_labels = sorted(centroids.keys(), key=lambda k: centroids[k][0])  # sort by x
    elif mode == "right-left":
        sorted_labels = sorted(centroids.keys(), key=lambda k: -centroids[k][0])
    elif mode == "top-bottom":
        sorted_labels = sorted(centroids.keys(), key=lambda k: centroids[k][1])  # sort by y
    elif mode == "bottom-top":
        sorted_labels = sorted(centroids.keys(), key=lambda k: -centroids[k][1])
    elif mode == "diagonal":
        sorted_labels = sorted(centroids.keys(), key=lambda k: centroids[k][0] + centroids[k][1])
    else:
        raise ValueError(f"Unsupported sweep mode: {mode}")

    return sorted_labels
