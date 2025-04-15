import numpy as np

def compute_centroids(cluster_to_points):
    """Compute the centroid of each cluster."""
    return {
        label: np.mean(list(points), axis=0)
        for label, points in cluster_to_points.items()
    }

def plan_zone_order(cluster_to_points, start_point):
    """Plan a greedy traversal order of zones based on centroid proximity."""
    centroids = compute_centroids(cluster_to_points)
    order = []
    unvisited = set(centroids.keys())
    current = start_point

    while unvisited:
        next_label = min(unvisited, key=lambda k: np.linalg.norm(centroids[k] - current))
        order.append(next_label)
        current = centroids[next_label]
        unvisited.remove(next_label)

    return order
