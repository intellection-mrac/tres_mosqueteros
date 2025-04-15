import numpy as np
from collections import defaultdict
import joblib

class ClusteredZoneManagerV2:
    def __init__(self, model_path):
        self.model = joblib.load(model_path)
        self.point_to_cluster = {}
        self.cluster_to_points = defaultdict(set)

    def assign_clusters(self, valid_points):
        coords = np.array(list(valid_points))
        labels = self.model.predict(coords)
        for pt, label in zip(coords, labels):
            self.point_to_cluster[tuple(pt)] = label
            self.cluster_to_points[label].add(tuple(pt))

    def get_clusters(self):
        return self.cluster_to_points

    def get_cluster_for_point(self, point):
        return self.point_to_cluster.get(point, None)

    def get_centroids(self):
        return {
            cluster: np.mean(list(points), axis=0)
            for cluster, points in self.cluster_to_points.items()
        }

    def get_nearest_point(self, from_point, cluster_id):
        points = self.cluster_to_points.get(cluster_id, set())
        if not points:
            return None
        return min(points, key=lambda p: np.linalg.norm(np.array(p) - np.array(from_point)))
