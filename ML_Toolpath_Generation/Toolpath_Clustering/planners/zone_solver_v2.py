import numpy as np

def solve_zone_path(points, entry):
    """
    Solve a path through the given set of points starting from the entry point.
    Uses a greedy nearest-neighbor strategy.
    """
    points = set(points)
    if entry not in points:
        raise ValueError("Entry point must be within the given zone points.")

    path = [entry]
    points.remove(entry)

    while points:
        last = path[-1]
        next_pt = min(points, key=lambda p: np.linalg.norm(np.array(p) - np.array(last)))
        path.append(next_pt)
        points.remove(next_pt)

    return path
