import numpy as np

def extract_local_mask(visited, x, y, radius):
    r = radius
    mask = []
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            nx, ny = x + dx, y + dy
            if 0 <= nx < visited.shape[0] and 0 <= ny < visited.shape[1]:
                mask.append(int(visited[nx, ny]))
            else:
                mask.append(1)  # treat out-of-bounds as visited
    return tuple(mask)

def generate_blob(grid_size, num_points=100, radius=15, noise_scale=1.0, seed=None):
    if seed is not None:
        np.random.seed(seed)

    center = (grid_size // 2, grid_size // 2)
    angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    noise = np.random.normal(0, 1, num_points) * noise_scale
    radii = radius + noise

    blob_mask = np.zeros((grid_size, grid_size), dtype=int)
    for x in range(grid_size):
        for y in range(grid_size):
            dx, dy = x - center[0], y - center[1]
            angle = (np.arctan2(dy, dx) + 2 * np.pi) % (2 * np.pi)
            index = int(angle / (2 * np.pi) * num_points)
            if np.hypot(dx, dy) <= radii[index]:
                blob_mask[x, y] = 1

    return blob_mask
