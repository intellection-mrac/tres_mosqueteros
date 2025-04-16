import matplotlib.pyplot as plt
import numpy as np

def plot_agent_path_with_heatmap_gr(points_uv, visit_counts, path, resolution, output_path):
    """
    Visualize the agent path and visit heatmap for (u, v) data using bounding box discretization.
    """
    points_uv = np.array(points_uv)
    min_bounds = points_uv.min(axis=0)
    max_bounds = points_uv.max(axis=0)
    scale = (max_bounds - min_bounds) / resolution

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_aspect('equal')

    for x in range(resolution):
        for y in range(resolution):
            visits = visit_counts[x, y]
            if visits > 0:
                world_x = min_bounds[0] + x * scale[0]
                world_y = min_bounds[1] + y * scale[1]
                color = plt.cm.Reds(min(1.0, visits / 5))
                ax.add_patch(plt.Rectangle((world_x, world_y), scale[0], scale[1],
                                           facecolor=color, edgecolor='lightgray'))

    if path:
        u, v = zip(*path)
        ax.plot(u, v, 'b-', linewidth=1)
        ax.plot(u[0], v[0], 'go')
        ax.plot(u[-1], v[-1], 'ro')

    ax.set_xlim(min_bounds[0], max_bounds[0])
    ax.set_ylim(min_bounds[1], max_bounds[1])
    ax.invert_yaxis()
    ax.axis('off')
    plt.title("Agent Path and Zone Heatmap (GR)")
    plt.savefig(output_path, bbox_inches='tight', dpi=300)
    plt.close()
