import matplotlib.pyplot as plt


def plot_agent_path_with_heatmap(mask, visit_counts, path, zone_size, output_path):
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_aspect('equal')

    for x in range(mask.shape[0]):
        for y in range(mask.shape[1]):
            if mask[x, y]:
                visits = visit_counts[x, y]
                color = plt.cm.Reds(min(1.0, visits / 5)) if visits > 0 else 'white'
                ax.add_patch(plt.Rectangle((y, x), 1, 1, facecolor=color, edgecolor='lightgray'))
                ax.plot(y + 0.5, x + 0.5, 'k.', markersize=1.5)

    if path:
        px, py = zip(*path)
        ax.plot([y + 0.5 for y in py], [x + 0.5 for x in px], 'b-', linewidth=1)
        ax.plot(py[0] + 0.5, px[0] + 0.5, 'go')
        ax.plot(py[-1] + 0.5, px[-1] + 0.5, 'ro')

    for i in range(1, mask.shape[0] // zone_size):
        ax.axhline(i * zone_size, color='black', linewidth=0.5)
        ax.axvline(i * zone_size, color='black', linewidth=0.5)

    ax.set_xlim(0, mask.shape[1])
    ax.set_ylim(0, mask.shape[0])
    ax.invert_yaxis()
    ax.axis('off')
    plt.title("Agent Path and Zone Heatmap")
    plt.savefig(output_path, bbox_inches='tight', dpi=300)
    plt.close()
