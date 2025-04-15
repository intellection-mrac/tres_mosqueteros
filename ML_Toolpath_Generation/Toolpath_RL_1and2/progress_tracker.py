import pandas as pd
import matplotlib.pyplot as plt
import os

# Load CSV
csv_path = "outputs/toolpath_rl_zonal/rl_toolpath_optimizer_zonal_memory/run_01/progress_log.csv"
df = pd.read_csv(csv_path)

# Plot configuration
fig, ax1 = plt.subplots(figsize=(10, 6))

# Total reward per episode
ax1.plot(df['episode'], df['total_reward'], label='Total Reward', color='tab:blue')
ax1.set_xlabel('Episode')
ax1.set_ylabel('Total Reward', color='tab:blue')
ax1.tick_params(axis='y', labelcolor='tab:blue')

# Create a second y-axis
ax2 = ax1.twinx()
ax2.plot(df['episode'], df['unique_visited'], label='Unique Visited Points', color='tab:green', linestyle='--')
ax2.plot(df['episode'], df['revisits'], label='Revisits', color='tab:red', linestyle=':')
ax2.set_ylabel('Visited / Revisits', color='gray')
ax2.tick_params(axis='y', labelcolor='gray')

# Combine legends
lines, labels = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(lines + lines2, labels + labels2, loc='upper left')

# Title and layout
plt.title('RL Training Progress')
plt.tight_layout()

# Save plot
output_path = os.path.join(os.path.dirname(csv_path), 'training_progress_plot.png')
plt.savefig(output_path, dpi=300)
plt.show()

print(f"Plot saved to: {output_path}")
