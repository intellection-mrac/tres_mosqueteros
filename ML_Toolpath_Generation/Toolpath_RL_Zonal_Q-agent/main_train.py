# Entry point for running training
from env.toolpath_env import ToolpathEnv
from env.zone_manager import ZoneManager
from agents.q_learning_agent import QLearningAgent
from utils.io_utils import get_version_folder
folder = get_version_folder([
    "main_train.py",
    "agents/q_learning_agent.py",
    "env/toolpath_env.py",
    "env/zone_manager.py"
])
from utils.io_utils import save_episode_csv
from utils.visualizer import plot_agent_path_with_heatmap
from utils.helpers import generate_blob
import pandas as pd
import matplotlib.pyplot as plt
import os

# ----------------------- CONFIG -----------------------
GRID_SIZE = 50
ZONE_COUNT = 5
VISION_RADIUS = max(1, GRID_SIZE // 25)
EPISODES = 500
EXPORTS = 10

print("Starting Q-learning...")
progress_log = []
agent = QLearningAgent(grid_size=GRID_SIZE, num_actions=8)

for episode in range(EPISODES):
    mask = generate_blob(GRID_SIZE, seed=episode)
    env = ToolpathEnv(mask, GRID_SIZE, VISION_RADIUS, ZONE_COUNT)
    obs, _ = env.reset()
    zone_mgr = ZoneManager(GRID_SIZE, ZONE_COUNT)
    zone_mgr.register_valid_points(env.total_to_cover)
    zone_mgr.enter_new_zone(env.agent_pos, env.total_to_cover)

    done = False
    total_reward = 0

    while not done:
        zone_mgr.update_coverage(*env.agent_pos)

        if zone_mgr.is_zone_complete(zone_mgr.current_zone):
            if env.agent_pos == zone_mgr.exit_point:
                zone_mgr.enter_new_zone(env.agent_pos, env.total_to_cover)

        action = agent.select_action(obs)
        next_obs, reward, done, _, _ = env.step(action)
        total_reward += reward
        agent.learn(obs, action, reward, next_obs)
        obs = next_obs

    agent.decay_epsilon()

    progress_log.append({
        'episode': episode + 1,
        'total_reward': total_reward,
        'steps_taken': env.steps,
        'unique_visited': len(env.covered),
        'revisits': env.revisits,
        'current_zone': zone_mgr.current_zone,
        'next_zone': zone_mgr.next_zone,
        'exit_point': zone_mgr.exit_point
    })

    if (episode + 1) % (EPISODES // EXPORTS) == 0 or episode == EPISODES - 1:
        img_path = os.path.join(folder, f"path_ep_{episode+1:03}.png")
        plot_agent_path_with_heatmap(env.visitable, env.visit_counts, env.path, env.zone_size, img_path)
        print(f"Episode {episode+1}/{EPISODES} - Total Reward: {total_reward:.2f}")

csv_path = os.path.join(folder, "progress_log.csv")
save_episode_csv(progress_log, csv_path)
print("Training complete. Outputs saved in:", folder)

# Plot summary
df = pd.read_csv(csv_path)
fig, ax1 = plt.subplots(figsize=(10, 6))
ax1.plot(df['episode'], df['total_reward'], label='Total Reward', color='tab:blue')
ax1.set_xlabel('Episode')
ax1.set_ylabel('Total Reward', color='tab:blue')
ax1.tick_params(axis='y', labelcolor='tab:blue')

ax2 = ax1.twinx()
ax2.plot(df['episode'], df['unique_visited'], label='Unique Visited Points', color='tab:green', linestyle='--')
ax2.plot(df['episode'], df['revisits'], label='Revisits', color='tab:red', linestyle=':')
ax2.set_ylabel('Visited / Revisits', color='gray')
ax2.tick_params(axis='y', labelcolor='gray')

lines, labels = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(lines + lines2, labels + labels2, loc='upper left')

plt.title('RL Training Progress')
plt.tight_layout()
plt.savefig(os.path.join(folder, 'training_progress_plot.png'), dpi=300)
plt.show()
