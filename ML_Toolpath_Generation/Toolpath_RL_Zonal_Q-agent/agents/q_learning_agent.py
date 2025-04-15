import numpy as np
import random

class QLearningAgent:
    def __init__(self, grid_size, num_actions, alpha=0.1, gamma=0.99, epsilon=1.0, epsilon_decay=0.995, min_epsilon=0.01):
        self.grid_size = grid_size
        self.num_actions = num_actions
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_decay = epsilon_decay
        self.min_epsilon = min_epsilon
        self.q_table = {}

    def get_state_key(self, obs):
        return obs  # Assumes obs is already hashable (tuple)

    def select_action(self, obs):
        key = self.get_state_key(obs)
        if key not in self.q_table:
            self.q_table[key] = np.zeros(self.num_actions)

        if random.random() < self.epsilon:
            return random.randint(0, self.num_actions - 1)
        return np.argmax(self.q_table[key])

    def learn(self, obs, action, reward, next_obs):
        key = self.get_state_key(obs)
        next_key = self.get_state_key(next_obs)

        if key not in self.q_table:
            self.q_table[key] = np.zeros(self.num_actions)
        if next_key not in self.q_table:
            self.q_table[next_key] = np.zeros(self.num_actions)

        best_next = np.max(self.q_table[next_key])
        td_target = reward + self.gamma * best_next
        td_error = td_target - self.q_table[key][action]
        self.q_table[key][action] += self.alpha * td_error

    def decay_epsilon(self):
        self.epsilon = max(self.min_epsilon, self.epsilon * self.epsilon_decay)
