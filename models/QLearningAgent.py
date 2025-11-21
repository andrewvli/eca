import pickle
import random
import os
import sys

# Add parent directory to Python path so we can import constants
# models/ is one level down from root, so go up one level
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from constants import *

class QLearningAgent:
    """
    Q-Learning agent for collision avoidance.
    """
    def __init__(self):
        self.q_table = {}  # { (lidar_front_bin, lidar_left_bin, lidar_right_bin, radar_bin, steering_bin, imu_yaw_bin, camera_object_count_bin, camera_distance_bin, camera_pos_x_bin, camera_pos_z_bin) : { steering_angle : q_value } }
        self.epsilon = EPSILON_START
        self.last_state = None
        self.last_action = None 
        # Learning statistics
        self.episode_count = 0
        self.step_count = 0
        self.total_reward = 0.0
        self.episode_reward = 0.0
        self.episode_steps = 0
        self.q_updates_count = 0
        self.explore_count = 0
        self.exploit_count = 0
        self.load_q_table()
        print("QLearningAgent Initialized")
    

    def load_q_table(self):
        """Load Q-learning progress from file if it exists."""
        if os.path.exists(Q_TABLE_FILE):
            try:
                with open(Q_TABLE_FILE, 'rb') as f:
                    self.q_table = pickle.load(f)
                print(f"Loaded Q-table with {len(self.q_table)} states")
            except Exception as e:
                print(f"Error loading Q-table: {e}")
                self.q_table = {}
        else:
            print("No existing Q-table found, starting fresh")
    

    def save_q_table(self):
        """Save Q-learning progress to file."""
        try:
            with open(Q_TABLE_FILE, 'wb') as f:
                pickle.dump(self.q_table, f)
            print(f"Saved Q-table with {len(self.q_table)} states")
        except Exception as e:
            print(f"Error saving Q-table: {e}")
    

    def get_q_values(self, state):
        """
        Get Q-values for a state, initializing if necessary.
        Returns: dictionary mapping steering angles to Q-values
        """
        # iI the state not yet in the Q-table, initialize it with zeros for all steering actions
        if state not in self.q_table:
            q_dict = {}
            for steering in STEERING_ADJUSTMENTS:
                q_dict[steering] = 0.0
            self.q_table[state] = q_dict
        
        return self.q_table[state]
    

    def get_valid_actions(self, current_steering_bin):
        """
        Based on current steering angle, extract only valid relative angle adjustments.
        Returns: List of all valid adjustments to the current angle.
        """
        # Convert from current_steering_bin back to the actual angle
        angle_bins = [-0.25, -0.20, -0.15, -0.10, -0.05, 0.0, 0.05, 0.10, 0.15, 0.20, 0.25]
        current_steering = angle_bins[current_steering_bin]

        return [action for action in STEERING_ADJUSTMENTS if MIN_ANGLE <= current_steering + action <= MAX_ANGLE]
    

    def select_action(self, state):
        """
        Select action using epsilon-greedy policy.
        With probability epsilon, explore (random action).
        With probability 1-epsilon, exploit (best action).
        Returns: steering angle (float)
        """
        q_values = self.get_q_values(state)
        
        valid_actions = self.get_valid_actions(state[STEERING_BIN_IDX])
        masked_q_values = {a: q_values[a] for a in valid_actions}
        
        if random.random() < self.epsilon: # Explore random valid action
            self.explore_count += 1
            steering = random.choice(valid_actions)
            return steering
        
        else: # Exploit best action
            self.exploit_count += 1
            best_action = max(masked_q_values.items(), key=lambda x: x[1])
            return best_action[0] 
    

    def update(self, state, action, reward, next_state):
        """
        Update Q-value using Q-learning update rule.
        action: steering angle (float)
        """
        q_values = self.get_q_values(state)
        next_q_values = self.get_q_values(next_state)

        # Mask invalid next actions
        valid_next_actions = self.get_valid_actions(next_state[STEERING_BIN_IDX])
        
        current_q = q_values[action]
        max_next_q = max(next_q_values[a] for a in valid_next_actions)
        new_q = current_q + ALPHA * (reward + GAMMA * max_next_q - current_q)

        q_values[action] = new_q
        self.q_updates_count += 1
        self.episode_reward += reward
        self.total_reward += reward
        
        # Print learning progress every N updates
        if self.q_updates_count % 100 == 0:
            avg_q = sum(q_values.values()) / len(q_values) if q_values else 0.0
            max_q = max(q_values.values()) if q_values else 0.0
            explore_rate = self.explore_count / (self.explore_count + self.exploit_count) * 100 if (self.explore_count + self.exploit_count) > 0 else 0.0
            print(f"[Learning] Updates: {self.q_updates_count}, States: {len(self.q_table)}, "
                  f"Epsilon: {self.epsilon:.4f}, Explore: {explore_rate:.1f}%, "
                  f"Avg Q: {avg_q:.2f}, Max Q: {max_q:.2f}, "
                  f"Q-change: {new_q - current_q:.3f}, Reward: {reward:.2f}")
    
    
    def decay_epsilon(self):
        """
        Decay the exploration rate (explore less often as iterations go on).
        """
        self.epsilon = max(EPSILON_MIN, self.epsilon * EPSILON_DECAY)
