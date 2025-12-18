---
title: "Week 12: Advanced AI Integration - Code Lab"
week: 12
module: "AI-Robot Integration"
difficulty: "advanced"
prerequisites: ["cognitive-planning", "vision-language", "machine-learning", "deep-learning"]
learning_objectives:
  - "Implement learning-based planning"
  - "Design multi-objective optimization systems"
  - "Integrate perception and action"
  - "Create end-to-end learning approaches"
tags: ["machine-learning", "deep-learning", "reinforcement-learning", "imitation-learning", "optimization", "end-to-end-learning"]
hardware_requirements:
  - gpu: "RTX 4080 or higher"
  - ram: "64GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "120 minutes"
---

# Week 12: Advanced AI Integration - Code Lab

## Learning Objectives
- Implement learning-based planning
- Design multi-objective optimization systems
- Integrate perception and action
- Create end-to-end learning approaches

## Prerequisites
- ROS 2 Humble Hawksbill installed
- Python 3.8+ with pip
- PyTorch and TensorFlow installed
- CUDA-compatible GPU with RTX 4080 or higher
- Basic understanding of reinforcement learning

## Setup Environment

### Install Required Dependencies
```bash
# Create a new workspace
mkdir -p ~/advanced_ai_ws/src
cd ~/advanced_ai_ws

# Install Python dependencies for AI integration
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip3 install tensorflow
pip3 install stable-baselines3[extra]
pip3 install gymnasium
pip3 install numpy scipy matplotlib
pip3 install opencv-python
pip3 install scikit-learn
pip3 install wandb  # For experiment tracking
```

### ROS 2 Package Structure
```bash
# Create ROS 2 packages
cd ~/advanced_ai_ws/src
ros2 pkg create --build-type ament_python advanced_ai_interfaces
ros2 pkg create --build-type ament_python reinforcement_learning
ros2 pkg create --build-type ament_python imitation_learning
ros2 pkg create --build-type ament_python multi_objective_optimizer
```

## Part 1: Reinforcement Learning Implementation

### Task 1.1: Create a DQN Agent
Implement a Deep Q-Network agent for robotic control:

```python
#!/usr/bin/env python3
# reinforcement_learning/reinforcement_learning/dqn_agent.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import random
from collections import deque
import copy

class DQN(nn.Module):
    def __init__(self, state_size, action_size):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(state_size, 128)
        self.fc2 = nn.Linear(128, 128)
        self.fc3 = nn.Linear(128, action_size)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        return self.fc3(x)

class DQNAgent(Node):
    def __init__(self):
        super().__init__('dqn_agent')

        # RL parameters
        self.state_size = 10  # Number of laser scan readings
        self.action_size = 3  # 0: left, 1: forward, 2: right
        self.memory = deque(maxlen=10000)
        self.epsilon = 1.0  # Exploration rate
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995
        self.learning_rate = 0.001
        self.gamma = 0.95  # Discount factor
        self.batch_size = 32

        # Neural networks
        self.q_network = DQN(self.state_size, self.action_size)
        self.target_network = DQN(self.state_size, self.action_size)
        self.optimizer = optim.Adam(self.q_network.parameters(), lr=self.learning_rate)

        # Publishers and subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.reward_pub = self.create_publisher(Float32, 'rl_reward', 10)

        # Internal state
        self.current_state = None
        self.previous_state = None
        self.previous_action = None
        self.step_count = 0

        # Update target network
        self.update_target_network()

        self.get_logger().info('DQN Agent initialized')

    def update_target_network(self):
        """Copy weights from main network to target network"""
        self.target_network.load_state_dict(self.q_network.state_dict())

    def remember(self, state, action, reward, next_state, done):
        """Store experience in replay memory"""
        self.memory.append((state, action, reward, next_state, done))

    def act(self, state):
        """Choose action using epsilon-greedy policy"""
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)

        state_tensor = torch.FloatTensor(state).unsqueeze(0)
        q_values = self.q_network(state_tensor)
        return np.argmax(q_values.cpu().data.numpy())

    def replay(self):
        """Train the model on a batch of experiences"""
        if len(self.memory) < self.batch_size:
            return

        batch = random.sample(self.memory, self.batch_size)
        states = torch.FloatTensor([e[0] for e in batch])
        actions = torch.LongTensor([e[1] for e in batch])
        rewards = torch.FloatTensor([e[2] for e in batch])
        next_states = torch.FloatTensor([e[3] for e in batch])
        dones = torch.BoolTensor([e[4] for e in batch])

        current_q_values = self.q_network(states).gather(1, actions.unsqueeze(1))
        next_q_values = self.target_network(next_states).max(1)[0].detach()
        target_q_values = rewards + (self.gamma * next_q_values * ~dones)

        loss = nn.MSELoss()(current_q_values.squeeze(), target_q_values)

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def scan_callback(self, msg):
        """Process laser scan and take action"""
        # Preprocess laser scan to get state
        scan_data = list(msg.ranges)
        # Filter out invalid readings
        scan_data = [r if not (r > msg.range_max or r < msg.range_min) else msg.range_max
                     for r in scan_data]

        # Take every nth reading to reduce state size
        n = len(scan_data) // self.state_size
        state = [np.mean(scan_data[i:i+n]) for i in range(0, len(scan_data), n)][:self.state_size]
        state = np.array(state)

        # Take action based on current state
        action = self.act(state)

        # Calculate reward based on action and state
        reward = self.calculate_reward(state, action)

        # Store experience if we have previous state
        if self.previous_state is not None and self.previous_action is not None:
            self.remember(self.previous_state, self.previous_action, reward, state, False)

        # Publish reward
        reward_msg = Float32()
        reward_msg.data = reward
        self.reward_pub.publish(reward_msg)

        # Execute action
        cmd_vel = Twist()
        if action == 0:  # Turn left
            cmd_vel.angular.z = 0.5
        elif action == 1:  # Go forward
            cmd_vel.linear.x = 0.3
        elif action == 2:  # Turn right
            cmd_vel.angular.z = -0.5

        self.cmd_vel_pub.publish(cmd_vel)

        # Store current state and action for next iteration
        self.previous_state = state
        self.previous_action = action

        # Train the agent periodically
        if self.step_count % 10 == 0:
            self.replay()

        # Update target network periodically
        if self.step_count % 100 == 0:
            self.update_target_network()

        self.step_count += 1

    def calculate_reward(self, state, action):
        """Calculate reward based on current state and action"""
        # Reward for moving forward in free space
        min_distance = min(state)

        if min_distance < 0.3:  # Too close to obstacle
            return -1.0
        elif min_distance > 1.0:  # Safe distance
            if action == 1:  # Moving forward
                return 0.5
            else:  # Turning when not necessary
                return 0.1
        else:  # Moderate distance
            if action == 1:  # Moving forward in safe space
                return 0.3
            else:  # Turning is okay here
                return 0.2

def main(args=None):
    rclpy.init(args=args)
    node = DQNAgent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 2: Imitation Learning Implementation

### Task 2.1: Create an Imitation Learning Node
Implement behavioral cloning for robot navigation:

```python
#!/usr/bin/env python3
# imitation_learning/imitation_learning/behavioral_cloning.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
from collections import deque
import pickle

class ImitationNet(nn.Module):
    def __init__(self, sensor_input_size, image_input_size, action_size):
        super(ImitationNet, self).__init__()

        # Sensor processing
        self.sensor_fc = nn.Sequential(
            nn.Linear(sensor_input_size, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU()
        )

        # Image processing (simplified for this example)
        self.image_conv = nn.Sequential(
            nn.Conv2d(3, 16, kernel_size=3, stride=2),
            nn.ReLU(),
            nn.Conv2d(16, 32, kernel_size=3, stride=2),
            nn.ReLU(),
            nn.Conv2d(32, 32, kernel_size=3, stride=2),
            nn.ReLU()
        )

        # Combined processing
        self.combined_fc = nn.Sequential(
            nn.Linear(32 + 32*4*4, 128),  # 32 from sensor + 32*4*4 from image
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, action_size)
        )

    def forward(self, sensor_input, image_input):
        sensor_features = self.sensor_fc(sensor_input)

        # Process image
        image_features = self.image_conv(image_input)
        image_features = image_features.view(image_features.size(0), -1)  # Flatten

        # Combine features
        combined = torch.cat((sensor_features, image_features), dim=1)
        return self.combined_fc(combined)

class BehavioralCloning(Node):
    def __init__(self):
        super().__init__('behavioral_cloning')

        # Network parameters
        self.sensor_input_size = 10
        self.image_input_size = (3, 64, 64)  # Channels, Height, Width
        self.action_size = 2  # linear.x and angular.z
        self.learning_rate = 0.001

        # Neural network
        self.network = ImitationNet(self.sensor_input_size, self.image_input_size[0]*self.image_input_size[1]*self.image_input_size[2], self.action_size)
        self.optimizer = optim.Adam(self.network.parameters(), lr=self.learning_rate)
        self.criterion = nn.MSELoss()

        # Publishers and subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel_expert', self.expert_cmd_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.train_mode_sub = self.create_subscription(
            Bool, 'train_mode', self.train_mode_callback, 10)

        # Data storage
        self.dataset = deque(maxlen=10000)
        self.current_sensor_data = None
        self.current_image_data = None
        self.current_expert_action = None
        self.is_training = True

        self.get_logger().info('Behavioral Cloning node initialized')

    def scan_callback(self, msg):
        """Process laser scan data"""
        scan_data = list(msg.ranges)
        # Filter and process scan data
        scan_data = [r if not (r > msg.range_max or r < msg.range_min) else msg.range_max
                     for r in scan_data]

        n = len(scan_data) // self.sensor_input_size
        processed_scan = [np.mean(scan_data[i:i+n]) for i in range(0, len(scan_data), n)][:self.sensor_input_size]
        self.current_sensor_data = np.array(processed_scan)

    def image_callback(self, msg):
        """Process image data"""
        # Convert ROS Image to tensor (simplified)
        # In practice, you'd convert the actual image data
        # For this example, we'll create a dummy tensor
        self.current_image_data = torch.randn(3, 64, 64)  # Dummy image

    def expert_cmd_callback(self, msg):
        """Store expert demonstration"""
        self.current_expert_action = np.array([msg.linear.x, msg.angular.z])

    def train_mode_callback(self, msg):
        """Toggle training mode"""
        self.is_training = msg.data

    def store_experience(self):
        """Store current state-action pair in dataset"""
        if (self.current_sensor_data is not None and
            self.current_image_data is not None and
            self.current_expert_action is not None):

            experience = {
                'sensor': self.current_sensor_data.copy(),
                'image': self.current_image_data.clone(),
                'action': self.current_expert_action.copy()
            }
            self.dataset.append(experience)

    def train_batch(self, batch_size=32):
        """Train on a batch of experiences"""
        if len(self.dataset) < batch_size:
            return

        # Sample random batch
        indices = np.random.choice(len(self.dataset), batch_size, replace=False)
        batch = [self.dataset[i] for i in indices]

        # Prepare batch tensors
        sensor_batch = torch.FloatTensor([exp['sensor'] for exp in batch])
        image_batch = torch.stack([exp['image'] for exp in batch])
        action_batch = torch.FloatTensor([exp['action'] for exp in batch])

        # Forward pass
        predicted_actions = self.network(sensor_batch, image_batch)
        loss = self.criterion(predicted_actions, action_batch)

        # Backward pass
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        return loss.item()

    def predict_action(self):
        """Predict action based on current sensor and image data"""
        if self.current_sensor_data is not None and self.current_image_data is not None:
            sensor_tensor = torch.FloatTensor(self.current_sensor_data).unsqueeze(0)
            image_tensor = self.current_image_data.unsqueeze(0)

            with torch.no_grad():
                action = self.network(sensor_tensor, image_tensor).squeeze(0).numpy()

            return action
        return None

    def timer_callback(self):
        """Main control loop"""
        if self.is_training and self.current_expert_action is not None:
            # Store experience for training
            self.store_experience()

            # Train the network periodically
            if len(self.dataset) >= 32:
                loss = self.train_batch()
                if loss:
                    self.get_logger().info(f'Training loss: {loss:.4f}')
        else:
            # Execute learned policy
            action = self.predict_action()
            if action is not None:
                cmd_vel = Twist()
                cmd_vel.linear.x = float(action[0])
                cmd_vel.angular.z = float(action[1])
                self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = BehavioralCloning()

    # Create a timer for the main loop
    node.timer = node.create_timer(0.1, node.timer_callback)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 3: Multi-Objective Optimization

### Task 3.1: Create a Multi-Objective Optimizer
Implement a system that balances multiple objectives:

```python
#!/usr/bin/env python3
# multi_objective_optimizer/multi_objective_optimizer/multi_objective_optimizer.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
import numpy as np
from scipy.optimize import minimize
from typing import List, Tuple

class MultiObjectiveOptimizer(Node):
    def __init__(self):
        super().__init__('multi_objective_optimizer')

        # Publishers and subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.objective_values_pub = self.create_publisher(
            Float32MultiArray, 'objective_values', 10)

        # Parameters
        self.current_scan = None
        self.current_goal = None
        self.weights = np.array([0.4, 0.3, 0.3])  # [safety, efficiency, comfort]

        # Internal state
        self.last_cmd_vel = Twist()

        self.get_logger().info('Multi-Objective Optimizer initialized')

    def scan_callback(self, msg):
        """Process laser scan data"""
        self.current_scan = list(msg.ranges)

    def goal_callback(self, msg):
        """Process goal pose"""
        self.current_goal = msg.pose

    def calculate_objectives(self, cmd_vel: Twist) -> List[float]:
        """Calculate multiple objectives for a given command"""
        objectives = []

        # Objective 1: Safety (minimize collision risk)
        safety_obj = self.calculate_safety_objective(cmd_vel)
        objectives.append(safety_obj)

        # Objective 2: Efficiency (minimize time to goal)
        efficiency_obj = self.calculate_efficiency_objective(cmd_vel)
        objectives.append(efficiency_obj)

        # Objective 3: Comfort (minimize jerky movements)
        comfort_obj = self.calculate_comfort_objective(cmd_vel)
        objectives.append(comfort_obj)

        return objectives

    def calculate_safety_objective(self, cmd_vel: Twist) -> float:
        """Calculate safety objective (lower is better)"""
        if not self.current_scan:
            return 0.0

        # Check distances in front of robot
        front_ranges = self.current_scan[len(self.current_scan)//2-30:len(self.current_scan)//2+30]
        min_dist = min([r for r in front_ranges if not (r > 10.0 or r < 0.1)], default=10.0)

        # Safety penalty increases as we get closer to obstacles
        if min_dist < 0.5:
            safety_penalty = 10.0 * (0.5 - min_dist)
        else:
            safety_penalty = 0.0

        # Also penalize high forward speed when close to obstacles
        if min_dist < 1.0 and cmd_vel.linear.x > 0.2:
            safety_penalty += cmd_vel.linear.x * (1.0 - min_dist)

        return safety_penalty

    def calculate_efficiency_objective(self, cmd_vel: Twist) -> float:
        """Calculate efficiency objective (lower is better)"""
        if not self.current_goal:
            return 0.0

        # Maximize forward progress toward goal
        # This is negative because we want to maximize progress (minimize negative progress)
        efficiency = -cmd_vel.linear.x  # Negative because we minimize this
        return efficiency

    def calculate_comfort_objective(self, cmd_vel: Twist) -> float:
        """Calculate comfort objective (lower is better)"""
        # Penalize rapid changes in velocity
        linear_change = abs(cmd_vel.linear.x - self.last_cmd_vel.linear.x)
        angular_change = abs(cmd_vel.angular.z - self.last_cmd_vel.angular.z)

        comfort_penalty = linear_change + angular_change
        return comfort_penalty

    def weighted_sum_objective(self, velocities: List[float]) -> float:
        """Calculate weighted sum of objectives"""
        cmd_vel = Twist()
        cmd_vel.linear.x = velocities[0]
        cmd_vel.angular.z = velocities[1]

        objectives = self.calculate_objectives(cmd_vel)

        # Normalize objectives
        normalized_objectives = []
        for obj in objectives:
            # Simple normalization (in practice, you'd use proper normalization)
            normalized_objectives.append(obj / (1.0 + abs(obj)))

        weighted_sum = np.dot(self.weights, normalized_objectives)
        return weighted_sum

    def optimize_control(self) -> Twist:
        """Optimize control command using multi-objective approach"""
        if not self.current_scan:
            # Default safe command
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = 0.0
            return cmd_vel

        # Define bounds for optimization
        bounds = [
            (0.0, 0.5),    # linear.x: 0 to 0.5 m/s
            (-1.0, 1.0)    # angular.z: -1 to 1 rad/s
        ]

        # Initial guess
        initial_guess = [0.2, 0.0]  # linear.x, angular.z

        # Optimize
        try:
            result = minimize(
                self.weighted_sum_objective,
                initial_guess,
                method='L-BFGS-B',
                bounds=bounds
            )

            if result.success:
                cmd_vel = Twist()
                cmd_vel.linear.x = max(0.0, min(0.5, result.x[0]))
                cmd_vel.angular.z = max(-1.0, min(1.0, result.x[1]))

                # Update last command for comfort calculation
                self.last_cmd_vel = cmd_vel

                return cmd_vel
        except Exception as e:
            self.get_logger().error(f'Optimization failed: {e}')

        # Fallback: safe command
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.1
        cmd_vel.angular.z = 0.0
        return cmd_vel

    def timer_callback(self):
        """Main optimization loop"""
        if self.current_scan:
            # Optimize control command
            cmd_vel = self.optimize_control()

            # Publish command
            self.cmd_vel_pub.publish(cmd_vel)

            # Publish objective values for monitoring
            objectives = self.calculate_objectives(cmd_vel)
            obj_msg = Float32MultiArray()
            obj_msg.data = objectives
            self.objective_values_pub.publish(obj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MultiObjectiveOptimizer()

    # Create a timer for the optimization loop
    node.timer = node.create_timer(0.1, node.timer_callback)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 4: End-to-End Learning System

### Task 4.1: Create an End-to-End Learning Node
Implement a complete perception-to-action system:

```python
#!/usr/bin/env python3
# reinforcement_learning/reinforcement_learning/end_to_end_learner.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
from collections import deque
import cv2

class EndToEndNet(nn.Module):
    def __init__(self):
        super(EndToEndNet, self).__init__()

        # Vision processing
        self.vision_conv = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            nn.ReLU()
        )

        # Calculate the size of flattened features
        conv_out_size = 64 * 7 * 7  # Adjust based on your image input size

        # Sensor processing
        self.sensor_fc = nn.Sequential(
            nn.Linear(10, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU()
        )

        # Combined processing
        self.combined_fc = nn.Sequential(
            nn.Linear(conv_out_size + 64, 512),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(512, 256),
            nn.ReLU(),
            nn.Linear(256, 2)  # Output: linear.x, angular.z
        )

    def forward(self, image, sensor):
        # Process image
        image_features = self.vision_conv(image)
        image_features = image_features.view(image_features.size(0), -1)  # Flatten

        # Process sensor data
        sensor_features = self.sensor_fc(sensor)

        # Combine features
        combined = torch.cat((image_features, sensor_features), dim=1)

        # Output control commands
        output = self.combined_fc(combined)

        # Apply tanh to bound outputs, then scale appropriately
        linear_x = torch.tanh(output[:, 0]) * 0.5  # Scale to [-0.5, 0.5]
        angular_z = torch.tanh(output[:, 1]) * 1.0  # Scale to [-1.0, 1.0]

        return torch.stack([linear_x, angular_z], dim=1)

class EndToEndLearner(Node):
    def __init__(self):
        super().__init__('end_to_end_learner')

        # Neural network
        self.network = EndToEndNet()
        self.optimizer = optim.Adam(self.network.parameters(), lr=0.0001)
        self.criterion = nn.MSELoss()

        # Publishers and subscribers
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel_expert', self.expert_cmd_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.loss_pub = self.create_publisher(Float32, 'training_loss', 10)

        # Data storage
        self.image_buffer = None
        self.scan_buffer = None
        self.expert_cmd_buffer = None
        self.training_data = deque(maxlen=1000)

        # Training parameters
        self.is_training = True
        self.training_step = 0

        self.get_logger().info('End-to-End Learner initialized')

    def image_callback(self, msg):
        """Process image data"""
        # Convert ROS Image to OpenCV and then to tensor
        # This is a simplified version - in practice you'd properly convert the image
        try:
            # For this example, we'll create a dummy tensor
            # In real implementation, you'd convert msg.data properly
            image_tensor = torch.randn(1, 3, 64, 64)  # Batch, Channels, Height, Width
            self.image_buffer = image_tensor
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def scan_callback(self, msg):
        """Process laser scan data"""
        scan_data = list(msg.ranges)
        # Filter and normalize scan data
        scan_data = [min(r, 10.0) for r in scan_data if r > 0.1]  # Remove invalid readings

        # Take every nth reading to reduce to 10 values
        if len(scan_data) >= 10:
            n = len(scan_data) // 10
            processed_scan = [np.mean(scan_data[i:i+n]) for i in range(0, len(scan_data), n)][:10]
        else:
            processed_scan = scan_data + [10.0] * (10 - len(scan_data))

        scan_tensor = torch.FloatTensor(processed_scan).unsqueeze(0)  # Add batch dimension
        self.scan_buffer = scan_tensor

    def expert_cmd_callback(self, msg):
        """Store expert command for training"""
        cmd_tensor = torch.FloatTensor([msg.linear.x, msg.angular.z]).unsqueeze(0)  # Add batch dimension
        self.expert_cmd_buffer = cmd_tensor

    def store_training_data(self):
        """Store current data for training"""
        if (self.image_buffer is not None and
            self.scan_buffer is not None and
            self.expert_cmd_buffer is not None):

            self.training_data.append({
                'image': self.image_buffer.clone(),
                'scan': self.scan_buffer.clone(),
                'expert_cmd': self.expert_cmd_buffer.clone()
            })

    def train_step(self):
        """Perform one training step"""
        if len(self.training_data) < 32:  # Batch size
            return None

        # Sample a batch
        batch_indices = np.random.choice(len(self.training_data), 32, replace=False)
        batch = [self.training_data[i] for i in batch_indices]

        # Combine batch data
        images = torch.cat([item['image'] for item in batch], dim=0)
        scans = torch.cat([item['scan'] for item in batch], dim=0)
        expert_cmds = torch.cat([item['expert_cmd'] for item in batch], dim=0)

        # Forward pass
        predicted_cmds = self.network(images, scans)

        # Calculate loss
        loss = self.criterion(predicted_cmds, expert_cmds)

        # Backward pass
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        return loss.item()

    def predict_control(self):
        """Predict control command based on current inputs"""
        if (self.image_buffer is not None and
            self.scan_buffer is not None):

            with torch.no_grad():
                predicted_cmd = self.network(self.image_buffer, self.scan_buffer)

            return predicted_cmd.squeeze(0).numpy()  # Remove batch dimension
        return None

    def timer_callback(self):
        """Main control loop"""
        if self.is_training and self.expert_cmd_buffer is not None:
            # Store data for training
            self.store_training_data()

            # Perform training step
            if len(self.training_data) >= 32:
                loss = self.train_step()
                if loss:
                    self.get_logger().info(f'Training loss: {loss:.6f}')

                    # Publish loss
                    loss_msg = Float32()
                    loss_msg.data = loss
                    self.loss_pub.publish(loss_msg)
        else:
            # Execute learned policy
            predicted_cmd = self.predict_control()
            if predicted_cmd is not None:
                cmd_vel = Twist()
                cmd_vel.linear.x = float(predicted_cmd[0])
                cmd_vel.angular.z = float(predicted_cmd[1])
                self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = EndToEndLearner()

    # Create a timer for the main loop
    node.timer = node.create_timer(0.1, node.timer_callback)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 5: Complete System Integration

### Task 5.1: Create Launch File
Create a launch file to start all advanced AI components:

```python
# reinforcement_learning/launch/advanced_ai_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # DQN Agent
        Node(
            package='reinforcement_learning',
            executable='dqn_agent',
            name='dqn_agent',
            output='screen',
        ),
        # Behavioral Cloning
        Node(
            package='imitation_learning',
            executable='behavioral_cloning',
            name='behavioral_cloning',
            output='screen',
        ),
        # Multi-Objective Optimizer
        Node(
            package='multi_objective_optimizer',
            executable='multi_objective_optimizer',
            name='multi_objective_optimizer',
            output='screen',
        ),
        # End-to-End Learner
        Node(
            package='reinforcement_learning',
            executable='end_to_end_learner',
            name='end_to_end_learner',
            output='screen',
        ),
    ])
```

## Exercises

### Exercise 1: Implement Proximal Policy Optimization (PPO)
Replace the DQN algorithm with PPO for better sample efficiency:
- Implement the PPO algorithm
- Compare performance with DQN
- Analyze sample efficiency improvements

### Exercise 2: Add Curriculum Learning
Implement a curriculum learning approach:
- Start with simple navigation tasks
- Gradually increase complexity
- Track learning progress across curriculum stages

### Exercise 3: Implement Safe RL
Add safety constraints to the learning process:
- Define safety boundaries
- Implement constrained optimization
- Test in simulation with safety violations

## Summary

In this lab, you implemented a complete advanced AI integration system with:
1. Reinforcement learning with DQN
2. Imitation learning with behavioral cloning
3. Multi-objective optimization
4. End-to-end learning approaches

The system demonstrates how different AI techniques can be integrated to create adaptive robotic systems.

## Next Steps

- Deploy on a physical robot
- Implement more sophisticated RL algorithms
- Add more sensory modalities
- Test in complex real-world environments