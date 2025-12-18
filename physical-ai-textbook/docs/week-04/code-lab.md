---
title: "Week 4: Digital Twins and Edge Computing Code Lab"
week: 4
module: "Foundations of Physical AI"
difficulty: "intermediate"
prerequisites: ["physics-simulation", "urdf", "ros2-basics", "sensor-integration"]
learning_objectives:
  - "Implement digital twin architecture patterns"
  - "Deploy simulation on Jetson Orin Nano"
  - "Create sensor simulation with realistic noise models"
  - "Validate 'Sim-to-Real' transfer capabilities"
tags: ["digital-twins", "edge-computing", "jetson", "simulation", "sensors"]
hardware_requirements:
  - gpu: "RTX 4070 or Jetson Orin Nano"
  - ram: "16GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "120 minutes"
---

# Week 4: Digital Twins and Edge Computing Code Lab

## Learning Objectives
- Implement digital twin architecture patterns
- Deploy simulation on Jetson Orin Nano
- Create sensor simulation with realistic noise models
- Validate 'Sim-to-Real' transfer capabilities

## 4.1 Setting Up Digital Twin Environment

First, let's set up the environment for both workstation and edge digital twins:

```bash
# Install required packages for digital twin
sudo apt update
sudo apt install ros-humble-rosbridge-suite  # For web communication
sudo apt install ros-humble-navigation2      # For navigation simulation
sudo apt install ros-humble-robot-localization # For localization

# Install Python dependencies
pip3 install numpy scipy matplotlib opencv-python
pip3 install transforms3d  # For 3D transformations
pip3 install plotly        # For 3D visualization
```

## 4.2 Digital Twin Architecture Implementation

Let's implement a flexible digital twin architecture that can work on both workstation and edge:

```python
#!/usr/bin/env python3
# digital_twin_architecture.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, Image, LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np
import json
import time
from enum import Enum

class TwinArchitecture(Enum):
    WORKSTATION = "workstation"
    EDGE = "edge"
    HYBRID = "hybrid"

class DigitalTwinNode(Node):
    """Digital twin node with configurable architecture"""

    def __init__(self):
        super().__init__('digital_twin_node')

        # Determine architecture based on available hardware
        self.architecture = self._detect_architecture()
        self.get_logger().info(f'Digital twin architecture: {self.architecture.value}')

        # Publishers for twin state
        self.twin_joint_pub = self.create_publisher(JointState, '/twin/joint_states', 10)
        self.twin_imu_pub = self.create_publisher(Imu, '/twin/imu', 10)
        self.twin_odom_pub = self.create_publisher(Odometry, '/twin/odometry', 10)

        # Subscribers for real robot data
        self.real_joint_sub = self.create_subscription(JointState, '/real/joint_states', self.real_joint_callback, 10)
        self.real_imu_sub = self.create_subscription(Imu, '/real/imu', self.real_imu_callback, 10)

        # Timer for twin update
        self.update_timer = self.create_timer(0.05, self.update_twin)  # 20 Hz

        # State storage
        self.real_state = {}
        self.twin_state = {}
        self.last_update_time = time.time()

        # Architecture-specific optimizations
        self._configure_for_architecture()

    def _detect_architecture(self):
        """Detect the appropriate architecture based on hardware"""
        # In a real implementation, this would check for Jetson hardware
        # For this example, we'll default to workstation
        return TwinArchitecture.WORKSTATION

    def _configure_for_architecture(self):
        """Configure twin based on architecture"""
        if self.architecture == TwinArchitecture.EDGE:
            # Use simplified physics and reduced update rates
            self.simulation_step = 0.02  # 50 Hz for edge
            self.use_approximate_physics = True
            self.reduced_visual_fidelity = True
        else:
            # Use full-fidelity simulation
            self.simulation_step = 0.001  # 1000 Hz for workstation
            self.use_approximate_physics = False
            self.reduced_visual_fidelity = False

    def real_joint_callback(self, msg):
        """Update real robot state"""
        for name, pos, vel, eff in zip(msg.name, msg.position, msg.velocity, msg.effort):
            self.real_state[name] = {
                'position': pos,
                'velocity': vel,
                'effort': eff
            }

    def real_imu_callback(self, msg):
        """Update real IMU data"""
        self.real_state['imu'] = {
            'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
            'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            'orientation': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        }

    def update_twin(self):
        """Update digital twin state"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        # Update twin state based on real state and physics simulation
        self._simulate_physics(dt)
        self._apply_sensor_noise()
        self._publish_twin_state()

    def _simulate_physics(self, dt):
        """Simulate physics for the twin"""
        # In a real implementation, this would integrate physics equations
        # For this example, we'll synchronize twin with real state
        for joint_name, joint_data in self.real_state.items():
            if joint_name != 'imu':  # Skip IMU, handle separately
                if joint_name not in self.twin_state:
                    self.twin_state[joint_name] = joint_data.copy()
                else:
                    # Apply some lag to simulate real physics
                    alpha = 0.95  # Lag factor
                    self.twin_state[joint_name]['position'] = (
                        alpha * self.twin_state[joint_name]['position'] +
                        (1 - alpha) * joint_data['position']
                    )

    def _apply_sensor_noise(self):
        """Apply realistic sensor noise to twin"""
        # Add noise to IMU data
        if 'imu' in self.real_state:
            noise_std = 0.01 if self.architecture == TwinArchitecture.EDGE else 0.005
            self.twin_state['imu'] = self._add_imu_noise(self.real_state['imu'], noise_std)

    def _add_imu_noise(self, imu_data, noise_std):
        """Add realistic noise to IMU data"""
        noisy_data = {}
        for key, value in imu_data.items():
            if isinstance(value, list):
                noisy_data[key] = [v + np.random.normal(0, noise_std) for v in value]
            else:
                noisy_data[key] = value
        return noisy_data

    def _publish_twin_state(self):
        """Publish twin state to ROS topics"""
        # Publish joint states
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.header.frame_id = 'twin_base_link'

        for joint_name, joint_data in self.twin_state.items():
            if joint_name != 'imu':  # Skip IMU data
                joint_msg.name.append(joint_name)
                joint_msg.position.append(joint_data['position'])
                joint_msg.velocity.append(joint_data.get('velocity', 0.0))
                joint_msg.effort.append(joint_data.get('effort', 0.0))

        self.twin_joint_pub.publish(joint_msg)

        # Publish IMU data
        if 'imu' in self.twin_state:
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'twin_imu_link'

            # Set orientation
            imu_msg.orientation.x = self.twin_state['imu']['orientation'][0]
            imu_msg.orientation.y = self.twin_state['imu']['orientation'][1]
            imu_msg.orientation.z = self.twin_state['imu']['orientation'][2]
            imu_msg.orientation.w = self.twin_state['imu']['orientation'][3]

            # Set angular velocity
            imu_msg.angular_velocity.x = self.twin_state['imu']['angular_velocity'][0]
            imu_msg.angular_velocity.y = self.twin_state['imu']['angular_velocity'][1]
            imu_msg.angular_velocity.z = self.twin_state['imu']['angular_velocity'][2]

            # Set linear acceleration
            imu_msg.linear_acceleration.x = self.twin_state['imu']['linear_acceleration'][0]
            imu_msg.linear_acceleration.y = self.twin_state['imu']['linear_acceleration'][1]
            imu_msg.linear_acceleration.z = self.twin_state['imu']['linear_acceleration'][2]

            self.twin_imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    twin_node = DigitalTwinNode()

    try:
        rclpy.spin(twin_node)
    except KeyboardInterrupt:
        pass
    finally:
        twin_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 4.3 Edge-Optimized Simulation

Let's create an optimized simulation specifically for Jetson Orin Nano:

```python
#!/usr/bin/env python3
# edge_optimized_simulator.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
import threading
from collections import deque

class EdgeOptimizedSimulator(Node):
    """Lightweight simulator optimized for Jetson Orin Nano"""

    def __init__(self):
        super().__init__('edge_simulator')

        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/edge/joint_states', 10)
        self.joint_command_sub = self.create_subscription(Float64MultiArray, '/edge/joint_commands', self.joint_command_callback, 10)

        # Timer for simulation update
        self.sim_timer = self.create_timer(0.02, self.simulation_step)  # 50 Hz for edge

        # Robot parameters (simplified for performance)
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.joint_positions = np.zeros(len(self.joint_names))
        self.joint_velocities = np.zeros(len(self.joint_names))
        self.joint_commands = np.zeros(len(self.joint_names))

        # Simplified physics parameters
        self.link_lengths = [0.5, 0.4, 0.3, 0.2, 0.15, 0.1]  # [meters]
        self.joint_limits = [(-np.pi, np.pi)] * len(self.joint_names)

        # Performance tracking
        self.update_times = deque(maxlen=100)
        self.target_dt = 0.02  # 50 Hz
        self.last_time = time.time()

        # Threading for performance
        self.sim_lock = threading.Lock()

        self.get_logger().info('Edge Optimized Simulator initialized')

    def joint_command_callback(self, msg):
        """Receive joint commands"""
        with self.sim_lock:
            # Update commands safely
            cmd_len = min(len(msg.data), len(self.joint_commands))
            self.joint_commands[:cmd_len] = msg.data[:cmd_len]

    def simulation_step(self):
        """Main simulation step optimized for edge"""
        start_time = time.time()

        with self.sim_lock:
            # Simple PD controller
            kp = 10.0
            kd = 1.0

            # Calculate control effort
            position_error = self.joint_commands - self.joint_positions
            velocity_error = -self.joint_velocities  # Assuming desired velocity is 0

            control_effort = kp * position_error + kd * velocity_error

            # Simple integration (Euler method) - optimized for performance
            dt = self.target_dt
            self.joint_velocities += control_effort * dt
            self.joint_positions += self.joint_velocities * dt

            # Apply joint limits
            for i, (lower, upper) in enumerate(self.joint_limits):
                self.joint_positions[i] = np.clip(self.joint_positions[i], lower, upper)

        # Publish joint states
        self.publish_joint_states()

        # Track performance
        elapsed_time = time.time() - start_time
        self.update_times.append(elapsed_time)

        # Log performance if needed
        if len(self.update_times) == 100:  # Every 100 updates
            avg_time = np.mean(self.update_times)
            self.get_logger().info(f'Avg update time: {avg_time*1000:.2f}ms, Target: {self.target_dt*1000}ms')

    def publish_joint_states(self):
        """Publish joint state message"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions.tolist()
        msg.velocity = self.joint_velocities.tolist()

        self.joint_state_pub.publish(msg)

    def calculate_forward_kinematics(self):
        """Simplified forward kinematics for performance"""
        # This is a simplified 2D example - extend for 6DOF as needed
        end_effector_x = 0
        end_effector_y = 0

        cumulative_angle = 0
        for i, (length, angle) in enumerate(zip(self.link_lengths, self.joint_positions)):
            cumulative_angle += angle
            end_effector_x += length * np.cos(cumulative_angle)
            end_effector_y += length * np.sin(cumulative_angle)

        return np.array([end_effector_x, end_effector_y, 0])  # 3D position

def main(args=None):
    rclpy.init(args=args)
    simulator = EdgeOptimizedSimulator()

    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        pass
    finally:
        simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 4.4 Sensor Simulation with Realistic Noise Models

Let's implement comprehensive sensor simulation with realistic noise models:

```python
#!/usr/bin/env python3
# sensor_simulation.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

class SensorSimulator(Node):
    """Realistic sensor simulation with noise models"""

    def __init__(self):
        super().__init__('sensor_simulator')

        # Create CV bridge
        self.cv_bridge = CvBridge()

        # Publishers
        self.camera_pub = self.create_publisher(Image, '/simulated_camera/image_raw', 10)
        self.imu_pub = self.create_publisher(Imu, '/simulated_imu', 10)
        self.lidar_pub = self.create_publisher(LaserScan, '/simulated_lidar', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/simulated_camera/camera_info', 10)

        # Timer for sensor simulation
        self.sensor_timer = self.create_timer(0.1, self.simulate_sensors)  # 10 Hz for all sensors

        # Sensor parameters
        self.camera_params = {
            'width': 640,
            'height': 480,
            'fov': 60,  # degrees
            'noise_std': 0.005,
            'update_rate': 30
        }

        self.imu_params = {
            'accel_noise_std': 0.01,
            'gyro_noise_std': 0.001,
            'accel_bias_std': 0.001,
            'gyro_bias_std': 0.0001,
            'update_rate': 100
        }

        self.lidar_params = {
            'angle_min': -np.pi/2,
            'angle_max': np.pi/2,
            'angle_increment': np.pi/180,  # 1 degree
            'range_min': 0.1,
            'range_max': 10.0,
            'noise_std': 0.01,
            'update_rate': 10
        }

        # Initialize sensor states
        self.robot_pose = np.array([0, 0, 0])  # x, y, theta
        self.robot_velocity = np.array([0, 0, 0])  # vx, vy, vtheta
        self.gravity = np.array([0, 0, -9.81])

        # Noise generators
        self.accel_bias = np.random.normal(0, self.imu_params['accel_bias_std'], 3)
        self.gyro_bias = np.random.normal(0, self.imu_params['gyro_bias_std'], 3)

        self.get_logger().info('Sensor Simulator initialized')

    def simulate_sensors(self):
        """Simulate all sensors with realistic noise"""
        # Update robot pose (simple kinematic model)
        self.update_robot_pose()

        # Simulate and publish each sensor
        self.simulate_camera()
        self.simulate_imu()
        self.simulate_lidar()

    def update_robot_pose(self):
        """Simple kinematic model to update robot pose"""
        # This would be connected to the physics simulation in a real implementation
        dt = 0.1  # Time step for this function
        v_linear = 0.1  # Constant forward velocity
        v_angular = 0.05 * np.sin(self.get_clock().now().nanoseconds / 1e9)  # Oscillating turn

        # Update pose
        self.robot_pose[0] += v_linear * np.cos(self.robot_pose[2]) * dt
        self.robot_pose[1] += v_linear * np.sin(self.robot_pose[2]) * dt
        self.robot_pose[2] += v_angular * dt

        # Normalize angle
        self.robot_pose[2] = np.arctan2(np.sin(self.robot_pose[2]), np.cos(self.robot_pose[2]))

    def simulate_camera(self):
        """Simulate realistic camera with noise"""
        # Create a synthetic image (in a real implementation, this would come from a 3D renderer)
        img = np.zeros((self.camera_params['height'], self.camera_params['width'], 3), dtype=np.uint8)

        # Add some synthetic features
        center_x, center_y = self.camera_params['width'] // 2, self.camera_params['height'] // 2
        cv2.circle(img, (center_x, center_y), 50, (255, 0, 0), -1)  # Blue circle
        cv2.rectangle(img, (100, 100), (200, 200), (0, 255, 0), 2)  # Green square

        # Add realistic noise
        noise = np.random.normal(0, self.camera_params['noise_std'] * 255, img.shape)
        noisy_img = np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)

        # Publish image
        img_msg = self.cv_bridge.cv2_to_imgmsg(noisy_img, encoding="bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_link'
        self.camera_pub.publish(img_msg)

        # Publish camera info
        camera_info = CameraInfo()
        camera_info.header = img_msg.header
        camera_info.width = self.camera_params['width']
        camera_info.height = self.camera_params['height']
        # Simple camera matrix (assumes no distortion)
        f = (self.camera_params['width']) / (2 * np.tan(np.radians(self.camera_params['fov']/2)))
        camera_info.k = [f, 0, self.camera_params['width']/2,
                        0, f, self.camera_params['height']/2,
                        0, 0, 1]
        self.camera_info_pub.publish(camera_info)

    def simulate_imu(self):
        """Simulate realistic IMU with bias and noise"""
        # True values (would come from physics simulation)
        true_linear_accel = self.gravity + np.array([0.1, 0.05, 0.02])  # Add some acceleration
        true_angular_vel = np.array([0.01, -0.02, 0.05])  # Angular velocities

        # Apply bias and noise
        measured_accel = (true_linear_accel +
                         self.accel_bias +
                         np.random.normal(0, self.imu_params['accel_noise_std'], 3))

        measured_gyro = (true_angular_vel +
                        self.gyro_bias +
                        np.random.normal(0, self.imu_params['gyro_noise_std'], 3))

        # Create and publish IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Set linear acceleration
        imu_msg.linear_acceleration.x = measured_accel[0]
        imu_msg.linear_acceleration.y = measured_accel[1]
        imu_msg.linear_acceleration.z = measured_accel[2]

        # Set angular velocity
        imu_msg.angular_velocity.x = measured_gyro[0]
        imu_msg.angular_velocity.y = measured_gyro[1]
        imu_msg.angular_velocity.z = measured_gyro[2]

        # For a real implementation, you would integrate to get orientation
        # Here we'll just set a dummy orientation
        imu_msg.orientation.w = 1.0  # No rotation

        # Set covariance (indicates uncertainty)
        imu_msg.linear_acceleration_covariance[0] = self.imu_params['accel_noise_std']**2
        imu_msg.linear_acceleration_covariance[4] = self.imu_params['accel_noise_std']**2
        imu_msg.linear_acceleration_covariance[8] = self.imu_params['accel_noise_std']**2

        imu_msg.angular_velocity_covariance[0] = self.imu_params['gyro_noise_std']**2
        imu_msg.angular_velocity_covariance[4] = self.imu_params['gyro_noise_std']**2
        imu_msg.angular_velocity_covariance[8] = self.imu_params['gyro_noise_std']**2

        self.imu_pub.publish(imu_msg)

    def simulate_lidar(self):
        """Simulate realistic LiDAR with noise"""
        # Calculate number of points
        num_points = int((self.lidar_params['angle_max'] - self.lidar_params['angle_min']) /
                        self.lidar_params['angle_increment']) + 1

        # Generate ideal ranges (in a real implementation, this would come from ray tracing)
        angles = np.linspace(self.lidar_params['angle_min'], self.lidar_params['angle_max'], num_points)

        # Create synthetic environment - a circular obstacle
        robot_x, robot_y = self.robot_pose[0], self.robot_pose[1]
        obstacle_x, obstacle_y, obstacle_r = 2.0, 0.0, 0.5  # Obstacle at (2,0) with radius 0.5

        # Calculate distances to obstacle
        distances = np.sqrt((obstacle_x - (robot_x + 2*np.cos(angles)))**2 +
                           (obstacle_y - (robot_y + 2*np.sin(angles)))**2) - obstacle_r

        # Add noise
        noisy_ranges = distances + np.random.normal(0, self.lidar_params['noise_std'], len(distances))

        # Ensure valid range values
        noisy_ranges = np.clip(noisy_ranges, self.lidar_params['range_min'], self.lidar_params['range_max'])

        # Create and publish LaserScan message
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'lidar_link'

        scan_msg.angle_min = self.lidar_params['angle_min']
        scan_msg.angle_max = self.lidar_params['angle_max']
        scan_msg.angle_increment = self.lidar_params['angle_increment']
        scan_msg.time_increment = 0.0  # Not used in simulation
        scan_msg.scan_time = 0.1  # 10 Hz
        scan_msg.range_min = self.lidar_params['range_min']
        scan_msg.range_max = self.lidar_params['range_max']

        scan_msg.ranges = noisy_ranges.tolist()
        # Intensities are optional, set to empty
        scan_msg.intensities = []

        self.lidar_pub.publish(scan_msg)

def main(args=None):
    rclpy.init(args=args)
    sensor_simulator = SensorSimulator()

    try:
        rclpy.spin(sensor_simulator)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 4.5 Jetson Orin Nano Deployment Script

Let's create a deployment script optimized for Jetson:

```bash
#!/bin/bash
# deploy_to_jetson.sh

# Script to deploy and run digital twin on Jetson Orin Nano

echo "Deploying Digital Twin to Jetson Orin Nano..."

# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS 2 Humble
if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install ros-humble-ros-base
fi

# Install additional packages for edge computing
sudo apt install python3-pip python3-colcon-common-extensions
pip3 install numpy scipy matplotlib opencv-python transforms3d

# Create workspace
mkdir -p ~/jetson_ws/src
cd ~/jetson_ws

# Source ROS environment
source /opt/ros/humble/setup.bash

# Build workspace
colcon build --packages-select edge_optimized_simulator

# Source the workspace
source install/setup.bash

# Run the edge-optimized simulator
echo "Starting Edge Optimized Simulator..."
ros2 run edge_optimized_simulator edge_simulator &
EDGE_SIM_PID=$!

# Run the sensor simulator
echo "Starting Sensor Simulator..."
ros2 run sensor_simulation sensor_simulator &
SENSOR_SIM_PID=$!

# Monitor resource usage
echo "Monitoring resource usage..."
htop &

# Function to cleanup on exit
cleanup() {
    echo "Stopping processes..."
    kill $EDGE_SIM_PID $SENSOR_SIM_PID
    exit 0
}

# Setup trap to handle Ctrl+C
trap cleanup INT

# Wait for processes
wait $EDGE_SIM_PID $SENSOR_SIM_PID
```

## 4.6 VSLAM Simulation Implementation

Let's implement a VSLAM simulation for the digital twin:

```python
#!/usr/bin/env python3
# vslam_simulation.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from collections import deque
import threading

class VSLAMSimulator(Node):
    """Visual SLAM simulation for digital twin"""

    def __init__(self):
        super().__init__('vslam_simulator')

        # Create CV bridge
        self.cv_bridge = CvBridge()

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/vslam/odom', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/vslam/pose', 10)
        self.feature_pub = self.create_publisher(PointStamped, '/vslam/features', 10)

        # Subscribers
        self.image_sub = self.create_subscription(Image, '/simulated_camera/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/simulated_camera/camera_info', self.camera_info_callback, 10)

        # Timer for VSLAM processing
        self.vslam_timer = self.create_timer(0.1, self.process_vslam)  # 10 Hz

        # VSLAM parameters
        self.feature_detector = cv2.SIFT_create(nfeatures=500)  # Limit features for performance
        self.matcher = cv2.BFMatcher()
        self.camera_matrix = None
        self.dist_coeffs = None

        # State variables
        self.prev_image = None
        self.current_image = None
        self.camera_pose = np.eye(4)  # 4x4 transformation matrix
        self.feature_points = deque(maxlen=1000)  # Store map features
        self.frame_count = 0
        self.processing_lock = threading.Lock()

        # Motion model parameters
        self.process_noise = 0.01
        self.measurement_noise = 0.1

        self.get_logger().info('VSLAM Simulator initialized')

    def camera_info_callback(self, msg):
        """Receive camera calibration parameters"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d) if msg.d else np.zeros(5)

    def image_callback(self, msg):
        """Receive camera image"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

            # Store current image
            with self.processing_lock:
                self.prev_image = self.current_image
                self.current_image = cv_image.copy()
                self.frame_count += 1

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def process_vslam(self):
        """Main VSLAM processing loop"""
        if self.prev_image is None or self.current_image is None:
            return

        with self.processing_lock:
            # Detect and match features
            pose_change = self.estimate_motion(self.prev_image, self.current_image)

            if pose_change is not None:
                # Update camera pose
                self.camera_pose = self.camera_pose @ pose_change

                # Publish odometry
                self.publish_odometry()

    def estimate_motion(self, prev_img, curr_img):
        """Estimate motion between two frames using feature matching"""
        if self.camera_matrix is None:
            return None

        # Detect features in both images
        kp_prev, desc_prev = self.feature_detector.detectAndCompute(prev_img, None)
        kp_curr, desc_curr = self.feature_detector.detectAndCompute(curr_img, None)

        if desc_prev is None or desc_curr is None:
            return None

        # Match features
        matches = self.matcher.knnMatch(desc_prev, desc_curr, k=2)

        # Apply Lowe's ratio test
        good_matches = []
        for match_pair in matches:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < 0.7 * n.distance:
                    good_matches.append(m)

        # Need minimum matches for reliable pose estimation
        if len(good_matches) < 10:
            return None

        # Extract matched keypoints
        src_pts = np.float32([kp_prev[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp_curr[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

        # Estimate essential matrix
        E, mask = cv2.findEssentialMat(src_pts, dst_pts, self.camera_matrix,
                                      threshold=1.0, prob=0.999)

        if E is None or E.size == 0:
            return None

        # Recover pose
        _, R, t, _ = cv2.recoverPose(E, src_pts, dst_pts, self.camera_matrix, mask=mask)

        # Create transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t.flatten()

        # Add some noise to simulate real conditions
        T = self.add_pose_noise(T)

        return T

    def add_pose_noise(self, T):
        """Add realistic noise to estimated pose"""
        # Add noise to rotation
        noise_rot = np.random.normal(0, self.process_noise, 3)
        R_noise = R.from_rotvec(noise_rot).as_matrix()
        T[:3, :3] = T[:3, :3] @ R_noise

        # Add noise to translation
        noise_trans = np.random.normal(0, self.measurement_noise, 3)
        T[:3, 3] += noise_trans

        return T

    def publish_odometry(self):
        """Publish odometry message"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'camera_link'

        # Set position
        odom_msg.pose.pose.position.x = self.camera_pose[0, 3]
        odom_msg.pose.pose.position.y = self.camera_pose[1, 3]
        odom_msg.pose.pose.position.z = self.camera_pose[2, 3]

        # Convert rotation matrix to quaternion
        r = R.from_matrix(self.camera_pose[:3, :3])
        quat = r.as_quat()
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # Set velocity (approximate)
        # In a real implementation, this would come from differentiation
        odom_msg.twist.twist.linear.x = 0.1  # Placeholder
        odom_msg.twist.twist.angular.z = 0.05  # Placeholder

        self.odom_pub.publish(odom_msg)

        # Publish pose separately
        pose_msg = PoseStamped()
        pose_msg.header = odom_msg.header
        pose_msg.pose = odom_msg.pose.pose
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    vslam_simulator = VSLAMSimulator()

    try:
        rclpy.spin(vslam_simulator)
    except KeyboardInterrupt:
        pass
    finally:
        vslam_simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 4.7 "Sim-to-Real" Transfer Validation

Let's create a validation framework for Sim-to-Real transfer:

```python
#!/usr/bin/env python3
# sim_to_real_validator.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import numpy as np
import pickle
import time
from collections import deque

class SimToRealValidator(Node):
    """Validate Sim-to-Real transfer capabilities"""

    def __init__(self):
        super().__init__('sim_to_real_validator')

        # Subscribers for both sim and real data
        self.sim_joint_sub = self.create_subscription(JointState, '/sim/joint_states', self.sim_joint_callback, 10)
        self.real_joint_sub = self.create_subscription(JointState, '/real/joint_states', self.real_joint_callback, 10)

        self.sim_imu_sub = self.create_subscription(Imu, '/sim/imu', self.sim_imu_callback, 10)
        self.real_imu_sub = self.create_subscription(Imu, '/real/imu', self.real_imu_callback, 10)

        # Timer for validation
        self.validation_timer = self.create_timer(1.0, self.run_validation)

        # Storage for comparison
        self.sim_data = {
            'joint_positions': deque(maxlen=100),
            'imu_linear_acc': deque(maxlen=100),
            'imu_angular_vel': deque(maxlen=100)
        }

        self.real_data = {
            'joint_positions': deque(maxlen=100),
            'imu_linear_acc': deque(maxlen=100),
            'imu_angular_vel': deque(maxlen=100)
        }

        self.validation_results = {
            'joint_error_mean': 0.0,
            'joint_error_std': 0.0,
            'imu_error_mean': 0.0,
            'imu_error_std': 0.0,
            'transfer_score': 0.0
        }

        # Validation parameters
        self.joint_tolerance = 0.1  # 0.1 rad tolerance
        self.imu_tolerance = 0.5    # 0.5 m/s² or rad/s tolerance

        self.get_logger().info('Sim-to-Real Validator initialized')

    def sim_joint_callback(self, msg):
        """Store simulated joint data"""
        positions = list(msg.position)
        self.sim_data['joint_positions'].append(positions)

    def real_joint_callback(self, msg):
        """Store real joint data"""
        positions = list(msg.position)
        self.real_data['joint_positions'].append(positions)

    def sim_imu_callback(self, msg):
        """Store simulated IMU data"""
        linear_acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        angular_vel = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        self.sim_data['imu_linear_acc'].append(linear_acc)
        self.sim_data['imu_angular_vel'].append(angular_vel)

    def real_imu_callback(self, msg):
        """Store real IMU data"""
        linear_acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        angular_vel = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        self.real_data['imu_linear_acc'].append(linear_acc)
        self.real_data['imu_angular_vel'].append(angular_vel)

    def run_validation(self):
        """Run comprehensive validation"""
        if not self.sim_data['joint_positions'] or not self.real_data['joint_positions']:
            return

        # Validate joint positions
        joint_validation = self.validate_joint_positions()

        # Validate IMU data
        imu_validation = self.validate_imu_data()

        # Calculate overall transfer score
        transfer_score = self.calculate_transfer_score(joint_validation, imu_validation)

        # Log results
        self.log_validation_results(joint_validation, imu_validation, transfer_score)

        # Store results
        self.validation_results.update({
            'joint_error_mean': joint_validation['mean_error'],
            'joint_error_std': joint_validation['std_error'],
            'imu_error_mean': imu_validation['mean_error'],
            'imu_error_std': imu_validation['std_error'],
            'transfer_score': transfer_score
        })

    def validate_joint_positions(self):
        """Validate joint position similarity"""
        if len(self.sim_data['joint_positions']) < 10 or len(self.real_data['joint_positions']) < 10:
            return {'mean_error': float('inf'), 'std_error': float('inf')}

        # Take the most recent data points
        sim_positions = np.array(self.sim_data['joint_positions'][-10:])
        real_positions = np.array(self.real_data['joint_positions'][-10:])

        # Ensure same number of joints
        min_joints = min(sim_positions.shape[1], real_positions.shape[1])
        sim_positions = sim_positions[:, :min_joints]
        real_positions = real_positions[:, :min_joints]

        # Calculate errors
        errors = np.abs(sim_positions - real_positions)
        mean_error = np.mean(errors)
        std_error = np.std(errors)

        return {
            'mean_error': mean_error,
            'std_error': std_error,
            'max_error': np.max(errors),
            'within_tolerance': np.mean(errors < self.joint_tolerance)
        }

    def validate_imu_data(self):
        """Validate IMU data similarity"""
        if len(self.sim_data['imu_linear_acc']) < 10 or len(self.real_data['imu_linear_acc']) < 10:
            return {'mean_error': float('inf'), 'std_error': float('inf')}

        # Validate linear acceleration
        sim_acc = np.array(self.sim_data['imu_linear_acc'][-10:])
        real_acc = np.array(self.real_data['imu_linear_acc'][-10:])

        # Calculate acceleration errors
        acc_errors = np.abs(sim_acc - real_acc)
        acc_mean_error = np.mean(acc_errors)
        acc_std_error = np.std(acc_errors)

        # Validate angular velocity
        sim_vel = np.array(self.sim_data['imu_angular_vel'][-10:])
        real_vel = np.array(self.real_data['imu_angular_vel'][-10:])

        vel_errors = np.abs(sim_vel - real_vel)
        vel_mean_error = np.mean(vel_errors)
        vel_std_error = np.std(vel_errors)

        # Combined IMU error
        combined_errors = np.concatenate([acc_errors.flatten(), vel_errors.flatten()])
        mean_error = np.mean(combined_errors)
        std_error = np.std(combined_errors)

        return {
            'mean_error': mean_error,
            'std_error': std_error,
            'acc_mean_error': acc_mean_error,
            'vel_mean_error': vel_mean_error,
            'within_tolerance': np.mean(combined_errors < self.imu_tolerance)
        }

    def calculate_transfer_score(self, joint_val, imu_val):
        """Calculate overall transfer quality score"""
        # Normalize errors to 0-1 scale (lower is better)
        joint_score = max(0, 1 - joint_val['mean_error'] / self.joint_tolerance)
        imu_score = max(0, 1 - imu_val['mean_error'] / self.imu_tolerance)

        # Weighted average (adjust weights as needed)
        transfer_score = 0.6 * joint_score + 0.4 * imu_score
        return max(0, min(1, transfer_score))  # Clamp to [0, 1]

    def log_validation_results(self, joint_val, imu_val, transfer_score):
        """Log validation results"""
        self.get_logger().info(f"=== Sim-to-Real Validation Results ===")
        self.get_logger().info(f"Joint Position Error - Mean: {joint_val['mean_error']:.4f}, Std: {joint_val['std_error']:.4f}")
        self.get_logger().info(f"IMU Data Error - Mean: {imu_val['mean_error']:.4f}, Std: {imu_val['std_error']:.4f}")
        self.get_logger().info(f"Transfer Quality Score: {transfer_score:.3f}")

        if transfer_score > 0.8:
            self.get_logger().info("✅ High transfer quality - suitable for real deployment")
        elif transfer_score > 0.5:
            self.get_logger().info("⚠️ Moderate transfer quality - may need calibration")
        else:
            self.get_logger().info("❌ Low transfer quality - significant domain gap exists")

    def save_validation_report(self, filename):
        """Save validation results to file"""
        report = {
            'timestamp': time.time(),
            'results': self.validation_results,
            'parameters': {
                'joint_tolerance': self.joint_tolerance,
                'imu_tolerance': self.imu_tolerance
            }
        }

        with open(filename, 'wb') as f:
            pickle.dump(report, f)

        self.get_logger().info(f'Validation report saved to {filename}')

def main(args=None):
    rclpy.init(args=args)
    validator = SimToRealValidator()

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        # Save final validation report
        validator.save_validation_report('/tmp/sim_to_real_validation_report.pkl')
        pass
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 4.8 Domain Randomization Implementation

Let's implement domain randomization for better Sim-to-Real transfer:

```python
#!/usr/bin/env python3
# domain_randomizer.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import random

class DomainRandomizer(Node):
    """Apply domain randomization to simulation parameters"""

    def __init__(self):
        super().__init__('domain_randomizer')

        # Publishers for randomized parameters
        self.param_pub = self.create_publisher(Float64MultiArray, '/domain_randomization/params', 10)

        # Timer for parameter randomization
        self.randomization_timer = self.create_timer(5.0, self.randomize_parameters)  # Every 5 seconds

        # Define parameter ranges for randomization
        self.param_ranges = {
            'friction': (0.1, 0.8),
            'restitution': (0.0, 0.2),
            'mass_multiplier': (0.8, 1.2),
            'inertia_multiplier': (0.8, 1.2),
            'gravity_multiplier': (0.9, 1.1),
            'camera_noise_std': (0.001, 0.01),
            'lidar_noise_std': (0.005, 0.02),
            'imu_accel_noise': (0.005, 0.02),
            'imu_gyro_noise': (0.0005, 0.002),
            'lighting_intensity': (0.5, 1.5)
        }

        # Current randomized parameters
        self.current_params = {}
        self.randomize_parameters()  # Initialize with random parameters

        self.get_logger().info('Domain Randomizer initialized')

    def randomize_parameters(self):
        """Randomize simulation parameters"""
        self.current_params = {}

        for param_name, (min_val, max_val) in self.param_ranges.items():
            self.current_params[param_name] = random.uniform(min_val, max_val)

        # Publish randomized parameters
        self.publish_parameters()

        self.log_current_parameters()

    def publish_parameters(self):
        """Publish current parameters to simulation"""
        param_msg = Float64MultiArray()

        # Add parameter values in a consistent order
        param_names = sorted(self.current_params.keys())
        param_msg.data = [self.current_params[name] for name in param_names]

        # Add parameter names as layout (simplified approach)
        # In a real implementation, you might use a custom message type
        self.param_pub.publish(param_msg)

    def log_current_parameters(self):
        """Log current parameter values"""
        self.get_logger().info("Current domain randomization parameters:")
        for name, value in self.current_params.items():
            self.get_logger().info(f"  {name}: {value:.4f}")

    def get_current_params(self):
        """Get current randomized parameters"""
        return self.current_params.copy()

    def get_param(self, param_name):
        """Get a specific parameter value"""
        return self.current_params.get(param_name, None)

class DomainRandomizedSimulator(Node):
    """Simulator that applies domain randomization"""

    def __init__(self):
        super().__init__('domain_randomized_simulator')

        # Subscribers for domain randomization parameters
        self.param_sub = self.create_subscription(Float64MultiArray, '/domain_randomization/params',
                                                self.param_callback, 10)

        # Publishers for simulated data
        self.joint_pub = self.create_publisher(JointState, '/domain_randomized/joint_states', 10)

        # Timer for simulation
        self.sim_timer = self.create_timer(0.01, self.simulation_step)

        # Initialize with default parameters
        self.current_params = {
            'friction': 0.3,
            'mass_multiplier': 1.0,
            'camera_noise_std': 0.005,
            'imu_accel_noise': 0.01
        }

        # Robot simulation state
        self.joint_positions = np.zeros(6)
        self.joint_velocities = np.zeros(6)

    def param_callback(self, msg):
        """Update parameters from domain randomizer"""
        # In a real implementation, this would update simulation parameters
        # For this example, we'll just log the update
        self.get_logger().info('Received new domain randomization parameters')

    def simulation_step(self):
        """Main simulation step with domain randomization"""
        # Apply current parameters to simulation
        self.apply_randomized_physics()
        self.apply_randomized_sensor_noise()

        # Update robot state
        self.update_robot_dynamics()

        # Publish simulated data
        self.publish_joint_states()

    def apply_randomized_physics(self):
        """Apply randomized physics parameters"""
        # Adjust physics based on current parameters
        friction = self.current_params.get('friction', 0.3)
        mass_mult = self.current_params.get('mass_multiplier', 1.0)

        # Apply effects to simulation (simplified)
        # In a real implementation, this would affect the physics engine directly

    def apply_randomized_sensor_noise(self):
        """Apply randomized sensor noise"""
        camera_noise = self.current_params.get('camera_noise_std', 0.005)
        imu_noise = self.current_params.get('imu_accel_noise', 0.01)

        # Use these parameters when generating sensor data
        # In a real implementation, this would affect sensor simulation

    def update_robot_dynamics(self):
        """Update robot dynamics with randomized parameters"""
        # Simple dynamics update
        dt = 0.01  # Time step

        # Apply some control input with randomized parameters
        control_input = np.random.normal(0, 0.1, 6)  # Random control with noise

        # Update velocities and positions
        self.joint_velocities += control_input * dt
        self.joint_positions += self.joint_velocities * dt

        # Apply joint limits
        self.joint_positions = np.clip(self.joint_positions, -np.pi, np.pi)

    def publish_joint_states(self):
        """Publish joint state message"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'joint_{i}' for i in range(6)]
        msg.position = self.joint_positions.tolist()
        msg.velocity = self.joint_velocities.tolist()

        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    # Create both nodes
    randomizer = DomainRandomizer()
    simulator = DomainRandomizedSimulator()

    # Create executor to run both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(randomizer)
    executor.add_node(simulator)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        randomizer.destroy_node()
        simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 4.9 Practical Exercise

Create a complete digital twin system that:
1. Implements both workstation and edge architectures
2. Includes sensor simulation with realistic noise models
3. Implements VSLAM simulation
4. Validates Sim-to-Real transfer capabilities
5. Applies domain randomization for better transfer

```python
# Student exercise - Complete implementation
class CompleteDigitalTwinSystem:
    """Student implementation of a complete digital twin system"""

    def __init__(self):
        """Initialize the complete digital twin system"""
        # TODO: Implement architecture selection (workstation/edge)
        # TODO: Implement sensor simulation
        # TODO: Implement VSLAM
        # TODO: Implement validation framework
        # TODO: Implement domain randomization
        pass

    def setup_architecture(self):
        """Setup digital twin architecture"""
        # TODO: Complete implementation
        pass

    def implement_sensors(self):
        """Implement realistic sensor simulation"""
        # TODO: Complete implementation
        pass

    def implement_vslam(self):
        """Implement VSLAM simulation"""
        # TODO: Complete implementation
        pass

    def validate_transfer(self):
        """Implement Sim-to-Real validation"""
        # TODO: Complete implementation
        pass

    def apply_domain_randomization(self):
        """Apply domain randomization"""
        # TODO: Complete implementation
        pass

print("Student Exercise: Implement a complete digital twin system")
print("Requirements:")
print("1. Dual architecture support (workstation and edge)")
print("2. Comprehensive sensor simulation with noise models")
print("3. VSLAM implementation")
print("4. Sim-to-Real validation framework")
print("5. Domain randomization for transfer improvement")
```

## Summary

In this lab, we've implemented a comprehensive digital twin system with both workstation and edge architectures, realistic sensor simulation, VSLAM capabilities, and Sim-to-Real validation. We've learned how to optimize simulation for resource-constrained environments like Jetson Orin Nano and how to apply domain randomization techniques to improve transfer learning between simulation and reality.

<div class="alert alert-warning">
  <h5>Hardware Requirement</h5>
  <div><strong>Requirement:</strong> GPU</div>
  <div><strong>Minimum:</strong> RTX 4070 or Jetson Orin Nano</div>
  <div><strong>Recommended:</strong> RTX 4080 or Jetson Orin Nano (higher power mode)</div>
  <div><strong>Purpose:</strong> Digital twin simulation requires significant computational resources, especially for real-time performance</div>
</div>