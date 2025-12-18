---
title: "Week 4: Digital Twins and Edge Computing"
week: 4
module: "Foundations of Physical AI"
difficulty: "intermediate"
prerequisites: ["physics-simulation", "urdf", "ros2-basics"]
learning_objectives:
  - "Differentiate between 'Digital Twin Workstation' and 'Edge Kit'"
  - "Implement sensor simulation"
  - "Understand 'Sim-to-Real' transfer challenges"
tags: ["digital-twins", "edge-computing", "jetson", "unity", "simulation"]
hardware_requirements:
  - gpu: "RTX 4070 or Jetson Orin Nano"
  - ram: "16GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "90 minutes"
---

# Week 4: Digital Twins and Edge Computing

## Learning Objectives
- Differentiate between 'Digital Twin Workstation' and 'Edge Kit' (Jetson Orin Nano)
- Implement sensor simulation for realistic perception
- Understand 'Sim-to-Real' transfer challenges and solutions
- Explore Unity for hardware simulation

## 4.1 Digital Twin Architecture: Workstation vs. Edge

Digital twins in robotics can be implemented in different architectural patterns depending on computational requirements and real-time constraints.

### Digital Twin Workstation

The workstation approach leverages powerful computing resources for high-fidelity simulation:

**Advantages:**
- High-fidelity physics simulation
- Complex sensor modeling
- Advanced rendering capabilities
- Extensive computational resources
- Support for multiple simultaneous simulations

**Disadvantages:**
- Higher cost
- Network latency concerns
- Less portable
- Power consumption

### Edge Kit (Jetson Orin Nano)

Edge computing brings digital twin capabilities closer to the physical robot:

**Advantages:**
- Reduced latency
- Lower bandwidth requirements
- Real-time processing
- Portability
- Cost-effective for deployment

**Disadvantages:**
- Limited computational resources
- Reduced simulation fidelity
- Thermal constraints
- Memory limitations

### Architecture Comparison

```python
class DigitalTwinArchitecture:
    def __init__(self, architecture_type):
        self.type = architecture_type
        self.computational_power = self._get_computational_power()
        self.latency = self._get_latency()
        self.power_consumption = self._get_power_consumption()

    def _get_computational_power(self):
        if self.type == "workstation":
            return "High (RTX 4070+)"
        elif self.type == "edge":
            return "Moderate (Jetson Orin Nano)"

    def _get_latency(self):
        if self.type == "workstation":
            return "Medium to High (network dependent)"
        elif self.type == "edge":
            return "Low (local processing)"

    def _get_power_consumption(self):
        if self.type == "workstation":
            return "High (500W+)"
        elif self.type == "edge":
            return "Low (15-25W)"
```

## 4.2 Jetson Orin Nano for Edge Deployment

### Hardware Specifications

The Jetson Orin Nano is designed for edge AI applications:

- **GPU**: 1024-core NVIDIA Ampere architecture GPU
- **CPU**: 6-core ARM Cortex-A78AE v8.2 64-bit CPU
- **Memory**: 4GB or 8GB LPDDR5
- **Power**: 15W to 25W TDP
- **Connectivity**: Gigabit Ethernet, M.2 Key M slot for NVMe storage

### Setting up Jetson for Robotics

```bash
# Update system
sudo apt update && sudo apt upgrade

# Install ROS 2 Humble for Jetson
sudo apt install ros-humble-ros-base
sudo apt install ros-humble-gazebo-ros-pkgs

# Install NVIDIA libraries
sudo apt install nvidia-jetpack

# Install robotics libraries
pip3 install numpy scipy matplotlib
```

### Optimized Simulation for Edge

```python
import numpy as np
import time

class EdgeOptimizedSimulator:
    """Lightweight simulator optimized for Jetson Orin Nano"""

    def __init__(self):
        self.dt = 0.01  # Larger time step for performance
        self.max_joints = 6  # Limit complexity
        self.use_approximate_physics = True

    def forward_kinematics(self, joint_angles):
        """Optimized forward kinematics for edge computing"""
        # Simplified kinematics calculation
        positions = []
        cumulative_angle = 0

        for i, (angle, length) in enumerate(zip(joint_angles, self.link_lengths)):
            cumulative_angle += angle
            x = sum([l * np.cos(sum(joint_angles[:j+1])) for j, l in enumerate(self.link_lengths[:i+1])])
            y = sum([l * np.sin(sum(joint_angles[:j+1])) for j, l in enumerate(self.link_lengths[:i+1])])
            positions.append([x, y])

        return positions

    def run_simulation_step(self):
        """Run a single simulation step optimized for edge"""
        start_time = time.time()

        # Perform simplified physics calculations
        self.update_positions()
        self.check_collisions_approximate()

        # Ensure we don't exceed timing constraints
        elapsed = time.time() - start_time
        if elapsed < self.dt:
            time.sleep(self.dt - elapsed)

    def update_positions(self):
        """Fast position update"""
        # Simplified position update logic
        pass

    def check_collisions_approximate(self):
        """Fast approximate collision detection"""
        # Use bounding boxes instead of detailed meshes
        pass
```

## 4.3 Unity for Hardware Simulation

Unity provides high-fidelity graphics and physics simulation capabilities for robotics:

### NVIDIA Isaac Unity Integration

NVIDIA Isaac Unity combines Unity's rendering capabilities with robotics simulation:

- **Photorealistic Rendering**: High-quality visual simulation
- **Physics Engine**: PhysX for realistic physics
- **ROS 2 Bridge**: Seamless integration with ROS 2
- **AI Training Environment**: Domain randomization support

### Unity Robotics Setup

```csharp
// Unity C# script for ROS 2 communication
using UnityEngine;
using Ros2UnityEx;
using std_msgs;

public class RobotController : MonoBehaviour
{
    private ROS2UnityEnvironment ros2Unity;
    private Publisher<std_msgs.msg.Float64MultiArray> jointPublisher;
    private Subscriber<std_msgs.msg.JointState> jointSubscriber;

    void Start()
    {
        ros2Unity = new ROS2UnityEnvironment();
        ros2Unity.Init();

        // Initialize publishers and subscribers
        jointPublisher = ros2Unity.CreatePublisher<std_msgs.msg.Float64MultiArray>("/joint_commands");
        jointSubscriber = ros2Unity.CreateSubscriber<std_msgs.msg.JointState>("/joint_states", JointStateCallback);
    }

    void JointStateCallback(JointState msg)
    {
        // Update robot model based on joint states
        UpdateRobotJoints(msg.position);
    }

    void UpdateRobotJoints(float[] positions)
    {
        // Apply positions to Unity robot model
        for (int i = 0; i < positions.Length && i < jointTransforms.Length; i++)
        {
            jointTransforms[i].localRotation = Quaternion.Euler(0, positions[i] * Mathf.Rad2Deg, 0);
        }
    }

    void OnDestroy()
    {
        ros2Unity.Dispose();
    }
}
```

## 4.4 Sensor Simulation in Digital Twins

### Realistic Sensor Modeling

Sensor simulation must account for real-world imperfections:

```python
import numpy as np

class SensorSimulator:
    """Realistic sensor simulation with noise models"""

    def __init__(self):
        self.camera_noise_params = {
            'gaussian_std': 0.005,
            'poisson_lambda': 0.01,
            'uniform_range': 0.001
        }

        self.lidar_noise_params = {
            'range_std': 0.01,  # 1cm standard deviation
            'angular_std': 0.001,  # 0.057 degrees
            'dropout_rate': 0.001  # 0.1% dropout
        }

    def simulate_camera(self, ideal_image):
        """Simulate realistic camera sensor with noise"""
        # Add Gaussian noise
        gaussian_noise = np.random.normal(0, self.camera_noise_params['gaussian_std'], ideal_image.shape)

        # Add Poisson noise (photon noise)
        poisson_noise = np.random.poisson(self.camera_noise_params['poisson_lambda'], ideal_image.shape)

        # Add uniform noise
        uniform_noise = np.random.uniform(-self.camera_noise_params['uniform_range'],
                                        self.camera_noise_params['uniform_range'],
                                        ideal_image.shape)

        noisy_image = ideal_image + gaussian_noise + poisson_noise + uniform_noise

        # Ensure values are in valid range
        noisy_image = np.clip(noisy_image, 0, 255)

        return noisy_image.astype(np.uint8)

    def simulate_lidar(self, ideal_ranges, angles):
        """Simulate realistic LiDAR sensor with noise"""
        # Add range noise
        range_noise = np.random.normal(0, self.lidar_noise_params['range_std'], len(ideal_ranges))
        noisy_ranges = ideal_ranges + range_noise

        # Simulate dropouts
        dropout_mask = np.random.random(len(noisy_ranges)) < self.lidar_noise_params['dropout_rate']
        noisy_ranges[dropout_mask] = float('inf')  # Represent as infinite range

        return noisy_ranges

    def simulate_imu(self, true_acceleration, true_angular_velocity):
        """Simulate realistic IMU with bias and noise"""
        # Accelerometer bias and noise
        accel_bias = np.random.normal(0, 0.01, 3)  # 0.01 m/s² bias
        accel_noise = np.random.normal(0, 0.02, 3)  # 0.02 m/s² noise

        # Gyroscope bias and noise
        gyro_bias = np.random.normal(0, 0.001, 3)  # 0.001 rad/s bias
        gyro_noise = np.random.normal(0, 0.002, 3)  # 0.002 rad/s noise

        measured_accel = true_acceleration + accel_bias + accel_noise
        measured_gyro = true_angular_velocity + gyro_bias + gyro_noise

        return measured_accel, measured_gyro
```

## 4.5 VSLAM (Visual Simultaneous Localization and Mapping)

### VSLAM in Digital Twins

Visual SLAM is crucial for autonomous navigation and environment understanding:

```python
import cv2
import numpy as np
from collections import deque

class VSLAMSimulator:
    """Visual SLAM simulation in digital twin environment"""

    def __init__(self):
        self.feature_detector = cv2.SIFT_create()
        self.matcher = cv2.BFMatcher()
        self.camera_matrix = None
        self.r_t_history = deque(maxlen=100)  # Store pose history
        self.map_points = {}  # Store 3D map points

    def process_frame(self, current_image, previous_image, camera_pose):
        """Process current frame for SLAM"""
        # Detect features in current frame
        kp_curr, desc_curr = self.feature_detector.detectAndCompute(current_image, None)
        kp_prev, desc_prev = self.feature_detector.detectAndCompute(previous_image, None)

        if desc_curr is not None and desc_prev is not None:
            # Match features between frames
            matches = self.matcher.knnMatch(desc_prev, desc_curr, k=2)

            # Apply Lowe's ratio test
            good_matches = []
            for match_pair in matches:
                if len(match_pair) == 2:
                    m, n = match_pair
                    if m.distance < 0.7 * n.distance:
                        good_matches.append(m)

            if len(good_matches) >= 10:  # Need minimum matches for pose estimation
                # Extract matched keypoints
                src_pts = np.float32([kp_prev[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                dst_pts = np.float32([kp_curr[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

                # Estimate essential matrix and decompose to get rotation and translation
                E, mask = cv2.findEssentialMat(src_pts, dst_pts, self.camera_matrix)
                _, R, t, _ = cv2.recoverPose(E, src_pts, dst_pts, self.camera_matrix)

                # Update camera pose
                new_pose = self.update_camera_pose(camera_pose, R, t)
                return new_pose, good_matches

        return camera_pose, []

    def update_camera_pose(self, current_pose, rotation, translation):
        """Update camera pose based on relative motion"""
        # Convert to homogeneous transformation
        T_rel = np.eye(4)
        T_rel[:3, :3] = rotation
        T_rel[:3, 3] = translation.flatten()

        # Apply transformation to current pose
        T_current = current_pose
        T_new = T_current @ T_rel

        return T_new
```

## 4.6 "Sim-to-Real" Transfer Challenges

### The Reality Gap Problem

The "Sim-to-Real" gap represents the differences between simulation and real-world performance:

**Physical Differences:**
- Exact robot dynamics modeling
- Surface friction variations
- Actuator response times
- Sensor noise characteristics

**Environmental Differences:**
- Lighting conditions
- Surface textures
- Temperature variations
- External disturbances

### Domain Randomization

Domain randomization helps bridge the reality gap:

```python
import random

class DomainRandomizer:
    """Randomize simulation parameters for domain randomization"""

    def __init__(self):
        self.param_ranges = {
            'friction': (0.1, 0.8),
            'mass_multiplier': (0.8, 1.2),
            'inertia_multiplier': (0.8, 1.2),
            'gravity_multiplier': (0.9, 1.1),
            'camera_noise_std': (0.001, 0.01),
            'lidar_noise_std': (0.005, 0.02)
        }

    def randomize_environment(self):
        """Randomize environment parameters"""
        randomized_params = {}
        for param, (min_val, max_val) in self.param_ranges.items():
            randomized_params[param] = random.uniform(min_val, max_val)
        return randomized_params

    def apply_randomization(self, simulation_env, params):
        """Apply randomization to simulation"""
        # Apply friction randomization
        simulation_env.set_friction(params['friction'])

        # Apply mass randomization
        simulation_env.scale_mass(params['mass_multiplier'])

        # Apply sensor noise randomization
        simulation_env.set_camera_noise_std(params['camera_noise_std'])
        simulation_env.set_lidar_noise_std(params['lidar_noise_std'])
```

## 4.7 Edge vs. Cloud Architecture Patterns

### Hybrid Architecture

A hybrid approach combines the benefits of both edge and cloud computing:

```python
class HybridDigitalTwin:
    """Hybrid digital twin architecture combining edge and cloud"""

    def __init__(self):
        self.edge_twin = EdgeDigitalTwin()
        self.cloud_twin = CloudDigitalTwin()
        self.sync_threshold = 0.1  # Sync when error exceeds threshold

    def update_local_prediction(self, sensor_data):
        """Update edge twin with sensor data"""
        local_state = self.edge_twin.predict_next_state(sensor_data)
        return local_state

    def request_cloud_validation(self, local_state, sensor_data):
        """Request cloud twin to validate local prediction"""
        cloud_prediction = self.cloud_twin.validate_prediction(local_state, sensor_data)

        # Calculate discrepancy
        discrepancy = self.calculate_discrepancy(local_state, cloud_prediction)

        if discrepancy > self.sync_threshold:
            # Update edge twin with cloud prediction
            self.edge_twin.update_state(cloud_prediction)
            return cloud_prediction

        return local_state

    def calculate_discrepancy(self, local_state, cloud_state):
        """Calculate discrepancy between local and cloud states"""
        # Calculate state difference
        diff = np.linalg.norm(local_state - cloud_state)
        return diff
```

## 4.8 Practical Implementation Considerations

### Performance Optimization for Edge

```python
class OptimizedEdgeTwin:
    """Optimized digital twin for edge deployment"""

    def __init__(self):
        self.use_approximate_models = True
        self.reduced_visual_fidelity = True
        self.lower_physics_accuracy = True
        self.optimized_meshes = True

    def optimize_for_edge(self, original_model):
        """Optimize robot model for edge deployment"""
        # Simplify collision meshes
        simplified_collision_mesh = self.simplify_mesh(original_model.collision_mesh)

        # Reduce visual mesh complexity
        simplified_visual_mesh = self.reduce_mesh_complexity(original_model.visual_mesh)

        # Use approximate physics models
        approximate_physics = self.create_approximate_physics(original_model.physics_params)

        optimized_model = {
            'collision_mesh': simplified_collision_mesh,
            'visual_mesh': simplified_visual_mesh,
            'physics': approximate_physics,
            'sensors': self.optimize_sensors(original_model.sensors)
        }

        return optimized_model

    def simplify_mesh(self, mesh):
        """Simplify mesh for better performance"""
        # Implementation for mesh simplification
        pass

    def optimize_sensors(self, sensor_config):
        """Optimize sensor configuration for edge"""
        optimized_config = sensor_config.copy()
        # Reduce update rates, simplify models
        return optimized_config
```

## 4.9 Jetson Orin Nano Specific Optimizations

### CUDA Optimization for Jetson

```python
import cupy as cp  # Use CuPy for GPU acceleration on Jetson

class JetsonOptimizedSimulator:
    """Simulator optimized for Jetson Orin Nano GPU"""

    def __init__(self):
        # Initialize GPU arrays
        self.joint_positions_gpu = cp.zeros(6, dtype=cp.float32)
        self.link_lengths_gpu = cp.array([0.5, 0.4, 0.3], dtype=cp.float32)
        self.transform_matrices_gpu = cp.zeros((4, 4, 6), dtype=cp.float32)

    def forward_kinematics_gpu(self, joint_angles):
        """GPU-accelerated forward kinematics"""
        # Copy to GPU
        joint_angles_gpu = cp.array(joint_angles, dtype=cp.float32)

        # Perform calculations on GPU
        positions_gpu = self._calculate_positions_gpu(joint_angles_gpu, self.link_lengths_gpu)

        # Copy result back to CPU
        positions = cp.asnumpy(positions_gpu)
        return positions

    def _calculate_positions_gpu(self, joint_angles, link_lengths):
        """Internal GPU calculation function"""
        # GPU implementation of forward kinematics
        pass
```

## Summary

Digital twins represent a critical component of Physical AI systems, bridging the gap between simulation and reality. The choice between workstation and edge architectures depends on specific application requirements, computational constraints, and real-time performance needs. Understanding "Sim-to-Real" transfer challenges and implementing appropriate domain randomization techniques is essential for deploying robust robotic systems.

## Next Steps

In the next module, we'll explore ROS 2 fundamentals and how digital twin architectures integrate with robotic middleware systems.

<div class="alert alert-warning">
  <h5>Hardware Requirement</h5>
  <div><strong>Requirement:</strong> GPU</div>
  <div><strong>Minimum:</strong> RTX 4070 or Jetson Orin Nano</div>
  <div><strong>Recommended:</strong> RTX 4080 or Jetson Orin Nano (higher power mode)</div>
  <div><strong>Purpose:</strong> Digital twin simulation requires significant computational resources, especially for real-time performance</div>
</div>