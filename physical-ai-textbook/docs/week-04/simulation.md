---
title: "Week 4: Digital Twins and Edge Computing Simulation"
week: 4
module: "Foundations of Physical AI"
difficulty: "intermediate"
prerequisites: ["physics-simulation", "urdf", "ros2-basics", "sensor-integration"]
learning_objectives:
  - "Simulate digital twin architectures for workstation and edge"
  - "Implement realistic sensor models for digital twins"
  - "Validate 'Sim-to-Real' transfer in simulation environments"
  - "Optimize simulation for resource-constrained devices"
tags: ["digital-twins", "edge-computing", "jetson", "simulation", "optimization"]
hardware_requirements:
  - gpu: "RTX 4070 or Jetson Orin Nano"
  - ram: "16GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "120 minutes"
---

# Week 4: Digital Twins and Edge Computing Simulation

## Learning Objectives
- Simulate digital twin architectures for workstation and edge
- Implement realistic sensor models for digital twins
- Validate 'Sim-to-Real' transfer in simulation environments
- Optimize simulation for resource-constrained devices

## 4.1 Digital Twin Architecture Simulation

### Workstation vs. Edge Architecture Simulation

Digital twin architectures require different simulation approaches based on the target platform. Let's explore how to simulate both workstation and edge environments:

```python
import numpy as np
import time
from abc import ABC, abstractmethod
from enum import Enum

class PlatformType(Enum):
    WORKSTATION = "workstation"
    EDGE = "edge"

class DigitalTwinSimulator(ABC):
    """Abstract base class for digital twin simulation"""

    def __init__(self, platform_type, max_joints=6):
        self.platform_type = platform_type
        self.max_joints = max_joints
        self.joint_positions = np.zeros(max_joints)
        self.joint_velocities = np.zeros(max_joints)
        self.last_update_time = time.time()

        # Platform-specific parameters
        if platform_type == PlatformType.WORKSTATION:
            self.simulation_accuracy = "high"
            self.update_rate = 1000  # 1000 Hz
            self.physics_fidelity = "full"
        else:  # EDGE
            self.simulation_accuracy = "medium"
            self.update_rate = 100   # 100 Hz
            self.physics_fidelity = "simplified"

    @abstractmethod
    def update_physics(self, dt):
        """Update physics simulation based on platform"""
        pass

    def simulate_step(self):
        """Main simulation step"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        # Apply control inputs
        self.apply_control_inputs(dt)

        # Update physics
        self.update_physics(dt)

        # Update sensors
        sensor_data = self.update_sensors()

        return sensor_data

    def apply_control_inputs(self, dt):
        """Apply control inputs to joints"""
        # In a real implementation, this would receive commands from control system
        # For simulation, we'll apply a simple PD control
        target_positions = np.zeros(self.max_joints)  # Default target
        kp = 10.0 if self.simulation_accuracy == "high" else 5.0
        kd = 1.0 if self.simulation_accuracy == "high" else 0.5

        position_error = target_positions - self.joint_positions
        control_effort = kp * position_error - kd * self.joint_velocities

        # Simple integration
        self.joint_velocities += control_effort * dt
        self.joint_positions += self.joint_velocities * dt

    def update_sensors(self):
        """Update sensor readings with platform-specific noise"""
        sensor_data = {}

        # Joint position sensors
        sensor_data['joint_positions'] = self.joint_positions + self._add_sensor_noise(
            self.joint_positions, self._get_position_noise_level()
        )

        # IMU simulation
        sensor_data['imu'] = self._simulate_imu()

        # Camera simulation
        sensor_data['camera'] = self._simulate_camera()

        return sensor_data

    def _add_sensor_noise(self, signal, noise_std):
        """Add platform-specific sensor noise"""
        return np.random.normal(0, noise_std, signal.shape)

    def _get_position_noise_level(self):
        """Get noise level based on platform"""
        return 0.001 if self.simulation_accuracy == "high" else 0.01

    def _simulate_imu(self):
        """Simulate IMU readings"""
        # True values
        true_acceleration = np.array([0, 0, -9.81])  # Gravity
        true_angular_velocity = np.zeros(3)  # No rotation initially

        # Add noise based on platform
        noise_level = 0.01 if self.simulation_accuracy == "high" else 0.05
        noisy_accel = true_acceleration + np.random.normal(0, noise_level, 3)
        noisy_gyro = true_angular_velocity + np.random.normal(0, noise_level/10, 3)

        return {
            'acceleration': noisy_accel,
            'angular_velocity': noisy_gyro
        }

    def _simulate_camera(self):
        """Simulate camera data"""
        # Camera parameters vary by platform
        if self.simulation_accuracy == "high":
            resolution = (1920, 1080)
            noise_std = 0.005
        else:
            resolution = (640, 480)
            noise_std = 0.02

        # Generate synthetic image (in real implementation, this would come from renderer)
        image = np.random.rand(resolution[1], resolution[0], 3) * 255
        noisy_image = image + np.random.normal(0, noise_std * 255, image.shape)
        return {
            'image': np.clip(noisy_image, 0, 255).astype(np.uint8),
            'resolution': resolution
        }

class WorkstationDigitalTwin(DigitalTwinSimulator):
    """High-fidelity digital twin for workstation"""

    def __init__(self):
        super().__init__(PlatformType.WORKSTATION)

    def update_physics(self, dt):
        """High-fidelity physics simulation"""
        # Complex physics calculations here
        # Full rigid body dynamics, detailed collision detection, etc.
        # This would use a sophisticated physics engine in practice

        # For simulation purposes, we'll use a more detailed model
        # with better numerical integration
        self._detailed_physics_integration(dt)

    def _detailed_physics_integration(self, dt):
        """Use Runge-Kutta 4th order integration for better accuracy"""
        # Simplified RK4 implementation
        # In practice, this would integrate full equations of motion
        k1 = self._compute_derivatives(self.joint_positions, self.joint_velocities)
        k2 = self._compute_derivatives(
            self.joint_positions + 0.5 * dt * k1[0],
            self.joint_velocities + 0.5 * dt * k1[1]
        )
        k3 = self._compute_derivatives(
            self.joint_positions + 0.5 * dt * k2[0],
            self.joint_velocities + 0.5 * dt * k2[1]
        )
        k4 = self._compute_derivatives(
            self.joint_positions + dt * k3[0],
            self.joint_velocities + dt * k3[1]
        )

        avg_deriv = (k1 + 2*k2 + 2*k3 + k4) / 6
        self.joint_positions += dt * avg_deriv[0]
        self.joint_velocities += dt * avg_deriv[1]

    def _compute_derivatives(self, positions, velocities):
        """Compute time derivatives for integration"""
        # In a real implementation, this would compute from physics equations
        # For now, return simple derivatives
        accelerations = np.zeros_like(velocities)  # Simplified
        return velocities, accelerations

class EdgeDigitalTwin(DigitalTwinSimulator):
    """Optimized digital twin for edge devices"""

    def __init__(self):
        super().__init__(PlatformType.EDGE)

    def update_physics(self, dt):
        """Simplified physics simulation optimized for edge"""
        # Use Euler integration instead of RK4
        # Simplified collision detection
        # Reduced number of physics calculations

        # Simple Euler integration
        accelerations = self._compute_simple_derivatives()
        self.joint_velocities += accelerations * dt
        self.joint_positions += self.joint_velocities * dt

    def _compute_simple_derivatives(self):
        """Simplified derivative computation for performance"""
        # In real implementation, this would still compute physics
        # but with simplified models and fewer calculations
        return np.zeros(self.max_joints)  # Simplified
```

## 4.2 Jetson Orin Nano Simulation Environment

Let's create a simulation that models the Jetson Orin Nano's constraints:

```python
import threading
import psutil
from collections import deque

class JetsonSimulator:
    """Simulate Jetson Orin Nano environment and constraints"""

    def __init__(self):
        self.max_power_draw = 25  # Watts
        self.current_power_draw = 0
        self.cpu_usage_history = deque(maxlen=100)
        self.gpu_usage_history = deque(maxlen=100)
        self.memory_usage_history = deque(maxlen=100)

        # Jetson-specific parameters
        self.gpu_compute_capability = "7.5"  # NVIDIA compute capability
        self.cpu_cores = 6  # ARM Cortex-A78AE
        self.memory_size = 8 * 1024 * 1024 * 1024  # 8GB in bytes

        # Thermal management
        self.temperature = 25  # Starting temperature in Celsius
        self.max_temperature = 95  # Maximum safe temperature

        # Performance scaling
        self.performance_mode = "default"
        self.cpu_frequency = 2200  # MHz
        self.gpu_frequency = 1300  # MHz

    def simulate_resource_usage(self, computational_load):
        """Simulate resource usage based on computational load"""
        # Calculate power draw based on load
        self.current_power_draw = min(self.max_power_draw,
                                    5 + computational_load * 20)  # 5W base + load

        # Update temperature based on power draw
        self.temperature += (self.current_power_draw / 50)  # Simplified heating model

        # Calculate CPU usage
        cpu_percent = min(100, computational_load * 80 + 20)  # Base 20% + load
        self.cpu_usage_history.append(cpu_percent)

        # Calculate GPU usage (simplified)
        gpu_percent = min(100, computational_load * 90)
        self.gpu_usage_history.append(gpu_percent)

        # Calculate memory usage
        memory_percent = min(95, computational_load * 60 + 30)
        self.memory_usage_history.append(memory_percent)

        # Thermal throttling simulation
        if self.temperature > 85:
            self.performance_mode = "throttled"
            self.cpu_frequency *= 0.8  # Reduce frequency by 20%
            self.gpu_frequency *= 0.8
        elif self.temperature < 60:
            self.performance_mode = "default"
            self.cpu_frequency *= 1.0  # Return to normal
            self.gpu_frequency *= 1.0

        return {
            'cpu_usage': cpu_percent,
            'gpu_usage': gpu_percent,
            'memory_usage': memory_percent,
            'temperature': self.temperature,
            'power_draw': self.current_power_draw,
            'performance_mode': self.performance_mode
        }

    def optimize_for_jetson(self, simulation_params):
        """Optimize simulation parameters for Jetson constraints"""
        optimized_params = simulation_params.copy()

        # Reduce complexity based on resource constraints
        if self.temperature > 75 or self.cpu_usage_history and max(self.cpu_usage_history) > 85:
            optimized_params['physics_fidelity'] = 'low'
            optimized_params['sensor_resolution'] = 'low'
            optimized_params['update_rate'] = max(50, simulation_params['update_rate'] * 0.5)
        else:
            optimized_params['physics_fidelity'] = simulation_params.get('physics_fidelity', 'medium')
            optimized_params['update_rate'] = simulation_params.get('update_rate', 100)

        return optimized_params

    def run_power_efficient_simulation(self, simulation_function, *args, **kwargs):
        """Run simulation with power and thermal constraints"""
        def constrained_simulation():
            while True:
                # Monitor current resource usage
                resources = self.simulate_resource_usage(0.5)  # Default load of 0.5

                # Adjust simulation load based on resource availability
                if resources['temperature'] > 80 or resources['cpu_usage'] > 90:
                    print("Thermal or CPU throttling detected, reducing simulation complexity")
                    # Reduce simulation complexity temporarily
                    time.sleep(0.1)  # Yield to allow cooling
                else:
                    # Run simulation step
                    result = simulation_function(*args, **kwargs)
                    time.sleep(0.01)  # Simulate real-time processing

        # Run in separate thread to not block main process
        simulation_thread = threading.Thread(target=constrained_simulation, daemon=True)
        simulation_thread.start()

        return simulation_thread
```

## 4.3 Sensor Simulation with Platform-Specific Models

```python
import cv2
import numpy as np

class MultiPlatformSensorSimulator:
    """Simulate sensors optimized for different platforms"""

    def __init__(self, platform_type):
        self.platform_type = platform_type
        self.sensor_config = self._get_platform_sensor_config()

    def _get_platform_sensor_config(self):
        """Get sensor configuration based on platform"""
        if self.platform_type == PlatformType.WORKSTATION:
            return {
                'camera': {
                    'resolution': (1920, 1080),
                    'fov': 60,  # degrees
                    'noise_std': 0.005,
                    'update_rate': 30
                },
                'lidar': {
                    'range_resolution': 0.01,  # 1cm
                    'angular_resolution': 0.01,  # ~0.57 degrees
                    'noise_std': 0.005,
                    'update_rate': 10
                },
                'imu': {
                    'accel_noise_density': 0.01,  # mg/sqrt(Hz)
                    'gyro_noise_density': 0.001,  # deg/s/sqrt(Hz)
                    'update_rate': 100
                }
            }
        else:  # EDGE
            return {
                'camera': {
                    'resolution': (640, 480),
                    'fov': 60,
                    'noise_std': 0.02,
                    'update_rate': 15
                },
                'lidar': {
                    'range_resolution': 0.05,  # 5cm
                    'angular_resolution': 0.1,  # ~5.7 degrees
                    'noise_std': 0.02,
                    'update_rate': 5
                },
                'imu': {
                    'accel_noise_density': 0.05,
                    'gyro_noise_density': 0.005,
                    'update_rate': 50
                }
            }

    def simulate_camera(self, scene_data):
        """Simulate camera with platform-specific parameters"""
        config = self.sensor_config['camera']

        # Generate synthetic image based on scene
        image = self._generate_synthetic_image(scene_data, config['resolution'])

        # Add noise based on platform
        noise = np.random.normal(0, config['noise_std'] * 255, image.shape)
        noisy_image = np.clip(image.astype(np.float32) + noise, 0, 255).astype(np.uint8)

        return {
            'image': noisy_image,
            'timestamp': time.time(),
            'resolution': config['resolution'],
            'fov': config['fov']
        }

    def _generate_synthetic_image(self, scene_data, resolution):
        """Generate synthetic image for the given scene"""
        height, width = resolution[1], resolution[0]
        image = np.zeros((height, width, 3), dtype=np.uint8)

        # Add scene elements (simplified)
        # In real implementation, this would use a 3D renderer
        center_x, center_y = width // 2, height // 2
        cv2.circle(image, (center_x, center_y), 50, (255, 0, 0), -1)  # Blue circle
        cv2.rectangle(image, (100, 100), (200, 200), (0, 255, 0), 2)  # Green square

        return image

    def simulate_lidar(self, environment_map):
        """Simulate LiDAR with platform-specific parameters"""
        config = self.sensor_config['lidar']

        # Generate angle array
        angles = np.linspace(-np.pi, np.pi, int(2 * np.pi / config['angular_resolution']))

        # Simulate distance measurements
        ranges = self._simulate_lidar_ranges(environment_map, angles)

        # Add noise
        noise = np.random.normal(0, config['noise_std'], len(ranges))
        noisy_ranges = np.maximum(0.1, ranges + noise)  # Ensure minimum range

        return {
            'ranges': noisy_ranges,
            'angles': angles,
            'timestamp': time.time(),
            'range_resolution': config['range_resolution'],
            'angular_resolution': config['angular_resolution']
        }

    def _simulate_lidar_ranges(self, environment_map, angles):
        """Simulate LiDAR range measurements"""
        # Simplified LiDAR simulation
        # In real implementation, this would perform ray tracing
        ranges = np.full(len(angles), 10.0)  # Default max range

        # Add some obstacles
        obstacle_angles = [0, np.pi/4, np.pi/2, 3*np.pi/4, np.pi]
        obstacle_distances = [2.0, 1.5, 3.0, 2.5, 1.8]

        for obs_angle, obs_dist in zip(obstacle_angles, obstacle_distances):
            # Find closest angle
            angle_diffs = np.abs(angles - obs_angle)
            closest_idx = np.argmin(angle_diffs)
            ranges[closest_idx] = obs_dist

        return ranges

    def simulate_imu(self, true_motion):
        """Simulate IMU with platform-specific parameters"""
        config = self.sensor_config['imu']

        # Get true values from motion
        true_accel = true_motion.get('acceleration', np.array([0, 0, -9.81]))
        true_gyro = true_motion.get('angular_velocity', np.array([0, 0, 0]))

        # Add noise based on platform
        accel_noise = np.random.normal(0, config['accel_noise_density'], 3)
        gyro_noise = np.random.normal(0, config['gyro_noise_density'], 3)

        measured_accel = true_accel + accel_noise
        measured_gyro = true_gyro + gyro_noise

        return {
            'acceleration': measured_accel,
            'angular_velocity': measured_gyro,
            'timestamp': time.time()
        }
```

## 4.4 "Sim-to-Real" Transfer Validation Simulation

```python
import pickle
import matplotlib.pyplot as plt

class SimToRealValidationSimulator:
    """Simulate and validate Sim-to-Real transfer capabilities"""

    def __init__(self):
        self.sim_data_buffer = []
        self.real_data_buffer = []
        self.transfer_metrics = {}
        self.domain_gap_analysis = {}

    def collect_simulation_data(self, sim_twin_data):
        """Collect data from simulation"""
        self.sim_data_buffer.append({
            'timestamp': time.time(),
            'joint_positions': sim_twin_data['joint_positions'],
            'sensor_readings': sim_twin_data['sensor_data'],
            'control_commands': sim_twin_data['control_commands']
        })

    def collect_real_data(self, real_robot_data):
        """Collect data from real robot"""
        self.real_data_buffer.append({
            'timestamp': time.time(),
            'joint_positions': real_robot_data['joint_positions'],
            'sensor_readings': real_robot_data['sensor_data'],
            'actual_commands': real_robot_data['control_commands']
        })

    def validate_transfer(self):
        """Validate Sim-to-Real transfer quality"""
        if len(self.sim_data_buffer) < 10 or len(self.real_data_buffer) < 10:
            return {'status': 'insufficient_data', 'score': 0.0}

        # Align data by timestamp (simplified alignment)
        aligned_sim, aligned_real = self._align_data()

        if not aligned_sim or not aligned_real:
            return {'status': 'alignment_failed', 'score': 0.0}

        # Calculate validation metrics
        metrics = self._calculate_validation_metrics(aligned_sim, aligned_real)

        # Analyze domain gap
        domain_gap = self._analyze_domain_gap(aligned_sim, aligned_real)

        self.transfer_metrics = metrics
        self.domain_gap_analysis = domain_gap

        # Calculate overall transfer score
        transfer_score = self._calculate_transfer_score(metrics)

        return {
            'status': 'completed',
            'score': transfer_score,
            'metrics': metrics,
            'domain_gap': domain_gap
        }

    def _align_data(self):
        """Align simulation and real data by timestamps"""
        # Simple temporal alignment
        # In real implementation, this would be more sophisticated
        min_len = min(len(self.sim_data_buffer), len(self.real_data_buffer))

        sim_aligned = self.sim_data_buffer[-min_len:]
        real_aligned = self.real_data_buffer[-min_len:]

        return sim_aligned, real_aligned

    def _calculate_validation_metrics(self, sim_data, real_data):
        """Calculate various validation metrics"""
        if not sim_data or not real_data:
            return {}

        # Joint position errors
        sim_positions = np.array([d['joint_positions'] for d in sim_data])
        real_positions = np.array([d['joint_positions'] for d in real_data])

        # Ensure same dimensions
        min_joints = min(sim_positions.shape[1], real_positions.shape[1])
        sim_positions = sim_positions[:, :min_joints]
        real_positions = real_positions[:, :min_joints]

        position_errors = np.abs(sim_positions - real_positions)

        metrics = {
            'joint_position_rmse': np.sqrt(np.mean(position_errors**2)),
            'joint_position_mean_error': np.mean(position_errors),
            'joint_position_max_error': np.max(position_errors),
            'joint_position_std_error': np.std(position_errors),
            'correlation_coefficient': np.corrcoef(
                sim_positions.flatten(), real_positions.flatten()
            )[0, 1]
        }

        # Add sensor validation metrics if available
        if sim_data[0]['sensor_readings'] and real_data[0]['sensor_readings']:
            sensor_sim = sim_data[0]['sensor_readings']
            sensor_real = real_data[0]['sensor_readings']

            if 'imu' in sensor_sim and 'imu' in sensor_real:
                sim_imu = np.array(sensor_sim['imu']['acceleration'])
                real_imu = np.array(sensor_real['imu']['acceleration'])

                imu_error = np.mean(np.abs(sim_imu - real_imu))
                metrics['imu_error'] = imu_error

        return metrics

    def _analyze_domain_gap(self, sim_data, real_data):
        """Analyze domain gap between simulation and reality"""
        analysis = {
            'data_distribution_similarity': self._compare_distributions(sim_data, real_data),
            'feature_space_alignment': self._analyze_feature_alignment(sim_data, real_data),
            'dynamics_difference': self._analyze_dynamics_difference(sim_data, real_data)
        }

        return analysis

    def _compare_distributions(self, sim_data, real_data):
        """Compare data distributions between sim and real"""
        # Calculate statistical moments
        sim_pos = np.array([d['joint_positions'] for d in sim_data])
        real_pos = np.array([d['joint_positions'] for d in real_data])

        # Ensure same dimensions
        min_joints = min(sim_pos.shape[1], real_pos.shape[1])
        sim_pos = sim_pos[:, :min_joints]
        real_pos = real_pos[:, :min_joints]

        # Calculate distribution similarity metrics
        sim_mean = np.mean(sim_pos, axis=0)
        real_mean = np.mean(real_pos, axis=0)

        sim_std = np.std(sim_pos, axis=0)
        real_std = np.std(real_pos, axis=0)

        # Simple distance between means and stds
        mean_distance = np.mean(np.abs(sim_mean - real_mean))
        std_distance = np.mean(np.abs(sim_std - real_std))

        return {
            'mean_distance': mean_distance,
            'std_distance': std_distance,
            'overall_similarity': 1.0 / (1.0 + mean_distance + std_distance)
        }

    def _analyze_dynamics_difference(self, sim_data, real_data):
        """Analyze differences in system dynamics"""
        # Calculate velocity and acceleration differences
        if len(sim_data) < 2:
            return {'status': 'insufficient_data'}

        # Calculate velocities
        sim_positions = np.array([d['joint_positions'] for d in sim_data])
        real_positions = np.array([d['joint_positions'] for d in real_data])

        min_joints = min(sim_positions.shape[1], real_positions.shape[1])
        sim_positions = sim_positions[:, :min_joints]
        real_positions = real_positions[:, :min_joints]

        sim_velocities = np.diff(sim_positions, axis=0)
        real_velocities = np.diff(real_positions, axis=0)

        # Calculate acceleration
        sim_accelerations = np.diff(sim_velocities, axis=0)
        real_accelerations = np.diff(real_velocities, axis=0)

        velocity_error = np.mean(np.abs(sim_velocities - real_velocities))
        acceleration_error = np.mean(np.abs(sim_accelerations - real_accelerations))

        return {
            'velocity_error': velocity_error,
            'acceleration_error': acceleration_error,
            'dynamics_fidelity': 1.0 / (1.0 + velocity_error + acceleration_error)
        }

    def _calculate_transfer_score(self, metrics):
        """Calculate overall transfer quality score"""
        if not metrics:
            return 0.0

        # Weight different metrics
        position_score = max(0, min(1, 1.0 / (1.0 + metrics.get('joint_position_rmse', 1.0))))
        correlation_score = max(0, min(1, (metrics.get('correlation_coefficient', 0) + 1) / 2))
        imu_score = max(0, min(1, 1.0 / (1.0 + metrics.get('imu_error', 1.0) * 10)))

        # Weighted average
        total_score = (0.5 * position_score + 0.3 * correlation_score + 0.2 * imu_score)
        return total_score

    def apply_domain_randomization(self, base_params, randomization_strength=0.1):
        """Apply domain randomization to simulation parameters"""
        randomized_params = {}

        for key, value in base_params.items():
            if isinstance(value, (int, float)):
                # Apply randomization
                variation = randomization_strength * abs(value)
                randomized_value = value + np.random.uniform(-variation, variation)
                randomized_params[key] = randomized_value
            elif isinstance(value, (list, np.ndarray)):
                # Randomize array elements
                variation = randomization_strength * np.abs(value)
                randomized_array = value + np.random.uniform(-variation, variation, value.shape)
                randomized_params[key] = randomized_array
            else:
                # Keep non-numeric parameters as is
                randomized_params[key] = value

        return randomized_params

    def generate_validation_report(self, filename):
        """Generate comprehensive validation report"""
        report = {
            'timestamp': time.time(),
            'transfer_metrics': self.transfer_metrics,
            'domain_gap_analysis': self.domain_gap_analysis,
            'recommendations': self._generate_recommendations()
        }

        with open(filename, 'wb') as f:
            pickle.dump(report, f)

        return report

    def _generate_recommendations(self):
        """Generate recommendations based on validation results"""
        recommendations = []

        score = self.transfer_metrics.get('score', 0.0)

        if score > 0.8:
            recommendations.append("High transfer quality achieved - suitable for real deployment")
        elif score > 0.5:
            recommendations.append("Moderate transfer quality - consider additional calibration or domain randomization")
        else:
            recommendations.append("Low transfer quality - significant domain gap exists, requires major adjustments")

        if self.transfer_metrics.get('joint_position_rmse', 1.0) > 0.1:
            recommendations.append("High joint position errors detected - review kinematic models")

        if self.transfer_metrics.get('imu_error', 1.0) > 0.5:
            recommendations.append("High IMU discrepancies - check sensor noise models")

        return recommendations
```

## 4.5 Unity Integration Simulation

Let's simulate Unity-based digital twin integration:

```python
class UnityDigitalTwinSimulator:
    """Simulate Unity-based digital twin for hardware simulation"""

    def __init__(self):
        self.unity_connection = None
        self.ros_bridge_active = False
        self.render_quality = "high"
        self.physics_accuracy = "high"
        self.simulation_speed = 1.0  # Real-time by default

    def connect_to_unity(self, ip_address="localhost", port=5005):
        """Simulate connection to Unity environment"""
        # In real implementation, this would use Unity ROS TCP connector
        print(f"Connecting to Unity at {ip_address}:{port}")
        self.unity_connection = {"ip": ip_address, "port": port, "connected": True}
        self.ros_bridge_active = True
        return True

    def setup_photorealistic_environment(self):
        """Setup photorealistic environment in Unity"""
        environment_config = {
            'lighting': {
                'type': 'HDRP',
                'quality': 'ultra',
                'shadows': 'high',
                'reflection_probes': True
            },
            'materials': {
                'pbr_enabled': True,
                'textures': 'high_resolution',
                'normal_maps': True
            },
            'rendering': {
                'quality': 'ultra',
                'aa_method': 'TAA',
                'post_processing': True
            }
        }
        return environment_config

    def simulate_physics_in_unity(self, robot_urdf, environment):
        """Simulate physics in Unity with PhysX"""
        # In real implementation, this would convert URDF to Unity physics objects
        # and simulate using PhysX engine
        physics_config = {
            'engine': 'PhysX',
            'gravity': [0, -9.81, 0],
            'solver_iterations': 10,
            'contact_offset': 0.01,
            'sleep_threshold': 0.005
        }
        return physics_config

    def setup_sensor_simulation(self, sensors_config):
        """Setup sensor simulation in Unity"""
        # Configure Unity sensors to match real hardware
        unity_sensors = {}
        for sensor_name, config in sensors_config.items():
            if config['type'] == 'camera':
                unity_sensors[sensor_name] = {
                    'type': 'UnityCamera',
                    'resolution': config['resolution'],
                    'fov': config['fov'],
                    'render_pipeline': 'HDRP',
                    'post_processing': True
                }
            elif config['type'] == 'lidar':
                unity_sensors[sensor_name] = {
                    'type': 'RaycastLiDAR',
                    'rays': config.get('rays', 64),
                    'range': config.get('range', 10.0),
                    'update_rate': config.get('update_rate', 10)
                }

        return unity_sensors

    def run_unity_simulation(self, scenario_config):
        """Run simulation in Unity environment"""
        # This would integrate with Unity engine
        simulation_results = {
            'frames_rendered': 0,
            'physics_steps': 0,
            'sensor_data': [],
            'performance_metrics': {
                'fps': 0,
                'physics_time': 0,
                'render_time': 0
            }
        }

        # Simulate Unity loop
        for step in range(scenario_config.get('steps', 1000)):
            # Update physics
            self._update_unity_physics()

            # Update sensors
            sensor_data = self._update_unity_sensors()
            simulation_results['sensor_data'].append(sensor_data)

            # Update rendering
            self._update_unity_rendering()

            simulation_results['frames_rendered'] += 1
            simulation_results['physics_steps'] += 1

            # Slow down simulation to match real-time if needed
            if self.simulation_speed < 1.0:
                time.sleep(0.01 * (1.0 / self.simulation_speed))

        return simulation_results

    def _update_unity_physics(self):
        """Update Unity physics simulation"""
        # In real implementation, this would call Unity physics update
        pass

    def _update_unity_sensors(self):
        """Update Unity sensor simulation"""
        # Simulate sensor readings
        return {
            'camera': np.random.rand(1080, 1920, 3) * 255,
            'lidar': np.random.rand(1080) * 10.0,
            'imu': {'accel': [0, 0, -9.81], 'gyro': [0, 0, 0]}
        }

    def _update_unity_rendering(self):
        """Update Unity rendering"""
        # In real implementation, this would update Unity rendering
        pass
```

## 4.6 Performance Optimization Techniques

```python
class SimulationOptimizer:
    """Optimize simulation performance for different platforms"""

    def __init__(self, platform_type):
        self.platform_type = platform_type
        self.optimization_level = "balanced" if platform_type == PlatformType.WORKSTATION else "aggressive"

    def optimize_meshes(self, mesh_data, target_complexity):
        """Optimize 3D meshes for performance"""
        if self.optimization_level == "aggressive":
            # Heavy optimization for edge devices
            return self._simplify_mesh_aggressive(mesh_data, target_complexity * 0.3)
        else:
            # Moderate optimization for workstation
            return self._simplify_mesh_moderate(mesh_data, target_complexity * 0.7)

    def _simplify_mesh_aggressive(self, mesh, target_triangles):
        """Aggressively simplify mesh for edge performance"""
        # In real implementation, this would use mesh simplification algorithms
        # like Quadric Error Metrics or Edge Collapse
        print("Applying aggressive mesh simplification for edge device")
        return mesh  # Placeholder

    def _simplify_mesh_moderate(self, mesh, target_triangles):
        """Moderately simplify mesh for workstation"""
        print("Applying moderate mesh simplification for workstation")
        return mesh  # Placeholder

    def optimize_physics(self, physics_params):
        """Optimize physics parameters for platform"""
        if self.platform_type == PlatformType.EDGE:
            # Reduce solver iterations
            physics_params['solver_iterations'] = max(2, physics_params.get('solver_iterations', 10) // 2)
            # Increase contact offset for stability
            physics_params['contact_offset'] = min(0.05, physics_params.get('contact_offset', 0.01) * 2)
            # Reduce update rate
            physics_params['update_rate'] = max(50, physics_params.get('update_rate', 100) // 2)
        else:
            # Workstation can handle full parameters
            pass

        return physics_params

    def optimize_sensors(self, sensor_configs):
        """Optimize sensor configurations for platform"""
        optimized_configs = {}

        for sensor_name, config in sensor_configs.items():
            if self.platform_type == PlatformType.EDGE:
                # Reduce resolution and update rates
                if config.get('type') == 'camera':
                    # Reduce resolution for edge
                    if config.get('resolution', (640, 480))[0] > 640:
                        config['resolution'] = (640, 480)
                    # Reduce update rate
                    config['update_rate'] = min(15, config.get('update_rate', 30))
                elif config.get('type') == 'lidar':
                    # Reduce number of rays and update rate
                    config['rays'] = min(32, config.get('rays', 64))
                    config['update_rate'] = min(5, config.get('update_rate', 10))
            else:
                # Workstation can handle higher settings
                pass

            optimized_configs[sensor_name] = config

        return optimized_configs

    def dynamic_load_balancing(self, tasks, available_resources):
        """Dynamically balance computational load"""
        if self.platform_type == PlatformType.EDGE:
            # Prioritize critical tasks, defer less important ones
            critical_tasks = [t for t in tasks if t.get('priority', 'low') == 'high']
            normal_tasks = [t for t in tasks if t.get('priority', 'low') == 'normal']
            low_tasks = [t for t in tasks if t.get('priority', 'low') == 'low']

            # Schedule based on available resources
            scheduled_tasks = self._schedule_tasks_edge(critical_tasks + normal_tasks, available_resources)
        else:
            # Workstation can handle all tasks more easily
            scheduled_tasks = tasks

        return scheduled_tasks

    def _schedule_tasks_edge(self, tasks, resources):
        """Schedule tasks with edge device constraints"""
        # Implement scheduling algorithm that considers
        # power, thermal, and performance constraints
        scheduled = []
        current_load = 0
        max_load = resources.get('cpu_cores', 1) * 0.8  # Use 80% of capacity

        for task in tasks:
            task_load = task.get('complexity', 1.0)
            if current_load + task_load <= max_load:
                scheduled.append(task)
                current_load += task_load

        return scheduled
```

## 4.7 Practical Exercise

Create a complete simulation environment that:
1. Implements both workstation and edge digital twin architectures
2. Includes platform-optimized sensor simulation
3. Implements "Sim-to-Real" validation
4. Applies domain randomization techniques
5. Includes performance optimization strategies

```python
# Student exercise - Complete implementation
class CompleteSimulationEnvironment:
    """Student implementation of a complete simulation environment"""

    def __init__(self, platform_type):
        """Initialize the complete simulation environment"""
        # TODO: Implement dual-architecture support
        # TODO: Implement sensor optimization
        # TODO: Implement validation framework
        # TODO: Implement domain randomization
        # TODO: Implement optimization strategies
        pass

    def setup_platform_simulation(self):
        """Setup simulation for specified platform"""
        # TODO: Complete implementation
        pass

    def implement_sensor_models(self):
        """Implement platform-specific sensor models"""
        # TODO: Complete implementation
        pass

    def validate_transfer_quality(self):
        """Implement Sim-to-Real validation"""
        # TODO: Complete implementation
        pass

    def optimize_for_platform(self):
        """Optimize simulation for target platform"""
        # TODO: Complete implementation
        pass

print("Student Exercise: Implement a complete simulation environment")
print("Requirements:")
print("1. Dual-architecture support (workstation and edge)")
print("2. Platform-optimized sensor models")
print("3. Comprehensive Sim-to-Real validation")
print("4. Domain randomization implementation")
print("5. Performance optimization strategies")
```

## 4.8 "Sim-to-Real" Transfer Considerations

### Key Factors Affecting Transfer Quality

When implementing digital twin simulations for "Sim-to-Real" transfer, several key factors must be considered:

1. **Model Fidelity vs. Performance**: Balance detailed physics models with real-time performance requirements
2. **Sensor Noise Modeling**: Accurately model real sensor characteristics including noise, bias, and drift
3. **Environmental Conditions**: Account for lighting, surface properties, and other environmental factors
4. **Actuator Dynamics**: Model real actuator limitations, delays, and non-linearities
5. **Computational Constraints**: Optimize for target platform capabilities

### Best Practices for Successful Transfer

- Implement comprehensive domain randomization to improve generalization
- Use system identification to calibrate simulation parameters to match real hardware
- Apply progressive transfer learning techniques
- Validate continuously throughout the development process
- Design robust control algorithms that can handle model inaccuracies

## Summary

Simulation of digital twins for both workstation and edge platforms requires careful consideration of computational constraints, sensor modeling, and "Sim-to-Real" transfer capabilities. By implementing platform-specific optimizations and validation frameworks, we can create effective digital twins that bridge the gap between simulation and reality.

<div class="alert alert-warning">
  <h5>Hardware Requirement</h5>
  <div><strong>Requirement:</strong> GPU</div>
  <div><strong>Minimum:</strong> RTX 4070 or Jetson Orin Nano</div>
  <div><strong>Recommended:</strong> RTX 4080 or Jetson Orin Nano (higher power mode)</div>
  <div><strong>Purpose:</strong> Digital twin simulation requires significant computational resources, especially for real-time performance</div>
</div>