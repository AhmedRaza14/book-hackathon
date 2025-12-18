---
title: "Week 3: Physics Simulation and Digital Twins"
week: 3
module: "Foundations of Physical AI"
difficulty: "intermediate"
prerequisites: ["kinematics", "python-basics", "physics-basics"]
learning_objectives:
  - "Set up and configure simulation environments"
  - "Create URDF models of robots"
  - "Understand physics simulation principles"
tags: ["gazebo", "urdf", "simulation", "physics", "digital-twins"]
hardware_requirements:
  - gpu: "RTX 4070 or higher"
  - ram: "16GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "90 minutes"
---

# Week 3: Physics Simulation and Digital Twins

## Learning Objectives
- Set up and configure simulation environments
- Create URDF models of robots
- Understand physics simulation principles
- Implement sensor simulation in virtual environments

## 3.1 Introduction to Physics Simulation

Physics simulation is a cornerstone of Physical AI development, providing a safe, cost-effective environment to test algorithms before deployment on real hardware. It enables:

- **Algorithm Validation**: Test control and planning algorithms in a controlled environment
- **Data Generation**: Create training datasets for machine learning models
- **Safety Testing**: Validate safety-critical systems without risk to hardware or humans
- **Performance Optimization**: Fine-tune parameters before real-world implementation

### The Digital Twin Concept

A digital twin is a virtual representation of a physical system that mirrors its properties, states, and behaviors in real-time. In robotics, digital twins enable:

- **Virtual Testing**: Validate algorithms before hardware deployment
- **Predictive Maintenance**: Monitor and predict system behavior
- **Optimization**: Fine-tune parameters in simulation before real-world implementation
- **Training**: Develop and test AI models without hardware constraints

## 3.2 Simulation Environments Overview

### Gazebo vs. Other Platforms

**Gazebo** (now Ignition Gazebo):
- Open-source physics simulator
- Strong ROS integration
- Realistic physics engine (ODE, Bullet, Simbody)
- Extensive model database

**Unity**:
- Game engine adapted for robotics
- High-quality graphics
- Cross-platform support
- NVIDIA Isaac integration

**PyBullet**:
- Python-friendly physics engine
- Real-time simulation capabilities
- Good for reinforcement learning
- Cross-platform

## 3.3 URDF (Unified Robot Description Format)

URDF is an XML-based format for representing robot models. It describes the robot's physical and visual properties.

### URDF Structure

A typical URDF file contains:

```xml
<?xml version="1.0"?>
<robot name="example_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="1000.0" velocity="1.0"/>
  </joint>
</robot>
```

### Key URDF Components

1. **Links**: Represent rigid bodies with visual, collision, and inertial properties
2. **Joints**: Define connections between links with specific degrees of freedom
3. **Materials**: Define visual appearance
4. **Transmissions**: Define how actuators connect to joints

## 3.4 Physics Simulation Principles

### Dynamics and Kinematics

Physics simulation combines kinematic and dynamic calculations:

- **Kinematics**: Geometric relationships (position, velocity) without considering forces
- **Dynamics**: Motion under the influence of forces and torques

### Integration Methods

Simulation engines use numerical integration to solve equations of motion:

- **Euler Method**: Simple but less accurate
- **Runge-Kutta**: More accurate but computationally expensive
- **Verlet Integration**: Good for stability in physics simulations

### Collision Detection

Collision detection systems identify when objects intersect:

- **Broad Phase**: Quick elimination of non-colliding pairs
- **Narrow Phase**: Precise collision detection
- **Continuous Collision Detection**: Prevents objects from passing through each other at high speeds

## 3.5 Setting Up a Physics Simulation

### Installing Gazebo (Ignition)

```bash
# For Ubuntu 22.04
sudo apt update
sudo apt install ignition-harmonic
# Or for ROS 2 Humble
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
```

### Basic Gazebo Launch

```bash
# Launch Gazebo with empty world
ign gazebo -r empty.sdf
# Or with ROS 2
ros2 launch gazebo_ros empty_world.launch.py
```

## 3.6 Creating Complex Robot Models

### Multi-Link Robot URDF

```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.15"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Shoulder joint and link -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="1000" velocity="1"/>
  </joint>

  <link name="shoulder_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.2"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- Elbow joint and link -->
  <joint name="elbow_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="elbow_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="1000" velocity="1"/>
  </joint>

  <link name="elbow_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0005"/>
    </inertial>
  </link>
</robot>
```

## 3.7 Sensor Simulation

### Types of Sensors in Simulation

1. **Camera Sensors**: Visual perception
2. **LiDAR**: 3D mapping and obstacle detection
3. **IMU**: Inertial measurement
4. **Force/Torque Sensors**: Contact forces
5. **GPS**: Position estimation

### Adding Sensors to URDF

```xml
<!-- Camera sensor -->
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <always_on>1</always_on>
    <update_rate>30</update_rate>
    <visualize>true</visualize>
  </sensor>
</gazebo>

<!-- IMU sensor -->
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>1</always_on>
    <update_rate>100</update_rate>
    <visualize>false</visualize>
  </sensor>
</gazebo>
```

## 3.8 Physics Parameters and Tuning

### Gravity and Environmental Settings

Gravity can be modified in world files to simulate different environments:

```xml
<world name="custom_gravity">
  <gravity>0 0 -1.62</gravity>  <!-- Moon gravity -->
  <physics type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
  </physics>
</world>
```

### Material Properties

Material properties affect collision behavior:

```xml
<gazebo reference="link_name">
  <mu1>0.2</mu1>  <!-- Friction coefficient -->
  <mu2>0.2</mu2>  <!-- Friction coefficient (second direction) -->
  <kp>1000000.0</kp>  <!-- Spring stiffness -->
  <kd>1.0</kd>        <!-- Damping coefficient -->
</gazebo>
```

## 3.9 Python Implementation for Simulation Control

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

class SimulationController(Node):
    """Controller for simulation robot"""

    def __init__(self):
        super().__init__('simulation_controller')

        # Publisher for joint commands
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Timer for periodic control updates
        self.timer = self.create_timer(0.1, self.control_loop)

        # Joint names for our robot
        self.joint_names = ['shoulder_joint', 'elbow_joint']

        # Desired joint positions
        self.desired_positions = [0.0, 0.0]

        # Time counter for trajectory generation
        self.time_counter = 0.0

    def control_loop(self):
        """Main control loop"""
        # Generate a simple oscillating trajectory
        self.time_counter += 0.1

        # Create a smooth oscillating motion
        shoulder_pos = 0.5 * np.sin(self.time_counter * 0.5)
        elbow_pos = 0.3 * np.cos(self.time_counter * 0.3)

        # Create trajectory message
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [shoulder_pos, elbow_pos]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000  # 0.1 seconds

        traj_msg.points = [point]

        # Publish the trajectory
        self.joint_pub.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)

    controller = SimulationController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3.10 Simulation Validation and Testing

### Testing Simulation Accuracy

To validate simulation accuracy:

1. **Compare with Analytical Models**: Verify kinematic solutions match mathematical calculations
2. **Check Conservation Laws**: Ensure energy and momentum are conserved appropriately
3. **Validate Sensor Models**: Compare simulated sensor data with expected values
4. **Test Edge Cases**: Validate behavior at joint limits and collision scenarios

### Performance Considerations

- **Time Step**: Smaller steps increase accuracy but decrease performance
- **Real-time Factor**: Ratio of simulation time to real time
- **Solver Parameters**: Balance between stability and computational cost

## 3.11 "Sim-to-Real" Transfer Challenges

### The Reality Gap

The "Sim-to-Real" gap refers to differences between simulation and reality:

- **Model Inaccuracies**: Real robots have unmodeled dynamics
- **Sensor Noise**: Simulated sensors are often too clean
- **Environmental Factors**: Lighting, surface properties, etc.
- **Actuator Dynamics**: Real actuators have delays and limitations

### Bridging the Gap

1. **Domain Randomization**: Randomize simulation parameters during training
2. **System Identification**: Calibrate simulation parameters to match real robot
3. **Robust Control**: Design controllers that handle uncertainty
4. **Progressive Transfer**: Gradually move from simulation to reality

## Summary

Physics simulation and digital twins are essential tools for Physical AI development. They provide a safe, efficient environment for testing and validating algorithms before deployment on real hardware. Understanding URDF, physics principles, and simulation parameters is crucial for creating effective digital twins that can bridge the gap between simulation and reality.

## Next Steps

In the next section, we'll explore digital twins in more detail, including "Digital Twin Workstation" vs. "Edge Kit" concepts and Unity integration for hardware simulation.

<div class="alert alert-warning">
  <h5>Hardware Requirement</h5>
  <div><strong>Requirement:</strong> GPU</div>
  <div><strong>Minimum:</strong> RTX 4070</div>
  <div><strong>Recommended:</strong> RTX 4080</div>
  <div><strong>Purpose:</strong> Physics simulation requires significant computational resources</div>
</div>