---
title: "Week 1: Physics Simulation Guide"
week: 1
module: "Foundations of Physical AI"
difficulty: "beginner"
prerequisites: ["physics-basics", "simulation-basics"]
learning_objectives:
  - "Set up basic physics simulation environment"
  - "Create simple URDF robot model"
  - "Simulate basic robot movements"
tags: ["gazebo", "urdf", "simulation", "physics"]
hardware_requirements:
  - gpu: "RTX 4070 or higher"
  - ram: "16GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "120 minutes"
---

# Week 1: Physics Simulation Guide

## Learning Objectives
- Set up basic physics simulation environment
- Create simple URDF robot model
- Simulate basic robot movements
- Understand physics simulation principles

## 3.1 Introduction to Physics Simulation

Physics simulation is a crucial component of Physical AI development. It allows us to:
- Test algorithms in a safe, controlled environment
- Bridge the gap between theory and real-world implementation
- Validate control algorithms before deployment
- Generate training data for machine learning models

### Digital Twins

A digital twin is a virtual representation of a physical system that mirrors its properties, states, and behaviors in real-time. In robotics, digital twins enable:

- **Virtual Testing**: Validate algorithms before hardware deployment
- **Predictive Maintenance**: Monitor and predict system behavior
- **Optimization**: Fine-tune parameters in simulation before real-world implementation

## 3.2 Setting Up Gazebo Simulation Environment

Gazebo is a powerful 3D simulation environment for robotics. To get started:

### Installation (Ubuntu 22.04 LTS)

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
```

### Basic Gazebo Launch

```bash
# Launch Gazebo with empty world
ros2 launch gazebo_ros empty_world.launch.py
```

## 3.3 Creating a Simple URDF Robot Model

URDF (Unified Robot Description Format) is XML-based format to describe robot models. Let's create a simple 2-link robot:

```xml
<!-- simple_robot.urdf -->
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base Link -->
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

  <!-- First Link -->
  <link name="link1">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.5"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint between base and first link -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="1000.0" velocity="1.0"/>
  </joint>
</robot>
```

## 3.4 Loading URDF in Gazebo

To load your URDF model in Gazebo:

```bash
# Launch Gazebo with your robot model
ros2 launch gazebo_ros spawn_entity.py -entity simple_robot -file /path/to/simple_robot.urdf
```

## 3.5 Creating a Gazebo World File

Create a simple world file to customize your simulation environment:

```xml
<!-- simple_world.world -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_world">
    <!-- Include the default sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Include the default ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Your robot model -->
    <include>
      <uri>model://simple_robot</uri>
    </include>

    <!-- Add a simple box obstacle -->
    <model name="box">
      <pose>1 0 0.5 0 0 0</pose>
      <link name="box_link">
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
        </collision>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.083</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.083</iyy>
            <iyz>0.0</iyz>
            <izz>0.083</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

## 3.6 Simulating Robot Movements

To control your robot in simulation, you can create a ROS 2 controller node. Here's a simple example:

```python
#!/usr/bin/env python3
# joint_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float64MultiArray()
        # Simple oscillating motion for joint1
        joint_pos = math.sin(self.i * 0.1) * 1.5
        msg.data = [joint_pos]  # Joint positions
        self.publisher.publish(msg)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    joint_publisher = JointPublisher()
    rclpy.spin(joint_publisher)
    joint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3.7 Understanding Physics Simulation Parameters

### Gravity
- Default value: 9.81 m/s² in the negative Z direction
- Can be modified in world files for different environments

### Time Step
- Smaller time steps provide more accurate simulation
- Larger time steps provide faster computation
- Balance accuracy and performance based on your needs

### Solver Parameters
- **Type**: Iterative or direct solvers
- **Iterations**: Number of iterations for constraint solving
- **Tolerance**: Convergence threshold for iterative solvers

## 3.8 Practical Exercise

Create a simulation environment with:
1. A simple 3-link robot arm
2. A target object to reach
3. A basic controller to move the arm to the target
4. Visualization of the robot's end-effector trajectory

## 3.9 "Sim-to-Real" Considerations

When developing with simulation, keep in mind:

- **Model Fidelity**: Ensure your simulation models accurately represent real hardware
- **Sensor Noise**: Add realistic noise models to simulated sensors
- **Actuator Dynamics**: Include realistic actuator limitations and delays
- **Environmental Factors**: Account for lighting, surface properties, and other environmental variables

## Summary

Physics simulation is a critical tool for Physical AI development. It allows us to test algorithms safely and efficiently before deployment on real hardware. Understanding the principles of physics simulation and how to create accurate digital twins is essential for bridging the gap between simulation and reality.

<div class="alert alert-warning">
  <h5>Hardware Requirement</h5>
  <div><strong>Requirement:</strong> GPU</div>
  <div><strong>Minimum:</strong> RTX 4070</div>
  <div><strong>Recommended:</strong> RTX 4080</div>
  <div><strong>Purpose:</strong> Physics simulation requires significant GPU resources</div>
</div>