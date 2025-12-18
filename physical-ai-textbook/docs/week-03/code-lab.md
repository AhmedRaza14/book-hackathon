---
title: "Week 3: Physics Simulation Code Lab"
week: 3
module: "Foundations of Physical AI"
difficulty: "intermediate"
prerequisites: ["python-basics", "ros2-basics", "kinematics", "urdf-basics"]
learning_objectives:
  - "Create and configure URDF robot models"
  - "Implement physics simulation in Gazebo"
  - "Simulate sensor data and robot control"
tags: ["gazebo", "urdf", "simulation", "physics", "sensors"]
hardware_requirements:
  - gpu: "RTX 4070 or higher"
  - ram: "16GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "120 minutes"
---

# Week 3: Physics Simulation Code Lab

## Learning Objectives
- Create and configure URDF robot models
- Implement physics simulation in Gazebo
- Simulate sensor data and robot control
- Validate simulation against mathematical models

## 3.1 Setting Up the Simulation Environment

First, let's install and verify our simulation environment:

```bash
# Install Gazebo and ROS 2 packages
sudo apt update
sudo apt install ignition-harmonic
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers

# Verify installation
ign --version
ros2 pkg list | grep gazebo
```

## 3.2 Creating a Simple Robot URDF

Let's create a complete URDF file for a simple 3-DOF manipulator:

```xml
<!-- simple_manipulator.urdf -->
<?xml version="1.0"?>
<robot name="simple_manipulator" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Materials -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.2 0.2 0.2 1.0"/>
  </material>
  <material name="orange">
    <color rgba="1.0 0.4235 0.0392 1.0"/>
  </material>
  <material name="brown">
    <color rgba="0.8706 0.8118 0.7647 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Joint 1: Base to Shoulder -->
  <joint name="shoulder_pan_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0.0 0.0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="1000" velocity="1.0"/>
  </joint>

  <link name="shoulder_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.15"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- Joint 2: Shoulder to Elbow -->
  <joint name="shoulder_lift_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="elbow_link"/>
    <origin xyz="0.0 0.0 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="1000" velocity="1.0"/>
  </joint>

  <link name="elbow_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.2"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- Joint 3: Elbow to Wrist -->
  <joint name="elbow_joint" type="revolute">
    <parent link="elbow_link"/>
    <child link="wrist_link"/>
    <origin xyz="0.0 0.0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="1000" velocity="1.0"/>
  </joint>

  <link name="wrist_link">
    <visual>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- End Effector -->
  <joint name="wrist_to_end_effector" type="fixed">
    <parent link="wrist_link"/>
    <child link="end_effector"/>
    <origin xyz="0.0 0.0 0.05" rpy="0 0 0"/>
  </joint>

  <link name="end_effector">
    <visual>
      <geometry>
        <sphere radius="0.01"/>
      </geometry>
      <material name="white"/>
    </visual>
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.00001" ixy="0.0" ixz="0.0" iyy="0.00001" iyz="0.0" izz="0.00001"/>
    </inertial>
  </link>

  <!-- Gazebo plugin for ROS control -->
  <gazebo>
    <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
      <parameters>$(find simple_manipulator_description)/config/simple_manipulator_controllers.yaml</parameters>
    </plugin>
  </gazebo>

</robot>
```

## 3.3 Robot Controller Configuration

Now let's create the controller configuration file:

```yaml
# config/simple_manipulator_controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

joint_trajectory_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

## 3.4 Launch Files for Simulation

Let's create a launch file to start our robot in Gazebo:

```python
# launch/simple_manipulator.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    headless = LaunchConfiguration('headless', default='false')

    # Paths
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    pkg_simple_manipulator_description = FindPackageShare('simple_manipulator_description')

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Set to "false" to run headless'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Set to "true" to run headless'
        ),

        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_gazebo_ros,
                    'launch',
                    'gazebo.launch.py'
                ])
            ]),
            launch_arguments={
                'gui': gui,
                'headless': headless,
                'use_sim_time': use_sim_time,
            }.items()
        ),

        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'simple_manipulator',
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_description': PathJoinSubstitution([
                    pkg_simple_manipulator_description,
                    'urdf',
                    'simple_manipulator.urdf'
                ])}
            ]
        ),

        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {'use_sim_time': use_sim_time}
            ]
        ),

        # Controller Manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                PathJoinSubstitution([
                    pkg_simple_manipulator_description,
                    'config',
                    'simple_manipulator_controllers.yaml'
                ]),
                {'use_sim_time': use_sim_time}
            ],
            output='both'
        )
    ])
```

## 3.5 Python Control Node

Let's create a Python node to control our simulated robot:

```python
#!/usr/bin/env python3
# robot_controller.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import math

class SimpleManipulatorController(Node):
    """Controller for the simple manipulator robot"""

    def __init__(self):
        super().__init__('simple_manipulator_controller')

        # Publisher for joint trajectory commands
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Subscriber for joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Robot parameters
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint']
        self.current_positions = [0.0, 0.0, 0.0]

        # Control parameters
        self.time_counter = 0.0
        self.control_mode = 'oscillate'  # 'oscillate', 'fixed', 'wave'

        self.get_logger().info('Simple Manipulator Controller initialized')

    def joint_state_callback(self, msg):
        """Update current joint positions"""
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                self.current_positions[idx] = pos

    def control_loop(self):
        """Main control loop"""
        self.time_counter += 0.1

        # Generate trajectory based on control mode
        if self.control_mode == 'oscillate':
            positions = self.generate_oscillating_trajectory()
        elif self.control_mode == 'fixed':
            positions = [0.5, 0.3, -0.2]  # Fixed position
        elif self.control_mode == 'wave':
            positions = self.generate_wave_trajectory()
        else:
            positions = [0.0, 0.0, 0.0]

        # Create and publish trajectory message
        self.publish_trajectory(positions)

        # Log current positions
        self.get_logger().info(
            f'Current positions: {self.current_positions}, '
            f'Desired positions: {positions}'
        )

    def generate_oscillating_trajectory(self):
        """Generate oscillating joint trajectory"""
        t = self.time_counter
        shoulder = 0.5 * math.sin(0.5 * t)
        elbow = 0.3 * math.sin(0.7 * t + math.pi/2)
        wrist = 0.2 * math.sin(0.9 * t + math.pi)

        return [shoulder, elbow, wrist]

    def generate_wave_trajectory(self):
        """Generate wave-like joint trajectory"""
        t = self.time_counter
        shoulder = 0.5 * math.sin(0.3 * t)
        elbow = 0.3 * math.sin(0.3 * t + math.pi/3)
        wrist = 0.2 * math.sin(0.3 * t + 2*math.pi/3)

        return [shoulder, elbow, wrist]

    def publish_trajectory(self, positions):
        """Publish joint trajectory command"""
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)  # Zero velocity
        point.accelerations = [0.0] * len(positions)  # Zero acceleration
        point.time_from_start = Duration(sec=0, nanosec=100000000)  # 0.1 seconds

        trajectory_msg.points = [point]
        self.trajectory_pub.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = SimpleManipulatorController()

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

## 3.6 Sensor Integration in Simulation

Let's enhance our URDF with sensor definitions:

```xml
<!-- Enhanced URDF with sensors -->
<?xml version="1.0"?>
<robot name="simple_manipulator_with_sensors">

  <!-- ... (previous links and joints) ... -->

  <!-- Camera sensor on end effector -->
  <joint name="end_effector_to_camera" type="fixed">
    <parent link="end_effector"/>
    <child link="camera_link"/>
    <origin xyz="0.0 0.0 0.02" rpy="0 0 0"/>
  </joint>

  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.02 0.02 0.01"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.02 0.02 0.01"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.00001" ixy="0.0" ixz="0.0" iyy="0.00001" iyz="0.0" izz="0.00001"/>
    </inertial>
  </link>

  <!-- IMU sensor -->
  <joint name="base_to_imu" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
  </joint>

  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.00001" ixy="0.0" ixz="0.0" iyy="0.00001" iyz="0.0" izz="0.00001"/>
    </inertial>
  </link>

  <!-- Gazebo sensor definitions -->
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
          <far>10</far>
        </clip>
      </camera>
      <always_on>1</always_on>
      <update_rate>30</update_rate>
      <visualize>true</visualize>
    </sensor>
  </gazebo>

  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>1</always_on>
      <update_rate>100</update_rate>
      <visualize>false</visualize>
    </sensor>
  </gazebo>

  <!-- Gazebo plugin for ROS control -->
  <gazebo>
    <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
      <parameters>$(find simple_manipulator_description)/config/simple_manipulator_controllers.yaml</parameters>
    </plugin>
  </gazebo>

</robot>
```

## 3.7 Sensor Data Processing Node

Now let's create a node to process sensor data from our simulated robot:

```python
#!/usr/bin/env python3
# sensor_processor.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, JointState
from cv_bridge import CvBridge
import cv2
import numpy as np

class SensorProcessor(Node):
    """Process sensor data from simulated robot"""

    def __init__(self):
        super().__init__('sensor_processor')

        # Create CV bridge for image processing
        self.cv_bridge = CvBridge()

        # Subscribers for sensor data
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu_sensor',
            self.imu_callback,
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer for periodic processing
        self.timer = self.create_timer(1.0, self.process_data)

        # Data storage
        self.latest_image = None
        self.latest_imu = None
        self.latest_joint_states = None

        self.get_logger().info('Sensor Processor initialized')

    def image_callback(self, msg):
        """Process incoming image data"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
            self.get_logger().info(f'Image received: {cv_image.shape}')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def imu_callback(self, msg):
        """Process incoming IMU data"""
        self.latest_imu = msg
        self.get_logger().info(
            f'IMU: Linear Acc = ({msg.linear_acceleration.x:.2f}, '
            f'{msg.linear_acceleration.y:.2f}, {msg.linear_acceleration.z:.2f})'
        )

    def joint_state_callback(self, msg):
        """Process incoming joint state data"""
        self.latest_joint_states = msg
        self.get_logger().info(f'Joint positions: {msg.position}')

    def process_data(self):
        """Periodically process collected sensor data"""
        if self.latest_image is not None:
            # Simple image processing - detect edges
            gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)

            # Display processed image (if display is available)
            # cv2.imshow('Processed Image', edges)
            # cv2.waitKey(1)

        if self.latest_imu is not None:
            # Calculate magnitude of acceleration
            acc = self.latest_imu.linear_acceleration
            acc_magnitude = np.sqrt(acc.x**2 + acc.y**2 + acc.z**2)
            self.get_logger().info(f'Acceleration magnitude: {acc_magnitude:.2f}')

def main(args=None):
    rclpy.init(args=args)
    processor = SensorProcessor()

    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3.8 Physics Simulation Parameters

Let's create a world file with custom physics parameters:

```xml
<!-- worlds/custom_physics.world -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="custom_physics_world">
    <!-- Include default sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Include default ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Custom gravity -->
    <gravity>0 0 -9.8</gravity>

    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <max_contacts>20</max_contacts>

      <!-- ODE specific parameters -->
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.000001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Add a simple box object for interaction -->
    <model name="box_object">
      <pose>1 0 0.5 0 0 0</pose>
      <link name="box_link">
        <visual name="visual">
          <geometry>
            <box>
              <size>0.2 0.2 0.2</size>
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
              <size>0.2 0.2 0.2</size>
            </box>
          </geometry>
        </collision>
        <inertial>
          <mass>0.5</mass>
          <inertia>
            <ixx>0.00333</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.00333</iyy>
            <iyz>0.0</iyz>
            <izz>0.00333</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

## 3.9 Simulation Testing and Validation

Let's create a test script to validate our simulation:

```python
#!/usr/bin/env python3
# simulation_validator.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import math

class SimulationValidator(Node):
    """Validate simulation behavior against expected models"""

    def __init__(self):
        super().__init__('simulation_validator')

        # Subscribers and publishers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Timer for validation
        self.timer = self.create_timer(0.5, self.validate_simulation)

        # Data storage
        self.current_positions = {}
        self.commanded_positions = {}
        self.test_counter = 0

        # Joint names for our robot
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint']

        self.get_logger().info('Simulation Validator initialized')

    def joint_state_callback(self, msg):
        """Update current joint positions"""
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_names:
                self.current_positions[name] = pos

    def validate_simulation(self):
        """Run validation tests"""
        self.test_counter += 1

        if self.test_counter % 4 == 1:
            self.test_position_accuracy()
        elif self.test_counter % 4 == 2:
            self.test_joint_limits()
        elif self.test_counter % 4 == 3:
            self.test_trajectory_following()
        else:
            self.test_stability()

    def test_position_accuracy(self):
        """Test if robot reaches commanded positions"""
        # Command a specific position
        target_positions = [0.5, 0.3, -0.2]
        self.command_trajectory(target_positions)

        # Check if we're close to target
        if all(name in self.current_positions for name in self.joint_names):
            current_pos = [self.current_positions[name] for name in self.joint_names]
            error = np.array(current_pos) - np.array(target_positions)
            max_error = np.max(np.abs(error))

            if max_error < 0.1:  # Within 0.1 rad tolerance
                self.get_logger().info(f'✓ Position accuracy test passed. Error: {max_error:.3f}')
            else:
                self.get_logger().warn(f'✗ Position accuracy test failed. Error: {max_error:.3f}')

    def test_joint_limits(self):
        """Test joint limit enforcement"""
        # Command positions near limits
        limit_positions = [1.5, 1.5, 1.5]  # Near upper limits
        self.command_trajectory(limit_positions)

        # Check if joints stay within limits
        limits_ok = True
        for name in self.joint_names:
            if name in self.current_positions:
                pos = self.current_positions[name]
                if abs(pos) > 1.6:  # Beyond our defined limits
                    limits_ok = False
                    self.get_logger().warn(f'Joint {name} exceeded limits: {pos}')

        if limits_ok:
            self.get_logger().info('✓ Joint limit test passed')

    def test_trajectory_following(self):
        """Test trajectory following performance"""
        # Generate a smooth trajectory
        t = self.get_clock().now().nanoseconds / 1e9
        positions = [
            0.3 * math.sin(0.5 * t),
            0.2 * math.sin(0.7 * t),
            0.1 * math.sin(0.9 * t)
        ]

        self.command_trajectory(positions)

    def test_stability(self):
        """Test simulation stability"""
        # Check if positions are reasonable (not NaN or extremely large)
        for name in self.joint_names:
            if name in self.current_positions:
                pos = self.current_positions[name]
                if math.isnan(pos) or abs(pos) > 100:
                    self.get_logger().error(f'Stability issue detected for {name}: {pos}')

    def command_trajectory(self, positions):
        """Command a joint trajectory"""
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.time_from_start = Duration(sec=0, nanosec=500000000)  # 0.5 seconds

        trajectory_msg.points = [point]
        self.trajectory_pub.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    validator = SimulationValidator()

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        pass
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3.10 Practical Exercise

Create a complete simulation environment that:
1. Implements a 4-DOF manipulator with proper URDF
2. Adds multiple sensor types (camera, IMU, force/torque)
3. Implements a control system that follows a trajectory
4. Validates the simulation against mathematical models

```python
# Student exercise - Complete implementation
class AdvancedManipulatorSimulator:
    """Student implementation of an advanced manipulator simulator"""

    def __init__(self):
        """Initialize the advanced simulator"""
        # TODO: Create URDF for 4-DOF manipulator
        # TODO: Implement sensor integration
        # TODO: Create control algorithms
        # TODO: Add validation tests
        pass

    def create_urdf(self):
        """Create URDF for 4-DOF manipulator"""
        # TODO: Complete URDF implementation
        pass

    def implement_control(self):
        """Implement control algorithms"""
        # TODO: Complete control implementation
        pass

    def add_sensors(self):
        """Add sensor models to simulation"""
        # TODO: Complete sensor implementation
        pass

    def validate_simulation(self):
        """Validate simulation accuracy"""
        # TODO: Complete validation implementation
        pass

print("Student Exercise: Implement an advanced manipulator simulator")
print("Requirements:")
print("1. 4-DOF manipulator with realistic dynamics")
print("2. Multiple sensor integration")
print("3. Advanced control algorithms")
print("4. Comprehensive validation framework")
```

## Summary

In this lab, we've implemented a complete physics simulation environment with URDF robot models, sensor integration, and control systems. We've learned how to create and configure robot models for simulation, implement control algorithms, and validate simulation accuracy. These skills are essential for developing and testing Physical AI systems in a safe, cost-effective environment.

<div class="alert alert-warning">
  <h5>Hardware Requirement</h5>
  <div><strong>Requirement:</strong> GPU</div>
  <div><strong>Minimum:</strong> RTX 4070</div>
  <div><strong>Recommended:</strong> RTX 4080</div>
  <div><strong>Purpose:</strong> Physics simulation requires significant computational resources</div>
</div>