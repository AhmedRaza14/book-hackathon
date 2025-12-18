---
title: "Week 14: Navigation and Manipulation - Simulation Environment"
week: 14
module: "Applications and Projects"
difficulty: "advanced"
prerequisites: ["week-13", "gazebo-simulation", "ros-navigation", "ros-manipulation"]
learning_objectives:
  - "Configure Gazebo simulation for mobile manipulator robots"
  - "Implement navigation and manipulation in simulated environments"
  - "Test path planning and obstacle avoidance algorithms"
  - "Evaluate manipulation strategies in simulation"
tags: ["navigation", "manipulation", "gazebo", "mobile-robot", "simulated-environment", "path-planning", "grasping"]
duration: "90 minutes"
---

# Week 14: Navigation and Manipulation - Simulation Environment

## Learning Objectives
- Configure Gazebo simulation for mobile manipulator robots
- Implement navigation and manipulation in simulated environments
- Test path planning and obstacle avoidance algorithms
- Evaluate manipulation strategies in simulation

## Introduction

This simulation environment provides a comprehensive platform for testing navigation and manipulation capabilities of mobile manipulator robots. The environment includes realistic physics, sensor models, and diverse scenarios for evaluating navigation and manipulation algorithms.

## Simulation Setup

### Environment Configuration

The simulation environment consists of a warehouse-like setting with dynamic obstacles, varied terrain, and manipulation targets:

```xml
<!-- Gazebo world file: mobile_manipulator.world -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="mobile_manipulator_world">
    <!-- Physics Engine -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Include outdoor lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Warehouse environment -->
    <model name="warehouse_floor">
      <pose>0 0 0 0 0 0</pose>
      <static>true</static>
      <link name="floor_link">
        <collision name="floor_collision">
          <geometry>
            <box>
              <size>20 20 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="floor_visual">
          <geometry>
            <box>
              <size>20 20 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Walls -->
    <model name="wall_north">
      <pose>0 10 1 0 0 0</pose>
      <static>true</static>
      <link name="wall_link">
        <collision name="wall_collision">
          <geometry>
            <box>
              <size>20 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="wall_visual">
          <geometry>
            <box>
              <size>20 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Furniture and obstacles -->
    <model name="table_1">
      <pose>5 5 0.5 0 0 0</pose>
      <static>true</static>
      <link name="table_link">
        <collision name="table_collision">
          <geometry>
            <box>
              <size>1.5 1.5 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="table_visual">
          <geometry>
            <box>
              <size>1.5 1.5 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Dynamic obstacles -->
    <model name="dynamic_obstacle_1">
      <pose>-3 0 0.5 0 0 0</pose>
      <static>false</static>
      <link name="obstacle_link">
        <collision name="obstacle_collision">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="obstacle_visual">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.2 0.6 0.8 1</ambient>
            <diffuse>0.2 0.6 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Objects for manipulation -->
    <model name="target_box_1">
      <pose>4 6 1.2 0 0 0</pose>
      <static>false</static>
      <link name="box_link">
        <collision name="box_collision">
          <geometry>
            <box>
              <size>0.2 0.2 0.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="box_visual">
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
      </link>
    </model>

    <model name="target_cylinder_1">
      <pose>-4 -6 1.2 0 0 0</pose>
      <static>false</static>
      <link name="cylinder_link">
        <collision name="cylinder_collision">
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>0.3</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="cylinder_visual">
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>0.3</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.2 0.8 0.2 1</ambient>
            <diffuse>0.2 0.8 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Mobile Manipulator Robot Model

The simulation includes a mobile manipulator robot with differential drive base and 6-DOF manipulator arm:

```xml
<!-- Robot URDF: mobile_manipulator.urdf.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="mobile_manipulator">

  <!-- Base properties -->
  <xacro:property name="base_mass" value="50.0"/>
  <xacro:property name="base_radius" value="0.3"/>
  <xacro:property name="base_length" value="0.4"/>

  <!-- Wheel properties -->
  <xacro:property name="wheel_mass" value="5.0"/>
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>
  <xacro:property name="wheel_y_offset" value="0.25"/>

  <!-- Manipulator properties -->
  <xacro:property name="arm_base_height" value="0.5"/>
  <xacro:property name="link1_length" value="0.3"/>
  <xacro:property name="link2_length" value="0.4"/>
  <xacro:property name="link3_length" value="0.3"/>
  <xacro:property name="link4_length" value="0.2"/>
  <xacro:property name="link5_length" value="0.1"/>
  <xacro:property name="link6_length" value="0.1"/>

  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base Footprint -->
  <link name="base_footprint">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.01"/>
      </geometry>
    </visual>
  </link>

  <!-- Base Link -->
  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 ${base_length/2}" rpy="0 0 0"/>
  </joint>

  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${base_radius}" length="${base_length}"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${base_radius}" length="${base_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${base_mass}"/>
      <inertia
        ixx="${base_mass / 12.0 * (3*base_radius*base_radius + base_length*base_length)}" ixy="0.0" ixz="0.0"
        iyy="${base_mass / 12.0 * (3*base_radius*base_radius + base_length*base_length)}" iyz="0.0"
        izz="${base_mass / 2.0 * (base_radius*base_radius)}"/>
    </inertial>
  </link>

  <!-- Wheels -->
  <xacro:macro name="wheel" params="prefix reflect joint_name">
    <link name="${prefix}_wheel">
      <visual>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black"/>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${wheel_mass}"/>
        <inertia
          ixx="${wheel_mass / 12.0 * (3*wheel_radius*wheel_radius + wheel_width*wheel_width)}" ixy="0.0" ixz="0.0"
          iyy="${wheel_mass / 12.0 * (3*wheel_radius*wheel_radius + wheel_width*wheel_width)}" iyz="0.0"
          izz="${wheel_mass / 2.0 * (wheel_radius*wheel_radius)}"/>
      </inertial>
    </link>

    <joint name="${joint_name}" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="0 ${reflect*wheel_y_offset} 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>

    <gazebo reference="${prefix}_wheel">
      <mu1>0.5</mu1>
      <mu2>0.5</mu2>
      <kp>10000000.0</kp>
      <kd>1.0</kd>
      <fdir1>1 0 0</fdir1>
      <material>Gazebo/Black</material>
    </gazebo>
  </xacro:macro>

  <xacro:wheel prefix="left" reflect="1" joint_name="left_wheel_joint"/>
  <xacro:wheel prefix="right" reflect="-1" joint_name="right_wheel_joint"/>

  <!-- Drive controller -->
  <gazebo>
    <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <commandTopic>cmd_vel</commandTopic>
      <odometryTopic>odom</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <robotBaseFrame>base_footprint</robotBaseFrame>
      <publishWheelJointState>true</publishWheelJointState>
      <publishWheelTF>false</publishWheelTF>
      <publishOdomTF>true</publishOdomTF>
      <legacyMode>false</legacyMode>
      <odometrySource>world</odometrySource>
      <updateRate>30</updateRate>
      <leftJoint>left_wheel_joint</leftJoint>
      <rightJoint>right_wheel_joint</rightJoint>
      <wheelSeparation>${2*wheel_y_offset}</wheelSeparation>
      <wheelDiameter>${2*wheel_radius}</wheelDiameter>
      <wheelAcceleration>1.0</wheelAcceleration>
      <wheelTorque>100.0</wheelTorque>
    </plugin>
  </gazebo>

  <!-- Manipulator base -->
  <joint name="base_arm_joint" type="fixed">
    <parent link="base_link"/>
    <child link="arm_base_link"/>
    <origin xyz="0 0 ${base_length/2 + arm_base_height}" rpy="0 0 0"/>
  </joint>

  <link name="arm_base_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Manipulator joints and links -->
  <joint name="shoulder_pan_joint" type="revolute">
    <parent link="arm_base_link"/>
    <child link="shoulder_pan_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.618" upper="2.618" effort="100" velocity="3.14"/>
  </joint>

  <link name="shoulder_pan_link">
    <visual>
      <geometry>
        <cylinder radius="0.08" length="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.08" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="shoulder_lift_joint" type="revolute">
    <parent link="shoulder_pan_link"/>
    <child link="shoulder_lift_link"/>
    <origin xyz="0 0 ${link1_length}" rpy="0 0 0"/>
    <axis xyz="0 -1 0"/>
    <limit lower="-1.571" upper="1.571" effort="100" velocity="3.14"/>
  </joint>

  <link name="shoulder_lift_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 ${link2_length}"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 ${link2_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <joint name="elbow_joint" type="revolute">
    <parent link="shoulder_lift_link"/>
    <child link="elbow_link"/>
    <origin xyz="0 0 ${link2_length}" rpy="0 0 0"/>
    <axis xyz="0 -1 0"/>
    <limit lower="-2.094" upper="1.571" effort="100" velocity="3.14"/>
  </joint>

  <link name="elbow_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 ${link3_length}"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 ${link3_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.2"/>
      <inertia ixx="0.015" ixy="0.0" ixz="0.0" iyy="0.015" iyz="0.0" izz="0.015"/>
    </inertial>
  </link>

  <joint name="wrist_1_joint" type="revolute">
    <parent link="elbow_link"/>
    <child link="wrist_1_link"/>
    <origin xyz="0 0 ${link3_length}" rpy="0 0 0"/>
    <axis xyz="0 -1 0"/>
    <limit lower="-1.571" upper="1.571" effort="50" velocity="3.14"/>
  </joint>

  <link name="wrist_1_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="wrist_2_joint" type="revolute">
    <parent link="wrist_1_link"/>
    <child link="wrist_2_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.618" upper="2.618" effort="50" velocity="3.14"/>
  </joint>

  <link name="wrist_2_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="wrist_3_joint" type="revolute">
    <parent link="wrist_2_link"/>
    <child link="ee_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 -1 0"/>
    <limit lower="-1.571" upper="1.571" effort="50" velocity="3.14"/>
  </joint>

  <link name="ee_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- End effector with gripper -->
  <joint name="ee_fixed_joint" type="fixed">
    <parent link="ee_link"/>
    <child link="gripper_base_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

  <link name="gripper_base_link">
    <visual>
      <geometry>
        <box size="0.03 0.08 0.03"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.03 0.08 0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Gripper fingers -->
  <joint name="left_gripper_joint" type="prismatic">
    <parent link="gripper_base_link"/>
    <child link="left_gripper_link"/>
    <origin xyz="0 0.05 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0.0" upper="0.04" effort="20" velocity="1.0"/>
  </joint>

  <link name="left_gripper_link">
    <visual>
      <geometry>
        <box size="0.02 0.02 0.03"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.02 0.02 0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="right_gripper_joint" type="prismatic">
    <parent link="gripper_base_link"/>
    <child link="right_gripper_link"/>
    <origin xyz="0 -0.05 0" rpy="0 0 0"/>
    <axis xyz="0 -1 0"/>
    <limit lower="0.0" upper="0.04" effort="20" velocity="1.0"/>
  </joint>

  <link name="right_gripper_link">
    <visual>
      <geometry>
        <box size="0.02 0.02 0.03"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.02 0.02 0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Gripper controller plugin -->
  <gazebo>
    <plugin name="gripper_controller" filename="libgazebo_ros_control.so">
      <robotNamespace>/mobile_manipulator</robotNamespace>
    </plugin>
  </gazebo>

  <!-- Sensors -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="${base_radius - 0.05} 0 ${base_length/2 + 0.1}" rpy="0 0 0"/>
  </joint>

  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.1 0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <gazebo reference="camera_link">
    <sensor type="camera" name="camera1">
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>800</width>
          <height>600</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>0.0</updateRate>
        <cameraName>camera1</cameraName>
        <imageTopicName>image_raw</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>camera_link</frameName>
        <hackBaseline>0.07</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
      </plugin>
    </sensor>
  </gazebo>

  <!-- IMU sensor -->
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 ${base_length/2 - 0.05}" rpy="0 0 0"/>
  </joint>

  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <gazebo reference="imu_link">
    <sensor type="imu" name="imu_sensor">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>false</visualize>
      <plugin filename="libgazebo_ros_imu.so" name="imu_plugin">
        <topicName>imu/data</topicName>
        <bodyName>imu_link</bodyName>
        <updateRateHZ>100.0</updateRateHZ>
        <gaussianNoise>0.01</gaussianNoise>
        <xyzOffset>0 0 0</xyzOffset>
        <rpyOffset>0 0 0</rpyOffset>
        <frameName>imu_link</frameName>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

## Launch Files

### Navigation Simulation Launch

```xml
<!-- Launch file: nav_manipulation_simulation.launch -->
<launch>
  <!-- Arguments -->
  <arg name="world_file" default="$(find mobile_manipulator_gazebo)/worlds/mobile_manipulator.world"/>
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="gui" default="true"/>
  <arg name="headless" default="false"/>
  <arg name="debug" default="false"/>

  <!-- Start Gazebo with the specified world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(arg world_file)"/>
    <arg name="paused" value="$(arg paused)"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    <arg name="gui" value="$(arg gui)"/>
    <arg name="headless" value="$(arg headless)"/>
    <arg name="debug" value="$(arg debug)"/>
  </include>

  <!-- Spawn the robot in Gazebo -->
  <param name="robot_description" command="$(find xacro)/xacro $(find mobile_manipulator_description)/urdf/mobile_manipulator.urdf.xacro"/>
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-param robot_description -urdf -model mobile_manipulator -x 0 -y 0 -z 0.5" respawn="false"/>

  <!-- Robot state publisher -->
  <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher">
    <param name="publish_frequency" type="double" value="30.0"/>
  </node>

  <!-- Joint state publisher -->
  <node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher">
    <param name="rate" type="double" value="30.0"/>
  </node>

  <!-- Navigation stack -->
  <include file="$(find nav_core)/launch/move_base.launch"/>

  <!-- Manipulation controllers -->
  <rosparam file="$(find mobile_manipulator_control)/config/controllers.yaml" command="load"/>
  <node name="controller_spawner" pkg="controller_manager" type="spawner" args="joint_state_controller
                mobile_base_controller
                arm_controller
                gripper_controller"/>

  <!-- RViz for visualization -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find mobile_manipulator_viz)/config/navigation_manipulation.rviz"/>
</launch>
```

### Controller Configuration

```yaml
# Config file: controllers.yaml
mobile_base_controller:
  type: "diff_drive_controller/DiffDriveController"
  left_wheel: ['left_wheel_joint']
  right_wheel: ['right_wheel_joint']
  publish_rate: 50
  pose_covariance_diagonal: [0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 1000.0]
  twist_covariance_diagonal: [0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 1000.0]
  cmd_vel_timeout: 0.25
  base_frame_id: base_footprint
  linear:
    x:
      has_velocity_limits: true
      max_velocity: 2.0
      min_velocity: -2.0
      has_acceleration_limits: true
      max_acceleration: 5.0
      min_acceleration: -5.0
  angular:
    z:
      has_velocity_limits: true
      max_velocity: 4.0
      min_velocity: -4.0
      has_acceleration_limits: true
      max_acceleration: 8.0
      min_acceleration: -8.0

arm_controller:
  type: "position_controllers/JointTrajectoryController"
  joints:
    - shoulder_pan_joint
    - shoulder_lift_joint
    - elbow_joint
    - wrist_1_joint
    - wrist_2_joint
    - wrist_3_joint
  gains:
    shoulder_pan_joint: {p: 100.0, i: 0.01, d: 10.0}
    shoulder_lift_joint: {p: 100.0, i: 0.01, d: 10.0}
    elbow_joint: {p: 100.0, i: 0.01, d: 10.0}
    wrist_1_joint: {p: 50.0, i: 0.01, d: 5.0}
    wrist_2_joint: {p: 50.0, i: 0.01, d: 5.0}
    wrist_3_joint: {p: 50.0, i: 0.01, d: 5.0}

gripper_controller:
  type: "position_controllers/JointTrajectoryController"
  joints:
    - left_gripper_joint
    - right_gripper_joint

joint_state_controller:
  type: "joint_state_controller/JointStateController"
  publish_rate: 50
```

## Simulation Scenarios

### Navigation Scenario

The navigation scenario tests the robot's ability to navigate through a cluttered environment to reach a goal location:

1. **Setup**: Robot starts at origin (0,0,0) in warehouse environment
2. **Goal**: Navigate to a specified location while avoiding static and dynamic obstacles
3. **Evaluation**: Measure path optimality, time to goal, and safety metrics

### Manipulation Scenario

The manipulation scenario tests the robot's ability to pick and place objects:

1. **Setup**: Robot approaches a table with objects
2. **Task**: Pick up a target object and place it at a designated location
3. **Evaluation**: Measure grasp success rate, task completion time, and accuracy

### Integrated Navigation-Manipulation Scenario

The integrated scenario combines navigation and manipulation:

1. **Setup**: Robot must navigate to multiple locations to perform manipulation tasks
2. **Task**: Collect objects from various locations and deposit them at a central location
3. **Evaluation**: Measure overall task completion, efficiency, and robustness

## Performance Evaluation Nodes

### Navigation Evaluation Node

```cpp
// navigation_evaluation.cpp
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <vector>

class NavigationEvaluator {
public:
    NavigationEvaluator() : nh_("~"), path_length_(0.0), start_time_(ros::Time::now()) {
        odom_sub_ = nh_.subscribe("/mobile_manipulator/odom", 1, &NavigationEvaluator::odomCallback, this);
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
        path_length_pub_ = nh_.advertise<std_msgs::Float64>("path_length", 1);

        // Set initial robot position as starting point
        prev_pose_.pose.position.x = 0.0;
        prev_pose_.pose.position.y = 0.0;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher goal_pub_, path_length_pub_;

    nav_msgs::Odometry prev_pose_;
    double path_length_;
    ros::Time start_time_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Calculate distance traveled since last update
        double dx = msg->pose.pose.position.x - prev_pose_.pose.position.x;
        double dy = msg->pose.pose.position.y - prev_pose_.pose.position.y;
        double dist = sqrt(dx*dx + dy*dy);

        path_length_ += dist;

        // Publish accumulated path length
        std_msgs::Float64 path_msg;
        path_msg.data = path_length_;
        path_length_pub_.publish(path_msg);

        // Store current pose for next calculation
        prev_pose_ = *msg;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigation_evaluator");
    NavigationEvaluator evaluator;
    ros::spin();
    return 0;
}
```

### Manipulation Evaluation Node

```cpp
// manipulation_evaluation.cpp
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <iostream>

class ManipulationEvaluator {
public:
    ManipulationEvaluator() : nh_("~"), attempts_(0), successes_(0) {
        success_sub_ = nh_.subscribe("grasp_success", 1, &ManipulationEvaluator::successCallback, this);
        success_rate_pub_ = nh_.advertise<std_msgs::Float64>("success_rate", 1);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber success_sub_;
    ros::Publisher success_rate_pub_;

    int attempts_, successes_;

    void successCallback(const std_msgs::Bool::ConstPtr& msg) {
        attempts_++;
        if (msg->data) {
            successes_++;
        }

        // Calculate and publish success rate
        std_msgs::Float64 rate_msg;
        if (attempts_ > 0) {
            rate_msg.data = (double)successes_ / (double)attempts_;
        } else {
            rate_msg.data = 0.0;
        }
        success_rate_pub_.publish(rate_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "manipulation_evaluator");
    ManipulationEvaluator evaluator;
    ros::spin();
    return 0;
}
```

## Isaac Sim Integration

For advanced simulation capabilities, the environment can be configured for Isaac Sim:

### Isaac Sim Configuration

```python
# isaac_sim_config.py
import omni
from pxr import UsdGeom
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core import World
from omni.isaac.franka import Franka
from omni.isaac.core.utils.prims import get_prim_at_path
import numpy as np

def setup_mobile_manipulator_simulation():
    """Setup mobile manipulator simulation in Isaac Sim"""
    # Initialize world
    world = World(stage_units_in_meters=1.0)

    # Add ground plane
    world.scene.add_ground_plane(static_friction=0.5, dynamic_friction=0.5, restitution=0.8)

    # Add mobile manipulator robot
    # The robot would be loaded from a USD file containing the complete URDF to USD conversion

    # Add objects for manipulation
    from omni.isaac.core.objects import DynamicCuboid

    cube = world.scene.add(
        DynamicCuboid(
            prim_path="/World/cube_1",
            name="cube_1",
            position=np.array([0.5, 0.5, 0.5]),
            size=0.2,
            color=np.array([0.8, 0.1, 0.1])
        )
    )

    # Add obstacles
    cylinder = world.scene.add(
        DynamicCuboid(
            prim_path="/World/cylinder_1",
            name="cylinder_1",
            position=np.array([-0.5, -0.5, 0.5]),
            size=0.15,
            color=np.array([0.2, 0.8, 0.2])
        )
    )

    return world

def run_navigation_manipulation_test():
    """Run integrated navigation and manipulation test"""
    world = setup_mobile_manipulator_simulation()
    world.reset()

    # Main simulation loop
    for i in range(10000):
        if i % 100 == 0:
            print(f"Simulation step: {i}")

        # Update simulation
        world.step(render=True)

        # Add navigation and manipulation logic here
        # This would include path planning, obstacle avoidance, and grasp planning

if __name__ == "__main__":
    run_navigation_manipulation_test()
```

## Testing and Validation

### Unit Tests for Navigation Components

```cpp
// test_navigation_components.cpp
#include <gtest/gtest.h>
#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>

class NavigationComponentTest : public ::testing::Test {
protected:
    virtual void SetUp() {
        // Setup test environment
    }

    virtual void TearDown() {
        // Cleanup test environment
    }
};

TEST_F(NavigationComponentTest, TestPathPlanning) {
    // Test path planning algorithm
    EXPECT_TRUE(true);  // Placeholder for actual test
}

TEST_F(NavigationComponentTest, TestObstacleAvoidance) {
    // Test obstacle avoidance
    EXPECT_TRUE(true);  // Placeholder for actual test
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
```

### Integration Tests

```bash
#!/bin/bash
# test_integration.sh

# Start Gazebo simulation
roslaunch mobile_manipulator_gazebo nav_manipulation_simulation.launch &
GAZEBO_PID=$!

# Wait for simulation to start
sleep 10

# Run navigation test
rosrun mobile_manipulator_tests navigation_test_node &
NAV_TEST_PID=$!

# Run manipulation test
rosrun mobile_manipulator_tests manipulation_test_node &
MANIP_TEST_PID=$!

# Wait for tests to complete
wait $NAV_TEST_PID
wait $MANIP_TEST_PID

# Shutdown simulation
kill $GAZEBO_PID

echo "Integration tests completed."
```

## Conclusion

This simulation environment provides a comprehensive platform for testing navigation and manipulation capabilities. The modular design allows for easy customization of environments, robot configurations, and test scenarios. The integration of both Gazebo and Isaac Sim provides flexibility in simulation approaches, from lightweight testing to high-fidelity physics simulation.

The environment includes realistic sensor models, dynamic obstacles, and performance evaluation tools that enable thorough testing of navigation and manipulation algorithms before deployment on real hardware.