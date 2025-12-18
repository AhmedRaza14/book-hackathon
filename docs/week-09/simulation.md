---
title: "Week 9: Navigation and Motion Planning - Simulation Environment"
week: 9
module: "Robotic Infrastructure"
difficulty: "advanced"
prerequisites: ["control-systems", "kinematics", "sensors", "ros2-fundamentals"]
learning_objectives:
  - "Configure Gazebo simulation for navigation"
  - "Implement path planning in simulated environments"
  - "Test localization and mapping algorithms"
  - "Evaluate navigation performance metrics"
tags: ["navigation", "motion-planning", "gazebo", "path-planning", "localization", "mapping", "simulation"]
hardware_requirements:
  - gpu: "RTX 4070 or higher"
  - ram: "16GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "90 minutes"
---

# Week 9: Navigation and Motion Planning - Simulation Environment

## Learning Objectives
- Configure Gazebo simulation for navigation
- Implement path planning in simulated environments
- Test localization and mapping algorithms
- Evaluate navigation performance metrics

## Introduction

This simulation environment provides a comprehensive platform for testing navigation and motion planning algorithms. The environment includes realistic physics, sensor models, and diverse scenarios for evaluating path planning, localization, and navigation systems. The simulation supports various navigation algorithms including A*, Dijkstra, RRT, and Dynamic Window Approach.

## Simulation Setup

### Environment Configuration

The simulation environment includes a complex indoor setting with static and dynamic obstacles:

```xml
<!-- Gazebo world file: navigation_world.world -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="navigation_world">
    <!-- Physics Engine -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Include outdoor lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Main arena -->
    <model name="main_arena">
      <pose>0 0 0 0 0 0</pose>
      <static>true</static>
      <link name="arena_link">
        <collision name="arena_collision">
          <geometry>
            <box>
              <size>20 20 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="arena_visual">
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

    <!-- Outer walls -->
    <model name="north_wall">
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
            <ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.4 0.4 0.4 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="south_wall">
      <pose>0 -10 1 0 0 0</pose>
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
            <ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.4 0.4 0.4 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="east_wall">
      <pose>10 0 1 0 0 1.5707</pose>
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
            <ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.4 0.4 0.4 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="west_wall">
      <pose>-10 0 1 0 0 1.5707</pose>
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
            <ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.4 0.4 0.4 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Static obstacles -->
    <model name="table_1">
      <pose>5 5 0.5 0 0 0</pose>
      <static>true</static>
      <link name="table_link">
        <collision name="table_collision">
          <geometry>
            <box>
              <size>2 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="table_visual">
          <geometry>
            <box>
              <size>2 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="shelf_1">
      <pose>-5 6 1 0 0 0</pose>
      <static>true</static>
      <link name="shelf_link">
        <collision name="shelf_collision">
          <geometry>
            <box>
              <size>0.5 3 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="shelf_visual">
          <geometry>
            <box>
              <size>0.5 3 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.3 0.1 1</ambient>
            <diffuse>0.5 0.3 0.1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="pillar_1">
      <pose>-3 -3 1.5 0 0 0</pose>
      <static>true</static>
      <link name="pillar_link">
        <collision name="pillar_collision">
          <geometry>
            <cylinder>
              <radius>0.8</radius>
              <length>3</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="pillar_visual">
          <geometry>
            <cylinder>
              <radius>0.8</radius>
              <length>3</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="chair_1">
      <pose>2 -4 0.4 0 0 0.785</pose>
      <static>true</static>
      <link name="chair_link">
        <collision name="chair_collision">
          <geometry>
            <box>
              <size>0.6 0.6 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="chair_visual">
          <geometry>
            <box>
              <size>0.6 0.6 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.6 0.2 1</ambient>
            <diffuse>0.8 0.6 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Dynamic obstacles -->
    <model name="dynamic_obstacle_1">
      <pose>-4 0 0.5 0 0 0</pose>
      <static>false</static>
      <link name="obstacle_link">
        <collision name="obstacle_collision">
          <geometry>
            <cylinder>
              <radius>0.4</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="obstacle_visual">
          <geometry>
            <cylinder>
              <radius>0.4</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.2 0.6 0.8 1</ambient>
            <diffuse>0.2 0.6 0.8 1</diffuse>
          </material>
        </visual>
        <velocity>0.2 0 0</velocity>
      </link>
      <plugin name="model_push" filename="libgazebo_ros_p3d.so">
        <always_on>true</always_on>
        <update_rate>10</update_rate>
        <body_name>obstacle_link</body_name>
        <topic_name>dynamic_obstacle_1/odom</topic_name>
      </plugin>
    </model>

    <model name="dynamic_obstacle_2">
      <pose>0 4 0.5 0 0 0</pose>
      <static>false</static>
      <link name="obstacle_link">
        <collision name="obstacle_collision">
          <geometry>
            <sphere>
              <radius>0.3</radius>
            </sphere>
          </geometry>
        </collision>
        <visual name="obstacle_visual">
          <geometry>
            <sphere>
              <radius>0.3</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.6 1</ambient>
            <diffuse>0.8 0.2 0.6 1</diffuse>
          </material>
        </visual>
        <velocity>0 -0.15 0</velocity>
      </link>
    </model>

    <!-- Navigation test areas -->
    <model name="start_area">
      <pose>-8 -8 0.01 0 0 0</pose>
      <static>true</static>
      <link name="area_link">
        <visual name="area_visual">
          <geometry>
            <box>
              <size>2 2 0.01</size>
            </box>
          </geometry>
          <material>
            <ambient>0.2 1.0 0.2 0.5</ambient>
            <diffuse>0.2 1.0 0.2 0.5</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="goal_area">
      <pose>8 8 0.01 0 0 0</pose>
      <static>true</static>
      <link name="area_link">
        <visual name="area_visual">
          <geometry>
            <box>
              <size>2 2 0.01</size>
            </box>
          </geometry>
          <material>
            <ambient>1.0 0.2 0.2 0.5</ambient>
            <diffuse>1.0 0.2 0.2 0.5</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Landmarks for localization -->
    <model name="landmark_1">
      <pose>7 7 1.5 0 0 0</pose>
      <static>true</static>
      <link name="landmark_link">
        <visual name="landmark_visual">
          <geometry>
            <box>
              <size>0.1 0.1 3</size>
            </box>
          </geometry>
          <material>
            <ambient>1.0 1.0 0.0 1</ambient>
            <diffuse>1.0 1.0 0.0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="landmark_2">
      <pose>-7 -7 1.5 0 0 0</pose>
      <static>true</static>
      <link name="landmark_link">
        <visual name="landmark_visual">
          <geometry>
            <box>
              <size>0.1 0.1 3</size>
            </box>
          </geometry>
          <material>
            <ambient>1.0 0.0 1.0 1</ambient>
            <diffuse>1.0 0.0 1.0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="landmark_3">
      <pose>0 9 1.5 0 0 0</pose>
      <static>true</static>
      <link name="landmark_link">
        <visual name="landmark_visual">
          <geometry>
            <box>
              <size>0.1 0.1 3</size>
            </box>
          </geometry>
          <material>
            <ambient>0.0 1.0 1.0 1</ambient>
            <diffuse>0.0 1.0 1.0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Robot Model for Navigation

The simulation includes a differential drive robot with navigation sensors:

```xml
<!-- Robot URDF: navigation_robot.urdf.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="navigation_robot">

  <!-- Base properties -->
  <xacro:property name="base_mass" value="20.0"/>
  <xacro:property name="base_radius" value="0.3"/>
  <xacro:property name="base_length" value="0.15"/>

  <!-- Wheel properties -->
  <xacro:property name="wheel_mass" value="3.0"/>
  <xacro:property name="wheel_radius" value="0.08"/>
  <xacro:property name="wheel_width" value="0.05"/>
  <xacro:property name="wheel_y_offset" value="0.25"/>

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
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
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
      <mu1>0.8</mu1>
      <mu2>0.8</mu2>
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
      <wheelTorque>50.0</wheelTorque>
    </plugin>
  </gazebo>

  <!-- 2D LIDAR -->
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 ${base_length/2 + 0.1}" rpy="0 0 0"/>
  </joint>

  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <gazebo reference="lidar_link">
    <sensor type="ray" name="laser_scanner">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1.0</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin filename="libgazebo_ros_laser.so" name="gazebo_ros_laser_controller">
        <topicName>scan</topicName>
        <frameName>lidar_link</frameName>
      </plugin>
    </sensor>
  </gazebo>

  <!-- RGBD Camera -->
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
    <sensor type="depth" name="camera">
      <update_rate>30.0</update_rate>
      <camera name="head">
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
      <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>0.0</updateRate>
        <cameraName>camera</cameraName>
        <imageTopicName>rgb/image_raw</imageTopicName>
        <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
        <depthImageTopicName>depth/image_raw</depthImageTopicName>
        <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>
        <pointCloudTopicName>depth/points</pointCloudTopicName>
        <frameName>camera_link</frameName>
      </plugin>
    </sensor>
  </gazebo>

  <!-- IMU Sensor -->
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
<!-- Launch file: nav_simulation.launch -->
<launch>
  <!-- Arguments -->
  <arg name="world_file" default="$(find nav_gazebo)/worlds/navigation_world.world"/>
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
  <param name="robot_description" command="$(find xacro)/xacro $(find nav_description)/urdf/navigation_robot.urdf.xacro"/>
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-param robot_description -urdf -model navigation_robot -x 0 -y 0 -z 0.1" respawn="false"/>

  <!-- Robot state publisher -->
  <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher">
    <param name="publish_frequency" type="double" value="50.0"/>
  </node>

  <!-- Joint state publisher -->
  <node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher">
    <param name="rate" type="double" value="50.0"/>
  </node>

  <!-- Navigation stack -->
  <include file="$(find nav2_bringup)/launch/navigation_launch.py"/>

  <!-- Map server -->
  <node pkg="nav2_map_server" exec="map_server" name="map_server">
    <param name="yaml_filename" value="$(find nav_maps)/maps/my_map.yaml"/>
  </node>

  <!-- AMCL localization -->
  <node pkg="nav2_amcl" exec="amcl" name="amcl">
    <param name="use_sim_time" value="true"/>
    <param name="initial_pose.x" value="0.0"/>
    <param name="initial_pose.y" value="0.0"/>
    <param name="initial_pose.z" value="0.0"/>
    <param name="initial_pose.yaw" value="0.0"/>
  </node>

  <!-- Controllers -->
  <rosparam file="$(find nav_control)/config/controllers.yaml" command="load"/>
  <node name="controller_spawner" pkg="controller_manager" type="spawner" args="
                joint_state_controller
                diff_drive_controller"/>

  <!-- RViz for visualization -->
  <node name="rviz" pkg="rviz2" type="rviz2" args="-d $(find nav_viz)/config/navigation.rviz"/>

  <!-- Launch custom navigation nodes -->
  <node name="dijkstra_planner" pkg="path_planning" exec="dijkstra_planner" output="screen"/>
  <node name="astar_planner" pkg="path_planning" exec="astar_planner" output="screen"/>
  <node name="dwa_planner" pkg="motion_planner" exec="dwa_planner" output="screen"/>
  <node name="particle_filter" pkg="localization_system" exec="particle_filter" output="screen"/>
  <node name="navigation_manager" pkg="navigation_stack" exec="navigation_manager" output="screen"/>

</launch>
```

### Controller Configuration

```yaml
# Config file: controllers.yaml
diff_drive_controller:
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
      max_velocity: 1.0
      min_velocity: -1.0
      has_acceleration_limits: true
      max_acceleration: 2.0
      min_acceleration: -2.0
  angular:
    z:
      has_velocity_limits: true
      max_velocity: 2.0
      min_velocity: -2.0
      has_acceleration_limits: true
      max_acceleration: 4.0
      min_acceleration: -4.0

joint_state_controller:
  type: "joint_state_controller/JointStateController"
  publish_rate: 50
```

## Simulation Scenarios

### Scenario 1: Static Obstacle Navigation

Test path planning algorithms with static obstacles:

1. **Setup**: Robot starts at (-8, -8) facing towards (8, 8)
2. **Goal**: Navigate to (8, 8) while avoiding static obstacles
3. **Evaluation**: Path optimality, execution time, safety metrics

### Scenario 2: Dynamic Obstacle Avoidance

Test local planning with moving obstacles:

1. **Setup**: Robot navigates path with moving obstacles
2. **Goal**: Reach destination while avoiding dynamic obstacles
3. **Evaluation**: Obstacle detection rate, path deviation, safety

### Scenario 3: Localization and Mapping

Test SLAM capabilities:

1. **Setup**: Robot with unknown initial position
2. **Goal**: Localize and build map of environment
3. **Evaluation**: Localization accuracy, map quality, convergence time

### Scenario 4: Multi-Goal Navigation

Test navigation with multiple waypoints:

1. **Setup**: Sequence of navigation goals
2. **Goal**: Visit all waypoints efficiently
3. **Evaluation**: Route optimization, goal completion rate

## Performance Evaluation Nodes

### Path Planning Evaluation Node

```cpp
// path_planning_evaluation.cpp
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <vector>
#include <cmath>

class PathPlanningEvaluator {
public:
    PathPlanningEvaluator() : nh_("~"), path_length_(0.0), start_time_(ros::Time::now()),
                             total_execution_time_(0.0), path_execution_start_(false) {
        // Publishers
        path_length_pub_ = nh_.advertise<std_msgs::Float64>("path_length", 10);
        execution_time_pub_ = nh_.advertise<std_msgs::Float64>("execution_time", 10);
        success_rate_pub_ = nh_.advertise<std_msgs::Float64>("success_rate", 10);
        path_efficiency_pub_ = nh_.advertise<std_msgs::Float64>("path_efficiency", 10);

        // Subscribers
        planned_path_sub_ = nh_.subscribe("plan", 10, &PathPlanningEvaluator::plannedPathCallback, this);
        executed_path_sub_ = nh_.subscribe("executed_path", 10, &PathPlanningEvaluator::executedPathCallback, this);
        odom_sub_ = nh_.subscribe("odom", 10, &PathPlanningEvaluator::odomCallback, this);
        goal_sub_ = nh_.subscribe("move_base_simple/goal", 10, &PathPlanningEvaluator::goalCallback, this);

        // Initialize
        prev_odom_pose_.pose.position.x = 0.0;
        prev_odom_pose_.pose.position.y = 0.0;
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher path_length_pub_, execution_time_pub_, success_rate_pub_, path_efficiency_pub_;
    ros::Subscriber planned_path_sub_, executed_path_sub_, odom_sub_, goal_sub_;

    nav_msgs::Path planned_path_;
    nav_msgs::Path executed_path_;
    nav_msgs::Odometry prev_odom_pose_;
    geometry_msgs::PoseStamped current_goal_;

    double path_length_;
    ros::Time start_time_;
    double total_execution_time_;
    bool path_execution_start_;
    int successful_navigations_;
    int total_navigations_;

    void plannedPathCallback(const nav_msgs::Path::ConstPtr& msg) {
        planned_path_ = *msg;
        calculatePathLength();
    }

    void executedPathCallback(const nav_msgs::Path::ConstPtr& msg) {
        executed_path_ = *msg;
        evaluatePathEfficiency();
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        if (!path_execution_start_) {
            path_execution_start_ = true;
            start_time_ = msg->header.stamp;
        }

        // Calculate distance traveled since last update
        double dx = msg->pose.pose.position.x - prev_odom_pose_.pose.position.x;
        double dy = msg->pose.pose.position.y - prev_odom_pose_.pose.position.y;
        double dist = sqrt(dx*dx + dy*dy);

        path_length_ += dist;

        // Update previous pose
        prev_odom_pose_ = *msg;

        // Publish path length
        std_msgs::Float64 path_length_msg;
        path_length_msg.data = path_length_;
        path_length_pub_.publish(path_length_msg);

        // Calculate and publish execution time
        ros::Duration execution_time = msg->header.stamp - start_time_;
        std_msgs::Float64 time_msg;
        time_msg.data = execution_time.toSec();
        execution_time_pub_.publish(time_msg);
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_goal_ = *msg;
        total_navigations_++;
        path_execution_start_ = false;
        path_length_ = 0.0;
    }

    void calculatePathLength() {
        if (planned_path_.poses.size() < 2) return;

        double length = 0.0;
        for (size_t i = 1; i < planned_path_.poses.size(); ++i) {
            double dx = planned_path_.poses[i].pose.position.x -
                       planned_path_.poses[i-1].pose.position.x;
            double dy = planned_path_.poses[i].pose.position.y -
                       planned_path_.poses[i-1].pose.position.y;
            length += sqrt(dx*dx + dy*dy);
        }

        std_msgs::Float64 planned_length_msg;
        planned_length_msg.data = length;
        path_length_pub_.publish(planned_length_msg);
    }

    void evaluatePathEfficiency() {
        if (planned_path_.poses.empty() || executed_path_.poses.empty()) return;

        // Calculate planned path length
        double planned_length = 0.0;
        for (size_t i = 1; i < planned_path_.poses.size(); ++i) {
            double dx = planned_path_.poses[i].pose.position.x -
                       planned_path_.poses[i-1].pose.position.x;
            double dy = planned_path_.poses[i].pose.position.y -
                       planned_path_.poses[i-1].pose.position.y;
            planned_length += sqrt(dx*dx + dy*dy);
        }

        // Calculate executed path length
        double executed_length = 0.0;
        for (size_t i = 1; i < executed_path_.poses.size(); ++i) {
            double dx = executed_path_.poses[i].pose.position.x -
                       executed_path_.poses[i-1].pose.position.x;
            double dy = executed_path_.poses[i].pose.position.y -
                       executed_path_.poses[i-1].pose.position.y;
            executed_length += sqrt(dx*dx + dy*dy);
        }

        // Calculate efficiency (lower is better, normalized)
        double efficiency = executed_length / planned_length;

        std_msgs::Float64 efficiency_msg;
        efficiency_msg.data = efficiency;
        path_efficiency_pub_.publish(efficiency_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planning_evaluator");
    PathPlanningEvaluator evaluator;
    ros::spin();
    return 0;
}
```

### Localization Evaluation Node

```cpp
// localization_evaluation.cpp
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include <cmath>

class LocalizationEvaluator {
public:
    LocalizationEvaluator() : nh_("~"), ground_truth_available_(false),
                             localization_error_(0.0), rotation_error_(0.0) {
        // Publishers
        position_error_pub_ = nh_.advertise<std_msgs::Float64>("position_error", 10);
        rotation_error_pub_ = nh_.advertise<std_msgs::Float64>("rotation_error", 10);
        accuracy_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("localization_accuracy", 10);

        // Subscribers
        estimated_pose_sub_ = nh_.subscribe("amcl_pose", 10, &LocalizationEvaluator::estimatedPoseCallback, this);
        ground_truth_sub_ = nh_.subscribe("ground_truth", 10, &LocalizationEvaluator::groundTruthCallback, this);

        // Timer for periodic evaluation
        eval_timer_ = nh_.createTimer(ros::Duration(1.0), &LocalizationEvaluator::evaluateLocalization, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher position_error_pub_, rotation_error_pub_, accuracy_pub_;
    ros::Subscriber estimated_pose_sub_, ground_truth_sub_;
    ros::Timer eval_timer_;

    geometry_msgs::PoseWithCovarianceStamped estimated_pose_;
    geometry_msgs::PoseStamped ground_truth_pose_;
    bool ground_truth_available_;
    double localization_error_;
    double rotation_error_;

    void estimatedPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        estimated_pose_ = *msg;
    }

    void groundTruthCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        ground_truth_pose_ = *msg;
        ground_truth_available_ = true;
    }

    void evaluateLocalization(const ros::TimerEvent&) {
        if (!ground_truth_available_) {
            ROS_WARN("Ground truth not available for localization evaluation");
            return;
        }

        // Calculate position error
        double dx = estimated_pose_.pose.pose.position.x - ground_truth_pose_.pose.position.x;
        double dy = estimated_pose_.pose.pose.position.y - ground_truth_pose_.pose.position.y;
        localization_error_ = sqrt(dx*dx + dy*dy);

        // Calculate rotation error
        double est_yaw = getYawFromQuaternion(estimated_pose_.pose.pose.orientation);
        double gt_yaw = getYawFromQuaternion(ground_truth_pose_.pose.orientation);
        rotation_error_ = std::abs(est_yaw - gt_yaw);

        // Normalize rotation error to [0, pi]
        if (rotation_error_ > M_PI) {
            rotation_error_ = 2 * M_PI - rotation_error_;
        }

        // Publish errors
        std_msgs::Float64 pos_error_msg;
        pos_error_msg.data = localization_error_;
        position_error_pub_.publish(pos_error_msg);

        std_msgs::Float64 rot_error_msg;
        rot_error_msg.data = rotation_error_;
        rotation_error_pub_.publish(rot_error_msg);

        // Publish comprehensive accuracy report
        std_msgs::Float64MultiArray accuracy_msg;
        accuracy_msg.data.push_back(localization_error_);
        accuracy_msg.data.push_back(rotation_error_);
        accuracy_msg.data.push_back(estimated_pose_.pose.covariance[0]);  // x variance
        accuracy_msg.data.push_back(estimated_pose_.pose.covariance[7]);  // y variance
        accuracy_msg.data.push_back(estimated_pose_.pose.covariance[35]); // theta variance
        accuracy_pub_.publish(accuracy_msg);

        ROS_INFO("Localization Error: %.3f m, Rotation Error: %.3f rad",
                 localization_error_, rotation_error_);
    }

    double getYawFromQuaternion(const geometry_msgs::Quaternion& q) {
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "localization_evaluator");
    LocalizationEvaluator evaluator;
    ros::spin();
    return 0;
}
```

## Isaac Sim Integration

For advanced simulation capabilities, the environment can be configured for Isaac Sim:

### Isaac Sim Configuration

```python
# isaac_sim_nav.py
import omni
from pxr import UsdGeom
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core import World
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.range_sensor import _range_sensor
import numpy as np

def setup_navigation_simulation():
    """Setup navigation simulation in Isaac Sim"""
    # Initialize world
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # Add differential drive robot
    robot = world.scene.add(
        WheeledRobot(
            prim_path="/World/Robot",
            name="nav_robot",
            wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
            create_robot=True,
            position=np.array([0.0, 0.0, 0.1]),
            orientation=np.array([0, 0, 0, 1])
        )
    )

    # Add static obstacles
    from omni.isaac.core.objects import DynamicCuboid, FixedCuboid

    obstacles = [
        world.scene.add(
            FixedCuboid(
                prim_path="/World/Obstacle1",
                name="obstacle1",
                position=np.array([5.0, 5.0, 0.5]),
                size=1.0,
                color=np.array([0.6, 0.4, 0.2])
            )
        ),
        world.scene.add(
            FixedCuboid(
                prim_path="/World/Obstacle2",
                name="obstacle2",
                position=np.array([-5.0, 6.0, 0.5]),
                size=0.5,
                color=np.array([0.5, 0.3, 0.1])
            )
        ),
        world.scene.add(
            FixedCuboid(
                prim_path="/World/Obstacle3",
                name="obstacle3",
                position=np.array([-3.0, -3.0, 1.0]),
                size=1.6,
                color=np.array([0.7, 0.7, 0.7])
            )
        )
    ]

    # Add landmarks for localization
    landmarks = [
        world.scene.add(
            FixedCuboid(
                prim_path="/World/Landmark1",
                name="landmark1",
                position=np.array([7.0, 7.0, 1.5]),
                size=np.array([0.1, 0.1, 3.0]),
                color=np.array([1.0, 1.0, 0.0])
            )
        ),
        world.scene.add(
            FixedCuboid(
                prim_path="/World/Landmark2",
                name="landmark2",
                position=np.array([-7.0, -7.0, 1.5]),
                size=np.array([0.1, 0.1, 3.0]),
                color=np.array([1.0, 0.0, 1.0])
            )
        )
    ]

    # Add LIDAR sensor
    lidar = _range_sensor.acquire_lidar_sensor_interface()
    lidar.add_lidar_to_stage(
        prim_path="/World/Robot/Lidar",
        translation=(0, 0, 0.2),
        orientation=(0, 0, 0, 1),
        config="Example_Rotary",
        min_range=0.1,
        max_range=10.0,
        draw_points=False,
        draw_lines=True
    )

    return world, robot, lidar

def run_navigation_test():
    """Run navigation test in Isaac Sim"""
    world, robot, lidar = setup_navigation_simulation()
    world.reset()

    # Main simulation loop
    for i in range(10000):
        if i % 100 == 0:
            print(f"Simulation step: {i}")

        # Get LIDAR data
        try:
            scan_data = lidar.get_linear_depth_data("/World/Robot/Lidar")
            # Process scan data for navigation
            print(f"LIDAR data shape: {len(scan_data)}")
        except Exception as e:
            print(f"LIDAR error: {e}")

        # Apply control commands to robot
        # In a real implementation, this would connect to ROS nodes
        if i < 1000:
            robot.apply_wheel_actions(
                omni.isaac.wheeled_robots.controllers.differential_inverse_kinematics.DifferentialController(
                    name="simple_control",
                    wheel_radius=0.1,
                    wheel_base=0.5
                ).forward(command=[0.5, 0.0])  # Move forward
            )
        elif i < 2000:
            robot.apply_wheel_actions(
                omni.isaac.wheeled_robots.controllers.differential_inverse_kinematics.DifferentialController(
                    name="simple_control",
                    wheel_radius=0.1,
                    wheel_base=0.5
                ).forward(command=[0.2, 0.5])  # Turn right
            )
        else:
            robot.apply_wheel_actions(
                omni.isaac.wheeled_robots.controllers.differential_inverse_kinematics.DifferentialController(
                    name="simple_control",
                    wheel_radius=0.1,
                    wheel_base=0.5
                ).forward(command=[0.0, 0.0])  # Stop
            )

        # Update simulation
        world.step(render=True)

if __name__ == "__main__":
    run_navigation_test()
```

## Testing and Validation

### Unit Tests for Navigation Components

```cpp
// test_navigation_components.cpp
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

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

TEST_F(NavigationComponentTest, TestLocalization) {
    // Test localization algorithm
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
# test_navigation_integration.sh

# Start Gazebo simulation
roslaunch nav_gazebo nav_simulation.launch &
GAZEBO_PID=$!

# Wait for simulation to start
sleep 15

# Start path planning tests
rosrun nav_test path_planning_test &
PATH_TEST_PID=$!

# Start localization tests
rosrun nav_test localization_test &
LOCAL_TEST_PID=$!

# Start navigation tests
rosrun nav_test navigation_test &
NAV_TEST_PID=$!

# Wait for tests to complete
sleep 60

# Shutdown all processes
kill $PATH_TEST_PID $LOCAL_TEST_PID $NAV_TEST_PID $GAZEBO_PID

echo "Navigation integration tests completed."
```

## Advanced Simulation Features

### Custom Sensors for Navigation

```xml
<!-- Custom sensor configuration -->
<gazebo reference="base_link">
  <!-- Multi-beam LIDAR for better obstacle detection -->
  <sensor type="ray" name="multi_lidar">
    <pose>0.3 0 0.2 0 0 0</pose>
    <visualize>false</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>0.5</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
        <vertical>
          <samples>16</samples>
          <resolution>1.0</resolution>
          <min_angle>-0.1745</min_angle>
          <max_angle>0.1745</max_angle>
        </vertical>
      </scan>
      <range>
        <min>0.1</min>
        <max>20.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin filename="libgazebo_ros_laser.so" name="multi_lidar_controller">
      <topicName>multi_scan</topicName>
      <frameName>lidar_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```

### Dynamic Environment Generation

```python
#!/usr/bin/env python3
# dynamic_env_generator.py

import rospy
import random
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String

class DynamicEnvironmentGenerator:
    def __init__(self):
        rospy.init_node('dynamic_env_generator', anonymous=True)

        # Service proxies
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        # Timer for dynamic changes
        self.env_timer = rospy.Timer(rospy.Duration(30.0), self.change_environment)

        rospy.loginfo("Dynamic Environment Generator initialized")

    def spawn_random_obstacle(self):
        """Spawn a random obstacle in the environment"""
        model_names = ["cylinder", "box", "sphere"]
        model_name = random.choice(model_names)

        # Generate random position
        x = random.uniform(-8, 8)
        y = random.uniform(-8, 8)

        # Define model based on type
        if model_name == "cylinder":
            model_xml = f"""
            <sdf version="1.6">
              <model name="dynamic_obstacle_{int(rospy.Time.now().to_sec())}">
                <pose>{x} {y} 0.5 0 0 0</pose>
                <link name="link">
                  <collision name="collision">
                    <geometry>
                      <cylinder>
                        <radius>0.3</radius>
                        <length>1.0</length>
                      </cylinder>
                    </geometry>
                  </collision>
                  <visual name="visual">
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
            </sdf>"""
        elif model_name == "box":
            model_xml = f"""
            <sdf version="1.6">
              <model name="dynamic_obstacle_{int(rospy.Time.now().to_sec())}">
                <pose>{x} {y} 0.5 0 0 0</pose>
                <link name="link">
                  <collision name="collision">
                    <geometry>
                      <box>
                        <size>0.6 0.6 1.0</size>
                      </box>
                    </geometry>
                  </collision>
                  <visual name="visual">
                    <geometry>
                      <box>
                        <size>0.6 0.6 1.0</size>
                      </box>
                    </geometry>
                    <material>
                      <ambient>0.8 0.2 0.2 1</ambient>
                      <diffuse>0.8 0.2 0.2 1</diffuse>
                    </material>
                  </visual>
                </link>
              </model>
            </sdf>"""
        else:  # sphere
            model_xml = f"""
            <sdf version="1.6">
              <model name="dynamic_obstacle_{int(rospy.Time.now().to_sec())}">
                <pose>{x} {y} 0.5 0 0 0</pose>
                <link name="link">
                  <collision name="collision">
                    <geometry>
                      <sphere>
                        <radius>0.4</radius>
                      </sphere>
                    </geometry>
                  </collision>
                  <visual name="visual">
                    <geometry>
                      <sphere>
                        <radius>0.4</radius>
                      </sphere>
                    </geometry>
                    <material>
                      <ambient>0.2 0.8 0.2 1</ambient>
                      <diffuse>0.2 0.8 0.2 1</diffuse>
                    </material>
                  </visual>
                </link>
              </model>
            </sdf>"""

        # Spawn the model
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.5

        try:
            self.spawn_model(f"dynamic_obstacle_{int(rospy.Time.now().to_sec())}",
                           model_xml, "", pose, "world")
            rospy.loginfo(f"Spawned {model_name} obstacle at ({x}, {y})")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def change_environment(self, event):
        """Change the environment by adding/removing obstacles"""
        # Add a new obstacle
        self.spawn_random_obstacle()

        # Optionally remove an old obstacle (implementation depends on tracking spawned models)
        rospy.loginfo("Environment changed")

def main():
    generator = DynamicEnvironmentGenerator()
    rospy.spin()

if __name__ == '__main__':
    main()
```

## Conclusion

This simulation environment provides a comprehensive platform for testing navigation and motion planning algorithms. The modular design allows for easy customization of environments, robot configurations, and test scenarios. The integration of both Gazebo and Isaac Sim provides flexibility in simulation approaches, from lightweight testing to high-fidelity physics simulation.

The environment includes realistic sensor models, dynamic obstacles, and performance evaluation tools that enable thorough testing of navigation and localization algorithms before deployment on real hardware.