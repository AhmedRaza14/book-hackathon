---
title: "Week 11: Cognitive Planning - Simulation"
week: 11
module: "AI-Robot Integration"
difficulty: "advanced"
prerequisites: ["motion-planning", "vision-language", "control-systems", "ros2-advanced"]
learning_objectives:
  - "Simulate task and motion planning systems"
  - "Test hierarchical planning architectures"
  - "Validate uncertainty handling in planning"
  - "Evaluate reactive vs. deliberative systems"
tags: ["simulation", "planning", "task-planning", "motion-planning", "hierarchical-planning", "decision-making", "uncertainty", "pddl"]
hardware_requirements:
  - gpu: "RTX 4070 or higher"
  - ram: "32GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "120 minutes"
---

# Week 11: Cognitive Planning - Simulation

## Learning Objectives
- Simulate task and motion planning systems
- Test hierarchical planning architectures
- Validate uncertainty handling in planning
- Evaluate reactive vs. deliberative systems

## Prerequisites
- ROS 2 Humble Hawksbill installed
- Gazebo or Isaac Sim installed
- Python 3.8+ with pip
- CUDA-compatible GPU with RTX 4070 or higher
- Completed Week 10 materials

## Simulation Environment Setup

### Option 1: Gazebo Simulation
Set up a Gazebo environment for cognitive planning:

```bash
# Install planning-specific Gazebo plugins
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
```

### Option 2: Isaac Sim Setup
For more advanced planning simulation:

```bash
# Ensure Isaac Sim is installed with planning extensions
# Install Isaac ROS navigation packages
sudo apt update
sudo apt install ros-humble-isaac-ros-navigation ros-humble-isaac-ros-occupancy-grid-localizer
```

## Part 1: Creating Planning Test Environments

### Task 1.1: Create a Complex World for Planning
Create a Gazebo world file with multiple rooms, obstacles, and objects for planning:

```xml
<!-- ~/.gazebo/worlds/planning_test_world.world -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="planning_test_world">
    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Add lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Create rooms with doors -->
    <model name="room_walls">
      <!-- Outer walls -->
      <link name="outer_wall_north">
        <pose>0 5 1 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box><size>10 0.2 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>10 0.2 2</size></box>
          </geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material>
        </visual>
      </link>
      <link name="outer_wall_south">
        <pose>0 -5 1 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box><size>10 0.2 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>10 0.2 2</size></box>
          </geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material>
        </visual>
      </link>
      <link name="outer_wall_east">
        <pose>5 0 1 0 0 1.57</pose>
        <collision name="collision">
          <geometry>
            <box><size>10 0.2 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>10 0.2 2</size></box>
          </geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material>
        </visual>
      </link>
      <link name="outer_wall_west">
        <pose>-5 0 1 0 0 1.57</pose>
        <collision name="collision">
          <geometry>
            <box><size>10 0.2 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>10 0.2 2</size></box>
          </geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material>
        </visual>
      </link>

      <!-- Interior walls with doors -->
      <link name="interior_wall_1">
        <pose>-2 0 1 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box><size>0.2 4 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.2 4 2</size></box>
          </geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient></material>
        </visual>
      </link>

      <!-- Door opening in interior wall -->
      <link name="interior_wall_2">
        <pose>2 0 1 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box><size>0.2 4 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.2 4 2</size></box>
          </geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Add objects for manipulation planning -->
    <model name="table_1">
      <pose>-3 -3 0.4 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box><size>1.0 0.8 0.8</size></box>
          </geometry>
          <material><ambient>0.8 0.6 0.4 1</ambient></material>
        </visual>
        <collision name="collision">
          <geometry>
            <box><size>1.0 0.8 0.8</size></box>
          </geometry>
        </collision>
      </link>
    </model>

    <model name="object_1">
      <pose>-3 -2.5 0.9 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder><radius>0.05</radius><length>0.1</length></cylinder>
          </geometry>
          <material><ambient>1 0 0 1</ambient></material>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder><radius>0.05</radius><length>0.1</length></cylinder>
          </geometry>
        </collision>
      </link>
    </model>

    <model name="table_2">
      <pose>3 3 0.4 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box><size>1.0 0.8 0.8</size></box>
          </geometry>
          <material><ambient>0.8 0.6 0.4 1</ambient></material>
        </visual>
        <collision name="collision">
          <geometry>
            <box><size>1.0 0.8 0.8</size></box>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Add a robot -->
    <include>
      <uri>model://turtlebot3_waffle</uri>
      <pose>-4 -4 0.1 0 0 0</pose>
    </include>
  </world>
</sdf>

## Part 2: Isaac Sim Planning Environment (Alternative)

### Task 2.1: Create Isaac Sim Planning Scene
If using Isaac Sim, create a more complex planning environment:

```python
# Create a Python script for Isaac Sim planning environment
# cognitive_planning_simulation.py

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.franka import Franka
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid
from omni.isaac.range_sensor import _range_sensor
import numpy as np

class PlanningWorld(World):
    def __init__(self):
        super().__init__(stage_units_in_meters=1.0)

        # Add a ground plane
        self.scene.add_default_ground_plane()

        # Create room boundaries
        self.walls = []
        # North wall
        self.walls.append(FixedCuboid(
            prim_path="/World/NorthWall",
            name="north_wall",
            position=[0.0, 5.0, 1.0],
            size=0.2,
            color=[0.5, 0.5, 0.5]
        ))
        # South wall
        self.walls.append(FixedCuboid(
            prim_path="/World/SouthWall",
            name="south_wall",
            position=[0.0, -5.0, 1.0],
            size=0.2,
            color=[0.5, 0.5, 0.5]
        ))
        # East wall
        self.walls.append(FixedCuboid(
            prim_path="/World/EastWall",
            name="east_wall",
            position=[5.0, 0.0, 1.0],
            size=0.2,
            color=[0.5, 0.5, 0.5]
        ))
        # West wall
        self.walls.append(FixedCuboid(
            prim_path="/World/WestWall",
            name="west_wall",
            position=[-5.0, 0.0, 1.0],
            size=0.2,
            color=[0.5, 0.5, 0.5]
        ))

        # Interior walls with gaps for doors
        self.interior_walls = []
        self.interior_walls.append(FixedCuboid(
            prim_path="/World/InteriorWall1",
            name="interior_wall_1",
            position=[-2.0, 0.0, 1.0],
            size=[0.2, 4.0, 2.0],
            color=[0.6, 0.6, 0.6]
        ))
        self.interior_walls.append(FixedCuboid(
            prim_path="/World/InteriorWall2",
            name="interior_wall_2",
            position=[2.0, 0.0, 1.0],
            size=[0.2, 4.0, 2.0],
            color=[0.6, 0.6, 0.6]
        ))

        # Add objects for planning tasks
        self.objects = []
        self.objects.append(DynamicCuboid(
            prim_path="/World/Object1",
            name="object_1",
            position=[-3.0, -2.5, 0.9],
            size=0.1,
            color=[1.0, 0.0, 0.0]
        ))

        # Add robot
        self.franka = Franka(
            prim_path="/World/Franka",
            name="franka",
            position=[-4.0, -4.0, 0.0],
            orientation=[1.0, 0.0, 0.0, 0.0]
        )

        # Add LIDAR sensor for mapping
        self.lidar = _range_sensor.acquire_lidar_sensor_interface()
        self.lidar.add_ground_truth_to_stage()
        self.lidar.add_lidar_to_stage(
            prim_path="/World/Franka/base_link/Lidar",
            translation=np.array([0.0, 0.0, 0.5]),
            orientation=np.array([0.0, 0.0, 0.0, 1.0])
        )

# Initialize the world
world = PlanningWorld()
world.reset()

# Run simulation
for i in range(100):
    world.step(render=True)

# Cleanup
world.clear()
```

## Part 3: Connecting Simulation to Planning System

### Task 3.1: Navigation Stack Configuration
Configure the ROS 2 navigation stack for the simulation environment:

```yaml
# cognitive_planning_simulation/config/nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 10.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_delay: 0.5
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.1
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: "map"
    robot_base_frame: "base_link"
    odom_topic: "/odom"
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Specify the path to the behavior tree XML file
    default_nav_through_poses_bt_xml: "nav2_bt_xml_v0/navigate_through_poses_w_replanning_and_recovery.xml"
    default_nav_to_pose_bt_xml: "nav2_bt_xml_v0/navigate_to_pose_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_smooth_path_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_assisted_teleop_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_drive_on_heading_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_truncate_path_local_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    - nav2_controller_cancel_bt_node
    - nav2_path_longer_on_approach_bt_node
    - nav2_wait_cancel_bt_node
    - nav2_spin_cancel_bt_node
    - nav2_back_up_cancel_bt_node
    - nav2_assisted_teleop_cancel_bt_node
    - nav2_drive_on_heading_cancel_bt_node

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # Goal checker parameters
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    # Controller parameters
    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      primary_controller: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      rotation_shim:
        plugin: "nav2_controller::SimplePureRotator"
        max_angular_accel: 1.0
        max_angular_velocity: 0.75
        goal_tolerance: 0.1
      regulated_pure_pursuit:
        plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
        desired_linear_vel: 0.5
        max_linear_accel: 2.5
        max_linear_decel: 2.5
        desired_angular_vel: 1.5
        max_angular_accel: 3.2
        min_turn_radius: 0.0
        lookahead_dist: 0.6
        lookahead_time: 1.5
        transform_tolerance: 0.1
        linear_heading_threshold: 0.75
        angular_dist_threshold: 0.75
        rotate_to_heading_angular_vel: 1.0
        use_velocity_scaled_lookahead_dist: false
        min_lookahead_dist: 0.3
        max_lookahead_dist: 0.9
        use_interpolation: true
        use_regulated_linear_velocity_scaling: true
        use_cost_regulated_linear_velocity_scaling: true
        regulated_linear_scaling_min_radius: 0.9
        regulated_linear_scaling_min_speed: 0.25
        use_rotate_to_heading: true
        rotate_to_heading_min_angle: 0.785
        max_angular_vel: 1.5
        min_linear_vel: 0.0
        max_linear_vel: 0.5

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: "odom"
      robot_base_frame: "base_link"
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      always_send_full_costmap: False
  local_costmap_client:
    ros__parameters:
      use_sim_time: True
  local_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: "map"
      robot_base_frame: "base_link"
      use_sim_time: True
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True
  global_costmap_client:
    ros__parameters:
      use_sim_time: True
  global_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: True

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

behavior_server:
  ros__parameters:
    costmap_topic: "local_costmap/costmap_raw"
    footprint_topic: "local_costmap/published_footprint"
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
      spin_dist: 1.57
    backup:
      plugin: "nav2_behaviors::BackUp"
      backup_dist: 0.15
      backup_speed: 0.025
    wait:
      plugin: "nav2_behaviors::Wait"
      wait_duration: 1.0
```

### Task 3.2: Launch Planning Simulation
Create a launch file to start the complete planning simulation:

```python
# cognitive_planning_simulation/launch/planning_simulation.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='planning_test_world.world')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('cognitive_planning_simulation'),
                'worlds',
                world
            ]),
            'verbose': 'true'
        }.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': '<robot name="turtlebot3_waffle">' +
                               '<link name="base_link">' +
                               '<visual>' +
                               '<geometry><box size="0.3 0.3 0.1"/></geometry>' +
                               '</visual>' +
                               '</link>' +
                               '</robot>'
        }]
    )

    # Navigation stack
    navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([
                FindPackageShare('cognitive_planning_simulation'),
                'config',
                'nav2_params.yaml'
            ])
        }.items()
    )

    # Task planner
    task_planner = Node(
        package='task_planning',
        executable='simple_planner',
        name='simple_planner',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Motion planner
    motion_planner = Node(
        package='motion_planning',
        executable='motion_planner',
        name='motion_planner',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Behavior tree planner
    behavior_tree_planner = Node(
        package='hierarchical_planner',
        executable='behavior_tree_planner',
        name='behavior_tree_planner',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Uncertainty planner
    uncertainty_planner = Node(
        package='hierarchical_planner',
        executable='uncertainty_planner',
        name='uncertainty_planner',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Hybrid planner
    hybrid_planner = Node(
        package='hierarchical_planner',
        executable='hybrid_planner',
        name='hybrid_planner',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=use_sim_time),
        gazebo,
        robot_state_publisher,
        navigation2,
        task_planner,
        motion_planner,
        behavior_tree_planner,
        uncertainty_planner,
        hybrid_planner,
    ])
```

## Part 4: Testing Planning Algorithms in Simulation

### Task 4.1: Create Planning Test Scenarios
Create test scenarios to validate different planning approaches:

```python
#!/usr/bin/env python3
# cognitive_planning_simulation/cognitive_planning_simulation/planning_tester.py

import rclpy
from rclpy.node import Node
from cognitive_planning_interfaces.msg import TaskAction
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import Path
from std_msgs.msg import String
import time
import random

class PlanningTester(Node):
    def __init__(self):
        super().__init__('planning_tester')

        # Publishers
        self.task_pub = self.create_publisher(TaskAction, 'high_level_task', 10)
        self.goal_pub = self.create_publisher(Pose, 'move_base_simple/goal', 10)
        self.test_status_pub = self.create_publisher(String, 'test_status', 10)

        # Timers for different tests
        self.test_timer = self.create_timer(15.0, self.run_next_test)
        self.test_count = 0

        # Define test scenarios
        self.test_scenarios = [
            self.test_simple_navigation,
            self.test_complex_navigation,
            self.test_task_planning,
            self.test_reactive_behavior,
            self.test_uncertainty_handling
        ]

        self.get_logger().info('Planning Tester node initialized')

    def run_next_test(self):
        if self.test_count < len(self.test_scenarios):
            scenario = self.test_scenarios[self.test_count]
            scenario_name = scenario.__name__

            self.get_logger().info(f'Running test scenario: {scenario_name}')

            status_msg = String()
            status_msg.data = f'Running {scenario_name}'
            self.test_status_pub.publish(status_msg)

            # Execute the test
            scenario()

            self.test_count += 1
        else:
            self.get_logger().info('All planning tests completed')
            status_msg = String()
            status_msg.data = 'All tests completed'
            self.test_status_pub.publish(status_msg)

    def test_simple_navigation(self):
        """Test simple navigation to a goal"""
        self.get_logger().info('Testing simple navigation')

        goal = Pose()
        goal.position.x = 4.0
        goal.position.y = 4.0
        goal.position.z = 0.0
        goal.orientation.w = 1.0

        self.goal_pub.publish(goal)
        time.sleep(1.0)

    def test_complex_navigation(self):
        """Test navigation through complex environment with obstacles"""
        self.get_logger().info('Testing complex navigation')

        # Navigate through the complex environment
        goals = [
            (0.0, 0.0),  # Start position
            (2.0, 0.0),  # Go through door
            (4.0, 3.0),  # Go to other side of room
        ]

        for x, y in goals:
            goal = Pose()
            goal.position.x = x
            goal.position.y = y
            goal.position.z = 0.0
            goal.orientation.w = 1.0

            self.goal_pub.publish(goal)
            time.sleep(3.0)  # Wait for navigation to progress

    def test_task_planning(self):
        """Test high-level task planning"""
        self.get_logger().info('Testing task planning')

        # Publish a complex task
        task = TaskAction()
        task.action_type = "move_and_pickup"
        task.parameters = "object_1 location_1 location_2"
        task.description = "Pick up object_1 from location_1 and move it to location_2"

        self.task_pub.publish(task)

    def test_reactive_behavior(self):
        """Test reactive behavior in dynamic environment"""
        self.get_logger().info('Testing reactive behavior')

        # Move to a location where obstacles might appear
        goal = Pose()
        goal.position.x = 0.0
        goal.position.y = 0.0
        goal.position.z = 0.0
        goal.orientation.w = 1.0

        self.goal_pub.publish(goal)

    def test_uncertainty_handling(self):
        """Test planning under uncertainty"""
        self.get_logger().info('Testing uncertainty handling')

        # Create a scenario where localization uncertainty might be high
        goal = Pose()
        goal.position.x = -3.0
        goal.position.y = -3.0
        goal.position.z = 0.0
        goal.orientation.w = 1.0

        self.goal_pub.publish(goal)

def main(args=None):
    rclpy.init(args=args)
    node = PlanningTester()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Task 4.2: Visualization and Monitoring
Create visualization tools to monitor planning performance:

```python
#!/usr/bin/env python3
# cognitive_planning_simulation/cognitive_planning_simulation/planning_visualizer.py

import rclpy
from rclpy.node import Node
from cognitive_planning_interfaces.msg import TaskPlan, MotionPlan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math

class PlanningVisualizer(Node):
    def __init__(self):
        super().__init__('planning_visualizer')

        # Subscribers
        self.task_plan_sub = self.create_subscription(
            TaskPlan, 'task_plan', self.task_plan_callback, 10)
        self.motion_plan_sub = self.create_subscription(
            MotionPlan, 'motion_plan', self.motion_plan_callback, 10)

        # Publishers
        self.task_viz_pub = self.create_publisher(Marker, 'task_plan_viz', 10)
        self.motion_viz_pub = self.create_publisher(Marker, 'motion_plan_viz', 10)

        self.get_logger().info('Planning Visualizer node initialized')

    def task_plan_callback(self, msg):
        """Visualize task plan"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "task_plan"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.0
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.text = f"Task Plan: {len(msg.actions)} actions"
        self.task_viz_pub.publish(marker)

    def motion_plan_callback(self, msg):
        """Visualize motion plan"""
        # Create path visualization
        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = "motion_path"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD

        path_marker.pose.orientation.w = 1.0
        path_marker.scale.x = 0.05
        path_marker.color.r = 0.0
        path_marker.color.g = 1.0
        path_marker.color.b = 0.0
        path_marker.color.a = 0.8

        for pose in msg.path:
            point = Point()
            point.x = pose.position.x
            point.y = pose.position.y
            point.z = 0.05  # Slightly above ground
            path_marker.points.append(point)

        self.motion_viz_pub.publish(path_marker)

        # Create start/end markers
        if len(msg.path) > 0:
            start_marker = Marker()
            start_marker.header.frame_id = "map"
            start_marker.header.stamp = self.get_clock().now().to_msg()
            start_marker.ns = "motion_start"
            start_marker.id = 1
            start_marker.type = Marker.SPHERE
            start_marker.action = Marker.ADD

            start_marker.pose.position = msg.path[0].position
            start_marker.pose.orientation.w = 1.0
            start_marker.scale.x = 0.2
            start_marker.scale.y = 0.2
            start_marker.scale.z = 0.2
            start_marker.color.r = 0.0
            start_marker.color.g = 0.0
            start_marker.color.b = 1.0
            start_marker.color.a = 1.0

            end_marker = Marker()
            end_marker.header.frame_id = "map"
            end_marker.header.stamp = self.get_clock().now().to_msg()
            end_marker.ns = "motion_end"
            end_marker.id = 2
            end_marker.type = Marker.SPHERE
            end_marker.action = Marker.ADD

            end_marker.pose.position = msg.path[-1].position
            end_marker.pose.orientation.w = 1.0
            end_marker.scale.x = 0.2
            end_marker.scale.y = 0.2
            end_marker.scale.z = 0.2
            end_marker.color.r = 1.0
            end_marker.color.g = 0.0
            end_marker.color.b = 0.0
            end_marker.color.a = 1.0

            self.motion_viz_pub.publish(start_marker)
            self.motion_viz_pub.publish(end_marker)

def main(args=None):
    rclpy.init(args=args)
    node = PlanningVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 5: Running the Planning Simulation

### Task 5.1: Build and Run
```bash
# Create the simulation package
cd ~/cognitive_planning_ws/src
ros2 pkg create --build-type ament_python cognitive_planning_simulation

# Build the workspace
cd ~/cognitive_planning_ws
colcon build --packages-select cognitive_planning_interfaces task_planning motion_planning hierarchical_planner cognitive_planning_simulation

# Source the workspace
source install/setup.bash

# Run the complete planning simulation system
ros2 launch cognitive_planning_simulation planning_simulation.launch.py
```

### Task 5.2: Monitor Planning Performance
Monitor the planning system performance:

```bash
# Monitor task plans
ros2 topic echo /task_plan

# Monitor motion plans
ros2 topic echo /motion_plan

# Monitor planning status
ros2 topic echo /test_status

# Visualize in RViz
ros2 run rviz2 rviz2 -d `ros2 pkg prefix cognitive_planning_simulation`/share/cognitive_planning_simulation/rviz/planning.rviz
```

## Exercises

### Exercise 1: Add Dynamic Obstacles
Extend the simulation environment with dynamic obstacles:
- Moving objects that change positions over time
- People walking through the environment
- Doors that open and close

### Exercise 2: Implement RRT* Algorithm
Replace the basic A* planner with an RRT* planner:
- Implement the RRT* algorithm
- Compare performance with A*
- Visualize the improvement over time

### Exercise 3: Multi-Robot Planning
Extend the system for multi-robot planning:
- Add a second robot to the simulation
- Implement coordination mechanisms
- Handle inter-robot collision avoidance

## Summary

In this simulation lab, you created a comprehensive cognitive planning environment with:
1. Complex simulation world with obstacles and rooms
2. Navigation stack integration
3. Task and motion planning systems
4. Uncertainty handling
5. Reactive-deliberative planning integration
6. Visualization and monitoring tools

The simulation provides a controlled environment to test and validate cognitive planning algorithms before deployment on real robots.

## Next Steps

- Integrate with real robot hardware
- Add more sophisticated planning algorithms
- Implement learning-based planning
- Test in more complex and dynamic environments