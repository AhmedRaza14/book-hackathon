---
title: "Week 14: Navigation and Manipulation - Code Lab"
week: 14
module: "Applications and Projects"
difficulty: "advanced"
prerequisites: ["human-robot-interaction", "motion-planning", "control-systems", "sensor-integration"]
learning_objectives:
  - "Implement autonomous navigation systems"
  - "Create manipulation planning algorithms"
  - "Integrate navigation and manipulation"
  - "Test mobile manipulation in simulation"
tags: ["navigation", "manipulation", "path-planning", "grasping", "mobile-robots", "manipulators", "slam"]
hardware_requirements:
  - gpu: "RTX 4070 or higher"
  - ram: "32GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "120 minutes"
---

# Week 14: Navigation and Manipulation - Code Lab

## Learning Objectives
- Implement autonomous navigation systems
- Create manipulation planning algorithms
- Integrate navigation and manipulation
- Test mobile manipulation in simulation

## Prerequisites
- ROS 2 Humble Hawksbill installed
- Python 3.8+ with pip
- MoveIt2 for manipulation planning
- Navigation2 for navigation
- Completed Week 13 materials

## Setup Environment

### Install Required Dependencies
```bash
# Create a new workspace
mkdir -p ~/nav_manip_ws/src
cd ~/nav_manip_ws

# Install navigation and manipulation dependencies
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-moveit ros-humble-moveit-ros
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```

### ROS 2 Package Structure
```bash
# Create ROS 2 packages
cd ~/nav_manip_ws/src
ros2 pkg create --build-type ament_python navigation_system
ros2 pkg create --build-type ament_python manipulation_planning
ros2 pkg create --build-type ament_python mobile_manipulation
ros2 pkg create --build-type ament_python nav_manip_interfaces
```

## Part 1: Navigation System Implementation

### Task 1.1: Create a Navigation Manager
Implement a system that manages navigation tasks:

```python
#!/usr/bin/env python3
# navigation_system/navigation_system/navigation_manager.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
import numpy as np
from scipy.spatial import distance

class NavigationManager(Node):
    def __init__(self):
        super().__init__('navigation_manager')

        # Publishers and subscribers
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.path_sub = self.create_subscription(
            Path, 'plan', self.path_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10)
        self.status_pub = self.create_publisher(String, 'navigation_status', 10)

        # Action client for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Internal state
        self.current_map = None
        self.current_scan = None
        self.current_path = None
        self.navigation_active = False
        self.goal_pose = None

        # Navigation parameters
        self.min_obstacle_distance = 0.5  # meters
        self.replan_threshold = 0.3  # meters

        self.get_logger().info('Navigation Manager initialized')

    def map_callback(self, msg):
        """Process occupancy grid map"""
        self.current_map = msg
        self.get_logger().info('Map received')

    def scan_callback(self, msg):
        """Process laser scan for obstacle detection"""
        self.current_scan = msg

        # Check for obstacles in path
        if self.current_path and self.navigation_active:
            if self.is_path_blocked():
                self.get_logger().warn('Path blocked, replanning needed')
                self.publish_status('path_blocked')

    def path_callback(self, msg):
        """Process planned path"""
        self.current_path = msg
        self.get_logger().info(f'Path received with {len(msg.poses)} waypoints')

    def is_path_blocked(self):
        """Check if current path is blocked by obstacles"""
        if not self.current_scan or not self.current_path:
            return False

        # Check if any obstacles are too close to the path
        for i in range(0, len(self.current_path.poses), 5):  # Check every 5th point
            path_point = self.current_path.poses[i].pose.position

            # Check laser readings around this path point
            for range_val in self.current_scan.ranges:
                if not (math.isnan(range_val) or math.isinf(range_val)) and range_val < self.min_obstacle_distance:
                    return True

        return False

    def send_navigation_goal(self, x, y, theta=0.0):
        """Send navigation goal to the system"""
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Navigation action server not available')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        # Set orientation (theta in radians)
        from math import sin, cos
        goal_msg.pose.pose.orientation.z = sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = cos(theta / 2.0)

        self.goal_pose = goal_msg.pose.pose
        self.navigation_active = True

        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

        self.get_logger().info(f'Navigation goal sent to ({x}, {y})')
        return True

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.navigation_active = False
            return

        self.get_logger().info('Goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.navigation_active = False
        self.get_logger().info(f'Navigation result: {result}')
        self.publish_status('navigation_completed')

    def publish_status(self, status):
        """Publish navigation status"""
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationManager()

    # Example: Send a navigation goal after a delay
    def send_example_goal():
        node.send_navigation_goal(2.0, 2.0)

    # Schedule example goal after 5 seconds
    node.create_timer(5.0, send_example_goal)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 2: Manipulation Planning System

### Task 2.1: Create a Manipulation Planner
Implement a system for planning manipulation tasks:

```python
#!/usr/bin/env python3
# manipulation_planning/manipulation_planning/manipulation_planner.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Vector3
from std_msgs.msg import String
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker
import numpy as np
from scipy.spatial.transform import Rotation as R

class ManipulationPlanner(Node):
    def __init__(self):
        super().__init__('manipulation_planner')

        # Publishers and subscribers
        self.object_pub = self.create_publisher(CollisionObject, 'collision_object', 10)
        self.marker_pub = self.create_publisher(Marker, 'grasp_visualization', 10)
        self.command_sub = self.create_subscription(
            String, 'manipulation_command', self.command_callback, 10)

        # Services
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        self.fk_client = self.create_client(GetPositionFK, 'compute_fk')

        # Internal state
        self.robot_state = None
        self.objects_in_environment = []

        # Wait for services
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('IK service not available, waiting...')

        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('FK service not available, waiting...')

        self.get_logger().info('Manipulation Planner initialized')

    def command_callback(self, msg):
        """Process manipulation commands"""
        command_data = msg.data.split()

        if len(command_data) < 2:
            self.get_logger().error(f'Invalid command: {msg.data}')
            return

        command = command_data[0]

        if command == 'pick':
            if len(command_data) >= 4:
                x, y, z = float(command_data[1]), float(command_data[2]), float(command_data[3])
                self.pick_object(x, y, z)
        elif command == 'place':
            if len(command_data) >= 4:
                x, y, z = float(command_data[1]), float(command_data[2]), float(command_data[3])
                self.place_object(x, y, z)
        elif command == 'move_to':
            if len(command_data) >= 4:
                x, y, z = float(command_data[1]), float(command_data[2]), float(command_data[3])
                self.move_to_position(x, y, z)

    def pick_object(self, x, y, z):
        """Plan and execute pick action"""
        self.get_logger().info(f'Planning to pick object at ({x}, {y}, {z})')

        # Plan approach pose (slightly above object)
        approach_pose = Pose()
        approach_pose.position.x = x
        approach_pose.position.y = y
        approach_pose.position.z = z + 0.1  # 10cm above object
        approach_pose.orientation.w = 1.0

        # Plan grasp pose (at object height)
        grasp_pose = Pose()
        grasp_pose.position.x = x
        grasp_pose.position.y = y
        grasp_pose.position.z = z + 0.05  # 5cm above base of object
        grasp_pose.orientation.w = 1.0

        # Calculate inverse kinematics for both poses
        approach_joints = self.calculate_ik(approach_pose)
        grasp_joints = self.calculate_ik(grasp_pose)

        if approach_joints and grasp_joints:
            self.get_logger().info('Pick trajectory calculated successfully')
            # In a real system, this would execute the trajectory
            self.execute_pick_trajectory(approach_joints, grasp_joints)
        else:
            self.get_logger().error('Could not calculate valid joint positions for pick')

    def place_object(self, x, y, z):
        """Plan and execute place action"""
        self.get_logger().info(f'Planning to place object at ({x}, {y}, {z})')

        # Plan place pose
        place_pose = Pose()
        place_pose.position.x = x
        place_pose.position.y = y
        place_pose.position.z = z + 0.05  # 5cm above placement surface
        place_pose.orientation.w = 1.0

        # Calculate inverse kinematics
        place_joints = self.calculate_ik(place_pose)

        if place_joints:
            self.get_logger().info('Place trajectory calculated successfully')
            # In a real system, this would execute the trajectory
            self.execute_place_trajectory(place_joints)
        else:
            self.get_logger().error('Could not calculate valid joint positions for place')

    def move_to_position(self, x, y, z):
        """Move end-effector to specific position"""
        self.get_logger().info(f'Moving to position ({x}, {y}, {z})')

        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.w = 1.0

        joints = self.calculate_ik(target_pose)

        if joints:
            self.get_logger().info('Movement trajectory calculated successfully')
            # In a real system, this would execute the trajectory
            self.execute_movement_trajectory(joints)
        else:
            self.get_logger().error('Could not calculate valid joint positions for movement')

    def calculate_ik(self, pose):
        """Calculate inverse kinematics for given pose"""
        try:
            request = GetPositionIK.Request()
            request.ik_request.group_name = 'manipulator'  # Adjust for your robot
            request.ik_request.pose_stamped.header.frame_id = 'base_link'
            request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
            request.ik_request.pose_stamped.pose = pose

            future = self.ik_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            result = future.result()
            if result and result.error_code.val == 1:  # SUCCESS
                return result.solution.joint_state.position
            else:
                self.get_logger().warn(f'IK failed with error code: {result.error_code.val}')
                return None
        except Exception as e:
            self.get_logger().error(f'IK calculation failed: {str(e)}')
            return None

    def execute_pick_trajectory(self, approach_joints, grasp_joints):
        """Execute pick trajectory (simulation)"""
        self.get_logger().info('Executing pick trajectory')
        # This would send the trajectory to the robot controller
        # For simulation, just log the steps
        self.get_logger().info(f'Approach joints: {approach_joints}')
        self.get_logger().info(f'Grasp joints: {grasp_joints}')

    def execute_place_trajectory(self, place_joints):
        """Execute place trajectory (simulation)"""
        self.get_logger().info('Executing place trajectory')
        self.get_logger().info(f'Place joints: {place_joints}')

    def execute_movement_trajectory(self, joints):
        """Execute movement trajectory (simulation)"""
        self.get_logger().info('Executing movement trajectory')
        self.get_logger().info(f'Movement joints: {joints}')

    def visualize_grasp_pose(self, pose, grasp_type='power'):
        """Visualize grasp pose in RViz"""
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'grasp_pose'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose = pose
        marker.scale.x = 0.1  # shaft diameter
        marker.scale.y = 0.02  # head diameter
        marker.scale.z = 0.02  # head length

        # Color based on grasp type
        if grasp_type == 'power':
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:  # precision
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

        marker.color.a = 1.0

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = ManipulationPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 3: Mobile Manipulation Integration

### Task 3.1: Create a Mobile Manipulation Coordinator
Implement a system that coordinates navigation and manipulation:

```python
#!/usr/bin/env python3
# mobile_manipulation/mobile_manipulation/mobile_manipulation_coordinator.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time
import json

class MobileManipulationCoordinator(Node):
    def __init__(self):
        super().__init__('mobile_manipulation_coordinator')

        # Publishers and subscribers
        self.command_sub = self.create_subscription(
            String, 'mobile_manip_command', self.command_callback, 10)
        self.status_pub = self.create_publisher(String, 'mobile_manip_status', 10)
        self.nav_command_pub = self.create_publisher(String, 'manipulation_command', 10)

        # Action client for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Internal state
        self.navigation_active = False
        self.manipulation_active = False
        self.current_task = None
        self.robot_pose = None

        self.get_logger().info('Mobile Manipulation Coordinator initialized')

    def command_callback(self, msg):
        """Process mobile manipulation commands"""
        try:
            command_data = json.loads(msg.data)
            task_type = command_data.get('type')
            params = command_data.get('parameters', {})

            if task_type == 'fetch_object':
                self.execute_fetch_task(params)
            elif task_type == 'deliver_object':
                self.execute_deliver_task(params)
            elif task_type == 'inspect_area':
                self.execute_inspect_task(params)
            else:
                self.get_logger().error(f'Unknown task type: {task_type}')
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON command: {msg.data}')

    def execute_fetch_task(self, params):
        """Execute fetch object task"""
        self.get_logger().info('Executing fetch object task')

        # Parameters: navigation_goal, object_position
        nav_goal = params.get('navigation_goal')
        obj_pos = params.get('object_position')

        if not nav_goal or not obj_pos:
            self.get_logger().error('Missing required parameters for fetch task')
            return

        # Step 1: Navigate to object location
        self.get_logger().info(f'Navigating to object location: {nav_goal}')
        self.current_task = 'fetch'
        self.navigation_active = True

        # Send navigation goal
        nav_success = self.send_navigation_goal(nav_goal['x'], nav_goal['y'], nav_goal.get('theta', 0.0))

        if nav_success:
            # Wait for navigation to complete
            self.wait_for_navigation_completion()

            # Step 2: Manipulate object
            self.get_logger().info(f'Picking up object at: {obj_pos}')
            self.manipulation_active = True

            # Send manipulation command
            cmd_msg = String()
            cmd_msg.data = f"pick {obj_pos['x']} {obj_pos['y']} {obj_pos['z']}"
            self.nav_command_pub.publish(cmd_msg)

            # Update status
            self.publish_status('fetch_completed')
        else:
            self.publish_status('navigation_failed')

    def execute_deliver_task(self, params):
        """Execute deliver object task"""
        self.get_logger().info('Executing deliver object task')

        # Parameters: navigation_goal, placement_position
        nav_goal = params.get('navigation_goal')
        place_pos = params.get('placement_position')

        if not nav_goal or not place_pos:
            self.get_logger().error('Missing required parameters for deliver task')
            return

        # Step 1: Navigate to delivery location
        self.get_logger().info(f'Navigating to delivery location: {nav_goal}')
        self.current_task = 'deliver'
        self.navigation_active = True

        # Send navigation goal
        nav_success = self.send_navigation_goal(nav_goal['x'], nav_goal['y'], nav_goal.get('theta', 0.0))

        if nav_success:
            # Wait for navigation to complete
            self.wait_for_navigation_completion()

            # Step 2: Place object
            self.get_logger().info(f'Placing object at: {place_pos}')
            self.manipulation_active = True

            # Send manipulation command
            cmd_msg = String()
            cmd_msg.data = f"place {place_pos['x']} {place_pos['y']} {place_pos['z']}"
            self.nav_command_pub.publish(cmd_msg)

            # Update status
            self.publish_status('delivery_completed')
        else:
            self.publish_status('navigation_failed')

    def execute_inspect_task(self, params):
        """Execute area inspection task"""
        self.get_logger().info('Executing area inspection task')

        # Parameters: navigation_goals (list of positions to visit)
        nav_goals = params.get('navigation_goals', [])

        if not nav_goals:
            self.get_logger().error('No navigation goals provided for inspection task')
            return

        self.get_logger().info(f'Inspecting {len(nav_goals)} locations')

        for i, goal in enumerate(nav_goals):
            self.get_logger().info(f'Navigating to inspection point {i+1}/{len(nav_goals)}: {goal}')

            # Send navigation goal
            nav_success = self.send_navigation_goal(goal['x'], goal['y'], goal.get('theta', 0.0))

            if nav_success:
                self.wait_for_navigation_completion()
                # In a real system, this would trigger sensor data collection
                time.sleep(2)  # Simulate inspection time
            else:
                self.get_logger().error(f'Failed to reach inspection point {i+1}')
                break

        self.publish_status('inspection_completed')

    def send_navigation_goal(self, x, y, theta=0.0):
        """Send navigation goal to the system"""
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Navigation action server not available')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        # Set orientation
        from math import sin, cos
        goal_msg.pose.pose.orientation.z = sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = cos(theta / 2.0)

        self.get_logger().info(f'Sending navigation goal to ({x}, {y})')

        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.navigation_goal_response_callback)

        return True

    def navigation_goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected')
            self.navigation_active = False
            return

        self.get_logger().info('Navigation goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.navigation_get_result_callback)

    def navigation_get_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.navigation_active = False
        self.get_logger().info(f'Navigation completed with result: {result}')

        # If we were in a fetch/deliver task, continue with manipulation
        if self.current_task in ['fetch', 'deliver']:
            self.continue_with_manipulation()

    def wait_for_navigation_completion(self):
        """Wait for navigation to complete (simplified)"""
        # In a real implementation, this would properly wait for the action to complete
        # For this example, we'll just wait a bit and assume it's done
        time.sleep(1)

    def continue_with_manipulation(self):
        """Continue with manipulation after navigation"""
        # This would be called after navigation completes
        # The actual manipulation commands are sent from the navigation callbacks
        pass

    def publish_status(self, status):
        """Publish mobile manipulation status"""
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MobileManipulationCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 4: Mobile Manipulation Task Executor

### Task 4.1: Create a Task Executor
Implement a system that executes complex mobile manipulation tasks:

```python
#!/usr/bin/env python3
# mobile_manipulation/mobile_manipulation/task_executor.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import json
import time
from enum import Enum

class TaskState(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    MANIPULATING = "manipulating"
    COMPLETED = "completed"
    FAILED = "failed"

class TaskExecutor(Node):
    def __init__(self):
        super().__init__('task_executor')

        # Publishers and subscribers
        self.command_sub = self.create_subscription(
            String, 'complex_task', self.task_callback, 10)
        self.status_pub = self.create_publisher(String, 'task_status', 10)
        self.nav_command_pub = self.create_publisher(String, 'mobile_manip_command', 10)
        self.manip_command_pub = self.create_publisher(String, 'manipulation_command', 10)

        # Internal state
        self.current_task = None
        self.task_state = TaskState.IDLE
        self.task_queue = []
        self.task_history = []

        # Timer for task monitoring
        self.task_timer = self.create_timer(1.0, self.monitor_task)

        self.get_logger().info('Task Executor initialized')

    def task_callback(self, msg):
        """Process complex task request"""
        try:
            task_data = json.loads(msg.data)
            task_id = task_data.get('id', f'task_{int(time.time())}')
            task_type = task_data.get('type')
            parameters = task_data.get('parameters', {})

            task = {
                'id': task_id,
                'type': task_type,
                'parameters': parameters,
                'created_time': time.time()
            }

            self.get_logger().info(f'Received task: {task_type} (ID: {task_id})')

            # Add to queue if idle, otherwise queue for later
            if self.task_state == TaskState.IDLE:
                self.execute_task(task)
            else:
                self.task_queue.append(task)
                self.get_logger().info(f'Task queued. Queue size: {len(self.task_queue)}')

        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON task: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error processing task: {str(e)}')

    def execute_task(self, task):
        """Execute a complex task"""
        self.current_task = task
        self.task_state = TaskState.NAVIGATING

        self.get_logger().info(f'Executing task: {task["type"]} (ID: {task["id"]})')

        if task['type'] == 'fetch_and_deliver':
            self.execute_fetch_and_deliver(task['parameters'])
        elif task['type'] == 'inspect_and_report':
            self.execute_inspect_and_report(task['parameters'])
        elif task['type'] == 'assemble_parts':
            self.execute_assemble_parts(task['parameters'])
        else:
            self.get_logger().error(f'Unknown task type: {task["type"]}')
            self.task_state = TaskState.FAILED

    def execute_fetch_and_deliver(self, params):
        """Execute fetch and deliver task"""
        # Create a mobile manipulation command for fetching
        fetch_cmd = {
            'type': 'fetch_object',
            'parameters': {
                'navigation_goal': params['fetch_location'],
                'object_position': params['object_position']
            }
        }

        # Publish fetch command
        cmd_msg = String()
        cmd_msg.data = json.dumps(fetch_cmd)
        self.nav_command_pub.publish(cmd_msg)

        # After fetch, we need to deliver
        # This would be handled by the mobile manipulation coordinator
        # For this example, we'll just log the intention
        self.get_logger().info('Fetch command sent, will deliver after completion')

    def execute_inspect_and_report(self, params):
        """Execute inspection task"""
        # Create a mobile manipulation command for inspection
        inspect_cmd = {
            'type': 'inspect_area',
            'parameters': {
                'navigation_goals': params['inspection_points']
            }
        }

        # Publish inspection command
        cmd_msg = String()
        cmd_msg.data = json.dumps(inspect_cmd)
        self.nav_command_pub.publish(cmd_msg)

    def execute_assemble_parts(self, params):
        """Execute parts assembly task"""
        # This would involve multiple navigation and manipulation steps
        # For this example, we'll break it down into steps
        assembly_steps = params.get('assembly_steps', [])

        for i, step in enumerate(assembly_steps):
            self.get_logger().info(f'Executing assembly step {i+1}/{len(assembly_steps)}')

            # Each step might involve navigation and manipulation
            if step['action'] == 'pick':
                # Navigate to part location and pick it up
                nav_cmd = {
                    'type': 'fetch_object',
                    'parameters': {
                        'navigation_goal': step['navigation_goal'],
                        'object_position': step['object_position']
                    }
                }

                cmd_msg = String()
                cmd_msg.data = json.dumps(nav_cmd)
                self.nav_command_pub.publish(cmd_msg)
            elif step['action'] == 'place':
                # Navigate to assembly location and place part
                nav_cmd = {
                    'type': 'deliver_object',
                    'parameters': {
                        'navigation_goal': step['navigation_goal'],
                        'placement_position': step['placement_position']
                    }
                }

                cmd_msg = String()
                cmd_msg.data = json.dumps(nav_cmd)
                self.nav_command_pub.publish(cmd_msg)

    def monitor_task(self):
        """Monitor task progress"""
        if self.task_state in [TaskState.COMPLETED, TaskState.FAILED]:
            # Task completed, check for next task
            if self.task_queue:
                next_task = self.task_queue.pop(0)
                self.execute_task(next_task)
            else:
                # No more tasks, go back to idle
                if self.current_task:
                    self.task_history.append({
                        'task_id': self.current_task['id'],
                        'status': self.task_state.value,
                        'completed_time': time.time()
                    })
                    self.current_task = None
                self.task_state = TaskState.IDLE

    def publish_task_status(self, task_id, status, details=""):
        """Publish task status"""
        status_data = {
            'task_id': task_id,
            'status': status,
            'details': details,
            'timestamp': time.time()
        }

        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TaskExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 5: Complete Mobile Manipulation System

### Task 5.1: Create Launch File
Create a launch file to start all navigation and manipulation components:

```python
# mobile_manipulation/launch/mobile_manipulation_system.launch.py
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

    # Navigation system
    navigation_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # MoveIt system (simplified)
    # In a real system, you would include MoveIt launch files

    return LaunchDescription([
        # Set parameters
        SetParameter(name='use_sim_time', value=use_sim_time),

        # Navigation system
        navigation_system,

        # Navigation Manager
        Node(
            package='navigation_system',
            executable='navigation_manager',
            name='navigation_manager',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # Manipulation Planner
        Node(
            package='manipulation_planning',
            executable='manipulation_planner',
            name='manipulation_planner',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # Mobile Manipulation Coordinator
        Node(
            package='mobile_manipulation',
            executable='mobile_manipulation_coordinator',
            name='mobile_manipulation_coordinator',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # Task Executor
        Node(
            package='mobile_manipulation',
            executable='task_executor',
            name='task_executor',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
```

## Exercises

### Exercise 1: Implement Dynamic Path Replanning
Enhance the navigation system with dynamic replanning:
- Detect moving obstacles
- Replan path in real-time
- Implement velocity obstacles

### Exercise 2: Add Force Control
Enhance the manipulation system with force control:
- Implement impedance control
- Add tactile feedback
- Create compliant grasping

### Exercise 3: Multi-Object Manipulation
Extend the system for multiple objects:
- Object recognition and localization
- Multi-step manipulation planning
- Task scheduling for multiple objects

## Summary

In this lab, you implemented a complete navigation and manipulation system with:
1. Navigation system with path planning and obstacle avoidance
2. Manipulation planning with grasp synthesis
3. Mobile manipulation coordination
4. Complex task execution framework

The system demonstrates how navigation and manipulation capabilities can be integrated to create mobile manipulator robots capable of complex autonomous behaviors.

## Next Steps

- Deploy on a physical robot platform
- Add more sophisticated manipulation skills
- Implement learning-based approaches
- Test in complex real-world environments