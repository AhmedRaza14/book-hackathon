---
title: "Week 11: Cognitive Planning - Code Lab"
week: 11
module: "AI-Robot Integration"
difficulty: "advanced"
prerequisites: ["motion-planning", "vision-language", "control-systems", "ros2-advanced"]
learning_objectives:
  - "Implement task and motion planning systems"
  - "Create hierarchical planning architectures"
  - "Handle uncertainty in decision making"
  - "Build reactive vs. deliberative systems"
tags: ["planning", "task-planning", "motion-planning", "hierarchical-planning", "decision-making", "uncertainty", "pddl"]
hardware_requirements:
  - gpu: "RTX 4070 or higher"
  - ram: "32GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "120 minutes"
---

# Week 11: Cognitive Planning - Code Lab

## Learning Objectives
- Implement task and motion planning systems
- Create hierarchical planning architectures
- Handle uncertainty in decision making
- Build reactive vs. deliberative systems

## Prerequisites
- ROS 2 Humble Hawksbill installed
- Python 3.8+ with pip
- Understanding of PDDL and planning algorithms
- Basic knowledge of behavior trees
- Familiarity with motion planning concepts

## Setup Environment

### Install Required Dependencies
```bash
# Create a new workspace
mkdir -p ~/cognitive_planning_ws/src
cd ~/cognitive_planning_ws

# Install Python dependencies for planning
pip3 install py_trees
pip3 install pypddl-parser
pip3 install networkx
pip3 install numpy scipy matplotlib
pip3 install transforms3d  # For transformation calculations
```

### ROS 2 Package Structure
```bash
# Create ROS 2 packages
cd ~/cognitive_planning_ws/src
ros2 pkg create --build-type ament_python cognitive_planning_interfaces
ros2 pkg create --build-type ament_python task_planning
ros2 pkg create --build-type ament_python motion_planning
ros2 pkg create --build-type ament_python hierarchical_planner
```

## Part 1: Task Planning Implementation

### Task 1.1: Create PDDL Domain and Problem Files
Create a simple domain for a mobile manipulator robot:

```pddl
; cognitive_planning_ws/src/task_planning/task_planning/domains/simple_domain.pddl
(define (domain simple_mobile_manipulation)
  (:requirements :strips :typing :equality :negative-preconditions)
  (:types robot location object)
  (:predicates
    (at ?r - robot ?loc - location)
    (has ?r - robot ?obj - object)
    (at_place ?obj - object ?loc - location)
    (clear ?loc - location)
    (connected ?l1 - location ?l2 - location)
  )

  (:action move
    :parameters (?r - robot ?from ?to - location)
    :precondition (and (at ?r ?from) (connected ?from ?to))
    :effect (and (at ?r ?to) (not (at ?r ?from)))
  )

  (:action pickup
    :parameters (?r - robot ?obj - object ?loc - location)
    :precondition (and (at ?r ?loc) (at_place ?obj ?loc) (clear ?loc))
    :effect (and (has ?r ?obj) (not (at_place ?obj ?loc)))
  )

  (:action place
    :parameters (?r - robot ?obj - object ?loc - location)
    :precondition (and (at ?r ?loc) (has ?r ?obj))
    :effect (and (at_place ?obj ?loc) (not (has ?r ?obj))))
)
```

Create a problem instance:

```pddl
; cognitive_planning_ws/src/task_planning/task_planning/problems/simple_problem.pddl
(define (problem simple_task)
  (:domain simple_mobile_manipulation)
  (:objects
    robot1 - robot
    location1 location2 location3 - location
    object1 object2 - object
  )
  (:init
    (at robot1 location1)
    (at_place object1 location2)
    (at_place object2 location3)
    (clear location1)
    (clear location2)
    (clear location3)
    (connected location1 location2)
    (connected location2 location1)
    (connected location2 location3)
    (connected location3 location2)
  )
  (:goal (and
    (at_place object1 location1)
    (at_place object2 location1)
  ))
)
```

### Task 1.2: Implement a Basic PDDL Parser and Planner
Create a simple task planner:

```python
#!/usr/bin/env python3
# task_planning/task_planning/simple_planner.py

import rclpy
from rclpy.node import Node
from cognitive_planning_interfaces.msg import TaskPlan, TaskAction
from std_msgs.msg import String
import pypddl
import networkx as nx
from typing import List, Dict, Tuple

class SimplePlanner(Node):
    def __init__(self):
        super().__init__('simple_planner')

        self.plan_pub = self.create_publisher(TaskPlan, 'task_plan', 10)
        self.goal_sub = self.create_subscription(
            String, 'task_goal', self.goal_callback, 10)

        self.get_logger().info('Simple Planner node initialized')

    def goal_callback(self, msg):
        goal_description = msg.data
        self.get_logger().info(f'Received goal: {goal_description}')

        # For this example, we'll simulate a simple plan
        plan = self.create_simple_plan(goal_description)

        # Publish the plan
        plan_msg = TaskPlan()
        plan_msg.plan_id = "simple_plan_001"
        plan_msg.actions = plan
        plan_msg.status = "generated"

        self.plan_pub.publish(plan_msg)
        self.get_logger().info(f'Published plan with {len(plan)} actions')

    def create_simple_plan(self, goal_description) -> List[TaskAction]:
        """Create a simple plan based on goal description"""
        actions = []

        # Parse the goal and create actions accordingly
        if "move object1 to location1" in goal_description.lower():
            # Move robot to location2 (where object1 is)
            action1 = TaskAction()
            action1.action_type = "move"
            action1.parameters = "robot1 location1 location2"
            action1.description = "Move robot to location of object1"
            actions.append(action1)

            # Pick up object1
            action2 = TaskAction()
            action2.action_type = "pickup"
            action2.parameters = "robot1 object1 location2"
            action2.description = "Pick up object1"
            actions.append(action2)

            # Move robot to location1
            action3 = TaskAction()
            action3.action_type = "move"
            action3.parameters = "robot1 location2 location1"
            action3.description = "Move robot to destination location1"
            actions.append(action3)

            # Place object1
            action4 = TaskAction()
            action4.action_type = "place"
            action4.parameters = "robot1 object1 location1"
            action4.description = "Place object1 at location1"
            actions.append(action4)

        return actions

def main(args=None):
    rclpy.init(args=args)
    node = SimplePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 2: Behavior Trees Implementation

### Task 2.1: Create a Behavior Tree Node
Implement a behavior tree for hierarchical planning:

```python
#!/usr/bin/env python3
# hierarchical_planner/hierarchical_planner/behavior_tree_planner.py

import rclpy
from rclpy.node import Node
from cognitive_planning_interfaces.msg import TaskAction
from std_msgs.msg import String
import py_trees
import py_trees_ros
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import LaserScan

class BehaviorTreePlanner(Node):
    def __init__(self):
        super().__init__('behavior_tree_planner')

        # Publishers and subscribers
        self.action_pub = self.create_publisher(TaskAction, 'planned_action', 10)
        self.feedback_sub = self.create_subscription(
            String, 'action_feedback', self.feedback_callback, 10)

        # Create the behavior tree
        self.setup_behavior_tree()

        # Timer to tick the tree
        self.tree_timer = self.create_timer(0.1, self.tick_tree)

        self.get_logger().info('Behavior Tree Planner node initialized')

    def setup_behavior_tree(self):
        """Setup the behavior tree structure"""
        # Root node
        self.root = py_trees.composites.Sequence("Root_Sequence")

        # Check if goal is reached
        check_goal = py_trees.behaviours.CheckGoalReached("Check_Goal")

        # Plan action sequence
        plan_action = py_trees.behaviours.PlanNextAction("Plan_Action")

        # Execute action
        execute_action = py_trees.behaviours.ExecuteAction("Execute_Action")

        # Add to root
        self.root.add_child(check_goal)
        self.root.add_child(plan_action)
        self.root.add_child(execute_action)

        # Setup the tree
        self.tree = py_trees.trees.BehaviourTree(self.root)

    def tick_tree(self):
        """Tick the behavior tree"""
        try:
            self.tree.tick()
        except Exception as e:
            self.get_logger().error(f'Error ticking behavior tree: {str(e)}')

    def feedback_callback(self, msg):
        feedback = msg.data
        self.get_logger().info(f'Action feedback: {feedback}')

        # Based on feedback, we might replan or continue
        if "success" in feedback.lower():
            # Publish next action if available
            next_action = self.get_next_action()
            if next_action:
                action_msg = TaskAction()
                action_msg.action_type = next_action['type']
                action_msg.parameters = next_action['params']
                action_msg.description = next_action['description']
                self.action_pub.publish(action_msg)

    def get_next_action(self):
        """Get the next action in the plan"""
        # This is a simplified version - in practice, this would come from a plan
        return {
            'type': 'move',
            'params': 'base_link target_location',
            'description': 'Move to target location'
        }

class CheckGoalReached(py_trees.behaviour.Behaviour):
    def __init__(self, name="CheckGoalReached"):
        super().__init__(name)
        self.feedback_message = "checking goal status"

    def initialise(self):
        self.logger.debug(f"{self.name}[{self.__class__.__name__}.initialise()]")

    def update(self):
        # In a real implementation, this would check actual robot state
        # For simulation, we'll return RUNNING to continue planning
        self.feedback_message = "Goal not reached yet"
        return py_trees.common.Status.RUNNING

class PlanNextAction(py_trees.behaviour.Behaviour):
    def __init__(self, name="PlanNextAction"):
        super().__init__(name)
        self.feedback_message = "planning next action"

    def update(self):
        # In a real implementation, this would generate the next action
        # For simulation, return SUCCESS to proceed to execution
        self.feedback_message = "Action planned"
        return py_trees.common.Status.SUCCESS

class ExecuteAction(py_trees.behaviour.Behaviour):
    def __init__(self, name="ExecuteAction"):
        super().__init__(name)
        self.feedback_message = "executing action"

    def update(self):
        # In a real implementation, this would execute the action
        # For simulation, return RUNNING to continue
        self.feedback_message = "Action executing"
        return py_trees.common.Status.RUNNING

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorTreePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 3: Motion Planning Integration

### Task 3.1: Implement Motion Planner Node
Create a motion planner that works with the task planner:

```python
#!/usr/bin/env python3
# motion_planning/motion_planning/motion_planner.py

import rclpy
from rclpy.node import Node
from cognitive_planning_interfaces.msg import MotionPlan
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
import numpy as np
from scipy.spatial import distance
import math

class MotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')

        # Publishers and subscribers
        self.motion_plan_pub = self.create_publisher(MotionPlan, 'motion_plan', 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10)
        self.goal_sub = self.create_subscription(
            Pose, 'move_base_simple/goal', self.goal_callback, 10)

        # Visualization
        self.marker_pub = self.create_publisher(Marker, 'path_visualization', 10)

        # Internal state
        self.map_data = None
        self.map_resolution = 0.05  # meters per cell
        self.map_width = 0
        self.map_height = 0
        self.map_origin = None

        self.get_logger().info('Motion Planner node initialized')

    def map_callback(self, msg):
        """Store map data for path planning"""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]

        self.get_logger().info(f'Map received: {self.map_width}x{self.map_height}')

    def goal_callback(self, msg):
        """Plan path to the goal pose"""
        if self.map_data is None:
            self.get_logger().warn('No map available for path planning')
            return

        # Convert goal pose to grid coordinates
        goal_x = int((msg.position.x - self.map_origin[0]) / self.map_resolution)
        goal_y = int((msg.position.y - self.map_origin[1]) / self.map_resolution)

        # Get current robot position (in a real system, this would come from TF or odometry)
        current_x, current_y = self.get_current_position()

        self.get_logger().info(f'Planning from ({current_x}, {current_y}) to ({goal_x}, {goal_y})')

        # Plan path using A* algorithm
        path = self.plan_path_astar(current_x, current_y, goal_x, goal_y)

        if path:
            # Create motion plan message
            motion_plan = MotionPlan()
            motion_plan.plan_id = f"path_{current_x}_{current_y}_to_{goal_x}_{goal_y}"
            motion_plan.path = []

            # Convert grid path back to world coordinates
            for grid_x, grid_y in path:
                world_x = grid_x * self.map_resolution + self.map_origin[0]
                world_y = grid_y * self.map_resolution + self.map_origin[1]

                pose = Pose()
                pose.position.x = float(world_x)
                pose.position.y = float(world_y)
                pose.position.z = 0.0
                motion_plan.path.append(pose)

            # Publish the plan
            self.motion_plan_pub.publish(motion_plan)

            # Visualize the path
            self.visualize_path(path)

            self.get_logger().info(f'Published motion plan with {len(path)} waypoints')
        else:
            self.get_logger().warn('No path found to goal')

    def get_current_position(self):
        """Get current robot position in grid coordinates"""
        # In a real system, this would come from TF or odometry
        # For simulation, return center of map
        return self.map_width // 2, self.map_height // 2

    def plan_path_astar(self, start_x, start_y, goal_x, goal_y):
        """A* path planning algorithm"""
        # Check if start and goal are valid
        if (not self.is_valid_cell(start_x, start_y) or
            not self.is_valid_cell(goal_x, goal_y) or
            self.is_occupied(start_x, start_y) or
            self.is_occupied(goal_x, goal_y)):
            return None

        # Initialize open and closed sets
        open_set = [(start_x, start_y)]
        came_from = {}
        g_score = {(start_x, start_y): 0}
        f_score = {(start_x, start_y): self.heuristic(start_x, start_y, goal_x, goal_y)}

        while open_set:
            # Get the node with lowest f_score
            current = min(open_set, key=lambda x: f_score.get(x, float('inf')))

            if current == (goal_x, goal_y):
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            open_set.remove(current)

            # Check neighbors (8-connected)
            for dx, dy in [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]:
                neighbor = (current[0] + dx, current[1] + dy)

                if not self.is_valid_cell(neighbor[0], neighbor[1]):
                    continue

                if self.is_occupied(neighbor[0], neighbor[1]):
                    continue

                tentative_g_score = g_score[current] + self.distance(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor[0], neighbor[1], goal_x, goal_y)

                    if neighbor not in open_set:
                        open_set.append(neighbor)

        return None  # No path found

    def is_valid_cell(self, x, y):
        """Check if cell coordinates are within map bounds"""
        return 0 <= x < self.map_width and 0 <= y < self.map_height

    def is_occupied(self, x, y):
        """Check if a cell is occupied (value > 50 in occupancy grid)"""
        if not self.is_valid_cell(x, y):
            return True
        return self.map_data[y, x] > 50

    def heuristic(self, x1, y1, x2, y2):
        """Heuristic function (Euclidean distance)"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def distance(self, pos1, pos2):
        """Distance between two positions"""
        return math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)

    def visualize_path(self, path):
        """Publish visualization marker for the path"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05  # Line width
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for x, y in path:
            world_x = x * self.map_resolution + self.map_origin[0]
            world_y = y * self.map_resolution + self.map_origin[1]

            point = Point()
            point.x = world_x
            point.y = world_y
            point.z = 0.0
            marker.points.append(point)

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 4: Uncertainty Handling

### Task 4.1: Create Uncertainty-Aware Planner
Implement a planner that handles uncertainty:

```python
#!/usr/bin/env python3
# hierarchical_planner/hierarchical_planner/uncertainty_planner.py

import rclpy
from rclpy.node import Node
from cognitive_planning_interfaces.msg import TaskAction, MotionPlan
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import math
from scipy.stats import norm

class UncertaintyPlanner(Node):
    def __init__(self):
        super().__init__('uncertainty_planner')

        # Publishers and subscribers
        self.action_pub = self.create_publisher(TaskAction, 'uncertain_action', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.pose_callback, 10)

        # Robot state with uncertainty
        self.robot_pose = None
        self.pose_covariance = None
        self.scan_data = None

        # Uncertainty thresholds
        self.position_uncertainty_threshold = 0.5  # meters
        self.orientation_uncertainty_threshold = 0.2  # radians

        self.get_logger().info('Uncertainty Planner node initialized')

    def scan_callback(self, msg):
        """Process laser scan data for obstacle uncertainty"""
        self.scan_data = msg.ranges
        self.get_logger().debug(f'Scan received with {len(msg.ranges)} points')

    def pose_callback(self, msg):
        """Process robot pose with covariance"""
        self.robot_pose = msg.pose.pose
        self.pose_covariance = np.array(msg.pose.covariance).reshape(6, 6)

        # Calculate uncertainty metrics
        pos_uncertainty = math.sqrt(
            self.pose_covariance[0, 0] + self.pose_covariance[1, 1]
        )
        orient_uncertainty = math.sqrt(self.pose_covariance[5, 5])

        self.get_logger().info(
            f'Pose uncertainty - Position: {pos_uncertainty:.3f}m, '
            f'Orientation: {orient_uncertainty:.3f}rad'
        )

        # If uncertainty is too high, replan or request localization update
        if (pos_uncertainty > self.position_uncertainty_threshold or
            orient_uncertainty > self.orientation_uncertainty_threshold):
            self.get_logger().warn('High uncertainty detected, requesting localization update')
            self.request_localization_update()

    def request_localization_update(self):
        """Request improved localization"""
        # In a real system, this might trigger:
        # - AMCL update
        # - Visual relocalization
        # - Sensor fusion update
        self.get_logger().info('Localization update requested')

    def calculate_path_robustness(self, path):
        """Calculate robustness of a path given uncertainty"""
        if self.scan_data is None or self.pose_covariance is None:
            return 1.0  # Unknown, assume path is robust

        min_clearance = float('inf')

        for pose in path:
            # Calculate distance to nearest obstacle
            robot_x = pose.position.x
            robot_y = pose.position.y

            min_dist = float('inf')
            for i, range_val in enumerate(self.scan_data):
                if not (math.isnan(range_val) or math.isinf(range_val)):
                    # Convert laser reading to world coordinates
                    angle = self.scan_data.angle_min + i * self.scan_data.angle_increment
                    world_x = robot_x + range_val * math.cos(angle)
                    world_y = robot_y + range_val * math.sin(angle)

                    dist = math.sqrt((world_x - robot_x)**2 + (world_y - robot_y)**2)
                    min_dist = min(min_dist, dist)

            min_clearance = min(min_clearance, min_dist)

        # Return robustness score (higher is more robust)
        return min_clearance if min_clearance != float('inf') else 1.0

def main(args=None):
    rclpy.init(args=args)
    node = UncertaintyPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 5: Reactive vs. Deliberative Integration

### Task 5.1: Create Hybrid Planning Node
Implement a system that combines reactive and deliberative planning:

```python
#!/usr/bin/env python3
# hierarchical_planner/hierarchical_planner/hybrid_planner.py

import rclpy
from rclpy.node import Node
from cognitive_planning_interfaces.msg import TaskAction, MotionPlan
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String
import math
import time

class HybridPlanner(Node):
    def __init__(self):
        super().__init__('hybrid_planner')

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.goal_sub = self.create_subscription(
            Pose, 'move_base_simple/goal', self.goal_callback, 10)
        self.task_sub = self.create_subscription(
            TaskAction, 'high_level_task', self.task_callback, 10)

        # Internal state
        self.scan_data = None
        self.current_goal = None
        self.current_task = None
        self.is_deliberative_active = True
        self.last_reactive_time = time.time()
        self.reactive_threshold = 0.5  # seconds between reactive checks

        # Reactive parameters
        self.safety_distance = 0.5  # meters
        self.reactive_turn_speed = 0.5  # rad/s
        self.forward_speed = 0.2  # m/s

        self.get_logger().info('Hybrid Planner node initialized')

    def scan_callback(self, msg):
        """Process laser scan for reactive planning"""
        self.scan_data = msg

        # Check for immediate obstacles
        if self.is_deliberative_active:
            self.check_reactive_safety()

    def goal_callback(self, msg):
        """Set new navigation goal"""
        self.current_goal = msg
        self.get_logger().info(f'New goal set: ({msg.position.x}, {msg.position.y})')

        # In a real system, this would trigger deliberative planning
        self.start_deliberative_navigation()

    def task_callback(self, msg):
        """Process high-level task"""
        self.current_task = msg
        self.get_logger().info(f'New task: {msg.action_type} with params {msg.parameters}')

    def check_reactive_safety(self):
        """Check for immediate collision risk and react if needed"""
        if self.scan_data is None:
            return

        current_time = time.time()
        if current_time - self.last_reactive_time < self.reactive_threshold:
            return  # Don't check too frequently

        self.last_reactive_time = current_time

        # Check for obstacles in front
        front_ranges = self.scan_data.ranges[
            len(self.scan_data.ranges)//2 - 30 : len(self.scan_data.ranges)//2 + 30
        ]

        min_front_dist = min([r for r in front_ranges if not (math.isnan(r) or math.isinf(r))], default=float('inf'))

        if min_front_dist < self.safety_distance:
            self.get_logger().warn(f'Obstacle detected at {min_front_dist:.2f}m, switching to reactive mode')

            # Stop deliberative planning temporarily
            self.is_deliberative_active = False

            # Execute reactive avoidance
            self.execute_reactive_avoidance(min_front_dist)

            # Resume deliberative planning after reactive action
            time.sleep(1.0)  # Wait for reactive action to take effect
            self.is_deliberative_active = True

    def execute_reactive_avoidance(self, obstacle_distance):
        """Execute immediate obstacle avoidance"""
        cmd_vel = Twist()

        # Turn away from obstacle
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = self.reactive_turn_speed

        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info('Executing reactive obstacle avoidance')

    def start_deliberative_navigation(self):
        """Start deliberative path planning and navigation"""
        if not self.current_goal:
            return

        self.get_logger().info('Starting deliberative navigation')

        # In a real system, this would call a path planner
        # For this example, we'll just move toward the goal with obstacle avoidance
        self.navigate_with_obstacle_avoidance()

    def navigate_with_obstacle_avoidance(self):
        """Navigate toward goal with obstacle avoidance"""
        if not self.current_goal or not self.scan_data:
            return

        # Calculate direction to goal
        cmd_vel = Twist()

        # This is a simplified example - in reality, you'd use the planned path
        cmd_vel.linear.x = self.forward_speed
        cmd_vel.angular.z = 0.0  # We rely on reactive avoidance for turning

        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = HybridPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 6: Complete System Integration

### Task 6.1: Create Launch File
Create a launch file to start all planning components:

```python
# hierarchical_planner/launch/cognitive_planning_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Task planner
        Node(
            package='task_planning',
            executable='simple_planner',
            name='simple_planner',
            output='screen',
        ),
        # Motion planner
        Node(
            package='motion_planning',
            executable='motion_planner',
            name='motion_planner',
            output='screen',
        ),
        # Behavior tree planner
        Node(
            package='hierarchical_planner',
            executable='behavior_tree_planner',
            name='behavior_tree_planner',
            output='screen',
        ),
        # Uncertainty planner
        Node(
            package='hierarchical_planner',
            executable='uncertainty_planner',
            name='uncertainty_planner',
            output='screen',
        ),
        # Hybrid planner
        Node(
            package='hierarchical_planner',
            executable='hybrid_planner',
            name='hybrid_planner',
            output='screen',
        ),
    ])
```

## Exercises

### Exercise 1: Extend PDDL Domain
Add more complex actions to the PDDL domain:
- Add grasping and manipulation actions
- Include object properties (color, size, weight)
- Add robot capabilities (arm reach, payload capacity)

### Exercise 2: Implement RRT Algorithm
Replace the A* algorithm with an RRT (Rapidly-exploring Random Tree) algorithm for motion planning:
- Implement basic RRT
- Add RRT* for optimal solutions
- Handle kinodynamic constraints

### Exercise 3: Add POMDP Support
Extend the uncertainty handling with POMDP (Partially Observable MDP) concepts:
- Implement belief state updates
- Add action-observation loops
- Include reward functions

## Summary

In this lab, you implemented a complete cognitive planning system with:
1. Task planning using PDDL
2. Motion planning with A* algorithm
3. Behavior trees for hierarchical control
4. Uncertainty handling
5. Hybrid reactive-deliberative planning

The system demonstrates how different planning approaches can be integrated to create robust autonomous behavior.

## Next Steps

- Integrate with a real robot platform
- Add learning-based planning components
- Implement more sophisticated PDDL planners
- Test in complex, dynamic environments