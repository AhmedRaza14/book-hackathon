---
title: "Week 9: Navigation and Motion Planning - Code Lab"
week: 9
module: "Robotic Infrastructure"
difficulty: "advanced"
prerequisites: ["control-systems", "kinematics", "sensors", "ros2-fundamentals"]
learning_objectives:
  - "Implement path planning algorithms in ROS 2"
  - "Create a navigation stack with localization"
  - "Test motion planning in simulation"
  - "Integrate path planning with robot control"
tags: ["navigation", "motion-planning", "path-planning", "a-star", "dijkstra", "rrt", "localization", "mapping"]
hardware_requirements:
  - gpu: "RTX 4070 or higher"
  - ram: "16GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "120 minutes"
---

# Week 9: Navigation and Motion Planning - Code Lab

## Learning Objectives
- Implement path planning algorithms in ROS 2
- Create a navigation stack with localization
- Test motion planning in simulation
- Integrate path planning with robot control

## Prerequisites
- ROS 2 Humble Hawksbill installed
- Python 3.8+ with pip
- Completed Week 8 materials
- Basic understanding of navigation concepts

## Setup Environment

### Install Required Dependencies
```bash
# Create a new workspace
mkdir -p ~/navigation_ws/src
cd ~/navigation_ws

# Install navigation dependencies
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
sudo apt install python3-pip python3-colcon-common-extensions

# Install Python dependencies
pip3 install numpy scipy matplotlib
```

### ROS 2 Package Structure
```bash
# Create ROS 2 packages for navigation
cd ~/navigation_ws/src
ros2 pkg create --build-type ament_python path_planning
ros2 pkg create --build-type ament_python localization_system
ros2 pkg create --build-type ament_python motion_planner
ros2 pkg create --build-type ament_python navigation_stack
```

## Part 1: Path Planning Implementation

### Task 1.1: Implement Dijkstra's Algorithm Node

Create a ROS 2 node that implements Dijkstra's path planning algorithm:

```python
#!/usr/bin/env python3
# path_planning/path_planning/dijkstra_planner.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header
import numpy as np
import heapq
from typing import List, Tuple

class DijkstraPlanner(Node):
    def __init__(self):
        super().__init__('dijkstra_planner')

        # Publishers and subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, 'dijkstra_path', 10)

        # Internal state
        self.map_data = None
        self.map_info = None
        self.current_goal = None

        self.get_logger().info('Dijkstra Planner node initialized')

    def map_callback(self, msg):
        """Handle incoming occupancy grid map"""
        self.map_info = msg.info
        self.map_data = np.array(msg.data).reshape(
            msg.info.height, msg.info.width)
        self.get_logger().info(f'Map received: {msg.info.width}x{msg.info.height}')

    def goal_callback(self, msg):
        """Handle new goal pose"""
        self.current_goal = msg.pose
        self.get_logger().info(f'New goal received: ({msg.pose.position.x}, {msg.pose.position.y})')

        # Plan path if map is available
        if self.map_data is not None:
            self.plan_path()

    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates"""
        if self.map_info is None:
            return (0, 0)

        grid_x = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        grid_y = int((y - self.map_info.origin.position.y) / self.map_info.resolution)

        # Clamp to grid bounds
        grid_x = max(0, min(grid_x, self.map_info.width - 1))
        grid_y = max(0, min(grid_y, self.map_info.height - 1))

        return (grid_y, grid_x)

    def grid_to_world(self, row, col):
        """Convert grid coordinates to world coordinates"""
        if self.map_info is None:
            return (0.0, 0.0)

        x = col * self.map_info.resolution + self.map_info.origin.position.x
        y = row * self.map_info.resolution + self.map_info.origin.position.y

        return (x, y)

    def dijkstra(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Dijkstra's algorithm for path planning in a 2D grid
        """
        if self.map_data is None:
            return []

        rows, cols = self.map_data.shape
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0),  # 4-connected
                      (1, 1), (1, -1), (-1, 1), (-1, -1)]  # Diagonals for 8-connected

        # Priority queue: (cost, row, col)
        pq = [(0, start[0], start[1])]

        # Costs dictionary
        costs = {start: 0}

        # Previous nodes for path reconstruction
        previous = {start: None}

        while pq:
            current_cost, current_row, current_col = heapq.heappop(pq)

            # If we reached the goal
            if (current_row, current_col) == goal:
                break

            # Skip if we've found a better path already
            if current_cost > costs.get((current_row, current_col), float('inf')):
                continue

            # Explore neighbors
            for dr, dc in directions:
                new_row, new_col = current_row + dr, current_col + dc

                # Check bounds
                if 0 <= new_row < rows and 0 <= new_col < cols:
                    # Skip obstacles (occupied cells have value > 50)
                    if self.map_data[new_row, new_col] > 50:
                        continue

                    # Calculate movement cost (diagonal = sqrt(2), orthogonal = 1)
                    move_cost = np.sqrt(2) if dr != 0 and dc != 0 else 1.0

                    new_cost = current_cost + move_cost

                    # If we found a cheaper path to this node
                    if new_cost < costs.get((new_row, new_col), float('inf')):
                        costs[(new_row, new_col)] = new_cost
                        previous[(new_row, new_col)] = (current_row, current_col)
                        heapq.heappush(pq, (new_cost, new_row, new_col))

        # Reconstruct path
        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = previous.get(current)

        path.reverse()

        # Return empty path if no path found
        if path[0] != start:
            return []

        return path

    def plan_path(self):
        """Plan path from current robot position to goal"""
        if self.map_data is None or self.current_goal is None:
            self.get_logger().warn('Cannot plan path: missing map or goal')
            return

        # For this example, assume start position is (0, 0) in world coordinates
        start_world = (0.0, 0.0)  # This would normally come from robot's current position
        goal_world = (self.current_goal.position.x, self.current_goal.position.y)

        # Convert to grid coordinates
        start_grid = self.world_to_grid(*start_world)
        goal_grid = self.world_to_grid(*goal_world)

        self.get_logger().info(f'Planning from {start_grid} to {goal_grid}')

        # Plan path using Dijkstra
        path_grid = self.dijkstra(start_grid, goal_grid)

        if not path_grid:
            self.get_logger().warn('No path found')
            return

        # Convert grid path to world coordinates
        path_world = [self.grid_to_world(row, col) for row, col in path_grid]

        # Publish path
        self.publish_path(path_world)

    def publish_path(self, path_world):
        """Publish path as Path message"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for x, y in path_world:
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = path_msg.header.frame_id
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # No rotation

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published path with {len(path_msg.poses)} waypoints')


def main(args=None):
    rclpy.init(args=args)
    node = DijkstraPlanner()

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

### Task 1.2: Implement A* Algorithm Node

Create a ROS 2 node that implements the A* path planning algorithm:

```python
#!/usr/bin/env python3
# path_planning/path_planning/astar_planner.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
import numpy as np
import heapq
from typing import List, Tuple

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')

        # Publishers and subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, 'astar_path', 10)

        # Internal state
        self.map_data = None
        self.map_info = None
        self.current_goal = None

        self.get_logger().info('A* Planner node initialized')

    def map_callback(self, msg):
        """Handle incoming occupancy grid map"""
        self.map_info = msg.info
        self.map_data = np.array(msg.data).reshape(
            msg.info.height, msg.info.width)
        self.get_logger().info(f'Map received: {msg.info.width}x{msg.info.height}')

    def goal_callback(self, msg):
        """Handle new goal pose"""
        self.current_goal = msg.pose
        self.get_logger().info(f'New goal received: ({msg.pose.position.x}, {msg.pose.position.y})')

        # Plan path if map is available
        if self.map_data is not None:
            self.plan_path()

    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates"""
        if self.map_info is None:
            return (0, 0)

        grid_x = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        grid_y = int((y - self.map_info.origin.position.y) / self.map_info.resolution)

        # Clamp to grid bounds
        grid_x = max(0, min(grid_x, self.map_info.width - 1))
        grid_y = max(0, min(grid_y, self.map_info.height - 1))

        return (grid_y, grid_x)

    def grid_to_world(self, row, col):
        """Convert grid coordinates to world coordinates"""
        if self.map_info is None:
            return (0.0, 0.0)

        x = col * self.map_info.resolution + self.map_info.origin.position.x
        y = row * self.map_info.resolution + self.map_info.origin.position.y

        return (x, y)

    def heuristic(self, pos: Tuple[int, int], goal: Tuple[int, int]) -> float:
        """Manhattan distance heuristic (adjusted for diagonal movement)"""
        return max(abs(pos[0] - goal[0]), abs(pos[1] - goal[1]))

    def astar(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        A* algorithm for path planning in a 2D grid
        """
        if self.map_data is None:
            return []

        rows, cols = self.map_data.shape
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0),  # 4-connected
                      (1, 1), (1, -1), (-1, 1), (-1, -1)]  # Diagonals for 8-connected

        # Priority queue: (f_score, g_score, row, col)
        pq = [(self.heuristic(start, goal), 0, start[0], start[1])]

        # Costs dictionary
        g_costs = {start: 0}
        f_costs = {start: self.heuristic(start, goal)}

        # Previous nodes for path reconstruction
        previous = {start: None}

        while pq:
            f_score, g_score, current_row, current_col = heapq.heappop(pq)

            # If we reached the goal
            if (current_row, current_col) == goal:
                break

            # Skip if we've found a better path already
            if g_score > g_costs.get((current_row, current_col), float('inf')):
                continue

            # Explore neighbors
            for dr, dc in directions:
                new_row, new_col = current_row + dr, current_col + dc

                # Check bounds
                if 0 <= new_row < rows and 0 <= new_col < cols:
                    # Skip obstacles (occupied cells have value > 50)
                    if self.map_data[new_row, new_col] > 50:
                        continue

                    # Calculate movement cost
                    move_cost = np.sqrt(2) if dr != 0 and dc != 0 else 1.0
                    tentative_g = g_score + move_cost

                    # If we found a cheaper path to this node
                    if tentative_g < g_costs.get((new_row, new_col), float('inf')):
                        previous[(new_row, new_col)] = (current_row, current_col)
                        g_costs[(new_row, new_col)] = tentative_g
                        f_costs[(new_row, new_col)] = tentative_g + self.heuristic((new_row, new_col), goal)

                        heapq.heappush(pq, (f_costs[(new_row, new_col)], tentative_g, new_row, new_col))

        # Reconstruct path
        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = previous.get(current)

        path.reverse()

        # Return empty path if no path found
        if path[0] != start:
            return []

        return path

    def plan_path(self):
        """Plan path from current robot position to goal"""
        if self.map_data is None or self.current_goal is None:
            self.get_logger().warn('Cannot plan path: missing map or goal')
            return

        # For this example, assume start position is (0, 0) in world coordinates
        start_world = (0.0, 0.0)  # This would normally come from robot's current position
        goal_world = (self.current_goal.position.x, self.current_goal.position.y)

        # Convert to grid coordinates
        start_grid = self.world_to_grid(*start_world)
        goal_grid = self.world_to_grid(*goal_world)

        self.get_logger().info(f'Planning from {start_grid} to {goal_grid}')

        # Plan path using A*
        path_grid = self.astar(start_grid, goal_grid)

        if not path_grid:
            self.get_logger().warn('No path found')
            return

        # Convert grid path to world coordinates
        path_world = [self.grid_to_world(row, col) for row, col in path_grid]

        # Publish path
        self.publish_path(path_world)

    def publish_path(self, path_world):
        """Publish path as Path message"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for x, y in path_world:
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = path_msg.header.frame_id
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # No rotation

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published path with {len(path_msg.poses)} waypoints')


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()

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

## Part 2: Motion Planning Implementation

### Task 2.1: Implement Dynamic Window Approach (DWA) Node

Create a ROS 2 node that implements the Dynamic Window Approach for local motion planning:

```python
#!/usr/bin/env python3
# motion_planner/motion_planner/dwa_planner.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np
from math import pi, sqrt, atan2, cos, sin
from collections import deque

class DynamicWindowApproach(Node):
    def __init__(self):
        super().__init__('dwa_planner')

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.local_path_pub = self.create_publisher(Path, 'local_path', 10)

        # Robot parameters
        self.robot_radius = 0.3
        self.max_speed = 1.0
        self.max_turn_rate = 1.0
        self.max_accel = 0.5
        self.max_domega = 1.0
        self.dt = 0.1
        self.lookahead_time = 1.0
        self.lookahead_steps = int(self.lookahead_time / self.dt)

        # Internal state
        self.current_pos = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.current_vel = np.array([0.0, 0.0])       # vx, omega
        self.goal_pos = np.array([0.0, 0.0])
        self.obstacles = []
        self.has_goal = False

        # Goal tolerance
        self.goal_tolerance = 0.5

        self.get_logger().info('DWA Planner node initialized')

    def odom_callback(self, msg):
        """Handle odometry data"""
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y

        # Convert quaternion to euler
        quat = msg.pose.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        self.current_pos[2] = atan2(siny_cosp, cosy_cosp)

        self.current_vel[0] = msg.twist.twist.linear.x
        self.current_vel[1] = msg.twist.twist.angular.z

    def laser_callback(self, msg):
        """Process laser scan for obstacle detection"""
        self.obstacles = []
        angle = msg.angle_min
        for range_val in msg.ranges:
            if msg.range_min <= range_val <= msg.range_max:
                # Convert polar to Cartesian
                x = range_val * cos(angle)
                y = range_val * sin(angle)
                # Transform to world coordinates
                world_x = x + self.current_pos[0]
                world_y = y + self.current_pos[1]
                self.obstacles.append((world_x, world_y))
            angle += msg.angle_increment

    def goal_callback(self, msg):
        """Handle new goal pose"""
        self.goal_pos[0] = msg.pose.position.x
        self.goal_pos[1] = msg.pose.position.y
        self.has_goal = True
        self.get_logger().info(f'New goal: ({self.goal_pos[0]:.2f}, {self.goal_pos[1]:.2f})')

    def calc_dynamic_window(self):
        """
        Calculate dynamic window based on current velocity and constraints
        Returns [v_min, v_max, omega_min, omega_max]
        """
        v_min = max(0, self.current_vel[0] - self.max_accel * self.dt)
        v_max = min(self.max_speed, self.current_vel[0] + self.max_accel * self.dt)
        omega_min = max(-self.max_turn_rate, self.current_vel[1] - self.max_domega * self.dt)
        omega_max = min(self.max_turn_rate, self.current_vel[1] + self.max_domega * self.dt)

        return [v_min, v_max, omega_min, omega_max]

    def predict_trajectory(self, vel_cmd):
        """Predict trajectory based on velocity command"""
        traj = []
        pos = self.current_pos.copy()

        for i in range(self.lookahead_steps):
            # Update position based on motion model
            pos[0] += vel_cmd[0] * self.dt * cos(pos[2])
            pos[1] += vel_cmd[0] * self.dt * sin(pos[2])
            pos[2] += vel_cmd[1] * self.dt

            traj.append(pos.copy())

        return np.array(traj)

    def calc_heading_score(self, traj, goal_pos):
        """Calculate score based on heading towards goal"""
        if len(traj) == 0:
            return 0

        # Calculate angle between robot direction and goal direction
        robot_pos = traj[-1][:2]
        goal_direction = goal_pos - robot_pos
        robot_direction = np.array([cos(traj[-1][2]), sin(traj[-1][2])])

        dot_product = np.dot(goal_direction, robot_direction)
        norm_product = np.linalg.norm(goal_direction) * np.linalg.norm(robot_direction)

        if norm_product == 0:
            return 0

        cos_angle = dot_product / norm_product
        return max(0, cos_angle)  # Return 0-1 score

    def calc_dist_score(self, traj, obstacles):
        """Calculate score based on distance from obstacles"""
        if len(obstacles) == 0:
            return 1.0

        min_dist = float('inf')
        for pos in traj:
            for obs in obstacles:
                dist = sqrt((pos[0] - obs[0])**2 + (pos[1] - obs[1])**2)
                min_dist = min(min_dist, dist)

        # Normalize score (higher is better)
        return max(0, min(1.0, min_dist / (2 * self.robot_radius)))

    def calc_vel_score(self, velocity):
        """Calculate score based on velocity (higher velocity is better)"""
        return min(1.0, velocity / self.max_speed)

    def plan_local_path(self):
        """
        Plan local path using Dynamic Window Approach
        """
        if not self.has_goal:
            return np.array([0.0, 0.0])

        # Calculate velocity windows
        vs = self.calc_dynamic_window()

        # Evaluate trajectories
        best_traj = None
        best_score = -float('inf')
        best_vel = np.array([0.0, 0.0])

        # Sample velocity space
        v_samples = np.linspace(vs[0], vs[1], 10)  # Linear velocity samples
        omega_samples = np.linspace(vs[2], vs[3], 10)  # Angular velocity samples

        for v in v_samples:
            for omega in omega_samples:
                vel_cmd = np.array([v, omega])
                traj = self.predict_trajectory(vel_cmd)

                # Evaluate trajectory
                heading_score = self.calc_heading_score(traj, self.goal_pos)
                dist_score = self.calc_dist_score(traj, self.obstacles)
                vel_score = self.calc_vel_score(v)

                # Combined score (tunable weights)
                score = 0.2 * heading_score + 0.3 * dist_score + 0.5 * vel_score

                if score > best_score:
                    best_score = score
                    best_traj = traj
                    best_vel = vel_cmd

        return best_vel

    def publish_local_path(self, vel_cmd):
        """Publish the local path for visualization"""
        if vel_cmd is None:
            return

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        # Generate path based on current velocity command
        pos = self.current_pos.copy()
        for i in range(10):  # Publish 10 points ahead
            pos[0] += vel_cmd[0] * 0.1 * cos(pos[2])
            pos[1] += vel_cmd[0] * 0.1 * sin(pos[2])
            pos[2] += vel_cmd[1] * 0.1

            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = path_msg.header.frame_id
            pose.pose.position.x = float(pos[0])
            pose.pose.position.y = float(pos[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.local_path_pub.publish(path_msg)

    def control_loop(self):
        """Main control loop"""
        if not self.has_goal:
            # Stop the robot if no goal
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            return

        # Check if we reached the goal
        dist_to_goal = np.linalg.norm(self.current_pos[:2] - self.goal_pos)
        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info('Goal reached!')
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            self.has_goal = False
            return

        # Plan local path using DWA
        best_vel = self.plan_local_path()

        # Create Twist message
        cmd = Twist()
        cmd.linear.x = float(best_vel[0])
        cmd.angular.z = float(best_vel[1])

        # Publish command
        self.cmd_vel_pub.publish(cmd)

        # Publish local path for visualization
        self.publish_local_path(best_vel)

        self.get_logger().info(f'Velocity command: v={cmd.linear.x:.2f}, omega={cmd.angular.z:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = DynamicWindowApproach()

    # Create a timer for the control loop
    timer = node.create_timer(0.1, node.control_loop)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot on shutdown
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Part 3: Localization Implementation

### Task 3.1: Implement Particle Filter Localization Node

Create a ROS 2 node that implements particle filter for robot localization:

```python
#!/usr/bin/env python3
# localization_system/localization_system/particle_filter.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from scipy.stats import norm
from scipy.spatial.distance import cdist
import math

class ParticleFilter(Node):
    def __init__(self):
        super().__init__('particle_filter')

        # Publishers and subscribers
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'initialpose', self.initial_pose_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10)

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'amcl_pose', 10)
        self.particles_pub = self.create_publisher(MarkerArray, 'particles', 10)

        # Particle filter parameters
        self.num_particles = 1000
        self.state_dim = 3  # x, y, theta
        self.motion_noise = [0.1, 0.1, 0.05]  # [x_noise, y_noise, theta_noise]
        self.sensor_noise = 0.1  # Range measurement noise
        self.resample_threshold = 0.5  # Resample when Neff < threshold * num_particles

        # Initialize particles
        self.particles = np.random.uniform(-10, 10, (self.num_particles, self.state_dim))
        self.weights = np.ones(self.num_particles) / self.num_particles

        # Robot state
        self.current_odom = None
        self.previous_odom = None
        self.map_data = None
        self.map_info = None
        self.laser_data = None
        self.landmarks = []  # Will be extracted from map

        # For motion model
        self.dt = 0.1
        self.last_time = self.get_clock().now()

        self.get_logger().info(f'Particle Filter initialized with {self.num_particles} particles')

    def initial_pose_callback(self, msg):
        """Initialize particles based on initial pose estimate"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Get orientation from quaternion
        quat = msg.pose.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        # Initialize particles around the given pose with some uncertainty
        std_x = msg.pose.covariance[0]  # Variance in x
        std_y = msg.pose.covariance[7]  # Variance in y
        std_theta = msg.pose.covariance[35]  # Variance in theta

        for i in range(self.num_particles):
            self.particles[i, 0] = np.random.normal(x, np.sqrt(std_x) if std_x > 0 else 0.1)
            self.particles[i, 1] = np.random.normal(y, np.sqrt(std_y) if std_y > 0 else 0.1)
            self.particles[i, 2] = np.random.normal(theta, np.sqrt(std_theta) if std_theta > 0 else 0.1)

        # Normalize weights
        self.weights.fill(1.0 / self.num_particles)

        self.get_logger().info(f'Particles initialized around ({x:.2f}, {y:.2f})')

    def map_callback(self, msg):
        """Handle incoming map data"""
        self.map_info = msg.info
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)

        # Extract landmarks from map (occupied cells)
        # For simplicity, we'll use every 10th occupied cell as a landmark
        occupied_indices = np.where(self.map_data > 50)
        occupied_coords = list(zip(occupied_indices[0], occupied_indices[1]))

        # Sample landmarks to reduce computational load
        if len(occupied_coords) > 100:  # Only use up to 100 landmarks
            indices = np.random.choice(len(occupied_coords), 100, replace=False)
            sampled_coords = [occupied_coords[i] for i in indices]
        else:
            sampled_coords = occupied_coords

        # Convert grid coordinates to world coordinates
        self.landmarks = []
        for row, col in sampled_coords:
            x = col * self.map_info.resolution + self.map_info.origin.position.x
            y = row * self.map_info.resolution + self.map_info.origin.position.y
            self.landmarks.append((x, y))

        self.get_logger().info(f'Map received with {len(self.landmarks)} landmarks')

    def odom_callback(self, msg):
        """Handle odometry data"""
        current_time = rclpy.time.Time.from_msg(msg.header.stamp)

        if self.previous_odom is not None:
            # Calculate time difference
            time_diff = (current_time.nanoseconds - self.last_time.nanoseconds) / 1e9
            if time_diff > 0:
                self.dt = time_diff
        else:
            self.dt = 0.1  # Default time step

        self.last_time = current_time
        self.previous_odom = self.current_odom
        self.current_odom = msg

    def laser_callback(self, msg):
        """Process laser scan data"""
        self.laser_data = msg
        self.update_filter()

    def motion_model(self, control):
        """Update particles based on motion model"""
        v, omega = control

        for i in range(self.num_particles):
            # Current particle state
            x, y, theta = self.particles[i]

            # Add noise to control
            v_noisy = v + np.random.normal(0, self.motion_noise[0])
            omega_noisy = omega + np.random.normal(0, self.motion_noise[2])

            # Update state using motion model
            if abs(omega_noisy) < 1e-6:  # Linear motion
                new_x = x + v_noisy * self.dt * np.cos(theta)
                new_y = y + v_noisy * self.dt * np.sin(theta)
                new_theta = theta
            else:  # Turning motion
                radius = v_noisy / omega_noisy
                new_x = x - radius * np.sin(theta) + radius * np.sin(theta + omega_noisy * self.dt)
                new_y = y + radius * np.cos(theta) - radius * np.cos(theta + omega_noisy * self.dt)
                new_theta = theta + omega_noisy * self.dt

            # Add process noise
            self.particles[i] = [
                new_x + np.random.normal(0, self.motion_noise[0]),
                new_y + np.random.normal(0, self.motion_noise[1]),
                new_theta + np.random.normal(0, self.motion_noise[2])
            ]

    def sensor_model(self, measurements):
        """Calculate likelihood of measurements for each particle"""
        if self.laser_data is None or len(self.landmarks) == 0:
            return self.weights

        for i in range(self.num_particles):
            particle_x, particle_y, particle_theta = self.particles[i]

            # Calculate expected measurements for this particle
            expected_measurements = []
            for lx, ly in self.landmarks:
                expected_dist = np.sqrt((lx - particle_x)**2 + (ly - particle_y)**2)
                expected_measurements.append(expected_dist)

            # Calculate likelihood of actual measurements given expected measurements
            likelihood = 1.0
            # For efficiency, only consider a subset of landmarks
            sample_size = min(20, len(measurements))
            for j in range(sample_size):
                if j < len(expected_measurements):
                    actual = measurements[j] if j < len(measurements) else 0
                    expected = expected_measurements[j]
                    # Use Gaussian likelihood
                    prob = norm.pdf(actual, expected, self.sensor_noise)
                    likelihood *= prob if prob > 0 else 1e-6

            self.weights[i] *= likelihood

    def resample(self):
        """Resample particles based on weights"""
        # Calculate Neff (effective sample size)
        neff = 1.0 / np.sum(self.weights**2)

        if neff < self.resample_threshold * self.num_particles:
            # Systematic resampling
            indices = []
            step = 1.0 / self.num_particles
            start = np.random.uniform(0, step)

            cumsum = np.cumsum(self.weights)
            i = 0
            for j in range(self.num_particles):
                thresh = start + j * step
                while cumsum[i] < thresh:
                    i += 1
                indices.append(i)

            # Resample particles
            self.particles = self.particles[indices]
            self.weights.fill(1.0 / self.num_particles)

    def estimate(self):
        """Estimate robot pose from particles"""
        mean_pose = np.average(self.particles, axis=0, weights=self.weights)
        return mean_pose

    def calculate_covariance(self, mean_pose):
        """Calculate covariance from particles"""
        # Calculate covariance matrix
        centered_particles = self.particles - mean_pose
        cov_matrix = np.cov(centered_particles.T, aweights=self.weights)

        # Flatten covariance matrix for PoseWithCovariance
        cov_flat = np.zeros(36)
        cov_flat[0] = cov_matrix[0, 0]  # xx
        cov_flat[7] = cov_matrix[1, 1]  # yy
        cov_flat[35] = cov_matrix[2, 2]  # theta-theta

        return cov_flat

    def update_filter(self):
        """Main update function called when new sensor data arrives"""
        if self.current_odom is None or self.laser_data is None:
            return

        # Calculate control input from odometry
        if self.previous_odom is not None:
            # Calculate velocity from odometry
            dx = (self.current_odom.pose.pose.position.x -
                  self.previous_odom.pose.pose.position.x)
            dy = (self.current_odom.pose.pose.position.y -
                  self.previous_odom.pose.pose.position.y)
            dt = self.dt

            if dt > 0:
                v = np.sqrt(dx*dx + dy*dy) / dt
                # Calculate angular velocity from orientation change
                prev_quat = self.previous_odom.pose.pose.orientation
                curr_quat = self.current_odom.pose.pose.orientation

                prev_yaw = math.atan2(2*(prev_quat.w*prev_quat.z + prev_quat.x*prev_quat.y),
                                      1 - 2*(prev_quat.y*prev_quat.y + prev_quat.z*prev_quat.z))
                curr_yaw = math.atan2(2*(curr_quat.w*curr_quat.z + curr_quat.x*curr_quat.y),
                                      1 - 2*(curr_quat.y*curr_quat.y + curr_quat.z*curr_quat.z))

                omega = (curr_yaw - prev_yaw) / dt
            else:
                v = 0.0
                omega = 0.0
        else:
            v = 0.0
            omega = 0.0

        control = [v, omega]

        # Predict step
        self.motion_model(control)

        # Update step
        # Convert laser scan to distance measurements
        measurements = [r for r in self.laser_data.ranges if not (math.isnan(r) or math.isinf(r))]
        if measurements:
            self.sensor_model(measurements)

        # Normalize weights
        if np.sum(self.weights) > 0:
            self.weights /= np.sum(self.weights)
        else:
            # Reset weights if they all became zero
            self.weights.fill(1.0 / self.num_particles)

        # Resample
        self.resample()

        # Publish estimated pose
        self.publish_estimate()

        # Publish particles for visualization
        self.publish_particles()

    def publish_estimate(self):
        """Publish the estimated pose"""
        if self.weights is None or len(self.weights) == 0:
            return

        estimated_pose = self.estimate()

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = float(estimated_pose[0])
        pose_msg.pose.pose.position.y = float(estimated_pose[1])
        pose_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        theta = estimated_pose[2]
        pose_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Calculate and set covariance
        cov_matrix = self.calculate_covariance(estimated_pose)
        pose_msg.pose.covariance = cov_matrix.tolist()

        self.pose_pub.publish(pose_msg)

    def publish_particles(self):
        """Publish particles for visualization"""
        marker_array = MarkerArray()

        for i, particle in enumerate(self.particles):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'particles'
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            # Position
            marker.pose.position.x = particle[0]
            marker.pose.position.y = particle[1]
            marker.pose.position.z = 0.0

            # Orientation (pointing in direction of heading)
            marker.pose.orientation.z = math.sin(particle[2] / 2.0)
            marker.pose.orientation.w = math.cos(particle[2] / 2.0)

            # Scale
            marker.scale.x = 0.3  # Length of arrow
            marker.scale.y = 0.1  # Width of arrow
            marker.scale.z = 0.1  # Height of arrow

            # Color (based on weight)
            weight = self.weights[i]
            marker.color.r = 1.0
            marker.color.g = 1.0 - weight  # Less confident particles are more red
            marker.color.b = 1.0 - weight
            marker.color.a = min(1.0, weight * 10)  # Alpha based on weight

            marker_array.markers.append(marker)

        self.particles_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ParticleFilter()

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

## Part 4: Navigation Stack Integration

### Task 4.1: Create Navigation Manager Node

Create a ROS 2 node that integrates all navigation components:

```python
#!/usr/bin/env python3
# navigation_stack/navigation_stack/navigation_manager.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np
from enum import Enum

class NavigationState(Enum):
    IDLE = "idle"
    PLANNING = "planning"
    EXECUTING = "executing"
    RECOVERY = "recovery"
    GOAL_REACHED = "goal_reached"

class NavigationManager(Node):
    def __init__(self):
        super().__init__('navigation_manager')

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.localization_callback, 10)
        self.global_path_sub = self.create_subscription(
            Path, 'global_plan', self.global_path_callback, 10)
        self.local_path_sub = self.create_subscription(
            Path, 'local_plan', self.local_path_callback, 10)

        self.status_pub = self.create_publisher(String, 'navigation_status', 10)

        # Internal state
        self.current_state = NavigationState.IDLE
        self.current_goal = None
        self.current_pose = None
        self.current_odom = None
        self.global_path = []
        self.local_path = []
        self.scan_data = None
        self.path_index = 0
        self.goal_tolerance = 0.5
        self.recovery_count = 0
        self.max_recovery_attempts = 3

        # Navigation parameters
        self.linear_vel = 0.5
        self.angular_vel = 0.5
        self.min_distance_to_path = 0.3

        # Timer for navigation control
        self.nav_timer = self.create_timer(0.1, self.navigation_control)

        self.get_logger().info('Navigation Manager initialized')

    def goal_callback(self, msg):
        """Handle new goal"""
        self.current_goal = msg
        self.path_index = 0
        self.recovery_count = 0
        self.current_state = NavigationState.PLANNING
        self.get_logger().info(f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def odom_callback(self, msg):
        """Handle odometry data"""
        self.current_odom = msg

    def scan_callback(self, msg):
        """Handle laser scan data"""
        self.scan_data = msg

    def localization_callback(self, msg):
        """Handle localization data"""
        self.current_pose = msg.pose.pose

    def global_path_callback(self, msg):
        """Handle global path"""
        self.global_path = msg.poses
        if self.current_state == NavigationState.PLANNING and len(self.global_path) > 0:
            self.current_state = NavigationState.EXECUTING
            self.path_index = 0
            self.get_logger().info(f'Global path received with {len(self.global_path)} waypoints')

    def local_path_callback(self, msg):
        """Handle local path"""
        self.local_path = msg.poses

    def navigation_control(self):
        """Main navigation control loop"""
        if self.current_state == NavigationState.IDLE:
            self.publish_status("idle")
            return
        elif self.current_state == NavigationState.PLANNING:
            self.publish_status("planning")
            # In a real system, this would trigger path planning
            # For this example, we assume path is provided externally
            pass
        elif self.current_state == NavigationState.EXECUTING:
            self.execute_navigation()
        elif self.current_state == NavigationState.RECOVERY:
            self.execute_recovery()
        elif self.current_state == NavigationState.GOAL_REACHED:
            self.publish_status("goal_reached")
            self.stop_robot()
            return

    def execute_navigation(self):
        """Execute navigation along path"""
        if not self.current_pose or not self.global_path:
            return

        # Check if we have reached the goal
        if self.is_goal_reached():
            self.current_state = NavigationState.GOAL_REACHED
            self.get_logger().info('Goal reached!')
            return

        # Check for obstacles
        if self.is_path_blocked():
            self.initiate_recovery()
            return

        # Follow the path
        cmd_vel = self.follow_path()
        self.cmd_vel_pub.publish(cmd_vel)
        self.publish_status("executing")

    def follow_path(self):
        """Follow the global path using local planning"""
        cmd = Twist()

        if not self.current_pose or not self.global_path:
            return cmd

        # Get current robot position
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y

        # Find closest point on path
        closest_idx = self.find_closest_waypoint(robot_x, robot_y)

        if closest_idx is None:
            return cmd

        # Determine target point ahead on the path
        target_idx = min(closest_idx + 5, len(self.global_path) - 1)
        target_pose = self.global_path[target_idx].pose.position

        # Calculate direction to target
        dx = target_pose.x - robot_x
        dy = target_pose.y - robot_y
        target_angle = np.arctan2(dy, dx)

        # Get current robot angle
        quat = self.current_pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        current_angle = np.arctan2(siny_cosp, cosy_cosp)

        # Calculate angle difference
        angle_diff = target_angle - current_angle
        # Normalize angle to [-pi, pi]
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))

        # Set velocities
        cmd.linear.x = min(self.linear_vel, np.sqrt(dx*dx + dy*dy))  # Slower when closer
        cmd.angular.z = max(-self.angular_vel, min(self.angular_vel, angle_diff * 1.5))  # P controller

        return cmd

    def find_closest_waypoint(self, x, y):
        """Find the closest waypoint on the path"""
        if not self.global_path:
            return None

        min_dist = float('inf')
        closest_idx = 0

        for i, pose in enumerate(self.global_path):
            dist = np.sqrt((pose.pose.position.x - x)**2 + (pose.pose.position.y - y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        return closest_idx

    def is_goal_reached(self):
        """Check if the goal has been reached"""
        if not self.current_pose or not self.current_goal:
            return False

        goal_x = self.current_goal.pose.position.x
        goal_y = self.current_goal.pose.position.y
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y

        distance = np.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
        return distance < self.goal_tolerance

    def is_path_blocked(self):
        """Check if the path is blocked by obstacles"""
        if not self.scan_data:
            return False

        # Check if there are obstacles in the forward direction
        forward_scan_start = len(self.scan_data.ranges) // 2 - 10
        forward_scan_end = len(self.scan_data.ranges) // 2 + 10

        if forward_scan_start < 0:
            forward_scan_start = 0
        if forward_scan_end > len(self.scan_data.ranges):
            forward_scan_end = len(self.scan_data.ranges)

        for i in range(forward_scan_start, forward_scan_end):
            if i < len(self.scan_data.ranges):
                if self.scan_data.ranges[i] < 0.5 and not (np.isnan(self.scan_data.ranges[i]) or np.isinf(self.scan_data.ranges[i])):
                    return True  # Obstacle detected

        return False

    def initiate_recovery(self):
        """Initiate recovery behavior when path is blocked"""
        self.recovery_count += 1
        if self.recovery_count > self.max_recovery_attempts:
            self.get_logger().warn('Max recovery attempts reached, aborting navigation')
            self.current_state = NavigationState.IDLE
            self.stop_robot()
            return

        self.get_logger().info(f'Path blocked, initiating recovery attempt {self.recovery_count}')
        self.current_state = NavigationState.RECOVERY

    def execute_recovery(self):
        """Execute recovery behavior"""
        cmd = Twist()
        # Simple recovery: turn in place for a bit, then continue
        cmd.angular.z = 0.5  # Turn right

        self.cmd_vel_pub.publish(cmd)
        self.publish_status("recovery")

        # After some time, go back to executing
        self.get_logger().info('Recovery completed, resuming navigation')
        self.current_state = NavigationState.EXECUTING

    def stop_robot(self):
        """Stop the robot"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def publish_status(self, status):
        """Publish navigation status"""
        status_msg = String()
        status_msg.data = f"{status}:{self.current_state.value}"
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot on shutdown
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Part 5: Launch Files and Configuration

### Task 5.1: Create Launch File

Create a launch file to start all navigation components:

```python
# navigation_stack/launch/navigation_system.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
import os

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file', default='nav_params.yaml')

    # Dijkstra planner node
    dijkstra_planner = Node(
        package='path_planning',
        executable='dijkstra_planner',
        name='dijkstra_planner',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # A* planner node
    astar_planner = Node(
        package='path_planning',
        executable='astar_planner',
        name='astar_planner',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # DWA planner node
    dwa_planner = Node(
        package='motion_planner',
        executable='dwa_planner',
        name='dwa_planner',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Particle filter node
    particle_filter = Node(
        package='localization_system',
        executable='particle_filter',
        name='particle_filter',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Navigation manager node
    nav_manager = Node(
        package='navigation_stack',
        executable='navigation_manager',
        name='navigation_manager',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        # Launch all nodes
        dijkstra_planner,
        astar_planner,
        dwa_planner,
        particle_filter,
        nav_manager,
    ])
```

### Task 5.2: Create Configuration File

Create a configuration file for navigation parameters:

```yaml
# navigation_stack/config/nav_params.yaml
/**:
  ros__parameters:
    # Robot parameters
    robot_radius: 0.3
    max_speed: 1.0
    max_turn_rate: 1.0
    max_accel: 0.5
    max_domega: 1.0

    # Particle filter parameters
    num_particles: 1000
    motion_noise: [0.1, 0.1, 0.05]
    sensor_noise: 0.1
    resample_threshold: 0.5

    # Navigation parameters
    goal_tolerance: 0.5
    linear_vel: 0.5
    angular_vel: 0.5
    min_distance_to_path: 0.3
    max_recovery_attempts: 3

    # Simulation time
    use_sim_time: true
```

## Exercises

### Exercise 1: Implement RRT Path Planning
Extend the path planning system with RRT algorithm:
- Create a new ROS 2 node for RRT
- Implement the RRT algorithm from the theory
- Compare performance with Dijkstra and A*

### Exercise 2: Add Trajectory Optimization
Enhance the motion planner with trajectory optimization:
- Implement the trajectory optimization algorithm from theory
- Add smooth trajectory generation
- Test with different objective functions

### Exercise 3: Improve Localization
Enhance the particle filter with:
- Adaptive particle count
- Better sensor models
- Map matching techniques

### Exercise 4: Add Dynamic Obstacle Avoidance
Implement dynamic obstacle handling:
- Track moving obstacles
- Predict obstacle trajectories
- Adjust paths in real-time

## Summary

In this lab, you implemented:
1. Path planning algorithms (Dijkstra, A*)
2. Local motion planning (DWA)
3. Localization system (Particle Filter)
4. Navigation stack integration
5. Complete navigation manager

The system demonstrates how different navigation components work together to enable autonomous robot navigation.

## Next Steps

- Deploy on a physical robot
- Test with real sensors
- Add more sophisticated planning algorithms
- Implement learning-based navigation