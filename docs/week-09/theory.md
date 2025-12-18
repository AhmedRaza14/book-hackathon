---
title: "Week 9: Navigation and Motion Planning"
week: 9
module: "Robotic Infrastructure"
difficulty: "advanced"
prerequisites: ["control-systems", "kinematics", "sensors", "ros2-fundamentals"]
learning_objectives:
  - "Implement navigation algorithms"
  - "Create motion planning systems"
  - "Integrate path planning with control"
  - "Handle dynamic environments"
tags: ["navigation", "motion-planning", "path-planning", "a-star", "dijkstra", "rrt", "localization", "mapping"]
hardware_requirements:
  - gpu: "RTX 4070 or higher"
  - ram: "16GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "90 minutes"
---

# Week 9: Navigation and Motion Planning

## Learning Objectives
- Implement navigation algorithms
- Create motion planning systems
- Integrate path planning with control
- Handle dynamic environments
- Understand SLAM and localization concepts

## 9.1 Introduction to Robot Navigation

Robot navigation is the ability of a robot to move through its environment safely and efficiently. It encompasses several key components:

1. **Localization**: Determining the robot's position in the environment
2. **Mapping**: Creating a representation of the environment
3. **Path Planning**: Finding a route from start to goal
4. **Motion Planning**: Generating feasible motions to follow the path
5. **Control**: Executing the planned motions

### Navigation Stack Architecture

The navigation stack typically follows this architecture:

```
High Level: Mission Planning → Path Planning → Motion Planning → Control
            ↓                  ↓               ↓                ↓
Low Level:  Tasks           Global Path    Local Path      Motor Commands
```

### Coordinate Frames in Navigation

Navigation relies on proper coordinate frame management:

- **World/Map Frame**: Global reference frame
- **Odom Frame**: Odometry-based reference frame
- **Base Frame**: Robot's body frame
- **Sensor Frames**: Individual sensor coordinate systems

## 9.2 Path Planning Algorithms

### Graph-Based Path Planning

Graph-based algorithms represent the environment as a graph of connected nodes.

#### Dijkstra's Algorithm

Dijkstra's algorithm finds the shortest path from a start node to all other nodes in a weighted graph.

```python
import heapq
import numpy as np
from typing import List, Tuple, Dict

def dijkstra(grid: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
    """
    Dijkstra's algorithm for path planning in a 2D grid

    Args:
        grid: 2D binary grid (0 = free space, 1 = obstacle)
        start: Starting position (row, col)
        goal: Goal position (row, col)

    Returns:
        List of positions forming the path from start to goal
    """
    rows, cols = grid.shape
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
                # Skip obstacles
                if grid[new_row, new_col] == 1:
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

# Example usage
def example_dijkstra():
    # Create a simple grid with obstacles
    grid = np.zeros((10, 10))
    # Add some obstacles
    grid[3, 3:7] = 1  # Horizontal wall
    grid[5:8, 6] = 1  # Vertical wall

    start = (0, 0)
    goal = (9, 9)

    path = dijkstra(grid, start, goal)
    print(f"Dijkstra path: {path}")

    return path
```

#### A* Algorithm

A* improves upon Dijkstra by using a heuristic function to guide the search.

```python
def a_star(grid: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
    """
    A* algorithm for path planning in a 2D grid

    Args:
        grid: 2D binary grid (0 = free space, 1 = obstacle)
        start: Starting position (row, col)
        goal: Goal position (row, col)

    Returns:
        List of positions forming the path from start to goal
    """
    rows, cols = grid.shape
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0),  # 4-connected
                  (1, 1), (1, -1), (-1, 1), (-1, -1)]  # Diagonals for 8-connected

    def heuristic(pos: Tuple[int, int]) -> float:
        """Manhattan distance heuristic (adjusted for diagonal movement)"""
        return max(abs(pos[0] - goal[0]), abs(pos[1] - goal[1]))

    # Priority queue: (f_score, g_score, row, col)
    pq = [(heuristic(start), 0, start[0], start[1])]

    # Costs dictionary
    g_costs = {start: 0}
    f_costs = {start: heuristic(start)}

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
                # Skip obstacles
                if grid[new_row, new_col] == 1:
                    continue

                # Calculate movement cost
                move_cost = np.sqrt(2) if dr != 0 and dc != 0 else 1.0
                tentative_g = g_score + move_cost

                # If we found a cheaper path to this node
                if tentative_g < g_costs.get((new_row, new_col), float('inf')):
                    previous[(new_row, new_col)] = (current_row, current_col)
                    g_costs[(new_row, new_col)] = tentative_g
                    f_costs[(new_row, new_col)] = tentative_g + heuristic((new_row, new_col))

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

def example_astar():
    # Create a simple grid with obstacles
    grid = np.zeros((10, 10))
    # Add some obstacles
    grid[3, 3:7] = 1  # Horizontal wall
    grid[5:8, 6] = 1  # Vertical wall

    start = (0, 0)
    goal = (9, 9)

    path = a_star(grid, start, goal)
    print(f"A* path: {path}")

    return path
```

### Sampling-Based Motion Planning

Sampling-based methods work well in high-dimensional spaces.

#### Rapidly-Exploring Random Trees (RRT)

```python
import numpy as np
from typing import List, Tuple
from scipy.spatial.distance import euclidean

class RRTNode:
    def __init__(self, position: np.ndarray):
        self.position = position
        self.parent = None
        self.children = []

class RRT:
    def __init__(self, start: np.ndarray, goal: np.ndarray, bounds: Tuple[float, float, float, float],
                 obstacle_list: List[Tuple[float, float, float]] = None, max_iterations: int = 1000):
        """
        RRT path planner

        Args:
            start: Start position [x, y]
            goal: Goal position [x, y]
            bounds: (x_min, x_max, y_min, y_max)
            obstacle_list: List of (x, y, radius) tuples for circular obstacles
            max_iterations: Maximum number of iterations
        """
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.bounds = bounds
        self.obstacles = obstacle_list or []
        self.max_iterations = max_iterations
        self.step_size = 0.5  # Step size for extending tree

        # Tree initialization
        self.nodes = [RRTNode(self.start)]
        self.goal_found = False

    def is_collision_free(self, pos: np.ndarray) -> bool:
        """Check if position is collision-free"""
        # Check bounds
        x_min, x_max, y_min, y_max = self.bounds
        if not (x_min <= pos[0] <= x_max and y_min <= pos[1] <= y_max):
            return False

        # Check obstacles
        for obs_x, obs_y, obs_radius in self.obstacles:
            dist = euclidean(pos, [obs_x, obs_y])
            if dist <= obs_radius:
                return False

        return True

    def get_random_node(self) -> np.ndarray:
        """Get random node with bias towards goal"""
        if np.random.random() < 0.1:  # 10% chance to sample goal
            return self.goal
        else:
            # Sample random position in bounds
            x_min, x_max, y_min, y_max = self.bounds
            return np.array([
                np.random.uniform(x_min, x_max),
                np.random.uniform(y_min, y_max)
            ])

    def find_nearest_node(self, target_pos: np.ndarray) -> RRTNode:
        """Find nearest node in tree to target position"""
        nearest_node = self.nodes[0]
        min_dist = euclidean(nearest_node.position, target_pos)

        for node in self.nodes[1:]:
            dist = euclidean(node.position, target_pos)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node

        return nearest_node

    def extend_towards(self, from_node: RRTNode, target_pos: np.ndarray) -> bool:
        """Extend tree towards target position"""
        direction = target_pos - from_node.position
        distance = np.linalg.norm(direction)

        if distance <= self.step_size:
            new_pos = target_pos
        else:
            # Normalize direction and scale to step size
            direction = direction / distance
            new_pos = from_node.position + direction * self.step_size

        # Check if new position is collision-free
        if not self.is_collision_free(new_pos):
            return False

        # Create new node
        new_node = RRTNode(new_pos)
        new_node.parent = from_node
        from_node.children.append(new_node)
        self.nodes.append(new_node)

        # Check if we reached goal (with tolerance)
        if euclidean(new_pos, self.goal) < self.step_size:
            self.goal_found = True

        return True

    def plan(self) -> List[np.ndarray]:
        """Plan path using RRT"""
        for i in range(self.max_iterations):
            # Get random node
            random_pos = self.get_random_node()

            # Find nearest node in tree
            nearest_node = self.find_nearest_node(random_pos)

            # Extend tree towards random position
            success = self.extend_towards(nearest_node, random_pos)

            if success and self.goal_found:
                # Extract path
                path = self.extract_path()
                return path

        # If we didn't find a path, try to return best path anyway
        path = self.extract_path()
        return path

    def extract_path(self) -> List[np.ndarray]:
        """Extract path from start to goal"""
        path = []

        # Find goal node
        goal_node = None
        for node in self.nodes:
            if euclidean(node.position, self.goal) < self.step_size:
                goal_node = node
                break

        if goal_node is None:
            # If exact goal not found, find closest node
            closest_node = self.nodes[0]
            min_dist = euclidean(closest_node.position, self.goal)
            for node in self.nodes[1:]:
                dist = euclidean(node.position, self.goal)
                if dist < min_dist:
                    min_dist = dist
                    closest_node = node
            goal_node = closest_node

        # Backtrack from goal to start
        current = goal_node
        while current is not None:
            path.append(current.position.copy())
            current = current.parent

        path.reverse()
        return path

def example_rrt():
    # Define bounds and obstacles
    bounds = (0, 10, 0, 10)
    obstacles = [(3, 3, 1), (7, 7, 1), (5, 2, 0.5)]  # (x, y, radius)

    start = [1, 1]
    goal = [9, 9]

    # Create and run RRT
    rrt = RRT(start, goal, bounds, obstacles, max_iterations=2000)
    path = rrt.plan()

    print(f"RRT path: {path}")
    return path
```

## 9.3 Motion Planning Algorithms

### Trajectory Optimization

Motion planning often involves optimizing trajectories for various objectives.

```python
import numpy as np
from scipy.optimize import minimize
from scipy.interpolate import CubicSpline

class TrajectoryOptimizer:
    def __init__(self, start_state, goal_state, obstacles=None):
        self.start_state = start_state  # [x, y, theta, vx, vy, omega]
        self.goal_state = goal_state
        self.obstacles = obstacles or []

    def optimize_trajectory(self, num_waypoints=10):
        """Optimize trajectory using direct collocation"""
        # State variables: [x, y, theta, vx, vy, omega] at each waypoint
        # Control variables: [ax, ay, alpha] at each waypoint

        n_states = 6  # x, y, theta, vx, vy, omega
        n_controls = 3  # ax, ay, alpha

        # Initial guess: straight line
        initial_guess = self.get_initial_guess(num_waypoints)

        # Define objective function
        def objective(vars):
            # Extract states and controls from variables
            states = vars[:num_waypoints * n_states].reshape(num_waypoints, n_states)
            controls = vars[num_waypoints * n_states:].reshape(num_waypoints, n_controls)

            # Cost function: minimize control effort + path length
            control_cost = np.sum(controls**2) * 0.1
            path_length_cost = np.sum(np.sqrt(np.sum(np.diff(states[:, :2], axis=0)**2, axis=1)))

            # Obstacle avoidance cost
            obstacle_cost = 0
            for obs_x, obs_y, obs_radius in self.obstacles:
                for state in states:
                    dist = np.sqrt((state[0] - obs_x)**2 + (state[1] - obs_y)**2)
                    if dist < obs_radius + 0.5:  # Safety margin
                        obstacle_cost += 1000 * (1 - dist / (obs_radius + 0.5))**2

            return control_cost + path_length_cost + obstacle_cost

        # Define constraints
        def dynamics_constraints(vars):
            """Constraint equations based on dynamics"""
            states = vars[:num_waypoints * n_states].reshape(num_waypoints, n_states)
            controls = vars[num_waypoints * n_states:].reshape(num_waypoints, n_controls)

            constraints = []

            # Dynamics constraints: state_{k+1} = f(state_k, control_k)
            dt = 1.0 / num_waypoints  # Time step
            for k in range(num_waypoints - 1):
                x_k, y_k, theta_k, vx_k, vy_k, omega_k = states[k]
                ax_k, ay_k, alpha_k = controls[k]

                # Simple kinematic model
                x_next = x_k + vx_k * dt
                y_next = y_k + vy_k * dt
                theta_next = theta_k + omega_k * dt
                vx_next = vx_k + ax_k * dt
                vy_next = vy_k + ay_k * dt
                omega_next = omega_k + alpha_k * dt

                # Constraint: predicted next state should match actual next state
                constraints.extend([
                    x_next - states[k+1, 0],
                    y_next - states[k+1, 1],
                    theta_next - states[k+1, 2],
                    vx_next - states[k+1, 3],
                    vy_next - states[k+1, 4],
                    omega_next - states[k+1, 5]
                ])

            return constraints

        # Boundary constraints
        def boundary_constraints(vars):
            states = vars[:num_waypoints * n_states].reshape(num_waypoints, n_states)

            constraints = []
            # Start state constraint
            constraints.extend(states[0] - self.start_state)
            # Goal state constraint
            constraints.extend(states[-1] - self.goal_state)

            return constraints

        # Set up optimization problem
        n_vars = num_waypoints * (n_states + n_controls)

        # Constraints
        con1 = {'type': 'eq', 'fun': dynamics_constraints}
        con2 = {'type': 'eq', 'fun': boundary_constraints}
        constraints = [con1, con2]

        # Bounds (optional)
        bounds = [(-10, 10)] * (num_waypoints * n_states) + [(-5, 5)] * (num_waypoints * n_controls)

        # Solve optimization problem
        result = minimize(
            objective,
            initial_guess,
            method='SLSQP',
            bounds=bounds,
            constraints=constraints,
            options={'maxiter': 1000}
        )

        if result.success:
            states = result.x[:num_waypoints * n_states].reshape(num_waypoints, n_states)
            controls = result.x[num_waypoints * n_states:].reshape(num_waypoints, n_controls)
            return states, controls
        else:
            raise RuntimeError(f"Optimization failed: {result.message}")

    def get_initial_guess(self, num_waypoints):
        """Generate initial guess for optimization"""
        n_states = 6
        n_controls = 3

        # Linear interpolation for states
        state_trajectory = np.zeros((num_waypoints, n_states))
        for i in range(n_states):
            state_trajectory[:, i] = np.linspace(
                self.start_state[i], self.goal_state[i], num_waypoints
            )

        # Zero controls initially
        control_trajectory = np.zeros((num_waypoints, n_controls))

        # Flatten and concatenate
        initial_guess = np.concatenate([state_trajectory.flatten(), control_trajectory.flatten()])
        return initial_guess
```

### Dynamic Window Approach (DWA)

The Dynamic Window Approach is a local path planning method for mobile robots.

```python
import numpy as np
from math import pi, sqrt, atan2, cos, sin

class DynamicWindowApproach:
    def __init__(self, robot_radius=0.5, max_speed=1.0, max_turn_rate=1.0,
                 max_accel=0.5, max_domega=1.0, dt=0.1, lookahead_time=1.0):
        """
        Dynamic Window Approach planner

        Args:
            robot_radius: Robot radius for collision checking
            max_speed: Maximum linear speed
            max_turn_rate: Maximum angular turn rate
            max_accel: Maximum linear acceleration
            max_domega: Maximum angular acceleration
            dt: Time step
            lookahead_time: Lookahead time for trajectory evaluation
        """
        self.robot_radius = robot_radius
        self.max_speed = max_speed
        self.max_turn_rate = max_turn_rate
        self.max_accel = max_accel
        self.max_domega = max_domega
        self.dt = dt
        self.lookahead_time = lookahead_time
        self.lookahead_steps = int(lookahead_time / dt)

    def plan_local_path(self, current_pos, current_vel, goal_pos, obstacles):
        """
        Plan local path using Dynamic Window Approach

        Args:
            current_pos: Current position [x, y, theta]
            current_vel: Current velocity [vx, omega]
            goal_pos: Goal position [x, y]
            obstacles: List of obstacle positions [(x1, y1), (x2, y2), ...]

        Returns:
            Best velocity command [vx, omega]
        """
        # Calculate velocity windows
        vs = self.calc_dynamic_window(current_vel)

        # Evaluate trajectories
        best_traj = None
        best_score = -float('inf')

        for v in np.linspace(vs[0], vs[1], 10):  # Linear velocity samples
            for omega in np.linspace(vs[2], vs[3], 10):  # Angular velocity samples
                traj = self.predict_trajectory(current_pos, [v, omega])

                # Evaluate trajectory
                heading_score = self.calc_heading_score(traj, goal_pos)
                dist_score = self.calc_dist_score(traj, obstacles)
                vel_score = self.calc_vel_score(v)

                # Combined score (tunable weights)
                score = 0.2 * heading_score + 0.3 * dist_score + 0.5 * vel_score

                if score > best_score:
                    best_score = score
                    best_traj = traj
                    best_vel = [v, omega]

        return best_vel

    def calc_dynamic_window(self, current_vel):
        """
        Calculate dynamic window based on current velocity and constraints
        Returns [v_min, v_max, omega_min, omega_max]
        """
        v_min = max(0, current_vel[0] - self.max_accel * self.dt)
        v_max = min(self.max_speed, current_vel[0] + self.max_accel * self.dt)
        omega_min = max(-self.max_turn_rate, current_vel[1] - self.max_domega * self.dt)
        omega_max = min(self.max_turn_rate, current_vel[1] + self.max_domega * self.dt)

        return [v_min, v_max, omega_min, omega_max]

    def predict_trajectory(self, start_pos, vel_cmd):
        """Predict trajectory based on velocity command"""
        traj = []
        pos = start_pos.copy()

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

def example_dwa():
    """Example of Dynamic Window Approach"""
    dwa = DynamicWindowApproach()

    current_pos = np.array([0.0, 0.0, 0.0])  # x, y, theta
    current_vel = np.array([0.0, 0.0])  # vx, omega
    goal_pos = np.array([5.0, 5.0])
    obstacles = [(2, 2), (3, 4), (4, 3)]

    best_vel = dwa.plan_local_path(current_pos, current_vel, goal_pos, obstacles)
    print(f"DWA best velocity: {best_vel}")

    return best_vel
```

## 9.4 Localization and Mapping

### Particle Filter for Localization

```python
import numpy as np
from scipy.stats import norm
import random

class ParticleFilter:
    def __init__(self, num_particles=1000, state_dim=3):
        """
        Particle Filter for robot localization

        Args:
            num_particles: Number of particles
            state_dim: State dimension (typically 3 for 2D localization: x, y, theta)
        """
        self.num_particles = num_particles
        self.state_dim = state_dim

        # Initialize particles
        self.particles = np.random.uniform(-10, 10, (num_particles, state_dim))
        self.weights = np.ones(num_particles) / num_particles

        # Motion and sensor noise
        self.motion_noise = [0.1, 0.1, 0.05]  # [x_noise, y_noise, theta_noise]
        self.sensor_noise = 0.1  # Range measurement noise

    def predict(self, control, dt=1.0):
        """
        Prediction step: update particles based on motion model

        Args:
            control: Control input [v, omega] (linear velocity, angular velocity)
            dt: Time step
        """
        v, omega = control

        for i in range(self.num_particles):
            # Current particle state
            x, y, theta = self.particles[i]

            # Add noise to control
            v_noisy = v + np.random.normal(0, self.motion_noise[0])
            omega_noisy = omega + np.random.normal(0, self.motion_noise[2])

            # Update state using motion model
            if abs(omega_noisy) < 1e-6:  # Linear motion
                new_x = x + v_noisy * dt * np.cos(theta)
                new_y = y + v_noisy * dt * np.sin(theta)
                new_theta = theta
            else:  # Turning motion
                radius = v_noisy / omega_noisy
                new_x = x - radius * np.sin(theta) + radius * np.sin(theta + omega_noisy * dt)
                new_y = y + radius * np.cos(theta) - radius * np.cos(theta + omega_noisy * dt)
                new_theta = theta + omega_noisy * dt

            # Add process noise
            self.particles[i] = [
                new_x + np.random.normal(0, self.motion_noise[0]),
                new_y + np.random.normal(0, self.motion_noise[1]),
                new_theta + np.random.normal(0, self.motion_noise[2])
            ]

    def update(self, measurements, landmarks):
        """
        Update step: update particle weights based on sensor measurements

        Args:
            measurements: List of observed distances to landmarks
            landmarks: List of landmark positions [(x1, y1), (x2, y2), ...]
        """
        for i in range(self.num_particles):
            particle_x, particle_y, particle_theta = self.particles[i]

            # Calculate expected measurements for this particle
            expected_measurements = []
            for lx, ly in landmarks:
                expected_dist = np.sqrt((lx - particle_x)**2 + (ly - particle_y)**2)
                expected_measurements.append(expected_dist)

            # Calculate likelihood of actual measurements given expected measurements
            likelihood = 1.0
            for actual, expected in zip(measurements, expected_measurements):
                # Use Gaussian likelihood
                prob = norm.pdf(actual, expected, self.sensor_noise)
                likelihood *= prob if prob > 0 else 1e-6

            self.weights[i] *= likelihood

        # Normalize weights
        self.weights /= np.sum(self.weights)

    def resample(self):
        """Resample particles based on weights"""
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

    def get_particles(self):
        """Get current particles"""
        return self.particles.copy()

def example_particle_filter():
    """Example of particle filter for localization"""
    pf = ParticleFilter(num_particles=1000)

    # Known landmarks
    landmarks = [(0, 0), (10, 0), (10, 10), (0, 10)]

    # Simulate robot movement and sensing
    true_pose = np.array([2.0, 2.0, 0.0])  # True robot pose
    control = [0.5, 0.1]  # v=0.5, omega=0.1

    for step in range(100):
        # Predict step
        pf.predict(control, dt=0.1)

        # Simulate sensor measurements (with noise)
        measurements = []
        for lx, ly in landmarks:
            true_dist = np.sqrt((lx - true_pose[0])**2 + (ly - true_pose[1])**2)
            noisy_dist = true_dist + np.random.normal(0, 0.1)
            measurements.append(noisy_dist)

        # Update step
        pf.update(measurements, landmarks)

        # Resample
        pf.resample()

        # Estimate and print
        estimated_pose = pf.estimate()
        print(f"Step {step}: True={true_pose[:2]}, Estimated={estimated_pose[:2]}")

        # Update true pose (for simulation)
        v, omega = control
        dt = 0.1
        if abs(omega) < 1e-6:
            true_pose[0] += v * dt * np.cos(true_pose[2])
            true_pose[1] += v * dt * np.sin(true_pose[2])
        else:
            radius = v / omega
            true_pose[0] -= radius * np.sin(true_pose[2]) + radius * np.sin(true_pose[2] + omega * dt)
            true_pose[1] += radius * np.cos(true_pose[2]) - radius * np.cos(true_pose[2] + omega * dt)
            true_pose[2] += omega * dt

        # Keep angle in [-pi, pi]
        true_pose[2] = (true_pose[2] + np.pi) % (2 * np.pi) - np.pi

def main():
    print("Testing different path planning algorithms...")

    # Test A* algorithm
    path_astar = example_astar()

    # Test RRT
    path_rrt = example_rrt()

    # Test DWA
    vel_cmd = example_dwa()

    # Test Particle Filter
    example_particle_filter()

    print("All algorithms tested successfully!")

if __name__ == '__main__':
    main()
```

## 9.5 ROS 2 Navigation Stack Integration

### Creating a Custom Planner Plugin

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from threading import Lock

class CustomPlannerNode(Node):
    def __init__(self):
        super().__init__('custom_planner')

        # Publishers
        self.global_plan_pub = self.create_publisher(Path, 'global_plan', 10)
        self.local_plan_pub = self.create_publisher(Path, 'local_plan', 10)
        self.debug_marker_pub = self.create_publisher(MarkerArray, 'planner_debug', 10)

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback, 10
        )
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )

        # Internal state
        self.map_data = None
        self.current_goal = None
        self.current_pose = None
        self.obstacles = []
        self.global_path = []
        self.local_path = []

        # Planner parameters
        self.planner_lock = Lock()
        self.resolution = 0.05
        self.origin = [0, 0]
        self.inflation_radius = 0.3

        # Timer for planning
        self.planning_timer = self.create_timer(0.5, self.plan_if_needed)

    def map_callback(self, msg):
        """Handle incoming map data"""
        with self.planner_lock:
            self.map_data = msg

    def goal_callback(self, msg):
        """Handle new goal pose"""
        with self.planner_lock:
            self.current_goal = msg.pose
            # Plan new path
            self.plan_global_path()

    def odom_callback(self, msg):
        """Update current pose"""
        with self.planner_lock:
            self.current_pose = msg.pose.pose

    def laser_callback(self, msg):
        """Process laser scan for local planning"""
        with self.planner_lock:
            # Convert laser scan to obstacle points in world coordinates
            self.obstacles = []
            angle = msg.angle_min
            for range_val in msg.ranges:
                if msg.range_min <= range_val <= msg.range_max:
                    # Convert polar to Cartesian
                    x = range_val * np.cos(angle)
                    y = range_val * np.sin(angle)
                    # Transform to world coordinates (simplified)
                    world_x = x + self.current_pose.position.x
                    world_y = y + self.current_pose.position.y
                    self.obstacles.append((world_x, world_y))
                angle += msg.angle_increment

    def plan_global_path(self):
        """Plan global path from current pose to goal"""
        if not self.map_data or not self.current_goal:
            return

        # Convert poses to grid coordinates
        start_grid = self.world_to_grid(
            self.current_pose.position.x,
            self.current_pose.position.y
        )
        goal_grid = self.world_to_grid(
            self.current_goal.position.x,
            self.current_goal.position.y
        )

        # Create grid from map data
        grid = np.array(self.map_data.data).reshape(
            self.map_data.info.height,
            self.map_data.info.width
        )
        grid = (grid > 50).astype(int)  # Convert to binary (occupied = 1)

        # Inflate obstacles
        inflated_grid = self.inflate_obstacles(grid, self.inflation_radius)

        # Plan path using A*
        path = a_star(inflated_grid, start_grid, goal_grid)

        # Convert grid path to world coordinates
        self.global_path = [self.grid_to_world(row, col) for row, col in path]

        # Publish global path
        self.publish_path(self.global_path, self.global_plan_pub)

    def inflate_obstacles(self, grid, inflation_radius):
        """Inflate obstacles by specified radius"""
        inflated_grid = grid.copy()
        inflation_cells = int(inflation_radius / self.map_data.info.resolution)

        rows, cols = grid.shape
        for r in range(rows):
            for c in range(cols):
                if grid[r, c] == 1:  # If obstacle
                    # Mark surrounding cells as occupied
                    for dr in range(-inflation_cells, inflation_cells + 1):
                        for dc in range(-inflation_cells, inflation_cells + 1):
                            nr, nc = r + dr, c + dc
                            if 0 <= nr < rows and 0 <= nc < cols:
                                dist = np.sqrt(dr**2 + dc**2)
                                if dist <= inflation_cells:
                                    inflated_grid[nr, nc] = 1

        return inflated_grid

    def plan_local_path(self):
        """Plan local path considering dynamic obstacles"""
        if not self.current_pose or not self.global_path:
            return

        # Get local area around robot
        local_area = self.get_local_area()

        # Use DWA or similar for local planning
        if self.obstacles:
            # Implement local obstacle avoidance
            self.local_path = self.avoid_local_obstacles()
        else:
            # Follow global path locally
            self.local_path = self.follow_global_path_locally()

        # Publish local path
        self.publish_path(self.local_path, self.local_plan_pub)

    def avoid_local_obstacles(self):
        """Local obstacle avoidance using dynamic approach"""
        # For this example, we'll use a simple approach
        # In practice, this would use DWA, RRT*, or other local planners
        if not self.global_path:
            return []

        # Find closest point on global path
        current_pos = (self.current_pose.position.x, self.current_pose.position.y)
        closest_idx = 0
        min_dist = float('inf')
        for i, point in enumerate(self.global_path):
            dist = np.sqrt((point[0] - current_pos[0])**2 + (point[1] - current_pos[1])**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Create local path by following global path but avoiding obstacles
        local_path = []
        start_idx = closest_idx
        for i in range(start_idx, min(start_idx + 20, len(self.global_path))):
            point = self.global_path[i]

            # Check if point is blocked by local obstacles
            blocked = False
            for obs_x, obs_y in self.obstacles:
                dist = np.sqrt((point[0] - obs_x)**2 + (point[1] - obs_y)**2)
                if dist < 0.5:  # Safety threshold
                    blocked = True
                    break

            if not blocked:
                local_path.append(point)

        return local_path

    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates"""
        if not self.map_data:
            return (0, 0)

        grid_x = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        grid_y = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

        # Clamp to grid bounds
        grid_x = max(0, min(grid_x, self.map_data.info.width - 1))
        grid_y = max(0, min(grid_y, self.map_data.info.height - 1))

        return (grid_y, grid_x)  # Row, Col format for numpy

    def grid_to_world(self, row, col):
        """Convert grid coordinates to world coordinates"""
        if not self.map_data:
            return (0.0, 0.0)

        x = col * self.map_data.info.resolution + self.map_data.info.origin.position.x
        y = row * self.map_data.info.resolution + self.map_data.info.origin.position.y

        return (x, y)

    def publish_path(self, path, publisher):
        """Publish path as Path message"""
        if not path:
            return

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for x, y in path:
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = path_msg.header.frame_id
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # No rotation

            path_msg.poses.append(pose)

        publisher.publish(path_msg)

    def plan_if_needed(self):
        """Plan paths if conditions are met"""
        with self.planner_lock:
            if self.current_pose and self.current_goal:
                self.plan_local_path()

def main(args=None):
    rclpy.init(args=args)
    node = CustomPlannerNode()

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

## 9.6 Practical Exercise

Create a complete navigation system that:
1. Implements multiple path planning algorithms
2. Integrates localization and mapping
3. Handles dynamic obstacles
4. Provides smooth trajectory execution
5. Includes safety systems

```python
# Student exercise - Complete implementation
class CompleteNavigationSystem:
    """Student implementation of a complete navigation system"""

    def __init__(self):
        """Initialize the complete navigation system"""
        # TODO: Implement multiple path planners (A*, RRT, Dijkstra)
        # TODO: Integrate localization (Particle Filter, EKF)
        # TODO: Implement mapping (Occupancy Grid, Octomap)
        # TODO: Handle dynamic obstacles
        # TODO: Implement trajectory smoothing
        # TODO: Add safety and emergency systems
        # TODO: Integrate with ROS 2 navigation stack
        pass

    def implement_path_planners(self):
        """Implement multiple path planning algorithms"""
        # TODO: Complete implementation
        pass

    def implement_localization(self):
        """Implement localization system"""
        # TODO: Complete implementation
        pass

    def implement_mapping(self):
        """Implement mapping system"""
        # TODO: Complete implementation
        pass

    def handle_dynamic_obstacles(self):
        """Handle dynamic obstacle avoidance"""
        # TODO: Complete implementation
        pass

    def implement_trajectory_smoothing(self):
        """Implement trajectory smoothing"""
        # TODO: Complete implementation
        pass

    def implement_safety_system(self):
        """Implement safety systems"""
        # TODO: Complete implementation
        pass

    def integrate_with_ros_navigation(self):
        """Integrate with ROS 2 navigation stack"""
        # TODO: Complete implementation
        pass

print("Student Exercise: Implement a complete navigation system")
print("Requirements:")
print("1. Multiple path planning algorithms (A*, RRT, Dijkstra)")
print("2. Localization system (Particle Filter, EKF)")
print("3. Mapping system (Occupancy Grid, Octomap)")
print("4. Dynamic obstacle handling")
print("5. Trajectory smoothing and execution")
print("6. Safety and emergency systems")
print("7. ROS 2 navigation stack integration")
```

## Summary

Navigation and motion planning are critical components of robotic systems, enabling robots to move safely and efficiently through their environment. Understanding path planning algorithms, localization techniques, and mapping methods is essential for creating autonomous robotic systems.

## Next Steps

In the next section, we'll explore machine learning integration in robotics, building on the navigation and motion planning concepts learned here to enable intelligent robotic behavior.

<div class="alert alert-warning">
  <h5>Hardware Requirement</h5>
  <div><strong>Requirement:</strong> GPU</div>
  <div><strong>Minimum:</strong> RTX 4070</div>
  <div><strong>Recommended:</strong> RTX 4080</div>
  <div><strong>Purpose:</strong> Navigation and motion planning algorithms require significant computational resources, especially for real-time operation</div>
</div>