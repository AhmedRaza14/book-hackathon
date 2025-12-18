---
title: "Week 8: Control Systems and Trajectory Planning Simulation"
week: 8
module: "Robotic Infrastructure"
difficulty: "advanced"
prerequisites: ["ros2-fundamentals", "python-basics", "robotics-concepts", "kinematics", "sensors"]
learning_objectives:
  - "Simulate control systems in virtual environments"
  - "Implement trajectory planning algorithms"
  - "Validate control performance in simulation"
  - "Test safety systems in safe environment"
tags: ["control", "simulation", "trajectory", "planning", "safety", "validation"]
hardware_requirements:
  - gpu: "Any"
  - ram: "8GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "120 minutes"
---

# Week 8: Control Systems and Trajectory Planning Simulation

## Learning Objectives
- Simulate control systems in virtual environments
- Implement trajectory planning algorithms
- Validate control performance in simulation
- Test safety systems in safe environment

## 8.1 Control System Simulation

### PID Controller Simulation

Let's create a comprehensive PID controller simulation that models real-world effects:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import time
from collections import deque

class SimulatedPIDController:
    """PID controller simulation with realistic effects"""

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limits=(-np.inf, np.inf),
                 noise_std=0.0, delay_ms=0, saturation=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.noise_std = noise_std
        self.delay_ms = delay_ms
        self.saturation = saturation

        # Internal state
        self.reset()

    def reset(self):
        """Reset controller state"""
        self.last_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.last_time = None
        self.delay_buffer = deque(maxlen=max(1, int(self.delay_ms / 10)))  # 10ms timestep assumption

    def compute(self, setpoint, measured_value, dt=None):
        """Compute PID output with realistic effects"""
        current_time = time.time()

        if dt is None:
            if self.last_time is None:
                dt = 0.01  # Default 10ms timestep
            else:
                dt = current_time - self.last_time
        self.last_time = current_time

        # Calculate error
        error = setpoint - measured_value

        # Proportional term
        proportional = self.kp * error

        # Integral term with anti-windup
        self.integral += error * dt
        # Anti-windup: clamp integral to prevent excessive accumulation
        max_integral = self.output_limits[1] / self.ki if self.ki != 0 else np.inf
        self.integral = np.clip(self.integral, -max_integral, max_integral)
        integral = self.ki * self.integral

        # Derivative term with noise filtering
        if dt > 0:
            raw_derivative = (error - self.last_error) / dt
            # Apply low-pass filter to derivative to reduce noise sensitivity
            alpha = 0.2  # Filter coefficient
            self.derivative = alpha * raw_derivative + (1 - alpha) * self.derivative
        derivative = self.kd * self.derivative

        # Calculate raw output
        raw_output = proportional + integral + derivative

        # Apply saturation
        if self.saturation > 0:
            raw_output = np.clip(raw_output, -self.saturation, self.saturation)

        # Add measurement noise
        if self.noise_std > 0:
            noise = np.random.normal(0, self.noise_std)
            raw_output += noise

        # Apply output limits
        output = np.clip(raw_output, self.output_limits[0], self.output_limits[1])

        # Apply delay simulation
        self.delay_buffer.append(output)
        if len(self.delay_buffer) > 1:
            output = self.delay_buffer[0]  # Use delayed output

        # Store for next iteration
        self.last_error = error

        return output, proportional, integral, derivative

class ControlSystemSimulationNode(Node):
    def __init__(self):
        super().__init__('control_system_simulation')

        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, 'simulated_joint_states', 10)
        self.control_status_pub = self.create_publisher(Float64, 'control_performance', 10)
        self.error_pub = self.create_publisher(Float64, 'tracking_error', 10)

        # Subscribers
        self.joint_cmd_sub = self.create_subscription(
            JointState, 'joint_commands', self.joint_command_callback, 10
        )
        self.trajectory_sub = self.create_subscription(
            JointTrajectory, 'joint_trajectory', self.trajectory_callback, 10
        )

        # Internal state
        self.current_positions = {}
        self.current_velocities = {}
        self.current_accelerations = {}
        self.joint_targets = {}
        self.pid_controllers = {}
        self.simulation_time = 0.0
        self.simulation_dt = 0.01  # 10ms timestep

        # Initialize robot simulation parameters
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.joint_limits = {
            'joint1': (-np.pi, np.pi),
            'joint2': (-np.pi/2, np.pi/2),
            'joint3': (-np.pi, np.pi),
            'joint4': (-np.pi/2, np.pi/2),
            'joint5': (-np.pi, np.pi),
            'joint6': (-np.pi/2, np.pi/2)
        }

        # Initialize PID controllers with realistic parameters
        for joint_name in self.joint_names:
            # Different parameters for different joint types
            if joint_name in ['joint1', 'joint2', 'joint3']:  # Larger joints
                params = {'kp': 50.0, 'ki': 1.0, 'kd': 5.0}
            else:  # Smaller joints
                params = {'kp': 25.0, 'ki': 0.5, 'kd': 2.5}

            self.pid_controllers[joint_name] = SimulatedPIDController(
                kp=params['kp'],
                ki=params['ki'],
                kd=params['kd'],
                output_limits=(-50.0, 50.0),  # Torque limits
                noise_std=0.01,  # 1% noise
                delay_ms=10,     # 10ms delay
                saturation=50.0  # Saturation at 50 Nm
            )
            self.current_positions[joint_name] = 0.0
            self.current_velocities[joint_name] = 0.0
            self.current_accelerations[joint_name] = 0.0
            self.joint_targets[joint_name] = 0.0

        # Simulation timer
        self.sim_timer = self.create_timer(self.simulation_dt, self.simulation_step)

        # Performance tracking
        self.error_history = {name: deque(maxlen=1000) for name in self.joint_names}
        self.performance_metrics = {name: {'mse': 0.0, 'mae': 0.0, 'max_error': 0.0} for name in self.joint_names}

        self.get_logger().info('Control system simulation initialized')

    def joint_command_callback(self, msg):
        """Update joint targets from commands"""
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_targets:
                self.joint_targets[name] = pos

    def trajectory_callback(self, msg):
        """Handle trajectory commands"""
        # This would implement trajectory following
        # For simulation, we'll use the trajectory points
        pass

    def simulation_step(self):
        """Main simulation step"""
        self.simulation_time += self.simulation_dt

        # Update each joint simulation
        for joint_name in self.joint_names:
            # Get PID control output
            target_pos = self.joint_targets[joint_name]
            current_pos = self.current_positions[joint_name]
            current_vel = self.current_velocities[joint_name]

            # Compute control effort
            control_effort, p_term, i_term, d_term = self.pid_controllers[joint_name].compute(
                target_pos, current_pos, self.simulation_dt
            )

            # Simulate motor dynamics
            # Simple second-order system: I*q_ddot + b*q_dot + k*q = tau
            # For simulation, we'll use: q_ddot = (tau - b*q_dot - k*q) / I
            inertia = 0.1  # kg*m^2
            damping = 0.05  # N*m*s/rad
            spring = 0.1    # N*m/rad

            acceleration = (control_effort - damping * current_vel - spring * current_pos) / inertia

            # Integrate to get velocity and position
            new_velocity = current_vel + acceleration * self.simulation_dt
            new_position = current_pos + new_velocity * self.simulation_dt

            # Apply joint limits
            min_pos, max_pos = self.joint_limits[joint_name]
            new_position = np.clip(new_position, min_pos, max_pos)

            # Update state
            self.current_accelerations[joint_name] = acceleration
            self.current_velocities[joint_name] = new_velocity
            self.current_positions[joint_name] = new_position

            # Calculate tracking error
            error = target_pos - new_position
            self.error_history[joint_name].append(error)

            # Update performance metrics
            self.update_performance_metrics(joint_name, error)

        # Publish simulation state
        self.publish_simulation_state()

        # Publish performance metrics
        self.publish_performance_metrics()

    def update_performance_metrics(self, joint_name, error):
        """Update performance metrics for a joint"""
        errors = list(self.error_history[joint_name])
        if errors:
            self.performance_metrics[joint_name]['mse'] = np.mean(np.array(errors)**2)
            self.performance_metrics[joint_name]['mae'] = np.mean(np.abs(errors))
            self.performance_metrics[joint_name]['max_error'] = np.max(np.abs(errors))

    def publish_simulation_state(self):
        """Publish simulated joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        for joint_name in self.joint_names:
            msg.name.append(joint_name)
            msg.position.append(self.current_positions[joint_name])
            msg.velocity.append(self.current_velocities[joint_name])
            msg.effort.append(self.current_accelerations[joint_name] * 0.1)  # Torque = I * alpha

        self.joint_state_pub.publish(msg)

    def publish_performance_metrics(self):
        """Publish performance metrics"""
        # Publish average performance metric
        avg_mse = np.mean([metrics['mse'] for metrics in self.performance_metrics.values()])
        perf_msg = Float64()
        perf_msg.data = avg_mse
        self.control_status_pub.publish(perf_msg)

        # Publish max error
        max_error = np.max([metrics['max_error'] for metrics in self.performance_metrics.values()])
        error_msg = Float64()
        error_msg.data = max_error
        self.error_pub.publish(error_msg)

    def simulate_disturbance(self, joint_name, magnitude=1.0):
        """Simulate external disturbance on a joint"""
        disturbance = np.random.normal(0, magnitude)
        self.current_accelerations[joint_name] += disturbance

def main(args=None):
    rclpy.init(args=args)
    node = ControlSystemSimulationNode()

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

## 8.2 Trajectory Planning Simulation

Let's create comprehensive trajectory planning simulation:

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Float64
import numpy as np
from scipy.interpolate import CubicSpline, BSpline
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from collections import deque

class TrajectoryPlanningSimulationNode(Node):
    def __init__(self):
        super().__init__('trajectory_planning_simulation')

        # Publishers
        self.trajectory_pub = self.create_publisher(JointTrajectory, 'simulated_trajectory', 10)
        self.performance_pub = self.create_publisher(Float64, 'trajectory_performance', 10)

        # Internal state
        self.planned_trajectories = {}
        self.executed_trajectories = {}
        self.trajectory_performance = {}

        # Initialize planners
        self.polynomial_planner = PolynomialTrajectoryPlanner()
        self.spline_planner = SplineTrajectoryPlanner()
        self.optimization_planner = OptimizationBasedPlanner()

        # Simulation parameters
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.max_velocities = {name: 2.0 for name in self.joint_names}  # rad/s
        self.max_accelerations = {name: 5.0 for name in self.joint_names}  # rad/s²
        self.max_jerk = {name: 10.0 for name in self.joint_names}  # rad/s³

        # Performance tracking
        self.performance_history = deque(maxlen=1000)

        # Timer for trajectory simulation
        self.trajectory_timer = self.create_timer(0.1, self.trajectory_simulation_step)

        self.get_logger().info('Trajectory planning simulation initialized')

    def plan_trajectory(self, start_positions, end_positions, duration, method='polynomial'):
        """Plan trajectory using specified method"""
        if method == 'polynomial':
            return self.polynomial_planner.plan_trajectory(start_positions, end_positions, duration)
        elif method == 'spline':
            return self.spline_planner.plan_trajectory(start_positions, end_positions, duration)
        elif method == 'optimization':
            return self.optimization_planner.plan_trajectory(start_positions, end_positions, duration)
        else:
            raise ValueError(f"Unknown planning method: {method}")

    def trajectory_simulation_step(self):
        """Simulate trajectory planning and execution"""
        # Example: Plan a trajectory from current position to random target
        start_pos = [0.0] * len(self.joint_names)
        end_pos = [np.random.uniform(-1, 1) for _ in self.joint_names]
        duration = 5.0

        try:
            # Plan trajectory using different methods
            poly_traj = self.plan_trajectory(start_pos, end_pos, duration, 'polynomial')
            spline_traj = self.plan_trajectory(start_pos, end_pos, duration, 'spline')
            opt_traj = self.plan_trajectory(start_pos, end_pos, duration, 'optimization')

            # Evaluate trajectory performance
            poly_perf = self.evaluate_trajectory_performance(poly_traj)
            spline_perf = self.evaluate_trajectory_performance(spline_traj)
            opt_perf = self.evaluate_trajectory_performance(opt_traj)

            # Publish best trajectory (lowest cost)
            best_traj = min([(poly_traj, poly_perf), (spline_traj, spline_perf), (opt_traj, opt_perf)], key=lambda x: x[1])[0]

            self.trajectory_pub.publish(best_traj)

            # Publish performance
            perf_msg = Float64()
            perf_msg.data = min(poly_perf, spline_perf, opt_perf)
            self.performance_pub.publish(perf_msg)

            # Store in history
            self.performance_history.append(perf_msg.data)

        except Exception as e:
            self.get_logger().error(f'Trajectory planning error: {e}')

    def evaluate_trajectory_performance(self, trajectory):
        """Evaluate trajectory performance based on various metrics"""
        if not trajectory.points:
            return float('inf')

        total_cost = 0.0

        # Evaluate smoothness (acceleration magnitude)
        for i in range(1, len(trajectory.points)):
            point = trajectory.points[i]
            prev_point = trajectory.points[i-1]

            # Smoothness cost based on acceleration
            if point.accelerations and prev_point.accelerations:
                acc_smoothness = np.sum(np.array(point.accelerations)**2)
                total_cost += acc_smoothness * 0.1

            # Velocity constraint violation
            if point.velocities:
                vel_violation = sum(max(0, abs(vel) - self.max_velocities[name])**2
                                  for vel, name in zip(point.velocities, trajectory.joint_names))
                total_cost += vel_violation * 0.05

            # Acceleration constraint violation
            if point.accelerations:
                acc_violation = sum(max(0, abs(acc) - self.max_accelerations[name])**2
                                  for acc, name in zip(point.accelerations, trajectory.joint_names))
                total_cost += acc_violation * 0.1

        # Duration cost (prefer shorter trajectories)
        total_duration = trajectory.points[-1].time_from_start.sec + \
                        trajectory.points[-1].time_from_start.nanosec / 1e9
        total_cost += total_duration * 0.01

        return total_cost

class PolynomialTrajectoryPlanner:
    """Plan trajectories using polynomial interpolation"""

    def plan_trajectory(self, start_positions, end_positions, duration, order=5):
        """Plan polynomial trajectory with smooth start/end conditions"""
        if len(start_positions) != len(end_positions):
            raise ValueError("Start and end positions must have same length")

        trajectory = JointTrajectory()
        trajectory.joint_names = [f'joint_{i}' for i in range(len(start_positions))]

        # Number of points based on desired resolution
        num_points = max(10, int(duration * 50))  # At least 10 points, 50 Hz minimum
        dt = duration / num_points

        for i in range(num_points + 1):
            t = i * dt
            fraction = min(1.0, t / duration)

            # Compute polynomial coefficients for smooth interpolation
            # Using quintic polynomial: s = 10*f^3 - 15*f^4 + 6*f^5
            # This provides zero velocity and acceleration at start and end
            s = 10 * fraction**3 - 15 * fraction**4 + 6 * fraction**5

            point = JointTrajectoryPoint()
            point.positions = []
            point.velocities = []
            point.accelerations = []

            for start_pos, end_pos in zip(start_positions, end_positions):
                # Position
                pos = start_pos + s * (end_pos - start_pos)
                point.positions.append(pos)

                # Velocity (derivative of position)
                if duration > 0:
                    ds_dt = (30 * fraction**2 - 60 * fraction**3 + 30 * fraction**4) / duration
                    vel = (end_pos - start_pos) * ds_dt
                    point.velocities.append(vel)

                    # Acceleration (derivative of velocity)
                    d2s_dt2 = (60 * fraction - 180 * fraction**2 + 120 * fraction**3) / (duration**2)
                    acc = (end_pos - start_pos) * d2s_dt2
                    point.accelerations.append(acc)
                else:
                    point.velocities.append(0.0)
                    point.accelerations.append(0.0)

            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            trajectory.points.append(point)

        return trajectory

class SplineTrajectoryPlanner:
    """Plan trajectories using spline interpolation"""

    def plan_trajectory(self, start_positions, end_positions, duration, num_waypoints=10):
        """Plan trajectory using cubic splines"""
        if len(start_positions) != len(end_positions):
            raise ValueError("Start and end positions must have same length")

        trajectory = JointTrajectory()
        trajectory.joint_names = [f'joint_{i}' for i in range(len(start_positions))]

        # Create intermediate waypoints
        num_points = max(10, int(duration * 50))  # 50 Hz minimum
        dt = duration / num_points

        # For each joint, create spline through waypoints
        for joint_idx in range(len(start_positions)):
            # Create waypoints with some variation
            waypoints = np.linspace(start_positions[joint_idx], end_positions[joint_idx], num_waypoints)
            # Add slight random variation to make it more interesting
            noise = np.random.normal(0, 0.05, len(waypoints))
            waypoints += noise

            # Times for waypoints
            waypoint_times = np.linspace(0, duration, num_waypoints)

            # Create spline
            spline = CubicSpline(waypoint_times, waypoints, bc_type='clamped')

            # Sample trajectory points
            for i in range(num_points + 1):
                t = i * dt

                # Get position, velocity, acceleration from spline
                pos = float(spline(t))
                vel = float(spline(t, 1))  # First derivative
                acc = float(spline(t, 2))  # Second derivative

                # Add to trajectory (we'll merge joint trajectories later)
                if i < len(trajectory.points):
                    trajectory.points[i].positions[joint_idx] = pos
                    trajectory.points[i].velocities[joint_idx] = vel
                    trajectory.points[i].accelerations[joint_idx] = acc
                else:
                    # Create new point
                    point = JointTrajectoryPoint()
                    point.positions = [0.0] * len(start_positions)
                    point.velocities = [0.0] * len(start_positions)
                    point.accelerations = [0.0] * len(start_positions)
                    point.positions[joint_idx] = pos
                    point.velocities[joint_idx] = vel
                    point.accelerations[joint_idx] = acc
                    point.time_from_start.sec = int(t)
                    point.time_from_start.nanosec = int((t - int(t)) * 1e9)
                    trajectory.points.append(point)

        return trajectory

class OptimizationBasedPlanner:
    """Plan trajectories using optimization techniques"""

    def plan_trajectory(self, start_positions, end_positions, duration, num_segments=20):
        """Plan trajectory by minimizing a cost function"""
        if len(start_positions) != len(end_positions):
            raise ValueError("Start and end positions must have same length")

        # Define the optimization problem
        # Minimize jerk (smoothness) while satisfying constraints
        def trajectory_cost(coefficients):
            """Cost function to minimize (jerk and constraint violations)"""
            # Reshape coefficients for each joint
            n_joints = len(start_positions)
            coeffs_per_joint = len(coefficients) // n_joints
            reshaped_coeffs = coefficients.reshape(n_joints, coeffs_per_joint)

            total_cost = 0.0

            for joint_idx, joint_coeffs in enumerate(reshaped_coeffs):
                # Create trajectory using polynomial coefficients
                # For simplicity, we'll use cubic polynomials
                # coeffs = [a0, a1, a2, a3] for polynomial: a0 + a1*t + a2*t^2 + a3*t^3
                if len(joint_coeffs) >= 4:
                    a0, a1, a2, a3 = joint_coeffs[:4]

                    # Evaluate polynomial and its derivatives over time
                    times = np.linspace(0, duration, 100)
                    positions = a0 + a1*times + a2*times**2 + a3*times**3
                    velocities = a1 + 2*a2*times + 3*a3*times**2
                    accelerations = 2*a2 + 6*a3*times
                    jerks = 6*a3  # Constant jerk for cubic polynomial

                    # Cost terms
                    jerk_cost = jerks**2 * duration
                    smoothness_cost = np.mean(accelerations**2)
                    boundary_cost = 0

                    # Boundary condition costs
                    if abs(positions[0] - start_positions[joint_idx]) > 0.01:
                        boundary_cost += 1000 * (positions[0] - start_positions[joint_idx])**2
                    if abs(positions[-1] - end_positions[joint_idx]) > 0.01:
                        boundary_cost += 1000 * (positions[-1] - end_positions[joint_idx])**2

                    total_cost += jerk_cost + smoothness_cost + boundary_cost

            return total_cost

        # Set up optimization problem
        n_joints = len(start_positions)
        n_coeffs_per_joint = 4  # cubic polynomial
        total_coeffs = n_joints * n_coeffs_per_joint

        # Initial guess: straight line (constant velocity)
        initial_coeffs = []
        for start_pos, end_pos in zip(start_positions, end_positions):
            # For straight line: pos(t) = start_pos + (end_pos - start_pos) * t / duration
            a0 = start_pos
            a1 = (end_pos - start_pos) / duration
            a2 = 0  # Zero acceleration for straight line
            a3 = 0  # Zero jerk for straight line
            initial_coeffs.extend([a0, a1, a2, a3])

        # Optimize
        result = minimize(
            trajectory_cost,
            np.array(initial_coeffs),
            method='BFGS',
            options={'maxiter': 1000}
        )

        # Create trajectory from optimized coefficients
        trajectory = JointTrajectory()
        trajectory.joint_names = [f'joint_{i}' for i in range(len(start_positions))]

        num_points = max(20, int(duration * 50))  # 50 Hz minimum
        dt = duration / num_points

        optimized_coeffs = result.x.reshape(n_joints, n_coeffs_per_joint)

        for i in range(num_points + 1):
            t = i * dt
            point = JointTrajectoryPoint()
            point.positions = []
            point.velocities = []
            point.accelerations = []

            for joint_idx, joint_coeffs in enumerate(optimized_coeffs):
                if len(joint_coeffs) >= 4:
                    a0, a1, a2, a3 = joint_coeffs[:4]

                    # Evaluate polynomial and derivatives
                    pos = a0 + a1*t + a2*t**2 + a3*t**3
                    vel = a1 + 2*a2*t + 3*a3*t**2
                    acc = 2*a2 + 6*a3*t

                    point.positions.append(pos)
                    point.velocities.append(vel)
                    point.accelerations.append(acc)
                else:
                    point.positions.append(0.0)
                    point.velocities.append(0.0)
                    point.accelerations.append(0.0)

            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            trajectory.points.append(point)

        return trajectory

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanningSimulationNode()

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

## 8.3 Safety System Simulation

Let's create a comprehensive safety system simulation:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64
from builtin_interfaces.msg import Time
import numpy as np
from collections import deque
import threading
import time

class SafetySystemSimulationNode(Node):
    def __init__(self):
        super().__init__('safety_system_simulation')

        # Publishers
        self.safety_status_pub = self.create_publisher(Bool, 'safety_status', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, 'emergency_stop', 10)
        self.safety_violation_pub = self.create_publisher(Float64, 'safety_violations', 10)
        self.hazard_level_pub = self.create_publisher(Float64, 'hazard_level', 10)

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 'simulated_joint_states', self.joint_state_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, 'simulated_imu', self.imu_callback, 10
        )
        self.velocity_cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.velocity_command_callback, 10
        )

        # Internal state
        self.current_joint_states = {}
        self.current_imu_data = {}
        self.current_velocity_cmd = None
        self.safety_violations = deque(maxlen=1000)
        self.hazard_history = deque(maxlen=100)
        self.emergency_active = False
        self.safety_override = False

        # Safety parameters
        self.joint_limits = {
            'joint1': (-np.pi, np.pi),
            'joint2': (-np.pi/2, np.pi/2),
            'joint3': (-np.pi, np.pi),
            'joint4': (-np.pi/2, np.pi/2),
            'joint5': (-np.pi, np.pi),
            'joint6': (-np.pi/2, np.pi/2)
        }
        self.velocity_limits = {name: 2.0 for name in self.joint_limits.keys()}
        self.acceleration_limits = {name: 5.0 for name in self.joint_limits.keys()}
        self.torque_limits = {name: 50.0 for name in self.joint_limits.keys()}

        # IMU safety parameters
        self.max_imu_acceleration = 19.62  # 2g
        self.max_imu_angular_velocity = 10.0  # rad/s

        # Velocity command limits
        self.max_linear_velocity = 1.0  # m/s
        self.max_angular_velocity = 1.0  # rad/s

        # Safety monitoring
        self.safety_lock = threading.Lock()
        self.violation_threshold = 0.1  # Fraction of limits that constitutes violation
        self.emergency_threshold = 0.5  # Fraction that triggers emergency

        # Safety timers
        self.safety_timer = self.create_timer(0.01, self.safety_monitoring_loop)  # 100 Hz
        self.hazard_assessment_timer = self.create_timer(0.1, self.assess_hazard_level)  # 10 Hz

        # Simulated hazards
        self.hazard_simulation_timer = self.create_timer(5.0, self.simulate_hazards)

        self.get_logger().info('Safety system simulation initialized')

    def joint_state_callback(self, msg):
        """Monitor joint states for safety violations"""
        with self.safety_lock:
            for name, pos, vel, effort in zip(msg.name, msg.position, msg.velocity, msg.effort):
                if name in self.joint_limits:
                    self.current_joint_states[name] = {
                        'position': pos,
                        'velocity': vel,
                        'effort': effort
                    }

    def imu_callback(self, msg):
        """Monitor IMU data for safety"""
        with self.safety_lock:
            self.current_imu_data = {
                'linear_acceleration': [
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z
                ],
                'angular_velocity': [
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z
                ],
                'orientation': [
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w
                ]
            }

    def velocity_command_callback(self, msg):
        """Monitor velocity commands"""
        with self.safety_lock:
            self.current_velocity_cmd = {
                'linear': [msg.linear.x, msg.linear.y, msg.linear.z],
                'angular': [msg.angular.x, msg.angular.y, msg.angular.z]
            }

    def safety_monitoring_loop(self):
        """Continuous safety monitoring"""
        with self.safety_lock:
            if self.emergency_active:
                # Publish emergency stop command
                self.publish_emergency_stop()
                return

            # Check all safety conditions
            self.check_joint_safety()
            self.check_imu_safety()
            self.check_command_safety()
            self.check_collision_risk()

            # Update safety status
            safe_status = Bool()
            safe_status.data = not self.emergency_active
            self.safety_status_pub.publish(safe_status)

    def check_joint_safety(self):
        """Check joint safety conditions"""
        for joint_name, state in self.current_joint_states.items():
            if joint_name in self.joint_limits:
                # Position limits
                min_pos, max_pos = self.joint_limits[joint_name]
                pos = state['position']

                pos_violation_ratio = max(
                    abs(pos - min_pos) / (max_pos - min_pos),
                    abs(pos - max_pos) / (max_pos - min_pos)
                )

                if pos_violation_ratio > self.violation_threshold:
                    self.record_safety_violation(
                        f'POSITION_LIMIT_WARNING: {joint_name}={pos:.3f}, ratio={pos_violation_ratio:.3f}',
                        severity='warning'
                    )

                if pos_violation_ratio > self.emergency_threshold:
                    self.record_safety_violation(
                        f'POSITION_LIMIT_EMERGENCY: {joint_name}={pos:.3f}, ratio={pos_violation_ratio:.3f}',
                        severity='emergency'
                    )
                    self.trigger_emergency_stop()

                # Velocity limits
                vel = abs(state['velocity'])
                max_vel = self.velocity_limits[joint_name]

                vel_violation_ratio = vel / max_vel

                if vel_violation_ratio > self.violation_threshold:
                    self.record_safety_violation(
                        f'VELOCITY_LIMIT_WARNING: {joint_name}={vel:.3f}, ratio={vel_violation_ratio:.3f}',
                        severity='warning'
                    )

                if vel_violation_ratio > self.emergency_threshold:
                    self.record_safety_violation(
                        f'VELOCITY_LIMIT_EMERGENCY: {joint_name}={vel:.3f}, ratio={vel_violation_ratio:.3f}',
                        severity='emergency'
                    )
                    self.trigger_emergency_stop()

                # Torque limits
                torque = abs(state['effort'])
                max_torque = self.torque_limits[joint_name]

                torque_violation_ratio = torque / max_torque

                if torque_violation_ratio > self.violation_threshold:
                    self.record_safety_violation(
                        f'TORQUE_LIMIT_WARNING: {joint_name}={torque:.3f}, ratio={torque_violation_ratio:.3f}',
                        severity='warning'
                    )

                if torque_violation_ratio > self.emergency_threshold:
                    self.record_safety_violation(
                        f'TORQUE_LIMIT_EMERGENCY: {joint_name}={torque:.3f}, ratio={torque_violation_ratio:.3f}',
                        severity='emergency'
                    )
                    self.trigger_emergency_stop()

    def check_imu_safety(self):
        """Check IMU-based safety conditions"""
        if self.current_imu_data:
            # Check linear acceleration magnitude
            accel = np.array(self.current_imu_data['linear_acceleration'])
            accel_magnitude = np.linalg.norm(accel)

            if accel_magnitude > self.max_imu_acceleration:
                self.record_safety_violation(
                    f'IMU_ACCELERATION_VIOLATION: {accel_magnitude:.3f} m/s² > {self.max_imu_acceleration:.3f} m/s²',
                    severity='emergency'
                )
                self.trigger_emergency_stop()

            # Check angular velocity magnitude
            ang_vel = np.array(self.current_imu_data['angular_velocity'])
            ang_vel_magnitude = np.linalg.norm(ang_vel)

            if ang_vel_magnitude > self.max_imu_angular_velocity:
                self.record_safety_violation(
                    f'IMU_ANGULAR_VELOCITY_VIOLATION: {ang_vel_magnitude:.3f} rad/s > {self.max_imu_angular_velocity:.3f} rad/s',
                    severity='emergency'
                )
                self.trigger_emergency_stop()

    def check_command_safety(self):
        """Check command safety"""
        if self.current_velocity_cmd:
            # Check linear velocity
            linear_vel = np.linalg.norm(self.current_velocity_cmd['linear'])
            if linear_vel > self.max_linear_velocity:
                self.record_safety_violation(
                    f'COMMAND_LINEAR_VEL_VIOLATION: {linear_vel:.3f} m/s > {self.max_linear_velocity:.3f} m/s',
                    severity='warning'
                )

            # Check angular velocity
            angular_vel = np.linalg.norm(self.current_velocity_cmd['angular'])
            if angular_vel > self.max_angular_velocity:
                self.record_safety_violation(
                    f'COMMAND_ANGULAR_VEL_VIOLATION: {angular_vel:.3f} rad/s > {self.max_angular_velocity:.3f} rad/s',
                    severity='warning'
                )

    def check_collision_risk(self):
        """Check for potential collision risks"""
        # This would typically use sensor data (LiDAR, cameras, etc.)
        # For simulation, we'll use a simplified approach
        # In a real system, this would integrate data from proximity sensors
        pass

    def simulate_hazards(self):
        """Simulate various hazards for testing"""
        # Simulate random hazards to test safety system response
        hazard_types = ['position_limit', 'velocity_limit', 'torque_limit', 'imu_violation']

        if np.random.random() < 0.1:  # 10% chance per 5 seconds
            hazard = np.random.choice(hazard_types)

            if hazard == 'position_limit':
                # Simulate position limit violation
                self.get_logger().info('Simulated position limit violation')
            elif hazard == 'velocity_limit':
                # Simulate velocity limit violation
                self.get_logger().info('Simulated velocity limit violation')
            elif hazard == 'torque_limit':
                # Simulate torque limit violation
                self.get_logger().info('Simulated torque limit violation')
            elif hazard == 'imu_violation':
                # Simulate IMU violation
                self.get_logger().info('Simulated IMU violation')

    def record_safety_violation(self, message, severity='warning'):
        """Record a safety violation"""
        violation = {
            'timestamp': time.time(),
            'message': message,
            'severity': severity
        }
        self.safety_violations.append(violation)

        # Log violation
        if severity == 'emergency':
            self.get_logger().fatal(f'SAFETY VIOLATION: {message}')
        elif severity == 'warning':
            self.get_logger().warn(f'Safety Warning: {message}')

        # Publish violation count
        violation_count_msg = Float64()
        violation_count_msg.data = len(self.safety_violations)
        self.safety_violation_pub.publish(violation_count_msg)

    def trigger_emergency_stop(self):
        """Trigger emergency stop"""
        if not self.emergency_active:
            self.emergency_active = True
            self.get_logger().fatal('EMERGENCY STOP ACTIVATED')

            # Publish emergency stop
            emergency_msg = Bool()
            emergency_msg.data = True
            self.emergency_stop_pub.publish(emergency_msg)

            # Log all violations that led to emergency
            for violation in list(self.safety_violations)[-10:]:  # Last 10 violations
                if violation['severity'] == 'emergency':
                    self.get_logger().error(f'Emergency-triggering violation: {violation["message"]}')

    def publish_emergency_stop(self):
        """Publish emergency stop command"""
        # This would publish zero commands to all controllers
        # For simulation, we just publish the emergency status
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_stop_pub.publish(emergency_msg)

    def assess_hazard_level(self):
        """Assess overall hazard level"""
        with self.safety_lock:
            # Calculate hazard level based on recent violations
            if self.safety_violations:
                recent_violations = [v for v in self.safety_violations
                                   if time.time() - v['timestamp'] < 5.0]  # Last 5 seconds

                # Weighted hazard assessment
                hazard_score = 0.0
                for violation in recent_violations:
                    weight = 1.0 if violation['severity'] == 'emergency' else 0.5
                    hazard_score += weight

                # Normalize by time window
                hazard_score = min(1.0, hazard_score / 10.0)  # Max 10 violations in window = 1.0
            else:
                hazard_score = 0.0

            # Add to history
            self.hazard_history.append(hazard_score)

            # Publish hazard level
            hazard_msg = Float64()
            hazard_msg.data = hazard_score
            self.hazard_level_pub.publish(hazard_msg)

    def reset_safety_system(self):
        """Reset safety system (simulated manual reset)"""
        with self.safety_lock:
            self.emergency_active = False
            self.safety_violations.clear()
            self.get_logger().info('Safety system reset')

            # Publish reset confirmation
            reset_msg = Bool()
            reset_msg.data = False
            self.emergency_stop_pub.publish(reset_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetySystemSimulationNode()

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

## 8.4 Cartesian Control Simulation

Let's create a Cartesian space control simulation:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import numpy as np
from scipy.spatial.transform import Rotation as R
import transforms3d

class CartesianControlSimulationNode(Node):
    def __init__(self):
        super().__init__('cartesian_control_simulation')

        # Publishers
        self.cartesian_pose_pub = self.create_publisher(Pose, 'simulated_cartesian_pose', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'simulated_joint_states', 10)
        self.error_pub = self.create_publisher(Float64, 'cartesian_error', 10)

        # Subscribers
        self.cartesian_cmd_sub = self.create_subscription(
            Pose, 'cartesian_command', self.cartesian_command_callback, 10
        )

        # Internal state
        self.current_cartesian_pose = np.array([0.5, 0.0, 0.3, 0, 0, 0, 1])  # x, y, z, qx, qy, qz, qw
        self.current_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 6 joints
        self.target_cartesian_pose = None
        self.cartesian_error_history = []

        # Robot kinematic model (simplified 6-DOF)
        self.robot_model = SimplifiedRobotModel()

        # Cartesian control parameters
        self.cartesian_gains = {
            'position': np.eye(3) * 1.0,
            'orientation': np.eye(3) * 0.5
        }
        self.max_cartesian_velocity = 0.1  # m/s
        self.max_orientation_velocity = 0.5  # rad/s

        # Control timer
        self.control_timer = self.create_timer(0.01, self.cartesian_control_step)  # 100 Hz

        self.get_logger().info('Cartesian control simulation initialized')

    def cartesian_command_callback(self, msg):
        """Handle Cartesian command"""
        self.target_cartesian_pose = np.array([
            msg.position.x,
            msg.position.y,
            msg.position.z,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])
        self.get_logger().info(f'New Cartesian target: [{msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f}]')

    def cartesian_control_step(self):
        """Main Cartesian control step"""
        if self.target_cartesian_pose is None:
            return

        # Get current pose
        current_pos = self.current_cartesian_pose[:3]
        current_rot = R.from_quat(self.current_cartesian_pose[3:])

        # Get target pose
        target_pos = self.target_cartesian_pose[:3]
        target_rot = R.from_quat(self.target_cartesian_pose[3:])

        # Calculate position error
        pos_error = target_pos - current_pos

        # Calculate orientation error using rotation vectors
        relative_rot = target_rot.inv() * current_rot
        rot_error = relative_rot.as_rotvec()

        # Apply Cartesian control gains
        pos_correction = self.cartesian_gains['position'] @ pos_error
        rot_correction = self.cartesian_gains['orientation'] @ rot_error

        # Limit Cartesian velocity
        pos_norm = np.linalg.norm(pos_correction)
        if pos_norm > self.max_cartesian_velocity:
            pos_correction = (pos_correction / pos_norm) * self.max_cartesian_velocity

        rot_norm = np.linalg.norm(rot_correction)
        if rot_norm > self.max_orientation_velocity:
            rot_correction = (rot_correction / rot_norm) * self.max_orientation_velocity

        # Convert Cartesian corrections to joint space using Jacobian
        try:
            jacobian = self.robot_model.compute_jacobian(self.current_joint_positions)

            # Combine position and orientation corrections
            cartesian_correction = np.concatenate([pos_correction, rot_correction])

            # Convert to joint space (transpose Jacobian for differential IK)
            joint_correction = np.linalg.pinv(jacobian) @ cartesian_correction

            # Apply joint corrections
            new_joint_positions = np.array(self.current_joint_positions) + joint_correction * 0.01  # Small step

            # Update joint positions (with limits)
            joint_limits = [(-np.pi, np.pi)] * 6
            for i, (min_lim, max_lim) in enumerate(joint_limits):
                new_joint_positions[i] = np.clip(new_joint_positions[i], min_lim, max_lim)

            self.current_joint_positions = new_joint_positions.tolist()

            # Update Cartesian pose using forward kinematics
            self.current_cartesian_pose = self.robot_model.forward_kinematics(self.current_joint_positions)

        except np.linalg.LinAlgError:
            self.get_logger().warn('Jacobian is singular, skipping control step')

        # Calculate tracking error
        tracking_error = np.linalg.norm(pos_error)
        self.cartesian_error_history.append(tracking_error)

        # Publish updated states
        self.publish_states()

        # Publish error
        error_msg = Float64()
        error_msg.data = tracking_error
        self.error_pub.publish(error_msg)

    def publish_states(self):
        """Publish current states"""
        # Publish Cartesian pose
        pose_msg = Pose()
        pose_msg.position.x = float(self.current_cartesian_pose[0])
        pose_msg.position.y = float(self.current_cartesian_pose[1])
        pose_msg.position.z = float(self.current_cartesian_pose[2])
        pose_msg.orientation.x = float(self.current_cartesian_pose[3])
        pose_msg.orientation.y = float(self.current_cartesian_pose[4])
        pose_msg.orientation.z = float(self.current_cartesian_pose[5])
        pose_msg.orientation.w = float(self.current_cartesian_pose[6])
        self.cartesian_pose_pub.publish(pose_msg)

        # Publish joint states
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [f'joint_{i}' for i in range(len(self.current_joint_positions))]
        joint_msg.position = self.current_joint_positions
        joint_msg.velocity = [0.0] * len(self.current_joint_positions)  # Simplified
        joint_msg.effort = [0.0] * len(self.current_joint_positions)   # Simplified
        self.joint_state_pub.publish(joint_msg)

class SimplifiedRobotModel:
    """Simplified 6-DOF robot model for simulation"""

    def __init__(self):
        # Simple 6-DOF manipulator with link lengths
        self.link_lengths = [0.5, 0.4, 0.3, 0.2, 0.15, 0.1]  # meters

    def forward_kinematics(self, joint_angles):
        """Compute forward kinematics"""
        if len(joint_angles) != 6:
            raise ValueError("Expected 6 joint angles")

        # Simplified FK computation
        # This is a significant simplification - real implementation would use DH parameters
        x = sum([l * np.cos(sum(joint_angles[:i+1])) for i, l in enumerate(self.link_lengths)])
        y = sum([l * np.sin(sum(joint_angles[:i+1])) for i, l in enumerate(self.link_lengths)])
        z = 0.1  # Fixed height for simplicity

        # For orientation, we'll use the sum of joint angles as a simple representation
        total_rotation = sum(joint_angles[:3])  # First 3 joints for position, last 3 for orientation
        orientation = R.from_rotvec([0, 0, total_rotation]).as_quat()

        return np.array([x, y, z, orientation[0], orientation[1], orientation[2], orientation[3]])

    def compute_jacobian(self, joint_angles):
        """Compute geometric Jacobian"""
        if len(joint_angles) != 6:
            raise ValueError("Expected 6 joint angles")

        jacobian = np.zeros((6, 6))  # 6 DOF (pos + rot), 6 joints

        # Compute end-effector position using FK
        ee_pos = self.forward_kinematics(joint_angles)[:3]

        # Compute Jacobian numerically (simplified approach)
        delta_q = 0.001  # Small change in joint angle

        for i in range(6):
            # Positive perturbation
            joint_angles_plus = joint_angles.copy()
            joint_angles_plus[i] += delta_q
            ee_pos_plus = self.forward_kinematics(joint_angles_plus)[:3]

            # Negative perturbation
            joint_angles_minus = joint_angles.copy()
            joint_angles_minus[i] -= delta_q
            ee_pos_minus = self.forward_kinematics(joint_angles_minus)[:3]

            # Compute position effect
            pos_effect = (ee_pos_plus - ee_pos_minus) / (2 * delta_q)
            jacobian[:3, i] = pos_effect

            # For orientation effect (simplified)
            jacobian[3:, i] = np.array([0, 0, 1])  # Simplified

        return jacobian

def main(args=None):
    rclpy.init(args=args)
    node = CartesianControlSimulationNode()

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

## 8.5 Performance Validation Simulation

Let's create a comprehensive performance validation system:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import numpy as np
from collections import deque
import time
import statistics

class PerformanceValidationNode(Node):
    def __init__(self):
        super().__init__('performance_validation')

        # Publishers
        self.performance_metrics_pub = self.create_publisher(Float64, 'performance_metrics', 10)
        self.validation_report_pub = self.create_publisher(String, 'validation_report', 10)

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 'simulated_joint_states', self.joint_state_callback, 10
        )
        self.cartesian_pose_sub = self.create_subscription(
            Pose, 'simulated_cartesian_pose', self.cartesian_pose_callback, 10
        )
        self.command_sub = self.create_subscription(
            JointState, 'joint_commands', self.command_callback, 10
        )
        self.error_sub = self.create_subscription(
            Float64, 'tracking_error', self.error_callback, 10
        )

        # Internal state
        self.joint_positions = {}
        self.joint_velocities = {}
        self.joint_commands = {}
        self.cartesian_errors = deque(maxlen=1000)
        self.performance_history = deque(maxlen=1000)
        self.validation_results = {}
        self.start_time = time.time()

        # Performance metrics
        self.metrics = {
            'tracking_accuracy': 0.0,
            'stability': 0.0,
            'responsiveness': 0.0,
            'energy_efficiency': 0.0,
            'overshoot': 0.0,
            'settling_time': 0.0
        }

        # Validation parameters
        self.validation_window = 100  # Number of samples for validation
        self.steady_state_threshold = 0.01  # Threshold for steady state
        self.error_threshold = 0.05  # Acceptable error threshold

        # Validation timer
        self.validation_timer = self.create_timer(0.1, self.validate_performance)

        self.get_logger().info('Performance validation system initialized')

    def joint_state_callback(self, msg):
        """Update joint state data"""
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            self.joint_positions[name] = pos
            self.joint_velocities[name] = vel

    def cartesian_pose_callback(self, msg):
        """Update Cartesian pose data"""
        # This would be used for Cartesian space validation
        pass

    def command_callback(self, msg):
        """Update command data"""
        for name, pos in zip(msg.name, msg.position):
            self.joint_commands[name] = pos

    def error_callback(self, msg):
        """Update error data"""
        self.cartesian_errors.append(msg.data)

    def validate_performance(self):
        """Validate control system performance"""
        if len(self.cartesian_errors) < self.validation_window:
            return

        # Calculate various performance metrics
        recent_errors = list(self.cartesian_errors)[-self.validation_window:]

        # Tracking accuracy (RMSE)
        rmse = np.sqrt(np.mean(np.array(recent_errors)**2))
        self.metrics['tracking_accuracy'] = 1.0 / (1.0 + rmse)  # Higher is better

        # Stability (variance of errors)
        stability = 1.0 / (1.0 + np.var(recent_errors))
        self.metrics['stability'] = stability

        # Responsiveness (based on how quickly error decreases)
        if len(recent_errors) > 10:
            early_errors = np.mean(recent_errors[:10])
            late_errors = np.mean(recent_errors[-10:])
            responsiveness = max(0, 1 - (late_errors / (early_errors + 1e-6)))
            self.metrics['responsiveness'] = responsiveness

        # Energy efficiency (based on control effort - simplified)
        # In a real system, this would measure actual power consumption
        avg_error = np.mean(recent_errors)
        self.metrics['energy_efficiency'] = 1.0 / (1.0 + avg_error)

        # Overshoot (difference between peak error and final error)
        if len(recent_errors) > 10:
            peak_error = max(recent_errors[:-10])  # Peak in early part
            final_error = np.mean(recent_errors[-10:])  # Final error
            overshoot = max(0, peak_error - final_error)
            self.metrics['overshoot'] = 1.0 / (1.0 + overshoot)

        # Settling time (time to reach and stay within threshold)
        settling_time = self.calculate_settling_time(recent_errors)
        self.metrics['settling_time'] = 1.0 / (1.0 + settling_time)  # Lower settling time = higher score

        # Calculate overall performance score
        weights = {
            'tracking_accuracy': 0.3,
            'stability': 0.2,
            'responsiveness': 0.2,
            'energy_efficiency': 0.15,
            'overshoot': 0.1,
            'settling_time': 0.05
        }

        overall_score = sum(self.metrics[key] * weights[key] for key in weights)

        # Publish performance metrics
        perf_msg = Float64()
        perf_msg.data = overall_score
        self.performance_metrics_pub.publish(perf_msg)

        # Store in history
        self.performance_history.append(overall_score)

        # Generate validation report if needed
        if overall_score < 0.5:  # Poor performance
            self.generate_validation_report(overall_score)

    def calculate_settling_time(self, errors):
        """Calculate settling time - time to stay within threshold"""
        threshold = self.steady_state_threshold
        settling_start = -1

        # Find when error stays within threshold
        for i, error in enumerate(errors):
            if abs(error) <= threshold:
                if settling_start == -1:
                    settling_start = i
            else:
                settling_start = -1  # Reset if error goes above threshold

        if settling_start != -1:
            settling_samples = len(errors) - settling_start
            settling_time = settling_samples * 0.1  # Assuming 10Hz sampling
            return settling_time
        else:
            return len(errors) * 0.1  # Didn't settle within window

    def generate_validation_report(self, performance_score):
        """Generate detailed validation report"""
        report = f"""
CONTROL SYSTEM VALIDATION REPORT
===============================
Timestamp: {time.time()}
Performance Score: {performance_score:.3f}

METRICS BREAKDOWN:
- Tracking Accuracy: {self.metrics['tracking_accuracy']:.3f}
- Stability: {self.metrics['stability']:.3f}
- Responsiveness: {self.metrics['responsiveness']:.3f}
- Energy Efficiency: {self.metrics['energy_efficiency']:.3f}
- Overshoot Control: {self.metrics['overshoot']:.3f}
- Settling Time: {self.metrics['settling_time']:.3f}

ANALYSIS:
- Average Error: {np.mean(list(self.cartesian_errors)[-self.validation_window:]):.4f}
- Error Variance: {np.var(list(self.cartesian_errors)[-self.validation_window:]):.4f}
- Max Error: {max(list(self.cartesian_errors)[-self.validation_window:]):.4f}
- Performance Trend: {'Improving' if len(self.performance_history) > 10 and np.polyfit(range(len(list(self.performance_history)[-10:])), list(self.performance_history)[-10:], 1)[0] > 0 else 'Declining'}

RECOMMENDATIONS:
"""

        if self.metrics['tracking_accuracy'] < 0.5:
            report += "- Improve tracking accuracy through PID tuning\n"
        if self.metrics['stability'] < 0.5:
            report += "- Investigate stability issues, consider reducing gains\n"
        if self.metrics['responsiveness'] < 0.5:
            report += "- System is too slow, consider increasing gains\n"
        if self.metrics['overshoot'] < 0.5:
            report += "- Reduce overshoot by adjusting D-term\n"

        report += "\nSYSTEM STATUS: "
        if performance_score > 0.8:
            report += "EXCELLENT"
        elif performance_score > 0.6:
            report += "GOOD"
        elif performance_score > 0.4:
            report += "FAIR"
        else:
            report += "POOR - IMMEDIATE ATTENTION REQUIRED"

        # Publish report
        report_msg = String()
        report_msg.data = report
        self.validation_report_pub.publish(report_msg)

        self.get_logger().info(f'Validation report generated: {report.split(chr(10))[0]}')

    def get_performance_summary(self):
        """Get summary of performance metrics"""
        if not self.performance_history:
            return "No performance data available"

        avg_performance = np.mean(list(self.performance_history))
        min_performance = min(list(self.performance_history))
        max_performance = max(list(self.performance_history))
        performance_std = np.std(list(self.performance_history))

        summary = f"""
Performance Summary:
- Average: {avg_performance:.3f}
- Min: {min_performance:.3f}
- Max: {max_performance:.3f}
- Std Dev: {performance_std:.3f}
- Total Samples: {len(self.performance_history)}
        """

        return summary

def main(args=None):
    rclpy.init(args=args)
    node = PerformanceValidationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Print final summary
        summary = node.get_performance_summary()
        node.get_logger().info(summary)
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 8.6 Practical Exercise

Create a complete control system simulation that:
1. Implements multiple control strategies
2. Includes trajectory planning and execution
3. Features comprehensive safety systems
4. Provides performance validation
5. Tests emergency procedures

```python
# Student exercise - Complete implementation
class CompleteControlSimulation:
    """Student implementation of a complete control system simulation"""

    def __init__(self):
        """Initialize the complete control simulation"""
        # TODO: Implement PID, LQR, MPC controllers
        # TODO: Implement trajectory planning algorithms
        # TODO: Add safety systems
        # TODO: Implement performance validation
        # TODO: Add emergency procedures
        # TODO: Create integrated simulation environment
        pass

    def implement_control_strategies(self):
        """Implement multiple control strategies"""
        # TODO: Complete implementation
        pass

    def implement_trajectory_planning(self):
        """Implement trajectory planning and execution"""
        # TODO: Complete implementation
        pass

    def implement_safety_systems(self):
        """Implement comprehensive safety systems"""
        # TODO: Complete implementation
        pass

    def implement_validation(self):
        """Implement performance validation"""
        # TODO: Complete implementation
        pass

    def implement_emergency_procedures(self):
        """Implement emergency procedures"""
        # TODO: Complete implementation
        pass

    def create_simulation_environment(self):
        """Create integrated simulation environment"""
        # TODO: Complete implementation
        pass

print("Student Exercise: Implement a complete control system simulation")
print("Requirements:")
print("1. Multiple control strategies (PID, LQR, MPC)")
print("2. Trajectory planning and execution")
print("3. Comprehensive safety systems")
print("4. Performance validation and monitoring")
print("5. Emergency procedures and recovery")
print("6. Integrated simulation environment")
```

## 8.7 Best Practices for Control System Simulation

### Performance Considerations
- Use appropriate simulation timestep (typically 1-10ms for control)
- Implement efficient numerical integration methods
- Consider computational complexity of algorithms
- Use multithreading for non-critical computations

### Safety in Simulation
- Always implement safety systems in simulation before real deployment
- Test edge cases and failure modes
- Validate safety systems with realistic sensor noise
- Implement comprehensive logging and monitoring

### Validation Approaches
- Compare simulation results with analytical models
- Use hardware-in-the-loop (HIL) testing
- Implement statistical validation methods
- Test across multiple operating conditions

## Summary

Control system simulation provides a safe and cost-effective environment to develop, test, and validate control algorithms before deployment on real hardware. Understanding how to simulate realistic control scenarios, including sensor noise, actuator dynamics, and safety constraints, is crucial for developing robust robotic systems.

<div class="alert alert-info">
  <h5>Hardware Requirement</h5>
  <div><strong>Requirement:</strong> GPU</div>
  <div><strong>Minimum:</strong> Any</div>
  <div><strong>Recommended:</strong> Any</div>
  <div><strong>Purpose:</strong> Basic computational requirements for Week 8 control system simulation</div>
</div>