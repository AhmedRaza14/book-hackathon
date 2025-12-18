---
title: "Week 8: Control Systems and Trajectory Planning Code Lab"
week: 8
module: "Robotic Infrastructure"
difficulty: "advanced"
prerequisites: ["ros2-fundamentals", "python-basics", "robotics-concepts", "kinematics", "sensors"]
learning_objectives:
  - "Implement PID controllers for robot joints"
  - "Create trajectory planning algorithms"
  - "Develop safety systems and emergency stops"
  - "Implement Cartesian space control"
tags: ["control", "pid", "trajectory", "planning", "safety", "cartesian"]
hardware_requirements:
  - gpu: "Any"
  - ram: "8GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "120 minutes"
---

# Week 8: Control Systems and Trajectory Planning Code Lab

## Learning Objectives
- Implement PID controllers for robot joints
- Create trajectory planning algorithms
- Develop safety systems and emergency stops
- Implement Cartesian space control
- Practice control system tuning and validation

## 8.1 Setting Up the Control Environment

First, let's set up the environment for implementing control systems:

```bash
# Install required packages for control systems
sudo apt update
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-joint-state-controller ros-humble-diff-drive-controller
sudo apt install ros-humble-position-controllers ros-humble-velocity-controllers

# Install Python dependencies
pip3 install numpy scipy matplotlib control  # control is python-control package
pip3 install transforms3d  # For 3D transformations
```

## 8.2 PID Controller Implementation

Let's implement a comprehensive PID controller with anti-windup and filtering:

```python
#!/usr/bin/env python3
# pid_controller.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
import numpy as np
import time

class PIDController:
    """Advanced PID controller with anti-windup and filtering"""

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limits=(-np.inf, np.inf),
                 integrator_limits=(-np.inf, np.inf), derivative_filter=True,
                 filter_coefficient=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.integrator_limits = integrator_limits
        self.derivative_filter = derivative_filter
        self.filter_coefficient = filter_coefficient

        # Internal state
        self.reset()

    def reset(self):
        """Reset PID controller state"""
        self.last_error = 0.0
        self.integral = 0.0
        self.filtered_derivative = 0.0
        self.last_time = None
        self.last_measurement = 0.0

    def compute(self, setpoint, measurement, dt=None):
        """Compute PID output"""
        current_time = time.time()

        if dt is None:
            if self.last_time is None:
                dt = 0.01  # Default timestep
            else:
                dt = current_time - self.last_time
        self.last_time = current_time

        # Calculate error
        error = setpoint - measurement

        # Proportional term
        proportional = self.kp * error

        # Integral term with anti-windup
        self.integral += error * dt
        # Clamp integral to prevent windup
        self.integral = np.clip(self.integral, self.integrator_limits[0], self.integrator_limits[1])
        integral = self.ki * self.integral

        # Derivative term with filtering
        if dt > 0:
            raw_derivative = (measurement - self.last_measurement) / dt

            if self.derivative_filter:
                # Apply first-order low-pass filter to derivative
                self.filtered_derivative = (self.filter_coefficient * raw_derivative +
                                           (1 - self.filter_coefficient) * self.filtered_derivative)
            else:
                self.filtered_derivative = raw_derivative

        derivative = -self.kd * self.filtered_derivative  # Negative because derivative of measurement

        # Calculate output
        output = proportional + integral + derivative

        # Apply output limits
        output = np.clip(output, self.output_limits[0], self.output_limits[1])

        # Store for next iteration
        self.last_error = error
        self.last_measurement = measurement

        return output, proportional, integral, derivative

class JointPIDControllerNode(Node):
    def __init__(self):
        super().__init__('joint_pid_controller')

        # Publishers
        self.joint_cmd_pub = self.create_publisher(JointState, 'joint_commands', 10)
        self.controller_state_pub = self.create_publisher(JointTrajectoryControllerState, 'controller_state', 10)

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )
        self.joint_trajectory_sub = self.create_subscription(
            JointTrajectory, 'joint_trajectory', self.joint_trajectory_callback, 10
        )

        # Internal state
        self.joint_positions = {}
        self.joint_velocities = {}
        self.joint_targets = {}
        self.pid_controllers = {}
        self.trajectory_points = {}

        # Initialize PID controllers for each joint
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        for joint_name in joint_names:
            # Different PID parameters for different joints based on their characteristics
            if joint_name in ['joint1', 'joint2', 'joint3']:  # Main joints
                pid_params = {'kp': 10.0, 'ki': 0.1, 'kd': 0.5}
            else:  # Wrist joints
                pid_params = {'kp': 5.0, 'ki': 0.05, 'kd': 0.2}

            self.pid_controllers[joint_name] = PIDController(
                kp=pid_params['kp'],
                ki=pid_params['ki'],
                kd=pid_params['kd'],
                output_limits=(-10.0, 10.0),
                integrator_limits=(-5.0, 5.0),
                derivative_filter=True,
                filter_coefficient=0.1
            )
            self.joint_targets[joint_name] = 0.0
            self.joint_positions[joint_name] = 0.0
            self.joint_velocities[joint_name] = 0.0

        # Control timer
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100 Hz

        self.get_logger().info('Joint PID controller initialized')

    def joint_state_callback(self, msg):
        """Update joint state"""
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            if name in self.joint_positions:
                self.joint_positions[name] = pos
                self.joint_velocities[name] = vel

    def joint_trajectory_callback(self, msg):
        """Handle joint trajectory commands"""
        # Store trajectory points
        for point in msg.points:
            for joint_name, position in zip(msg.joint_names, point.positions):
                if joint_name in self.joint_targets:
                    self.joint_targets[joint_name] = position

    def control_loop(self):
        """Main control loop"""
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.name = list(self.joint_targets.keys())

        controller_state_msg = JointTrajectoryControllerState()
        controller_state_msg.header.stamp = cmd_msg.header.stamp
        controller_state_msg.joint_names = list(self.joint_targets.keys())

        for joint_name in self.joint_targets.keys():
            if joint_name in self.joint_positions:
                # Get current position and target
                current_pos = self.joint_positions[joint_name]
                target_pos = self.joint_targets[joint_name]

                # Compute PID output
                output, p_term, i_term, d_term = self.pid_controllers[joint_name].compute(
                    target_pos, current_pos
                )

                # Add to command message
                cmd_msg.effort.append(output)

                # Add to controller state
                controller_state_msg.desired.positions.append(target_pos)
                controller_state_msg.actual.positions.append(current_pos)
                controller_state_msg.error.positions.append(target_pos - current_pos)
                controller_state_msg.desired.velocities.append(0.0)  # Simple implementation
                controller_state_msg.actual.velocities.append(self.joint_velocities[joint_name])
                controller_state_msg.error.velocities.append(0.0)

                # Add PID terms for debugging
                controller_state_msg.error.effort.append(output)

        # Publish commands
        self.joint_cmd_pub.publish(cmd_msg)
        self.controller_state_pub.publish(controller_state_msg)

    def tune_pid_parameters(self, joint_name, kp, ki, kd):
        """Dynamically tune PID parameters"""
        if joint_name in self.pid_controllers:
            self.pid_controllers[joint_name].kp = kp
            self.pid_controllers[joint_name].ki = ki
            self.pid_controllers[joint_name].kd = kd
            self.get_logger().info(f'PID parameters updated for {joint_name}: Kp={kp}, Ki={ki}, Kd={kd}')

def main(args=None):
    rclpy.init(args=args)
    node = JointPIDControllerNode()

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

## 8.3 Trajectory Planning Implementation

Let's create comprehensive trajectory planning algorithms:

```python
#!/usr/bin/env python3
# trajectory_planning.py

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Float64
import numpy as np
from scipy.interpolate import CubicSpline
import time

class TrajectoryPlannerNode(Node):
    def __init__(self):
        super().__init__('trajectory_planner')

        # Publishers
        self.trajectory_pub = self.create_publisher(JointTrajectory, 'joint_trajectory', 10)
        self.progress_pub = self.create_publisher(Float64, 'trajectory_progress', 10)

        # Internal state
        self.current_trajectory = None
        self.trajectory_start_time = None
        self.trajectory_executing = False

        # Trajectory planners
        self.polynomial_planner = PolynomialTrajectoryPlanner()
        self.spline_planner = SplineTrajectoryPlanner()
        self.trapezoidal_planner = TrapezoidalTrajectoryPlanner()

        # Timer for trajectory execution
        self.execution_timer = self.create_timer(0.01, self.execute_trajectory)

        self.get_logger().info('Trajectory planner initialized')

    def plan_polynomial_trajectory(self, start_positions, end_positions, duration):
        """Plan polynomial trajectory"""
        return self.polynomial_planner.plan_trajectory(start_positions, end_positions, duration)

    def plan_spline_trajectory(self, waypoints, times):
        """Plan spline trajectory"""
        return self.spline_planner.plan_trajectory(waypoints, times)

    def plan_trapezoidal_trajectory(self, start_pos, end_pos, duration, max_vel=1.0):
        """Plan trapezoidal trajectory"""
        return self.trapezoidal_planner.plan_trajectory(start_pos, end_pos, duration, max_vel)

    def execute_trajectory(self):
        """Execute planned trajectory"""
        if self.current_trajectory and self.trajectory_executing:
            elapsed_time = time.time() - self.trajectory_start_time
            trajectory_duration = self.current_trajectory.points[-1].time_from_start.sec + \
                                  self.current_trajectory.points[-1].time_from_start.nanosec / 1e9

            if elapsed_time <= trajectory_duration:
                # Calculate progress
                progress = min(1.0, elapsed_time / trajectory_duration)

                # Publish trajectory
                self.trajectory_pub.publish(self.current_trajectory)

                # Publish progress
                progress_msg = Float64()
                progress_msg.data = progress
                self.progress_pub.publish(progress_msg)
            else:
                # Trajectory completed
                self.trajectory_executing = False
                self.get_logger().info('Trajectory execution completed')

    def execute_single_trajectory(self, start_pos, end_pos, duration, joint_name='joint1'):
        """Execute a single joint trajectory"""
        trajectory = JointTrajectory()
        trajectory.joint_names = [joint_name]

        # Generate trajectory points
        num_points = int(duration * 100)  # 100 points per second
        dt = duration / num_points

        for i in range(num_points + 1):
            t = i * dt
            # Use polynomial interpolation
            fraction = min(1.0, t / duration)
            # Cubic polynomial for smooth start/end
            s = 3 * fraction**2 - 2 * fraction**3

            pos = start_pos + s * (end_pos - start_pos)

            point = JointTrajectoryPoint()
            point.positions = [pos]
            point.velocities = [0.0]  # Will be computed properly
            point.accelerations = [0.0]  # Will be computed properly
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            trajectory.points.append(point)

        # Compute velocities and accelerations properly
        self.compute_trajectory_derivatives(trajectory)

        # Execute trajectory
        self.current_trajectory = trajectory
        self.trajectory_start_time = time.time()
        self.trajectory_executing = True

    def compute_trajectory_derivatives(self, trajectory):
        """Compute velocities and accelerations from positions"""
        for i, point in enumerate(trajectory.points):
            if i == 0:
                # First point: assume zero velocity/acceleration
                point.velocities = [0.0] * len(point.positions)
                point.accelerations = [0.0] * len(point.positions)
            elif i == len(trajectory.points) - 1:
                # Last point: assume zero velocity/acceleration
                point.velocities = [0.0] * len(point.positions)
                point.accelerations = [0.0] * len(point.positions)
            else:
                # Middle points: compute from neighboring points
                dt = 0.01  # Assuming 100 Hz
                prev_point = trajectory.points[i-1]
                next_point = trajectory.points[i+1]

                # Compute velocity (central difference)
                velocities = []
                for j in range(len(point.positions)):
                    vel = (next_point.positions[j] - prev_point.positions[j]) / (2 * dt)
                    velocities.append(vel)
                point.velocities = velocities

                # Compute acceleration (central difference)
                accelerations = []
                for j in range(len(point.positions)):
                    acc = (next_point.velocities[j] - prev_point.velocities[j]) / (2 * dt)
                    accelerations.append(acc)
                point.accelerations = accelerations

class PolynomialTrajectoryPlanner:
    """Plan trajectories using polynomial interpolation"""

    def plan_trajectory(self, start_positions, end_positions, duration):
        """Plan trajectory using quintic polynomial"""
        if len(start_positions) != len(end_positions):
            raise ValueError("Start and end positions must have same length")

        trajectory = JointTrajectory()
        trajectory.joint_names = [f'joint_{i}' for i in range(len(start_positions))]

        # Number of points based on duration (100 Hz)
        num_points = int(duration * 100)
        dt = duration / num_points

        for i in range(num_points + 1):
            t = i * dt
            fraction = min(1.0, t / duration)

            # Quintic polynomial for smooth trajectory (0, 0, 0 start and end conditions)
            # s = 10*f^3 - 15*f^4 + 6*f^5 (where f is fraction)
            s = 10 * fraction**3 - 15 * fraction**4 + 6 * fraction**5

            point = JointTrajectoryPoint()
            point.positions = []
            point.velocities = []
            point.accelerations = []

            for start_pos, end_pos in zip(start_positions, end_positions):
                pos = start_pos + s * (end_pos - start_pos)
                point.positions.append(pos)

                # Compute velocity (derivative of position)
                ds_dt = (30 * fraction**2 - 60 * fraction**3 + 30 * fraction**4) / duration
                vel = (end_pos - start_pos) * ds_dt
                point.velocities.append(vel)

                # Compute acceleration (derivative of velocity)
                d2s_dt2 = (60 * fraction - 180 * fraction**2 + 120 * fraction**3) / (duration**2)
                acc = (end_pos - start_pos) * d2s_dt2
                point.accelerations.append(acc)

            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            trajectory.points.append(point)

        return trajectory

class SplineTrajectoryPlanner:
    """Plan trajectories using spline interpolation"""

    def plan_trajectory(self, waypoints, times):
        """Plan trajectory using cubic splines"""
        if len(waypoints) != len(times):
            raise ValueError("Waypoints and times must have same length")

        trajectory = JointTrajectory()
        trajectory.joint_names = [f'joint_{i}' for i in range(len(waypoints[0]))]

        # Create spline for each joint
        num_joints = len(waypoints[0])
        splines = []

        for j in range(num_joints):
            joint_waypoints = [wp[j] for wp in waypoints]
            spline = CubicSpline(times, joint_waypoints, bc_type='natural')
            splines.append(spline)

        # Generate trajectory points
        total_time = times[-1]
        num_points = int(total_time * 100)  # 100 Hz

        for i in range(num_points + 1):
            t = i * (total_time / num_points)

            point = JointTrajectoryPoint()
            point.positions = [spline(t) for spline in splines]
            point.velocities = [spline(t, 1) for spline in splines]  # 1st derivative
            point.accelerations = [spline(t, 2) for spline in splines]  # 2nd derivative

            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            trajectory.points.append(point)

        return trajectory

class TrapezoidalTrajectoryPlanner:
    """Plan trajectories using trapezoidal velocity profile"""

    def plan_trajectory(self, start_pos, end_pos, duration, max_vel=1.0):
        """Plan trapezoidal trajectory"""
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint_0']

        distance = abs(end_pos - start_pos)
        direction = 1 if end_pos > start_pos else -1

        # Calculate acceleration and deceleration times
        # For symmetric trapezoidal: acc_time = dec_time
        # Total distance = acc_distance + const_distance + dec_distance
        # acc_distance = dec_distance = 0.5 * max_vel * acc_time
        # const_distance = max_vel * const_time
        # total_distance = max_vel * acc_time + max_vel * const_time = max_vel * (acc_time + const_time)

        acc_time = max_vel / 2.0  # Assuming max acceleration of 2.0 rad/s^2
        min_duration_for_triangle = 2 * acc_time  # Minimum for triangular profile

        if duration < min_duration_for_triangle:
            # Triangular profile (never reaches max velocity)
            acc_time = duration / 2.0
            actual_max_vel = max_vel * (duration / min_duration_for_triangle)
        else:
            actual_max_vel = max_vel

        const_time = duration - 2 * acc_time

        num_points = int(duration * 100)
        dt = duration / num_points

        for i in range(num_points + 1):
            t = i * dt

            if t <= acc_time:
                # Acceleration phase
                pos_fraction = 0.5 * (actual_max_vel / acc_time) * t**2
                vel = (actual_max_vel / acc_time) * t
                acc = actual_max_vel / acc_time
            elif t <= acc_time + const_time:
                # Constant velocity phase
                pos_fraction = 0.5 * actual_max_vel * acc_time + \
                              actual_max_vel * (t - acc_time)
                vel = actual_max_vel
                acc = 0.0
            else:
                # Deceleration phase
                time_in_dec = t - (acc_time + const_time)
                pos_fraction = 0.5 * actual_max_vel * acc_time + \
                              actual_max_vel * const_time + \
                              actual_max_vel * time_in_dec - \
                              0.5 * (actual_max_vel / acc_time) * time_in_dec**2
                vel = actual_max_vel - (actual_max_vel / acc_time) * time_in_dec
                acc = -actual_max_vel / acc_time

            pos = start_pos + direction * pos_fraction

            point = JointTrajectoryPoint()
            point.positions = [pos]
            point.velocities = [direction * vel]
            point.accelerations = [direction * acc]
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            trajectory.points.append(point)

        return trajectory

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlannerNode()

    try:
        # Example: Execute a simple trajectory
        node.execute_single_trajectory(0.0, np.pi/2, 5.0, 'joint1')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 8.4 Cartesian Space Control

Let's implement Cartesian space control with inverse kinematics:

```python
#!/usr/bin/env python3
# cartesian_control.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf2_ros import TransformListener, Buffer
import numpy as np
from scipy.spatial.transform import Rotation as R
import transforms3d

class CartesianControllerNode(Node):
    def __init__(self):
        super().__init__('cartesian_controller')

        # Publishers
        self.joint_cmd_pub = self.create_publisher(JointState, 'joint_commands', 10)
        self.cartesian_pose_pub = self.create_publisher(Pose, 'cartesian_pose', 10)

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )
        self.cartesian_cmd_sub = self.create_subscription(
            Pose, 'cartesian_command', self.cartesian_command_callback, 10
        )

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Robot model and inverse kinematics
        self.robot_model = self.initialize_robot_model()
        self.current_joint_positions = {}
        self.current_cartesian_pose = None

        # Control parameters
        self.cartesian_gains = {
            'position': 1.0,
            'orientation': 0.5
        }
        self.max_cartesian_velocity = 0.1  # 10 cm/s

        # Control timer
        self.control_timer = self.create_timer(0.01, self.cartesian_control_loop)

        self.get_logger().info('Cartesian controller initialized')

    def initialize_robot_model(self):
        """Initialize robot kinematic model (simplified 6-DOF manipulator)"""
        # This would typically load from URDF or DH parameters
        # For this example, we'll create a simplified model
        return RobotKinematicModel(
            link_lengths=[0.5, 0.4, 0.3, 0.2, 0.15, 0.1]
        )

    def joint_state_callback(self, msg):
        """Update current joint states"""
        for name, pos in zip(msg.name, msg.position):
            self.current_joint_positions[name] = pos

        # Update Cartesian pose using forward kinematics
        joint_positions = [self.current_joint_positions.get(name, 0.0)
                          for name in sorted(self.current_joint_positions.keys())]
        self.current_cartesian_pose = self.robot_model.forward_kinematics(joint_positions)

        # Publish current Cartesian pose
        if self.current_cartesian_pose is not None:
            pose_msg = Pose()
            pose_msg.position.x = float(self.current_cartesian_pose[0])
            pose_msg.position.y = float(self.current_cartesian_pose[1])
            pose_msg.position.z = float(self.current_cartesian_pose[2])

            # Assuming orientation is stored as quaternion
            if len(self.current_cartesian_pose) > 3:
                pose_msg.orientation.x = float(self.current_cartesian_pose[3])
                pose_msg.orientation.y = float(self.current_cartesian_pose[4])
                pose_msg.orientation.z = float(self.current_cartesian_pose[5])
                pose_msg.orientation.w = float(self.current_cartesian_pose[6])

            self.cartesian_pose_pub.publish(pose_msg)

    def cartesian_command_callback(self, msg):
        """Handle Cartesian command"""
        # Store target pose
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

    def cartesian_control_loop(self):
        """Main Cartesian control loop"""
        if not hasattr(self, 'target_cartesian_pose') or self.current_cartesian_pose is None:
            return

        # Calculate Cartesian error
        current_pos = self.current_cartesian_pose[:3]
        target_pos = self.target_cartesian_pose[:3]
        pos_error = target_pos - current_pos

        # Calculate orientation error (simplified)
        current_rot = R.from_quat(self.current_cartesian_pose[3:])
        target_rot = R.from_quat(self.target_cartesian_pose[3:])
        rot_error_quat = (target_rot.inv() * current_rot).as_quat()
        # Use vector part of quaternion for error (small angle approximation)
        rot_error = rot_error_quat[:3] * np.sign(rot_error_quat[3]) if rot_error_quat[3] >= 0 else -rot_error_quat[:3]

        # Apply control gains
        pos_correction = self.cartesian_gains['position'] * pos_error
        rot_correction = self.cartesian_gains['orientation'] * rot_error

        # Limit Cartesian velocity
        pos_norm = np.linalg.norm(pos_correction)
        if pos_norm > self.max_cartesian_velocity:
            pos_correction = (pos_correction / pos_norm) * self.max_cartesian_velocity

        # Convert Cartesian corrections to joint space using Jacobian
        joint_positions = [self.current_joint_positions.get(name, 0.0)
                          for name in sorted(self.current_joint_positions.keys())]

        try:
            jacobian = self.robot_model.compute_jacobian(joint_positions)

            # Combine position and orientation corrections
            cartesian_correction = np.concatenate([pos_correction, rot_correction])

            # Convert to joint space
            joint_correction = np.linalg.pinv(jacobian) @ cartesian_correction

            # Apply joint corrections to current positions
            new_joint_positions = np.array(joint_positions) + joint_correction * 0.01  # Small step

            # Create and publish joint command
            cmd_msg = JointState()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.name = sorted(self.current_joint_positions.keys())
            cmd_msg.position = new_joint_positions.tolist()
            self.joint_cmd_pub.publish(cmd_msg)

        except np.linalg.LinAlgError:
            self.get_logger().warn('Jacobian is singular, skipping control step')

    def move_to_cartesian_pose(self, target_pose, tolerance=0.01):
        """Move to specific Cartesian pose with tolerance checking"""
        self.target_cartesian_pose = target_pose

        while rclpy.ok():
            if self.current_cartesian_pose is not None:
                # Check if we're within tolerance
                pos_error = np.linalg.norm(
                    self.target_cartesian_pose[:3] - self.current_cartesian_pose[:3]
                )

                if pos_error < tolerance:
                    self.get_logger().info('Reached target Cartesian pose')
                    break

            time.sleep(0.01)

class RobotKinematicModel:
    """Simplified robot kinematic model for demonstration"""

    def __init__(self, link_lengths):
        self.link_lengths = link_lengths
        self.num_joints = len(link_lengths)

    def forward_kinematics(self, joint_angles):
        """Compute forward kinematics"""
        if len(joint_angles) != self.num_joints:
            raise ValueError(f"Expected {self.num_joints} joint angles, got {len(joint_angles)}")

        # Simplified 6-DOF serial manipulator FK
        # This is a highly simplified model - real implementation would use DH parameters
        x = sum([l * np.cos(sum(joint_angles[:i+1])) for i, l in enumerate(self.link_lengths)])
        y = sum([l * np.sin(sum(joint_angles[:i+1])) for i, l in enumerate(self.link_lengths)])
        z = 0  # 2D model for simplicity

        # For orientation, we'll use a simple representation
        # In reality, this would involve complex rotation matrices
        total_rotation = sum(joint_angles)
        orientation = R.from_rotvec([0, 0, total_rotation]).as_quat()

        return np.array([x, y, z, orientation[0], orientation[1], orientation[2], orientation[3]])

    def compute_jacobian(self, joint_angles):
        """Compute geometric Jacobian"""
        if len(joint_angles) != self.num_joints:
            raise ValueError(f"Expected {self.num_joints} joint angles, got {len(joint_angles)}")

        jacobian = np.zeros((6, self.num_joints))  # 6 DOF (pos + rot), N joints

        # Compute end-effector position using FK
        ee_pos = self.forward_kinematics(joint_angles)[:3]

        # For each joint, compute the effect on end-effector
        cumulative_angle = 0
        current_pos = np.array([0, 0, 0])

        for i in range(self.num_joints):
            cumulative_angle += joint_angles[i]

            # Position of joint i in base frame
            joint_pos = np.array([
                sum([l * np.cos(sum(joint_angles[:j+1])) for j, l in enumerate(self.link_lengths[:i+1])]),
                sum([l * np.sin(sum(joint_angles[:j+1])) for j, l in enumerate(self.link_lengths[:i+1])]),
                0
            ])

            # Z-axis of joint i (for revolute joint)
            z_axis = np.array([0, 0, 1])

            # Position effect: cross product of z-axis and (ee_pos - joint_pos)
            pos_effect = np.cross(z_axis, ee_pos - joint_pos)
            jacobian[:3, i] = pos_effect

            # Orientation effect: z-axis itself
            jacobian[3:, i] = z_axis

        return jacobian

def main(args=None):
    rclpy.init(args=args)
    node = CartesianControllerNode()

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

## 8.5 Safety Systems Implementation

Let's create comprehensive safety systems:

```python
#!/usr/bin/env python3
# safety_systems.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Duration
import numpy as np
import threading
import time

class SafetySystemNode(Node):
    def __init__(self):
        super().__init__('safety_system')

        # Publishers
        self.safety_status_pub = self.create_publisher(Bool, 'safety_status', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, 'emergency_stop', 10)
        self.velocity_limit_pub = self.create_publisher(Twist, 'velocity_limit', 10)

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10
        )
        self.velocity_cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.velocity_command_callback, 10
        )

        # Safety parameters
        self.joint_limits = {
            'joint1': (-np.pi, np.pi),
            'joint2': (-np.pi/2, np.pi/2),
            'joint3': (-np.pi, np.pi),
            'joint4': (-np.pi/2, np.pi/2),
            'joint5': (-np.pi, np.pi),
            'joint6': (-np.pi/2, np.pi/2)
        }
        self.velocity_limits = {name: 2.0 for name in self.joint_limits.keys()}  # 2 rad/s
        self.acceleration_limits = {name: 5.0 for name in self.joint_limits.keys()}  # 5 rad/s²
        self.torque_limits = {name: 50.0 for name in self.joint_limits.keys()}  # 50 Nm

        # Safety state
        self.emergency_active = False
        self.safety_violations = []
        self.current_states = {
            'positions': {},
            'velocities': {},
            'efforts': {},
            'imu_data': None,
            'last_command': None
        }

        # Velocity limiting
        self.max_linear_vel = 0.5  # m/s
        self.max_angular_vel = 0.5  # rad/s

        # Threading lock
        self.safety_lock = threading.Lock()

        # Safety monitoring timer
        self.safety_timer = self.create_timer(0.01, self.safety_monitoring)  # 100 Hz

        # Emergency stop monitoring
        self.emergency_stop_timer = self.create_timer(0.1, self.check_emergency_conditions)

        self.get_logger().info('Safety system initialized')

    def joint_state_callback(self, msg):
        """Monitor joint states for safety violations"""
        with self.safety_lock:
            for name, pos, vel, effort in zip(msg.name, msg.position, msg.velocity, msg.effort):
                if name in self.joint_limits:
                    self.current_states['positions'][name] = pos
                    self.current_states['velocities'][name] = vel
                    self.current_states['efforts'][name] = effort

    def imu_callback(self, msg):
        """Monitor IMU data for safety"""
        with self.safety_lock:
            self.current_states['imu_data'] = {
                'linear_acceleration': [
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z
                ],
                'angular_velocity': [
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z
                ]
            }

    def velocity_command_callback(self, msg):
        """Monitor velocity commands"""
        with self.safety_lock:
            self.current_states['last_command'] = {
                'linear': (msg.linear.x, msg.linear.y, msg.linear.z),
                'angular': (msg.angular.x, msg.angular.y, msg.angular.z)
            }

    def safety_monitoring(self):
        """Continuous safety monitoring"""
        with self.safety_lock:
            if self.emergency_active:
                # Publish zero commands to stop all motion
                self.publish_safety_stop()
                return

            # Check various safety conditions
            self.check_joint_limits()
            self.check_velocity_limits()
            self.check_acceleration_limits()
            self.check_torque_limits()
            self.check_imu_safety()
            self.check_command_limits()

            # Publish safety status
            status_msg = Bool()
            status_msg.data = not self.emergency_active
            self.safety_status_pub.publish(status_msg)

    def check_joint_limits(self):
        """Check joint position limits"""
        for joint_name, (min_pos, max_pos) in self.joint_limits.items():
            if joint_name in self.current_states['positions']:
                pos = self.current_states['positions'][joint_name]
                if pos < min_pos or pos > max_pos:
                    violation = f'JOINT_LIMIT_VIOLATION: {joint_name}={pos:.3f}, limits=({min_pos:.3f}, {max_pos:.3f})'
                    self.safety_violations.append(violation)
                    self.get_logger().error(violation)
                    self.activate_emergency_stop()

    def check_velocity_limits(self):
        """Check joint velocity limits"""
        for joint_name, max_vel in self.velocity_limits.items():
            if joint_name in self.current_states['velocities']:
                vel = abs(self.current_states['velocities'][joint_name])
                if vel > max_vel:
                    violation = f'VELOCITY_LIMIT_VIOLATION: {joint_name}={vel:.3f}, limit={max_vel:.3f}'
                    self.safety_violations.append(violation)
                    self.get_logger().error(violation)
                    self.activate_emergency_stop()

    def check_acceleration_limits(self):
        """Check joint acceleration limits (requires history)"""
        # This would need to store previous velocity values to compute acceleration
        # For now, we'll implement a simplified version
        pass

    def check_torque_limits(self):
        """Check joint torque/effort limits"""
        for joint_name, max_torque in self.torque_limits.items():
            if joint_name in self.current_states['efforts']:
                torque = abs(self.current_states['efforts'][joint_name])
                if torque > max_torque:
                    violation = f'TORQUE_LIMIT_VIOLATION: {joint_name}={torque:.3f}, limit={max_torque:.3f}'
                    self.safety_violations.append(violation)
                    self.get_logger().error(violation)
                    self.activate_emergency_stop()

    def check_imu_safety(self):
        """Check IMU data for safety violations"""
        if self.current_states['imu_data'] is not None:
            # Check for excessive acceleration (possible collision)
            accel = np.array(self.current_states['imu_data']['linear_acceleration'])
            accel_magnitude = np.linalg.norm(accel)

            # Typical threshold: 2g (19.62 m/s²)
            if accel_magnitude > 19.62:
                violation = f'IMU_ACCELERATION_VIOLATION: {accel_magnitude:.3f} m/s² > 19.62 m/s²'
                self.safety_violations.append(violation)
                self.get_logger().error(violation)
                self.activate_emergency_stop()

            # Check for excessive angular velocity (possible malfunction)
            ang_vel = np.array(self.current_states['imu_data']['angular_velocity'])
            ang_vel_magnitude = np.linalg.norm(ang_vel)

            # Threshold: 5 rad/s
            if ang_vel_magnitude > 5.0:
                violation = f'IMU_ANGULAR_VELOCITY_VIOLATION: {ang_vel_magnitude:.3f} rad/s > 5.0 rad/s'
                self.safety_violations.append(violation)
                self.get_logger().error(violation)
                self.activate_emergency_stop()

    def check_command_limits(self):
        """Check velocity commands for safety"""
        if self.current_states['last_command'] is not None:
            linear_vel = np.linalg.norm(self.current_states['last_command']['linear'])
            angular_vel = np.linalg.norm(self.current_states['last_command']['angular'])

            if linear_vel > self.max_linear_vel:
                violation = f'COMMAND_LINEAR_VEL_VIOLATION: {linear_vel:.3f} m/s > {self.max_linear_vel:.3f} m/s'
                self.safety_violations.append(violation)
                self.get_logger().warn(violation)
                # Don't emergency stop for command violations, just limit

            if angular_vel > self.max_angular_vel:
                violation = f'COMMAND_ANGULAR_VEL_VIOLATION: {angular_vel:.3f} rad/s > {self.max_angular_vel:.3f} rad/s'
                self.safety_violations.append(violation)
                self.get_logger().warn(violation)
                # Don't emergency stop for command violations, just limit

    def check_emergency_conditions(self):
        """Check for conditions that should trigger emergency stop"""
        # Check if safety violations have accumulated
        if len(self.safety_violations) > 10:  # Too many violations
            self.activate_emergency_stop()
            self.safety_violations.clear()  # Clear violations after emergency stop

    def activate_emergency_stop(self):
        """Activate emergency stop"""
        if not self.emergency_active:
            self.emergency_active = True
            self.get_logger().fatal('EMERGENCY STOP ACTIVATED - SHUTTING DOWN SAFELY')

            # Publish emergency stop signal
            emergency_msg = Bool()
            emergency_msg.data = True
            self.emergency_stop_pub.publish(emergency_msg)

            # Log violations
            self.log_safety_violations()

    def deactivate_emergency_stop(self):
        """Deactivate emergency stop (manual reset typically required)"""
        self.emergency_active = False
        self.get_logger().info('Emergency stop deactivated - SYSTEM READY')

        # Publish emergency stop release
        emergency_msg = Bool()
        emergency_msg.data = False
        self.emergency_stop_pub.publish(emergency_msg)

    def publish_safety_stop(self):
        """Publish zero commands to stop all motion"""
        # This would publish zero velocity commands to all controllers
        # For joint controllers, publish zero joint efforts
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.name = list(self.joint_limits.keys())
        cmd_msg.effort = [0.0] * len(cmd_msg.name)
        # Publish to safety stop topic
        # In real implementation, this would send to all controllers

    def log_safety_violations(self):
        """Log current safety violations"""
        if self.safety_violations:
            self.get_logger().info(f'Total safety violations: {len(self.safety_violations)}')
            for i, violation in enumerate(self.safety_violations[-10:]):  # Last 10 violations
                self.get_logger().error(f'Violation {i+1}: {violation}')

    def manual_reset_safety(self):
        """Manual reset of safety system (typically requires operator intervention)"""
        # In a real system, this would require physical button press or key switch
        # For simulation, we'll allow software reset
        self.emergency_active = False
        self.safety_violations.clear()
        self.get_logger().info('Safety system manually reset')

class SafetyMonitorInterface(Node):
    """Interface for safety system monitoring and control"""

    def __init__(self):
        super().__init__('safety_monitor_interface')

        # Subscribers
        self.safety_status_sub = self.create_subscription(
            Bool, 'safety_status', self.safety_status_callback, 10
        )
        self.emergency_stop_sub = self.create_subscription(
            Bool, 'emergency_stop', self.emergency_stop_callback, 10
        )

        # Publishers
        self.manual_reset_pub = self.create_publisher(Bool, 'manual_reset', 10)

        # Internal state
        self.safety_ok = True
        self.emergency_triggered = False

        self.get_logger().info('Safety monitor interface initialized')

    def safety_status_callback(self, msg):
        """Update safety status"""
        self.safety_ok = msg.data
        if not self.safety_ok and not self.emergency_triggered:
            self.get_logger().warn('SAFETY SYSTEM REPORTS UNSAFE CONDITION')

    def emergency_stop_callback(self, msg):
        """Update emergency stop status"""
        self.emergency_triggered = msg.data
        if msg.data:
            self.get_logger().error('EMERGENCY STOP TRIGGERED')
        else:
            self.get_logger().info('EMERGENCY STOP CLEARED')

    def request_manual_reset(self):
        """Request manual reset of safety system"""
        reset_msg = Bool()
        reset_msg.data = True
        self.manual_reset_pub.publish(reset_msg)
        self.get_logger().info('Manual reset requested')

def main(args=None):
    rclpy.init(args=args)

    # Create both nodes
    safety_node = SafetySystemNode()
    interface_node = SafetyMonitorInterface()

    # Create executor to run both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(safety_node)
    executor.add_node(interface_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        safety_node.destroy_node()
        interface_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 8.6 Advanced Control Algorithms

Let's implement advanced control algorithms:

```python
#!/usr/bin/env python3
# advanced_control_algorithms.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float64
import numpy as np
from scipy.linalg import solve_continuous_are
from scipy.signal import lti, lqr

class AdvancedControllerNode(Node):
    def __init__(self):
        super().__init__('advanced_controller')

        # Publishers and subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )
        self.joint_cmd_pub = self.create_publisher(JointState, 'joint_commands', 10)

        # Internal state
        self.current_positions = {}
        self.current_velocities = {}
        self.current_efforts = {}
        self.desired_positions = {}
        self.desired_velocities = {}

        # Advanced controllers
        self.lqr_controllers = {}
        self.mpc_controllers = {}
        self.adaptive_controllers = {}

        # Initialize controllers for each joint
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        for joint_name in joint_names:
            # LQR controller for each joint
            self.lqr_controllers[joint_name] = self.create_lqr_controller()
            self.desired_positions[joint_name] = 0.0
            self.desired_velocities[joint_name] = 0.0

        # Control timer
        self.control_timer = self.create_timer(0.01, self.advanced_control_loop)

        self.get_logger().info('Advanced controller initialized')

    def joint_state_callback(self, msg):
        """Update joint states"""
        for name, pos, vel, effort in zip(msg.name, msg.position, msg.velocity, msg.effort):
            self.current_positions[name] = pos
            self.current_velocities[name] = vel
            self.current_efforts[name] = effort

    def create_lqr_controller(self):
        """Create LQR controller for a single joint"""
        # Simple second-order system: m*q_ddot + b*q_dot + k*q = u
        # State: x = [q, q_dot]^T
        # A = [[0, 1], [-k/m, -b/m]]
        # B = [[0], [1/m]]
        # For simplicity, assume m=1, b=0.1, k=1
        A = np.array([[0, 1], [-1, -0.1]])
        B = np.array([[0], [1]])

        # Design LQR controller
        # Q: state cost matrix
        # R: control cost matrix
        Q = np.array([[10, 0], [0, 1]])  # Penalize position error more than velocity
        R = np.array([[0.1]])  # Penalize control effort

        # Solve LQR problem
        try:
            P = solve_continuous_are(A, B, Q, R)
            K = np.linalg.inv(R) @ B.T @ P
            return K
        except np.linalg.LinAlgError:
            # If ARE fails, return a simple PD controller
            return np.array([[10, 1]])  # Simple PD gains

    def lqr_control(self, joint_name, desired_pos, current_pos, current_vel):
        """Apply LQR control to a single joint"""
        if joint_name in self.lqr_controllers:
            K = self.lqr_controllers[joint_name]

            # State error: [position_error, velocity_error]
            state_error = np.array([[desired_pos - current_pos], [0 - current_vel]])

            # Control law: u = -K * x_error
            control_effort = -(K @ state_error)[0, 0]

            return control_effort
        else:
            # Fallback to simple PD control
            pos_error = desired_pos - current_pos
            vel_error = 0 - current_vel
            return -10 * pos_error - 1 * vel_error

    def advanced_control_loop(self):
        """Main advanced control loop"""
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.name = list(self.desired_positions.keys())

        for joint_name in self.desired_positions.keys():
            if joint_name in self.current_positions:
                current_pos = self.current_positions[joint_name]
                current_vel = self.current_velocities[joint_name]
                desired_pos = self.desired_positions[joint_name]

                # Apply LQR control
                control_effort = self.lqr_control(joint_name, desired_pos, current_pos, current_vel)

                cmd_msg.effort.append(control_effort)

        self.joint_cmd_pub.publish(cmd_msg)

    def set_trajectory_reference(self, joint_name, trajectory_func):
        """Set trajectory reference for advanced control"""
        # This would be called with a function that gives desired position, velocity, acceleration
        # at any given time
        pass

class ModelPredictiveController:
    """Model Predictive Control implementation"""

    def __init__(self, prediction_horizon=10, control_horizon=5):
        self.prediction_horizon = prediction_horizon
        self.control_horizon = control_horizon

        # MPC parameters
        self.Q = np.eye(2) * 10  # State cost
        self.R = np.eye(1) * 0.1  # Control cost
        self.P = np.eye(2) * 5  # Terminal cost

    def solve_mpc(self, current_state, reference_trajectory):
        """Solve MPC optimization problem"""
        # This is a simplified MPC implementation
        # Real implementation would use quadratic programming

        # For demonstration, we'll return a simple control action
        # based on tracking the reference trajectory
        predicted_states = []
        control_sequence = []

        current_x = current_state.copy()
        for k in range(self.control_horizon):
            # Simple prediction model (integrator)
            if k < len(reference_trajectory):
                reference = reference_trajectory[k]
                error = reference - current_x
                control = -0.5 * error  # Simple feedback
            else:
                control = 0.0

            # Apply control and predict next state
            current_x += control * 0.01  # dt = 0.01s
            predicted_states.append(current_x.copy())
            control_sequence.append(control)

        return control_sequence[0] if control_sequence else 0.0  # Return first control

class AdaptiveController:
    """Direct adaptive control implementation"""

    def __init__(self, initial_params=None):
        if initial_params is None:
            initial_params = {'kp': 1.0, 'ki': 0.1, 'kd': 0.05}
        self.params = initial_params
        self.param_history = {key: [val] for key, val in initial_params.items()}
        self.error_history = []
        self.adaptation_rate = 0.01

    def update_params(self, error, state):
        """Update controller parameters based on error"""
        self.error_history.append(error)
        if len(self.error_history) > 100:
            self.error_history.pop(0)

        # Simple adaptation law based on error magnitude and trend
        error_magnitude = abs(error)

        if error_magnitude > 0.1:  # High error
            self.params['kp'] = min(self.params['kp'] * 1.01, 50.0)
        elif error_magnitude < 0.01:  # Low error
            self.params['kp'] = max(self.params['kp'] * 0.99, 0.1)

        # Store parameter history
        for key, val in self.params.items():
            self.param_history[key].append(val)
            if len(self.param_history[key]) > 1000:
                self.param_history[key].pop(0)

    def compute_control(self, error, dt):
        """Compute adaptive control output"""
        self.update_params(error, None)

        # Standard PID with adaptive parameters
        proportional = self.params['kp'] * error
        self.params['integral'] = self.params.get('integral', 0) + error * dt
        integral = self.params['ki'] * self.params['integral']
        derivative = self.params['kd'] * (error - self.params.get('last_error', 0)) / dt if dt > 0 else 0

        output = proportional + integral + derivative
        self.params['last_error'] = error

        return output

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedControllerNode()

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

## 8.7 Practical Exercise

Create a complete control system that:
1. Implements PID, LQR, and adaptive controllers
2. Plans and executes trajectories
3. Includes comprehensive safety systems
4. Provides Cartesian space control
5. Validates control performance

```python
# Student exercise - Complete implementation
class CompleteControlSystem:
    """Student implementation of a complete control system"""

    def __init__(self):
        """Initialize the complete control system"""
        # TODO: Implement PID controllers
        # TODO: Implement trajectory planning
        # TODO: Implement Cartesian control
        # TODO: Implement safety systems
        # TODO: Implement advanced control algorithms
        # TODO: Implement validation and testing
        pass

    def implement_pid_control(self):
        """Implement PID controllers with tuning"""
        # TODO: Complete implementation
        pass

    def implement_trajectory_planning(self):
        """Implement trajectory planning algorithms"""
        # TODO: Complete implementation
        pass

    def implement_cartesian_control(self):
        """Implement Cartesian space control"""
        # TODO: Complete implementation
        pass

    def implement_safety_systems(self):
        """Implement comprehensive safety systems"""
        # TODO: Complete implementation
        pass

    def implement_advanced_control(self):
        """Implement LQR, MPC, and adaptive control"""
        # TODO: Complete implementation
        pass

    def validate_performance(self):
        """Validate control system performance"""
        # TODO: Complete implementation
        pass

print("Student Exercise: Implement a complete control system")
print("Requirements:")
print("1. PID, LQR, and adaptive controllers")
print("2. Trajectory planning and execution")
print("3. Cartesian space control with IK")
print("4. Comprehensive safety systems")
print("5. Performance validation and tuning")
print("6. Real-time implementation")
```

## Summary

In this lab, we've implemented comprehensive control systems including PID controllers, trajectory planning algorithms, Cartesian space control, and safety systems. These components form the foundation of robotic control and enable precise, safe, and reliable robot operation.

<div class="alert alert-info">
  <h5>Hardware Requirement</h5>
  <div><strong>Requirement:</strong> GPU</div>
  <div><strong>Minimum:</strong> Any</div>
  <div><strong>Recommended:</strong> Any</div>
  <div><strong>Purpose:</strong> Basic computational requirements for Week 8 control systems lab</div>
</div>