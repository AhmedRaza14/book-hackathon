---
title: "Week 8: Control Systems and Trajectory Planning"
week: 8
module: "Robotic Infrastructure"
difficulty: "advanced"
prerequisites: ["ros2-fundamentals", "python-basics", "robotics-concepts", "kinematics", "sensors"]
learning_objectives:
  - "Implement robot control systems"
  - "Design trajectory planning algorithms"
  - "Tune PID controllers"
  - "Plan and execute trajectories"
tags: ["control", "pid", "trajectory", "planning", "navigation", "motion"]
hardware_requirements:
  - gpu: "Any"
  - ram: "8GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "90 minutes"
---

# Week 8: Control Systems and Trajectory Planning

## Learning Objectives
- Implement robot control systems
- Design trajectory planning algorithms
- Tune PID controllers
- Plan and execute trajectories
- Understand safety systems and emergency stops

## 8.1 Introduction to Robot Control Systems

Robot control systems are essential for translating high-level commands into precise motor actions. They ensure that robots move accurately and safely in their environment.

### Control System Architecture

A typical robot control system consists of several layers:

1. **High-Level Planner**: Generates desired trajectories
2. **Motion Controller**: Converts trajectories to joint commands
3. **Low-Level Controller**: Manages individual joint motors
4. **Safety System**: Monitors and enforces safety constraints

### Control System Types

1. **Open-Loop Control**: Commands sent without feedback
2. **Closed-Loop Control**: Uses feedback for error correction
3. **Feedforward Control**: Predictive control based on model
4. **Adaptive Control**: Adjusts parameters based on changing conditions

## 8.2 PID Controllers

Proportional-Integral-Derivative (PID) controllers are fundamental in robot control systems.

### PID Controller Theory

The PID controller output is calculated as:

$$u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}$$

Where:
- $u(t)$: Controller output
- $e(t)$: Error signal (desired - actual)
- $K_p$: Proportional gain
- $K_i$: Integral gain
- $K_d$: Derivative gain

### PID Implementation

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(-np.inf, np.inf)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits

        self.reset()

    def reset(self):
        """Reset PID controller state"""
        self.last_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.last_time = None

    def compute(self, setpoint, measured_value, dt=None):
        """Compute PID output"""
        current_time = time.time()

        if dt is None:
            if self.last_time is None:
                dt = 0.01  # Default timestep
            else:
                dt = current_time - self.last_time
        self.last_time = current_time

        # Calculate error
        error = setpoint - measured_value

        # Proportional term
        proportional = self.kp * error

        # Integral term
        self.integral += error * dt
        integral = self.ki * self.integral

        # Derivative term
        if dt > 0:
            self.derivative = (error - self.last_error) / dt
        derivative = self.kd * self.derivative

        # Calculate output
        output = proportional + integral + derivative

        # Apply output limits
        output = np.clip(output, self.output_limits[0], self.output_limits[1])

        # Store for next iteration
        self.last_error = error

        return output

class JointControllerNode(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Publishers and subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )
        self.joint_cmd_pub = self.create_publisher(
            JointState, 'joint_commands', 10
        )

        # PID controllers for each joint
        self.pid_controllers = {}
        self.joint_positions = {}
        self.joint_velocities = {}
        self.joint_targets = {}

        # Initialize PID controllers
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        for joint_name in joint_names:
            self.pid_controllers[joint_name] = PIDController(
                kp=10.0, ki=0.1, kd=0.5, output_limits=(-10.0, 10.0)
            )
            self.joint_targets[joint_name] = 0.0

        # Control timer
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100 Hz

    def joint_state_callback(self, msg):
        """Update joint state"""
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            if name in self.pid_controllers:
                self.joint_positions[name] = pos
                self.joint_velocities[name] = vel

    def control_loop(self):
        """Main control loop"""
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.name = list(self.joint_targets.keys())

        for joint_name in self.joint_targets.keys():
            if joint_name in self.joint_positions:
                # Compute PID output
                current_pos = self.joint_positions[joint_name]
                target_pos = self.joint_targets[joint_name]

                control_output = self.pid_controllers[joint_name].compute(
                    target_pos, current_pos
                )

                cmd_msg.position.append(control_output)

        self.joint_cmd_pub.publish(cmd_msg)

    def set_target_position(self, joint_name, target):
        """Set target position for a joint"""
        if joint_name in self.joint_targets:
            self.joint_targets[joint_name] = target
            self.pid_controllers[joint_name].reset()
```

## 8.3 Advanced Control Techniques

### Feedforward Control

```python
class FeedforwardController:
    """Feedforward controller for predictive control"""

    def __init__(self, mass_matrix_func, coriolis_matrix_func, gravity_vector_func):
        self.mass_matrix_func = mass_matrix_func
        self.coriolis_matrix_func = coriolis_matrix_func
        self.gravity_vector_func = gravity_vector_func

    def compute_feedforward_torque(self, q, dq, ddq):
        """Compute feedforward torque based on robot dynamics model"""
        # Mass matrix
        M = self.mass_matrix_func(q)

        # Coriolis and centrifugal terms
        C = self.coriolis_matrix_func(q, dq)

        # Gravity vector
        g = self.gravity_vector_func(q)

        # Feedforward torque: τ = M(q)ddq + C(q,dq)dq + g(q)
        tau_ff = M @ ddq + C @ dq + g

        return tau_ff

class ModelBasedController:
    """Controller using robot dynamics model"""

    def __init__(self, robot_model):
        self.model = robot_model
        self.feedforward_controller = FeedforwardController(
            robot_model.mass_matrix,
            robot_model.coriolis_matrix,
            robot_model.gravity_vector
        )
        self.feedback_pid = PIDController(kp=50.0, ki=1.0, kd=2.0)

    def compute_control_torque(self, q_desired, q_actual, dq_desired, dq_actual, ddq_desired):
        """Compute total control torque using feedforward + feedback"""
        # Feedforward torque
        tau_ff = self.feedforward_controller.compute_feedforward_torque(
            q_actual, dq_actual, ddq_desired
        )

        # Feedback torque
        position_error = q_desired - q_actual
        velocity_error = dq_desired - dq_actual

        # Combine position and velocity feedback
        tau_fb = self.feedback_pid.compute(position_error, velocity_error)

        # Total control torque
        tau_total = tau_ff + tau_fb

        return tau_total
```

### Adaptive Control

```python
class AdaptiveController:
    """Adaptive controller for changing conditions"""

    def __init__(self, initial_params, adaptation_rate=0.01):
        self.params = initial_params
        self.adaptation_rate = adaptation_rate
        self.error_history = []

    def update_parameters(self, error, state):
        """Adapt controller parameters based on error"""
        # Store error for history
        self.error_history.append(error)
        if len(self.error_history) > 100:
            self.error_history.pop(0)

        # Adapt parameters based on error magnitude and trends
        error_magnitude = np.abs(error)

        # Adjust gains based on error
        if error_magnitude > 0.1:  # High error threshold
            self.params['kp'] *= 1.01  # Increase proportional gain
        elif error_magnitude < 0.01:  # Low error threshold
            self.params['kp'] *= 0.99  # Decrease proportional gain

        # Adaptation based on error trend
        if len(self.error_history) > 10:
            recent_errors = np.array(self.error_history[-10:])
            error_trend = np.polyfit(range(len(recent_errors)), recent_errors, 1)[0]

            if error_trend > 0:  # Error increasing
                self.params['ki'] = min(self.params['ki'] * 1.05, 10.0)  # Increase integral gain
            elif error_trend < 0:  # Error decreasing
                self.params['ki'] = max(self.params['ki'] * 0.95, 0.1)  # Decrease integral gain

    def compute_control(self, error, dt):
        """Compute control output with adaptive parameters"""
        # Update parameters based on current error
        self.update_parameters(error, None)

        # Apply PID with adaptive parameters
        proportional = self.params['kp'] * error
        self.params['integral'] += error * dt
        integral = self.params['ki'] * self.params['integral']
        derivative = self.params['kd'] * (error - self.params['last_error']) / dt if dt > 0 else 0

        output = proportional + integral + derivative
        self.params['last_error'] = error

        return output
```

## 8.4 Trajectory Planning

### Joint Space Trajectory Planning

```python
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

class TrajectoryPlanner:
    """Plan smooth trajectories in joint space"""

    def __init__(self):
        self.current_trajectory = None
        self.trajectory_time = 0.0

    def plan_polynomial_trajectory(self, start_pos, end_pos, duration, max_vel=1.0, max_acc=2.0):
        """Plan polynomial trajectory using quintic polynomial"""
        # Quintic polynomial: q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
        # Boundary conditions: q(0)=start, q'(0)=0, q''(0)=0, q(T)=end, q'(T)=0, q''(T)=0

        T = duration
        a0 = start_pos
        a1 = 0
        a2 = 0
        a3 = (20*(end_pos - start_pos))/(2*T**3)
        a4 = (-30*(end_pos - start_pos))/(2*T**4)
        a5 = (12*(end_pos - start_pos))/(2*T**5)

        def trajectory_func(t):
            t = np.clip(t, 0, T)
            q = a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5
            q_dot = a1 + 2*a2*t + 3*a3*t**2 + 4*a4*t**3 + 5*a5*t**4
            q_ddot = 2*a2 + 6*a3*t + 12*a4*t**2 + 20*a5*t**3
            return q, q_dot, q_ddot

        return trajectory_func

    def plan_trapezoidal_trajectory(self, start_pos, end_pos, duration, max_vel=1.0):
        """Plan trapezoidal velocity profile"""
        distance = end_pos - start_pos
        direction = 1 if distance > 0 else -1
        abs_distance = abs(distance)

        # Calculate acceleration time to reach max velocity
        # For symmetric acceleration/deceleration: v_max = a * t_acc
        # Distance during acceleration: d_acc = 0.5 * a * t_acc^2
        # Total distance: d_total = 2 * d_acc + d_const
        # So: d_const = d_total - 2 * d_acc

        # Assume acceleration = max_vel / 2 (arbitrary choice for symmetric profile)
        acceleration = max_vel**2 / abs_distance if abs_distance > 0 else 1.0
        acc_time = max_vel / acceleration if acceleration > 0 else 0.0

        # Check if we can reach max velocity within the duration
        min_duration_for_max_vel = 2 * acc_time
        if duration < min_duration_for_max_vel:
            # Cannot reach max velocity, adjust acceleration
            acc_time = duration / 2
            acceleration = max_vel / acc_time if acc_time > 0 else 1.0

        const_time = duration - 2 * acc_time

        def trajectory_func(t):
            t = np.clip(t, 0, duration)

            if t < acc_time:
                # Acceleration phase
                pos = start_pos + 0.5 * acceleration * direction * t**2
                vel = acceleration * direction * t
                acc = acceleration * direction
            elif t < acc_time + const_time:
                # Constant velocity phase
                pos = start_pos + 0.5 * acceleration * direction * acc_time**2 + \
                      max_vel * direction * (t - acc_time)
                vel = max_vel * direction
                acc = 0.0
            else:
                # Deceleration phase
                time_in_dec = t - (acc_time + const_time)
                pos = start_pos + 0.5 * acceleration * direction * acc_time**2 + \
                      max_vel * direction * const_time + \
                      max_vel * direction * time_in_dec - \
                      0.5 * acceleration * direction * time_in_dec**2
                vel = max_vel * direction - acceleration * direction * time_in_dec
                acc = -acceleration * direction

            return pos, vel, acc

        return trajectory_func

    def plan_spline_trajectory(self, waypoints, times):
        """Plan trajectory using cubic splines"""
        # Create cubic spline interpolation
        positions = np.array(waypoints)
        time_points = np.array(times)

        # Create spline functions for position, velocity, acceleration
        pos_spline = interp1d(time_points, positions, kind='cubic', fill_value='extrapolate')

        # Create velocity and acceleration splines by differentiation
        # For simplicity, we'll use finite differences
        dt = 0.01
        smooth_times = np.arange(time_points[0], time_points[-1], dt)
        smooth_positions = pos_spline(smooth_times)

        # Compute velocity and acceleration using finite differences
        velocities = np.gradient(smooth_positions, dt)
        accelerations = np.gradient(velocities, dt)

        # Create interpolating functions
        vel_spline = interp1d(smooth_times, velocities, kind='cubic', fill_value='extrapolate')
        acc_spline = interp1d(smooth_times, accelerations, kind='cubic', fill_value='extrapolate')

        def trajectory_func(t):
            pos = pos_spline(t)
            vel = vel_spline(t)
            acc = acc_spline(t)
            return pos, vel, acc

        return trajectory_func
```

### Cartesian Space Trajectory Planning

```python
from scipy.spatial.transform import Rotation as R

class CartesianTrajectoryPlanner:
    """Plan trajectories in Cartesian space"""

    def __init__(self):
        self.trajectory_planner = TrajectoryPlanner()

    def plan_linear_trajectory(self, start_pose, end_pose, duration):
        """Plan linear trajectory in Cartesian space"""
        # Extract position and orientation
        start_pos = np.array(start_pose[:3])
        end_pos = np.array(end_pose[:3])

        start_rot = R.from_quat(start_pose[3:])  # [x, y, z, w]
        end_rot = R.from_quat(end_pose[3:])

        # Plan position trajectory
        pos_trajectory_func = self.trajectory_planner.plan_polynomial_trajectory(
            start_pos, end_pos, duration
        )

        # Plan orientation trajectory (SLERP - Spherical Linear Interpolation)
        def rot_trajectory_func(t):
            t_norm = np.clip(t / duration, 0, 1)
            interp_rot = start_rot.slerp(end_rot, t_norm)
            return interp_rot.as_quat()

        def trajectory_func(t):
            pos, vel, acc = pos_trajectory_func(t)
            rot = rot_trajectory_func(t)
            return pos, rot, vel, acc

        return trajectory_func

    def plan_circular_trajectory(self, center, radius, start_angle, end_angle, duration):
        """Plan circular trajectory in 2D"""
        def trajectory_func(t):
            t_norm = np.clip(t / duration, 0, 1)
            angle = start_angle + t_norm * (end_angle - start_angle)

            x = center[0] + radius * np.cos(angle)
            y = center[1] + radius * np.sin(angle)
            z = center[2]  # Constant height

            pos = np.array([x, y, z])

            # Calculate velocity (tangent to circle)
            vel_x = -radius * np.sin(angle) * (end_angle - start_angle) / duration
            vel_y = radius * np.cos(angle) * (end_angle - start_angle) / duration
            vel_z = 0.0
            vel = np.array([vel_x, vel_y, vel_z])

            # Calculate acceleration (centripetal)
            acc_x = -radius * np.cos(angle) * ((end_angle - start_angle) / duration)**2
            acc_y = -radius * np.sin(angle) * ((end_angle - start_angle) / duration)**2
            acc_z = 0.0
            acc = np.array([acc_x, acc_y, acc_z])

            # Identity rotation for simplicity
            rot = np.array([0, 0, 0, 1])  # [x, y, z, w]

            return pos, rot, vel, acc

        return trajectory_func
```

## 8.5 Robot Control Implementation

### Joint Space Control

```python
class JointSpaceController(Node):
    def __init__(self):
        super().__init__('joint_space_controller')

        # Publishers and subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )
        self.joint_cmd_pub = self.create_publisher(
            JointState, 'joint_commands', 10
        )
        self.trajectory_sub = self.create_subscription(
            JointTrajectory, 'joint_trajectory', self.trajectory_callback, 10
        )

        # Internal state
        self.current_positions = {}
        self.current_velocities = {}
        self.current_efforts = {}
        self.trajectory_generator = TrajectoryPlanner()
        self.current_trajectory = {}
        self.trajectory_start_time = {}

        # PID controllers for each joint
        self.pid_controllers = {}
        self.joint_limits = {}  # {joint_name: (min, max)}

        # Initialize controllers
        self.initialize_controllers()

        # Control timer
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100 Hz

    def initialize_controllers(self):
        """Initialize PID controllers and joint limits"""
        # Example joint names and limits
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        joint_limits = [
            (-np.pi, np.pi),    # joint1
            (-np.pi/2, np.pi/2), # joint2
            (-np.pi, np.pi),    # joint3
            (-np.pi/2, np.pi/2), # joint4
            (-np.pi, np.pi),    # joint5
            (-np.pi/2, np.pi/2)  # joint6
        ]

        for joint_name, limits in zip(joint_names, joint_limits):
            self.pid_controllers[joint_name] = PIDController(
                kp=10.0, ki=0.1, kd=0.5, output_limits=(-10.0, 10.0)
            )
            self.joint_limits[joint_name] = limits

    def joint_state_callback(self, msg):
        """Update current joint states"""
        for name, pos, vel, eff in zip(msg.name, msg.position, msg.velocity, msg.effort):
            if name in self.pid_controllers:
                self.current_positions[name] = pos
                self.current_velocities[name] = vel
                self.current_efforts[name] = eff

    def trajectory_callback(self, msg):
        """Handle incoming trajectory commands"""
        for point in msg.points:
            # Process trajectory point
            for joint_name, pos in zip(msg.joint_names, point.positions):
                if joint_name in self.pid_controllers:
                    # Plan trajectory from current position to target
                    current_pos = self.current_positions.get(joint_name, 0.0)
                    duration = rclpy.duration.Duration.from_msg(point.time_from_start).nanoseconds / 1e9

                    if duration > 0:
                        trajectory_func = self.trajectory_generator.plan_polynomial_trajectory(
                            current_pos, pos, duration
                        )
                        self.current_trajectory[joint_name] = trajectory_func
                        self.trajectory_start_time[joint_name] = time.time()

    def control_loop(self):
        """Main control loop"""
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.name = list(self.pid_controllers.keys())

        for joint_name in self.pid_controllers.keys():
            if joint_name in self.current_positions:
                current_pos = self.current_positions[joint_name]

                # Get trajectory target if available
                if joint_name in self.current_trajectory:
                    trajectory_time = time.time() - self.trajectory_start_time[joint_name]
                    target_pos, target_vel, target_acc = self.current_trajectory[joint_name](trajectory_time)

                    # Use trajectory target
                    control_output = self.pid_controllers[joint_name].compute(target_pos, current_pos)
                else:
                    # Hold current position
                    control_output = self.pid_controllers[joint_name].compute(current_pos, current_pos)

                cmd_msg.effort.append(control_output)

        self.joint_cmd_pub.publish(cmd_msg)

    def check_safety_constraints(self):
        """Check safety constraints and apply limits"""
        for joint_name in self.current_positions:
            pos = self.current_positions[joint_name]
            min_pos, max_pos = self.joint_limits[joint_name]

            if pos < min_pos or pos > max_pos:
                self.get_logger().warn(f'Joint {joint_name} exceeded limits: {pos}')
                # Emergency stop or limit adjustment
                return False

        return True
```

### Cartesian Space Control with Inverse Kinematics

```python
class CartesianController(Node):
    def __init__(self):
        super().__init__('cartesian_controller')

        # Publishers and subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )
        self.cartesian_cmd_sub = self.create_subscription(
            PoseStamped, 'cartesian_command', self.cartesian_command_callback, 10
        )
        self.joint_cmd_pub = self.create_publisher(
            JointState, 'joint_commands', 10
        )

        # Robot model for inverse kinematics
        self.robot_model = self.initialize_robot_model()
        self.cartesian_trajectory_planner = CartesianTrajectoryPlanner()

        # Internal state
        self.current_joint_positions = []
        self.current_cartesian_pose = None

        # Control timer
        self.control_timer = self.create_timer(0.01, self.cartesian_control_loop)

    def initialize_robot_model(self):
        """Initialize robot kinematic model"""
        # This would typically load from URDF or DH parameters
        # For example, using PyKDL, OpenRAVE, or custom implementation
        return RobotKinematicModel()

    def joint_state_callback(self, msg):
        """Update current joint states"""
        self.current_joint_positions = msg.position
        # Update Cartesian pose using forward kinematics
        self.current_cartesian_pose = self.robot_model.forward_kinematics(msg.position)

    def cartesian_command_callback(self, msg):
        """Handle Cartesian space commands"""
        target_pose = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])

        # Plan trajectory in Cartesian space
        current_pose = self.current_cartesian_pose if self.current_cartesian_pose else target_pose
        duration = 5.0  # 5 seconds to reach target

        self.cartesian_trajectory = self.cartesian_trajectory_planner.plan_linear_trajectory(
            current_pose, target_pose, duration
        )
        self.trajectory_start_time = time.time()

    def cartesian_control_loop(self):
        """Main Cartesian control loop"""
        if hasattr(self, 'cartesian_trajectory'):
            # Get target Cartesian pose from trajectory
            elapsed_time = time.time() - self.trajectory_start_time
            target_pos, target_rot, target_vel, target_acc = self.cartesian_trajectory(elapsed_time)

            # Calculate inverse kinematics
            target_joints = self.robot_model.inverse_kinematics(
                target_pos, target_rot, self.current_joint_positions
            )

            # Send joint commands
            cmd_msg = JointState()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.name = [f'joint_{i}' for i in range(len(target_joints))]
            cmd_msg.position = target_joints
            self.joint_cmd_pub.publish(cmd_msg)

    def execute_cartesian_trajectory(self, trajectory_func, duration):
        """Execute a planned Cartesian trajectory"""
        start_time = time.time()
        dt = 0.01  # 100 Hz

        while time.time() - start_time < duration:
            elapsed = time.time() - start_time
            target_pos, target_rot, target_vel, target_acc = trajectory_func(elapsed)

            # Calculate inverse kinematics
            target_joints = self.robot_model.inverse_kinematics(
                target_pos, target_rot, self.current_joint_positions
            )

            # Send commands
            cmd_msg = JointState()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.name = [f'joint_{i}' for i in range(len(target_joints))]
            cmd_msg.position = target_joints
            self.joint_cmd_pub.publish(cmd_msg)

            time.sleep(dt)
```

## 8.6 Safety Systems and Emergency Stops

### Safety Monitoring

```python
class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')

        # Subscribers for monitoring
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )
        self.emergency_stop_sub = self.create_subscription(
            Bool, 'emergency_stop', self.emergency_stop_callback, 10
        )

        # Publishers for safety commands
        self.safety_cmd_pub = self.create_publisher(JointState, 'safety_commands', 10)
        self.emergency_status_pub = self.create_publisher(Bool, 'emergency_status', 10)

        # Safety parameters
        self.joint_limits = {}  # {joint_name: (min, max)}
        self.velocity_limits = {}  # {joint_name: max_vel}
        self.acceleration_limits = {}  # {joint_name: max_acc}
        self.force_limits = {}  # {joint_name: max_force}

        # Safety state
        self.emergency_active = False
        self.safety_violations = []

        # Initialize safety parameters
        self.initialize_safety_limits()

        # Safety monitoring timer
        self.safety_timer = self.create_timer(0.01, self.safety_check)

    def initialize_safety_limits(self):
        """Initialize safety limits for each joint"""
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        for i, joint_name in enumerate(joint_names):
            # Position limits (example values)
            self.joint_limits[joint_name] = (-np.pi, np.pi)

            # Velocity limits (1 rad/s)
            self.velocity_limits[joint_name] = 1.0

            # Acceleration limits (5 rad/s²)
            self.acceleration_limits[joint_name] = 5.0

            # Force/torque limits (50 Nm)
            self.force_limits[joint_name] = 50.0

    def joint_state_callback(self, msg):
        """Monitor joint states for safety violations"""
        if self.emergency_active:
            return

        for name, pos, vel, effort in zip(msg.name, msg.position, msg.velocity, msg.effort):
            if name in self.joint_limits:
                # Check position limits
                min_pos, max_pos = self.joint_limits[name]
                if pos < min_pos or pos > max_pos:
                    self.safety_violations.append(f'Position violation: {name}={pos:.3f}, limits=({min_pos:.3f}, {max_pos:.3f})')
                    self.trigger_emergency_stop()

                # Check velocity limits
                max_vel = self.velocity_limits[name]
                if abs(vel) > max_vel:
                    self.safety_violations.append(f'Velocity violation: {name}={vel:.3f}, limit={max_vel:.3f}')
                    self.trigger_emergency_stop()

                # Check force/torque limits
                max_force = self.force_limits[name]
                if abs(effort) > max_force:
                    self.safety_violations.append(f'Force violation: {name}={effort:.3f}, limit={max_force:.3f}')
                    self.trigger_emergency_stop()

    def emergency_stop_callback(self, msg):
        """Handle external emergency stop commands"""
        if msg.data:
            self.trigger_emergency_stop()
        else:
            self.clear_emergency_stop()

    def safety_check(self):
        """Perform safety checks"""
        if self.emergency_active:
            # Publish zero commands to stop all joints
            cmd_msg = JointState()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.name = list(self.joint_limits.keys())
            cmd_msg.position = [0.0] * len(cmd_msg.name)
            cmd_msg.velocity = [0.0] * len(cmd_msg.name)
            cmd_msg.effort = [0.0] * len(cmd_msg.name)
            self.safety_cmd_pub.publish(cmd_msg)

        # Publish emergency status
        status_msg = Bool()
        status_msg.data = self.emergency_active
        self.emergency_status_pub.publish(status_msg)

        # Log safety violations
        if self.safety_violations:
            for violation in self.safety_violations[-5:]:  # Log last 5 violations
                self.get_logger().error(violation)
            self.safety_violations.clear()

    def trigger_emergency_stop(self):
        """Trigger emergency stop"""
        if not self.emergency_active:
            self.emergency_active = True
            self.get_logger().error('EMERGENCY STOP ACTIVATED')
            self.log_safety_violations()

    def clear_emergency_stop(self):
        """Clear emergency stop (requires manual reset typically)"""
        self.emergency_active = False
        self.get_logger().info('Emergency stop cleared')

    def log_safety_violations(self):
        """Log current safety violations"""
        if self.safety_violations:
            self.get_logger().info(f'Safety violations: {len(self.safety_violations)}')
            for violation in self.safety_violations[-10:]:  # Last 10 violations
                self.get_logger().info(violation)
```

## 8.7 Practical Control Examples

### Pick and Place Controller

```python
class PickPlaceController(Node):
    def __init__(self):
        super().__init__('pick_place_controller')

        # Publishers and subscribers
        self.joint_cmd_pub = self.create_publisher(JointState, 'joint_commands', 10)
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

        # Internal state
        self.current_positions = {}
        self.gripper_open = True

        # Initialize trajectory planner
        self.trajectory_planner = TrajectoryPlanner()
        self.cartesian_planner = CartesianTrajectoryPlanner()

        # Define pick and place poses
        self.pre_pick_pose = [0.5, 0.0, 0.3, 0, 0, 0, 1]  # Above object
        self.pick_pose = [0.5, 0.0, 0.1, 0, 0, 0, 1]      # At object
        self.pre_place_pose = [0.0, 0.5, 0.3, 0, 0, 0, 1] # Above destination
        self.place_pose = [0.0, 0.5, 0.1, 0, 0, 0, 1]     # At destination

    def execute_pick_place(self):
        """Execute complete pick and place operation"""
        try:
            # Move to pre-pick position
            self.get_logger().info('Moving to pre-pick position')
            self.move_to_cartesian_pose(self.pre_pick_pose, duration=3.0)

            # Move down to pick position
            self.get_logger().info('Moving to pick position')
            self.move_to_cartesian_pose(self.pick_pose, duration=2.0)

            # Close gripper
            self.get_logger().info('Closing gripper')
            self.close_gripper()
            time.sleep(1.0)

            # Lift object
            self.get_logger().info('Lifting object')
            self.move_to_cartesian_pose(self.pre_pick_pose, duration=2.0)

            # Move to pre-place position
            self.get_logger().info('Moving to pre-place position')
            self.move_to_cartesian_pose(self.pre_place_pose, duration=4.0)

            # Move down to place position
            self.get_logger().info('Moving to place position')
            self.move_to_cartesian_pose(self.place_pose, duration=2.0)

            # Open gripper
            self.get_logger().info('Opening gripper')
            self.open_gripper()
            time.sleep(1.0)

            # Lift gripper
            self.get_logger().info('Lifting gripper')
            self.move_to_cartesian_pose(self.pre_place_pose, duration=2.0)

            self.get_logger().info('Pick and place operation completed successfully')

        except Exception as e:
            self.get_logger().error(f'Pick and place operation failed: {e}')
            raise

    def move_to_cartesian_pose(self, target_pose, duration=3.0):
        """Move end-effector to Cartesian pose"""
        # Plan trajectory
        current_pose = self.get_current_cartesian_pose()
        trajectory_func = self.cartesian_planner.plan_linear_trajectory(
            current_pose, target_pose, duration
        )

        # Execute trajectory
        start_time = time.time()
        dt = 0.01  # 100 Hz

        while time.time() - start_time < duration:
            elapsed = time.time() - start_time
            target_pos, target_rot, target_vel, target_acc = trajectory_func(elapsed)

            # Calculate joint positions using inverse kinematics
            current_joints = list(self.current_positions.values())
            target_joints = self.inverse_kinematics(target_pos, target_rot, current_joints)

            # Send joint commands
            cmd_msg = JointState()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.name = [f'joint_{i}' for i in range(len(target_joints))]
            cmd_msg.position = target_joints
            self.joint_cmd_pub.publish(cmd_msg)

            time.sleep(dt)

    def close_gripper(self):
        """Close the gripper"""
        # Implementation depends on gripper type
        # This is a placeholder
        self.gripper_open = False
        self.get_logger().info('Gripper closed')

    def open_gripper(self):
        """Open the gripper"""
        # Implementation depends on gripper type
        # This is a placeholder
        self.gripper_open = True
        self.get_logger().info('Gripper opened')

    def get_current_cartesian_pose(self):
        """Get current end-effector Cartesian pose"""
        # This would use forward kinematics
        # Placeholder implementation
        return [0.0, 0.0, 0.0, 0, 0, 0, 1]

    def inverse_kinematics(self, target_pos, target_rot, current_joints):
        """Calculate inverse kinematics"""
        # This would use a proper IK solver
        # Placeholder implementation
        return current_joints
```

## Summary

Control systems and trajectory planning are fundamental to robot operation, enabling precise movement and safe execution of tasks. Understanding PID controllers, advanced control techniques, trajectory planning algorithms, and safety systems is crucial for developing robust robotic applications.

## Next Steps

In the next section, we'll explore navigation and motion planning, building on the control systems developed here to enable autonomous robot navigation.

<div class="alert alert-info">
  <h5>Hardware Requirement</h5>
  <div><strong>Requirement:</strong> GPU</div>
  <div><strong>Minimum:</strong> Any</div>
  <div><strong>Recommended:</strong> Any</div>
  <div><strong>Purpose:</strong> Basic computational requirements for Week 8 control systems and trajectory planning</div>
</div>