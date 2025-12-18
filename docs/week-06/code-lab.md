---
title: "Week 6: Advanced ROS 2 Concepts Code Lab"
week: 6
module: "Robotic Infrastructure"
difficulty: "advanced"
prerequisites: ["ros2-fundamentals", "python-basics", "robotics-concepts"]
learning_objectives:
  - "Create complex launch files for system composition"
  - "Implement distributed robotics systems"
  - "Use advanced debugging and visualization tools"
tags: ["ros2", "launch", "composition", "debugging", "visualization", "distributed"]
hardware_requirements:
  - gpu: "Any"
  - ram: "8GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "120 minutes"
---

# Week 6: Advanced ROS 2 Concepts Code Lab

## Learning Objectives
- Create complex launch files for system composition
- Implement distributed robotics systems
- Use advanced debugging and visualization tools
- Practice advanced ROS 2 development workflows

## 6.1 Complex Launch Files

Let's create sophisticated launch files that demonstrate system composition:

```python
# launch/advanced_robot_system_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_namespace = LaunchConfiguration('robot_namespace')
    enable_diagnostics = LaunchConfiguration('enable_diagnostics')
    enable_visualization = LaunchConfiguration('enable_visualization')
    enable_logging = LaunchConfiguration('enable_logging')
    log_level = LaunchConfiguration('log_level')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='robot1',
            description='Robot namespace'
        ),
        DeclareLaunchArgument(
            'enable_diagnostics',
            default_value='true',
            description='Enable diagnostic nodes'
        ),
        DeclareLaunchArgument(
            'enable_visualization',
            default_value='true',
            description='Enable visualization nodes'
        ),
        DeclareLaunchArgument(
            'enable_logging',
            default_value='true',
            description='Enable detailed logging'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            choices=['debug', 'info', 'warn', 'error', 'fatal'],
            description='Log level'
        ),

        # Set global parameters
        SetParameter(name='use_sim_time', value=use_sim_time),

        # Robot controller group
        GroupAction(
            actions=[
                PushRosNamespace(robot_namespace),

                # Main robot controller
                Node(
                    package='my_robot_package',
                    executable='robot_controller',
                    name='robot_controller',
                    parameters=[
                        {'use_sim_time': use_sim_time},
                        {'robot_id': robot_namespace},
                        {'control_frequency': 50},
                        {'max_velocity': 1.0},
                        {'safety_timeout': 5.0}
                    ],
                    remappings=[
                        ('/cmd_vel', 'cmd_vel'),
                        ('/odom', 'odom'),
                        ('/scan', 'scan')
                    ],
                    respawn=True,
                    respawn_delay=2.0,
                    arguments=['--ros-args', '--log-level', log_level],
                    output='screen'
                ),

                # Sensor processor
                Node(
                    package='my_robot_package',
                    executable='sensor_processor',
                    name='sensor_processor',
                    parameters=[
                        {'use_sim_time': use_sim_time},
                        {'sensor_timeout': 1.0}
                    ],
                    arguments=['--ros-args', '--log-level', log_level],
                    output='screen'
                )
            ]
        ),

        # Diagnostic nodes (conditional)
        GroupAction(
            condition=IfCondition(enable_diagnostics),
            actions=[
                PushRosNamespace(robot_namespace),

                # Diagnostic aggregator
                Node(
                    package='diagnostics',
                    executable='diagnostic_aggregator',
                    name='diagnostic_aggregator',
                    parameters=[
                        {'use_sim_time': use_sim_time},
                        {'diagnostic_period': 1.0}
                    ],
                    arguments=['--ros-args', '--log-level', log_level],
                    output='screen'
                ),

                # Performance monitor
                Node(
                    package='my_robot_package',
                    executable='performance_monitor',
                    name='performance_monitor',
                    parameters=[
                        {'use_sim_time': use_sim_time}
                    ],
                    arguments=['--ros-args', '--log-level', log_level],
                    output='screen'
                )
            ]
        ),

        # Visualization nodes (conditional)
        GroupAction(
            condition=IfCondition(enable_visualization),
            actions=[
                PushRosNamespace(robot_namespace),

                # RViz2
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', PathJoinSubstitution([
                        FindPackageShare('my_robot_package'),
                        'rviz',
                        'robot_view.rviz'
                    ])],
                    condition=IfCondition(PythonExpression(['"', robot_namespace, '" == "robot1"']))
                ),

                # Custom visualization
                Node(
                    package='my_robot_package',
                    executable='advanced_visualization',
                    name='advanced_visualization',
                    parameters=[
                        {'use_sim_time': use_sim_time}
                    ],
                    arguments=['--ros-args', '--log-level', log_level],
                    output='screen'
                )
            ]
        )
    ])
```

## 6.2 Composable Nodes Implementation

Let's create composable nodes that can be loaded as components:

```python
# composable_nodes.py
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String, Bool, Float64
from sensor_msgs.msg import JointState, LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from threading import Lock

class JointControllerComponent(Node):
    """Composable joint controller component"""

    def __init__(self):
        super().__init__('joint_controller_component')

        # Declare parameters
        self.declare_parameter('control_frequency', 50.0, ParameterDescriptor(
            description='Joint controller update frequency'
        ))
        self.declare_parameter('max_velocity', 1.0, ParameterDescriptor(
            description='Maximum joint velocity'
        ))

        # Initialize internal state
        self.joint_positions = [0.0, 0.0, 0.0]
        self.joint_velocities = [0.0, 0.0, 0.0]
        self.joint_targets = [0.0, 0.0, 0.0]
        self.controller_enabled = True
        self.state_lock = Lock()

        # Create publishers and subscribers
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.joint_cmd_sub = self.create_subscription(
            JointState, 'joint_commands', self.joint_command_callback, qos_profile
        )
        self.enable_sub = self.create_subscription(
            Bool, 'controller_enable', self.enable_callback, 10
        )

        # Create timer
        freq = self.get_parameter('control_frequency').value
        self.control_timer = self.create_timer(1.0/freq, self.control_loop)

    def joint_command_callback(self, msg):
        """Handle joint command messages"""
        with self.state_lock:
            if len(msg.position) >= len(self.joint_targets):
                self.joint_targets = list(msg.position[:len(self.joint_targets)])

    def enable_callback(self, msg):
        """Handle controller enable/disable messages"""
        self.controller_enabled = msg.data
        self.get_logger().info(f'Controller {"enabled" if self.controller_enabled else "disabled"}')

    def control_loop(self):
        """Main control loop"""
        if not self.controller_enabled:
            return

        with self.state_lock:
            # Simple PD control
            kp = 2.0
            kd = 0.5

            for i in range(len(self.joint_positions)):
                error = self.joint_targets[i] - self.joint_positions[i]
                velocity_error = -self.joint_velocities[i]

                control_effort = kp * error + kd * velocity_error
                self.joint_velocities[i] = control_effort

                # Update positions
                dt = 1.0 / self.get_parameter('control_frequency').value
                self.joint_positions[i] += self.joint_velocities[i] * dt

        # Publish joint states
        self.publish_joint_states()

    def publish_joint_states(self):
        """Publish current joint states"""
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.header.frame_id = 'base_link'
        joint_msg.name = ['joint1', 'joint2', 'joint3']

        with self.state_lock:
            joint_msg.position = self.joint_positions.copy()
            joint_msg.velocity = self.joint_velocities.copy()

        self.joint_state_pub.publish(joint_msg)

class SensorProcessorComponent(Node):
    """Composable sensor processor component"""

    def __init__(self):
        super().__init__('sensor_processor_component')

        # Declare parameters
        self.declare_parameter('sensor_timeout', 1.0, ParameterDescriptor(
            description='Timeout for sensor data'
        ))
        self.declare_parameter('laser_range_min', 0.1, ParameterDescriptor(
            description='Minimum valid laser range'
        ))
        self.declare_parameter('laser_range_max', 10.0, ParameterDescriptor(
            description='Maximum valid laser range'
        ))

        # Initialize internal state
        self.last_scan_time = self.get_clock().now()
        self.valid_scan_ranges = []
        self.scan_valid = False

        # Create publishers and subscribers
        qos_profile = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile
        )
        self.processed_scan_pub = self.create_publisher(LaserScan, 'processed_scan', qos_profile)
        self.scan_valid_pub = self.create_publisher(Bool, 'scan_valid', 10)

        # Create timer for timeout checking
        self.timeout_timer = self.create_timer(0.1, self.check_scan_timeout)

    def scan_callback(self, msg):
        """Process incoming laser scan data"""
        self.last_scan_time = self.get_clock().now()

        # Process scan ranges
        min_range = self.get_parameter('laser_range_min').value
        max_range = self.get_parameter('laser_range_max').value

        processed_ranges = []
        for range_val in msg.ranges:
            if min_range <= range_val <= max_range:
                processed_ranges.append(range_val)
            else:
                processed_ranges.append(float('inf'))  # Mark as invalid

        self.valid_scan_ranges = processed_ranges
        self.scan_valid = len([r for r in processed_ranges if r != float('inf')]) > 0

        # Publish processed scan
        processed_msg = msg
        processed_msg.ranges = processed_ranges
        self.processed_scan_pub.publish(processed_msg)

        # Publish validity status
        valid_msg = Bool()
        valid_msg.data = self.scan_valid
        self.scan_valid_pub.publish(valid_msg)

    def check_scan_timeout(self):
        """Check if sensor data has timed out"""
        current_time = self.get_clock().now()
        time_since_last_scan = (current_time - self.last_scan_time).nanoseconds / 1e9

        timeout = self.get_parameter('sensor_timeout').value
        if time_since_last_scan > timeout:
            self.scan_valid = False
            valid_msg = Bool()
            valid_msg.data = False
            self.scan_valid_pub.publish(valid_msg)

class MotionControllerComponent(Node):
    """Composable motion controller component"""

    def __init__(self):
        super().__init__('motion_controller_component')

        # Declare parameters
        self.declare_parameter('linear_vel_max', 1.0, ParameterDescriptor(
            description='Maximum linear velocity'
        ))
        self.declare_parameter('angular_vel_max', 1.0, ParameterDescriptor(
            description='Maximum angular velocity'
        ))
        self.declare_parameter('acceleration_limit', 2.0, ParameterDescriptor(
            description='Maximum acceleration'
        ))

        # Initialize internal state
        self.current_twist = Twist()
        self.target_twist = Twist()
        self.controller_active = True

        # Create publishers and subscribers
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, qos_profile
        )
        self.motion_cmd_pub = self.create_publisher(Twist, 'motion_commands', qos_profile)

        # Create timer
        self.control_timer = self.create_timer(0.02, self.motion_control_loop)  # 50 Hz

    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        self.target_twist = msg

    def motion_control_loop(self):
        """Motion control loop with acceleration limiting"""
        if not self.controller_active:
            self.current_twist.linear.x = 0.0
            self.current_twist.angular.z = 0.0

        # Apply acceleration limits
        max_accel = self.get_parameter('acceleration_limit').value
        dt = 0.02  # Timer period

        # Limit linear acceleration
        linear_diff = self.target_twist.linear.x - self.current_twist.linear.x
        max_change = max_accel * dt
        limited_diff = max(min(linear_diff, max_change), -max_change)
        self.current_twist.linear.x += limited_diff

        # Limit angular acceleration
        angular_diff = self.target_twist.angular.z - self.current_twist.angular.z
        limited_angular_diff = max(min(angular_diff, max_change), -max_change)
        self.current_twist.angular.z += limited_angular_diff

        # Apply velocity limits
        linear_max = self.get_parameter('linear_vel_max').value
        angular_max = self.get_parameter('angular_vel_max').value

        self.current_twist.linear.x = max(min(self.current_twist.linear.x, linear_max), -linear_max)
        self.current_twist.angular.z = max(min(self.current_twist.angular.z, angular_max), -angular_max)

        # Publish motion commands
        self.motion_cmd_pub.publish(self.current_twist)

# Register components for composition
from rclpy_components import register_component

def register_components():
    register_component(JointControllerComponent)
    register_component(SensorProcessorComponent)
    register_component(MotionControllerComponent)

if __name__ == '__main__':
    register_components()
```

## 6.3 Distributed System Implementation

Let's create a distributed robotics system:

```python
# distributed_robot_system.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import JointState
import socket
import threading
import json
import time
from collections import defaultdict

class DistributedRobotNode(Node):
    """Distributed robot node for multi-robot systems"""

    def __init__(self):
        super().__init__('distributed_robot_node')

        # Get network information
        self.host_ip = self._get_host_ip()
        self.robot_id = self._generate_robot_id()

        # Declare parameters
        self.declare_parameter('network_port', 5555)
        self.declare_parameter('discovery_interval', 5.0)
        self.declare_parameter('heartbeat_interval', 1.0)
        self.declare_parameter('max_robots', 10)

        # Initialize robot network state
        self.known_robots = {}  # {robot_id: {'ip': ip, 'last_seen': time, 'status': status}}
        self.robot_states = {}  # {robot_id: {'pose': pose, 'status': status}}

        # Create publishers and subscribers
        self.status_pub = self.create_publisher(String, 'robot_status', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'robot_pose', 10)
        self.discovery_pub = self.create_publisher(String, 'robot_discovery', 10)

        # Create subscribers
        self.discovery_sub = self.create_subscription(
            String, 'robot_discovery', self._discovery_callback, 10
        )

        # Create timers
        self.discovery_timer = self.create_timer(
            self.get_parameter('discovery_interval').value,
            self._publish_discovery_message
        )
        self.heartbeat_timer = self.create_timer(
            self.get_parameter('heartbeat_interval').value,
            self._publish_heartbeat
        )
        self.status_timer = self.create_timer(0.5, self._publish_status)

        # Start network thread
        self.network_thread = threading.Thread(target=self._network_worker)
        self.network_thread.daemon = True
        self.network_thread.start()

        self.get_logger().info(f'Distributed robot node started: {self.robot_id} at {self.host_ip}')

    def _get_host_ip(self):
        """Get the host machine's IP address"""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                return s.getsockname()[0]
        except Exception:
            return "127.0.0.1"

    def _generate_robot_id(self):
        """Generate unique robot ID"""
        import uuid
        return f"robot_{uuid.uuid4().hex[:8]}"

    def _publish_discovery_message(self):
        """Publish discovery message to find other robots"""
        discovery_msg = {
            'robot_id': self.robot_id,
            'ip_address': self.host_ip,
            'timestamp': time.time(),
            'status': 'active'
        }

        msg = String()
        msg.data = json.dumps(discovery_msg)
        self.discovery_pub.publish(msg)

    def _publish_heartbeat(self):
        """Publish heartbeat to maintain network presence"""
        heartbeat_msg = {
            'robot_id': self.robot_id,
            'timestamp': time.time(),
            'status': 'active'
        }

        # Update known robots
        self.known_robots[self.robot_id] = {
            'ip': self.host_ip,
            'last_seen': time.time(),
            'status': 'active'
        }

    def _discovery_callback(self, msg):
        """Handle discovery messages from other robots"""
        try:
            robot_info = json.loads(msg.data)
            robot_id = robot_info['robot_id']

            # Update known robots
            self.known_robots[robot_id] = {
                'ip': robot_info.get('ip_address', 'unknown'),
                'last_seen': robot_info['timestamp'],
                'status': robot_info['status']
            }

            self.get_logger().info(f'Discovered robot: {robot_id} at {self.known_robots[robot_id]["ip"]}')
        except json.JSONDecodeError:
            self.get_logger().error('Invalid discovery message received')

    def _publish_status(self):
        """Publish robot status"""
        status_msg = String()
        status_msg.data = f'Robot {self.robot_id} active, {len(self.known_robots)} robots discovered'
        self.status_pub.publish(status_msg)

    def _network_worker(self):
        """Background network worker thread"""
        # This could implement additional network protocols
        # for direct robot-to-robot communication
        while rclpy.ok():
            # Check for stale robots
            current_time = time.time()
            timeout = self.get_parameter('discovery_interval').value * 3  # 3x timeout

            stale_robots = []
            for robot_id, info in self.known_robots.items():
                if current_time - info['last_seen'] > timeout and robot_id != self.robot_id:
                    stale_robots.append(robot_id)

            for robot_id in stale_robots:
                del self.known_robots[robot_id]
                self.get_logger().info(f'Removed stale robot: {robot_id}')

            time.sleep(1.0)  # Check every second

    def get_network_topology(self):
        """Get current network topology"""
        return {
            'local_robot': self.robot_id,
            'local_ip': self.host_ip,
            'discovered_robots': dict(self.known_robots)
        }

def main(args=None):
    rclpy.init(args=args)
    node = DistributedRobotNode()

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

## 6.4 Advanced Debugging Tools

Let's create comprehensive debugging tools:

```python
# advanced_debugging_tools.py
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Float64, String, Bool
from rclpy.qos import QoSProfile
import psutil
import time
from collections import deque
import threading
import traceback
from functools import wraps

def debug_timer(func):
    """Decorator to time function execution"""
    @wraps(func)
    def wrapper(*args, **kwargs):
        start = time.time()
        result = func(*args, **kwargs)
        end = time.time()
        print(f"{func.__name__} took {(end-start)*1000:.2f}ms")
        return result
    return wrapper

class AdvancedDebugNode(Node):
    """Advanced debugging and diagnostic node"""

    def __init__(self):
        super().__init__('advanced_debug_node')

        # Publishers
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.cpu_pub = self.create_publisher(Float64, 'debug/cpu_usage', 10)
        self.memory_pub = self.create_publisher(Float64, 'debug/memory_usage', 10)
        self.log_pub = self.create_publisher(String, 'debug/log_messages', 10)

        # Internal state
        self.message_counts = {}
        self.performance_data = {}
        self.error_log = deque(maxlen=100)
        self.warning_log = deque(maxlen=100)

        # Timers
        self.diag_timer = self.create_timer(1.0, self._publish_diagnostics)
        self.monitor_timer = self.create_timer(0.5, self._monitor_system)

        # Thread for background monitoring
        self.monitor_thread = threading.Thread(target=self._background_monitor)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

    def _monitor_system(self):
        """Monitor system resources"""
        # CPU usage
        cpu_percent = psutil.cpu_percent()
        cpu_msg = Float64()
        cpu_msg.data = cpu_percent
        self.cpu_pub.publish(cpu_msg)

        # Memory usage
        memory_percent = psutil.virtual_memory().percent
        memory_msg = Float64()
        memory_msg.data = memory_percent
        self.memory_pub.publish(memory_msg)

    def _background_monitor(self):
        """Background monitoring thread"""
        while rclpy.ok():
            try:
                # Monitor disk usage
                disk_percent = psutil.disk_usage('/').percent
                if disk_percent > 90:
                    self._log_warning(f'Disk usage high: {disk_percent}%')

                # Monitor network connections
                connections = psutil.net_connections()
                if len(connections) > 100:  # Arbitrary threshold
                    self._log_warning(f'High number of network connections: {len(connections)}')

                time.sleep(2.0)
            except Exception as e:
                self._log_error(f'Background monitor error: {e}')
                time.sleep(1.0)

    def _publish_diagnostics(self):
        """Publish comprehensive diagnostic information"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # System health diagnostic
        sys_diag = DiagnosticStatus()
        sys_diag.name = 'System Health'
        sys_diag.hardware_id = self.get_name()

        # Calculate overall health score
        cpu_percent = psutil.cpu_percent()
        memory_percent = psutil.virtual_memory().percent
        disk_percent = psutil.disk_usage('/').percent

        health_score = (cpu_percent + memory_percent + disk_percent) / 3

        if health_score < 50:
            sys_diag.level = DiagnosticStatus.OK
            sys_diag.message = 'System healthy'
        elif health_score < 80:
            sys_diag.level = DiagnosticStatus.WARN
            sys_diag.message = 'System under moderate load'
        else:
            sys_diag.level = DiagnosticStatus.ERROR
            sys_diag.message = 'System overloaded'

        # Add detailed metrics
        sys_diag.values = [
            KeyValue(key='cpu_percent', value=f'{cpu_percent}%'),
            KeyValue(key='memory_percent', value=f'{memory_percent}%'),
            KeyValue(key='disk_percent', value=f'{disk_percent}%'),
            KeyValue(key='process_count', value=str(len(psutil.pids()))),
            KeyValue(key='uptime_seconds', value=str(time.time() - psutil.boot_time()))
        ]

        diag_array.status.append(sys_diag)

        # ROS communication diagnostic
        ros_diag = DiagnosticStatus()
        ros_diag.name = 'ROS Communication'
        ros_diag.hardware_id = f'{self.get_name()}_ros'

        # Count active topics
        topic_names_and_types = self.get_topic_names_and_types()
        topic_count = len(topic_names_and_types)

        if topic_count > 50:  # Arbitrary threshold
            ros_diag.level = DiagnosticStatus.WARN
            ros_diag.message = f'High topic count: {topic_count}'
        else:
            ros_diag.level = DiagnosticStatus.OK
            ros_diag.message = f'Normal topic count: {topic_count}'

        ros_diag.values = [
            KeyValue(key='topic_count', value=str(topic_count)),
            KeyValue(key='node_count', value=str(len(self.get_node_names())))
        ]

        diag_array.status.append(ros_diag)

        # Publish diagnostics
        self.diag_pub.publish(diag_array)

    def _log_error(self, message):
        """Log an error message"""
        error_entry = {
            'timestamp': time.time(),
            'message': message,
            'traceback': traceback.format_stack()
        }
        self.error_log.append(error_entry)

        log_msg = String()
        log_msg.data = f'ERROR: {message}'
        self.log_pub.publish(log_msg)

    def _log_warning(self, message):
        """Log a warning message"""
        warning_entry = {
            'timestamp': time.time(),
            'message': message
        }
        self.warning_log.append(warning_entry)

        log_msg = String()
        log_msg.data = f'WARNING: {message}'
        self.log_pub.publish(log_msg)

    def increment_message_count(self, topic_name):
        """Increment message count for a topic"""
        if topic_name not in self.message_counts:
            self.message_counts[topic_name] = 0
        self.message_counts[topic_name] += 1

    def get_performance_metrics(self):
        """Get current performance metrics"""
        return {
            'message_counts': dict(self.message_counts),
            'error_count': len(self.error_log),
            'warning_count': len(self.warning_log),
            'cpu_percent': psutil.cpu_percent(),
            'memory_percent': psutil.virtual_memory().percent
        }

def main(args=None):
    rclpy.init(args=args)
    debug_node = AdvancedDebugNode()

    try:
        rclpy.spin(debug_node)
    except KeyboardInterrupt:
        pass
    finally:
        debug_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 6.5 Advanced Visualization Implementation

Let's create advanced visualization tools:

```python
# advanced_visualization.py
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped, Vector3
from std_msgs.msg import ColorRGBA, Float64
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
import math
import numpy as np
from collections import deque

class AdvancedVisualizationNode(Node):
    """Advanced visualization node for complex robotic data"""

    def __init__(self):
        super().__init__('advanced_visualization_node')

        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, 'advanced_markers', 10)
        self.path_pub = self.create_publisher(Path, 'robot_path', 10)
        self.scan_viz_pub = self.create_publisher(MarkerArray, 'scan_visualization', 10)

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, 'robot_pose', self._pose_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self._scan_callback, 10
        )
        self.velocity_sub = self.create_subscription(
            Float64, 'robot_velocity', self._velocity_callback, 10
        )

        # Internal state
        self.robot_path = deque(maxlen=500)  # Keep last 500 points
        self.scan_points = []
        self.current_velocity = 0.0
        self.time_counter = 0.0

        # Create timer for visualization updates
        self.vis_timer = self.create_timer(0.05, self._publish_visualization)  # 20 Hz

    def _pose_callback(self, msg):
        """Handle robot pose updates"""
        # Add to path
        self.robot_path.append((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))

    def _scan_callback(self, msg):
        """Handle laser scan updates"""
        # Convert scan to 3D points
        self.scan_points = []
        angle = msg.angle_min

        for range_val in msg.ranges:
            if not (math.isnan(range_val) or math.isinf(range_val)):
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                self.scan_points.append((x, y, 0.0))
            angle += msg.angle_increment

    def _velocity_callback(self, msg):
        """Handle velocity updates"""
        self.current_velocity = msg.data

    def _publish_visualization(self):
        """Publish all visualization elements"""
        self.time_counter += 0.05

        # Create marker array
        marker_array = MarkerArray()

        # Robot position and orientation
        if self.robot_path:
            last_pos = self.robot_path[-1]
            robot_marker = self._create_robot_marker(last_pos, len(marker_array.markers))
            marker_array.markers.append(robot_marker)

        # Robot path
        if len(self.robot_path) > 1:
            path_marker = self._create_path_marker(len(marker_array.markers))
            marker_array.markers.append(path_marker)

        # Laser scan points
        if self.scan_points:
            scan_marker = self._create_scan_marker(len(marker_array.markers))
            marker_array.markers.append(scan_marker)

        # Velocity indicator
        velocity_marker = self._create_velocity_marker(len(marker_array.markers))
        marker_array.markers.append(velocity_marker)

        # Publish markers
        self.marker_pub.publish(marker_array)

        # Publish path separately
        self._publish_path()

    def _create_robot_marker(self, pos, marker_id):
        """Create robot visualization marker"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = pos[0]
        marker.pose.position.y = pos[1]
        marker.pose.position.z = pos[2]

        # Orientation (simplified - pointing in direction of movement)
        marker.pose.orientation.w = 1.0

        # Scale
        marker.scale = Vector3(x=0.3, y=0.1, z=0.1)

        # Color based on velocity
        velocity_color = self._velocity_to_color(self.current_velocity)
        marker.color = ColorRGBA(r=velocity_color[0], g=velocity_color[1], b=velocity_color[2], a=1.0)

        return marker

    def _create_path_marker(self, marker_id):
        """Create path visualization marker"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "path"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Scale
        marker.scale = Vector3(x=0.02, y=0.02, z=0.02)

        # Color
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)

        # Points
        for x, y, z in self.robot_path:
            point = Point()
            point.x = x
            point.y = y
            point.z = z + 0.05  # Slightly above ground
            marker.points.append(point)

        return marker

    def _create_scan_marker(self, marker_id):
        """Create laser scan visualization marker"""
        marker = Marker()
        marker.header.frame_id = "laser_frame"  # Assuming laser frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "scan"
        marker.id = marker_id
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        # Scale
        marker.scale = Vector3(x=0.05, y=0.05, z=0.05)

        # Color
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)

        # Points
        for x, y, z in self.scan_points:
            point = Point()
            point.x = x
            point.y = y
            point.z = z
            marker.points.append(point)

        return marker

    def _create_velocity_marker(self, marker_id):
        """Create velocity visualization marker"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "velocity"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Position (slightly above robot)
        if self.robot_path:
            last_pos = self.robot_path[-1]
            marker.pose.position.x = last_pos[0]
            marker.pose.position.y = last_pos[1]
            marker.pose.position.z = last_pos[2] + 0.5

        # Scale
        marker.scale = Vector3(x=0.2, y=0.2, z=0.2)

        # Color
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)

        # Text
        marker.text = f'Velocity: {self.current_velocity:.2f} m/s'

        return marker

    def _velocity_to_color(self, velocity):
        """Convert velocity to RGB color (blue=slow, red=fast)"""
        # Normalize velocity (assuming max of 2.0 m/s)
        norm_vel = min(abs(velocity) / 2.0, 1.0)

        # Blue to red gradient
        if norm_vel < 0.5:
            # Blue to green
            r = 0.0
            g = norm_vel * 2.0
            b = 1.0 - (norm_vel * 2.0)
        else:
            # Green to red
            r = (norm_vel - 0.5) * 2.0
            g = 1.0 - ((norm_vel - 0.5) * 2.0)
            b = 0.0

        return (r, g, b)

    def _publish_path(self):
        """Publish path as Path message"""
        if len(self.robot_path) < 2:
            return

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y, z in self.robot_path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    vis_node = AdvancedVisualizationNode()

    try:
        rclpy.spin(vis_node)
    except KeyboardInterrupt:
        pass
    finally:
        vis_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 6.6 Practical Exercise

Create a complete distributed robotics system that:
1. Implements multiple composable nodes
2. Uses advanced launch file composition
3. Includes comprehensive debugging and diagnostics
4. Provides advanced visualization capabilities
5. Demonstrates distributed communication

```python
# Student exercise - Complete implementation
class CompleteDistributedSystem:
    """Student implementation of a complete distributed system"""

    def __init__(self):
        """Initialize the complete distributed system"""
        # TODO: Create multiple composable nodes
        # TODO: Implement launch file composition
        # TODO: Add debugging and diagnostic capabilities
        # TODO: Implement visualization
        # TODO: Create distributed communication
        # TODO: Add error handling and recovery
        pass

    def create_composable_nodes(self):
        """Create multiple composable nodes"""
        # TODO: Complete implementation
        pass

    def implement_distributed_communication(self):
        """Implement distributed communication"""
        # TODO: Complete implementation
        pass

    def add_diagnostics(self):
        """Add comprehensive diagnostics"""
        # TODO: Complete implementation
        pass

    def implement_visualization(self):
        """Implement advanced visualization"""
        # TODO: Complete implementation
        pass

    def create_launch_files(self):
        """Create comprehensive launch files"""
        # TODO: Complete implementation
        pass

print("Student Exercise: Implement a complete distributed system")
print("Requirements:")
print("1. Multiple composable nodes")
print("2. Advanced launch file composition")
print("3. Comprehensive debugging and diagnostics")
print("4. Advanced visualization capabilities")
print("5. Distributed communication system")
print("6. Error handling and recovery mechanisms")
```

## Summary

In this lab, you've learned advanced ROS 2 concepts including complex launch files, composable nodes, distributed systems, advanced debugging tools, and comprehensive visualization. These concepts enable the development of sophisticated robotic systems with proper system composition and monitoring.

<div class="alert alert-info">
  <h5>Hardware Requirement</h5>
  <div><strong>Requirement:</strong> GPU</div>
  <div><strong>Minimum:</strong> Any</div>
  <div><strong>Recommended:</strong> Any</div>
  <div><strong>Purpose:</strong> Basic computational requirements for Week 6 advanced ROS 2 lab</div>
</div>