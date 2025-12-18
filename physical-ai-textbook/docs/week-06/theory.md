---
title: "Week 6: Advanced ROS 2 Concepts"
week: 6
module: "Robotic Infrastructure"
difficulty: "advanced"
prerequisites: ["ros2-fundamentals", "python-basics", "robotics-concepts"]
learning_objectives:
  - "Work with launch files and system composition"
  - "Implement distributed robotics systems"
  - "Use debugging and visualization tools"
tags: ["ros2", "launch", "composition", "debugging", "visualization"]
hardware_requirements:
  - gpu: "Any"
  - ram: "8GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "90 minutes"
---

# Week 6: Advanced ROS 2 Concepts

## Learning Objectives
- Work with launch files and system composition
- Implement distributed robotics systems
- Use debugging and visualization tools
- Understand best practices for ROS 2 development

## 6.1 Advanced Launch Files and System Composition

### Complex Launch File Structures

Launch files in ROS 2 provide powerful mechanisms for starting and configuring multiple nodes simultaneously. Advanced launch files can include conditional execution, parameter management, and complex node configurations.

```python
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.descriptions import ParameterFile

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_namespace = LaunchConfiguration('robot_namespace')
    enable_logging = LaunchConfiguration('enable_logging')

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
            'enable_logging',
            default_value='true',
            description='Enable detailed logging'
        ),

        # Set global parameters
        SetParameter(name='use_sim_time', value=use_sim_time),

        # Group nodes with conditional execution
        GroupAction(
            condition=IfCondition(enable_logging),
            actions=[
                # Diagnostic node
                Node(
                    package='diagnostics',
                    executable='diagnostic_aggregator',
                    name='diagnostic_aggregator',
                    parameters=[{'use_sim_time': use_sim_time}]
                ),
                # Performance monitor
                Node(
                    package='performance_monitor',
                    executable='perf_monitor',
                    name='performance_monitor',
                    parameters=[{'use_sim_time': use_sim_time}]
                )
            ]
        ),

        # Robot controller node
        Node(
            package='my_robot_package',
            executable='robot_controller',
            name='robot_controller',
            namespace=robot_namespace,
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_id': robot_namespace},
                {'control_frequency': 50},
                {'max_velocity': 1.0}
            ],
            remappings=[
                ('/cmd_vel', 'cmd_vel'),
                ('/odom', 'odom'),
                ('/scan', 'scan')
            ],
            respawn=True,
            respawn_delay=2.0,
            output='screen'
        )
    ])
```

### Composable Nodes and Components

Node composition allows multiple nodes to run in the same process, reducing communication overhead and improving performance.

```python
# Composable node example
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String

class ComposableNode(Node):
    """Example of a composable node that can be loaded as a component"""

    def __init__(self):
        super().__init__('composable_node')

        # Declare parameters
        self.declare_parameter('publish_rate', 1.0, ParameterDescriptor(
            description='Rate at which to publish messages'
        ))

        # Create publisher and subscriber
        self.publisher = self.create_publisher(String, 'composed_topic', 10)
        self.subscription = self.create_subscription(
            String, 'composed_input', self.subscription_callback, 10
        )

        # Create timer
        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0/rate, self.timer_callback)

    def subscription_callback(self, msg):
        self.get_logger().info(f'Received in composed node: {msg.data}')

    def timer_callback(self):
        msg = String()
        msg.data = f'Composed node message at {self.get_clock().now().nanoseconds}'
        self.publisher.publish(msg)

# Register the component
from rclpy_components import register_component
register_component(ComposableNode)
```

### Lifecycle Nodes

Lifecycle nodes provide better control over node state management and system initialization.

```python
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn
from std_msgs.msg import String

class LifecycleManagerNode(LifecycleNode):
    """Example lifecycle node with state management"""

    def __init__(self):
        super().__init__('lifecycle_node')
        self.publisher = None

    def on_configure(self, state):
        """Called when node transitions to CONFIGURING state"""
        self.get_logger().info('Configuring lifecycle node')

        # Create publisher
        self.publisher = self.create_publisher(String, 'lifecycle_topic', 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        """Called when node transitions to ACTIVATING state"""
        self.get_logger().info('Activating lifecycle node')

        # Activate publisher
        self.publisher.on_activate()

        # Create timer
        self.timer = self.create_timer(1.0, self.timer_callback)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        """Called when node transitions to DEACTIVATING state"""
        self.get_logger().info('Deactivating lifecycle node')

        # Deactivate publisher
        self.publisher.on_deactivate()

        # Destroy timer
        self.timer.destroy()

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        """Called when node transitions to CLEANINGUP state"""
        self.get_logger().info('Cleaning up lifecycle node')

        # Destroy publisher
        self.publisher.destroy()
        self.publisher = None

        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        msg = String()
        msg.data = f'Lifecycle node active at {self.get_clock().now().nanoseconds}'
        self.publisher.publish(msg)
```

## 6.2 Distributed Robotics Systems

### Multi-Machine Communication

ROS 2's DDS (Data Distribution Service) middleware enables robust distributed communication between machines.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket

class DistributedNode(Node):
    def __init__(self):
        super().__init__('distributed_node')

        # Get this machine's IP
        self.host_ip = self.get_host_ip()
        self.get_logger().info(f'Node running on IP: {self.host_ip}')

        # Publisher for distributed communication
        self.dist_publisher = self.create_publisher(String, 'distributed_topic', 10)

        # Timer to periodically publish location info
        self.timer = self.create_timer(5.0, self.publish_location_info)

    def get_host_ip(self):
        """Get the host machine's IP address"""
        try:
            # Connect to a remote server to determine local IP
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                return s.getsockname()[0]
        except Exception:
            return "127.0.0.1"

    def publish_location_info(self):
        """Publish information about this node's location"""
        msg = String()
        msg.data = f'Node {self.get_name()} on host {self.host_ip} at {self.get_clock().now().nanoseconds}'
        self.dist_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DistributedNode()

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

### DDS Configuration for Distributed Systems

Proper DDS configuration is crucial for distributed systems:

```python
# Configure DDS for distributed communication
import os

# Set DDS environment variables for distributed operation
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'  # Or rmw_fastrtps_cpp
os.environ['CYCLONEDDS_URI'] = """
<?xml version="1.0" encoding="UTF-8" ?>
<dds>
    <general>
        <network>
            <allow_multicast>true</allow_multicast>
            <interface>auto</interface>
        </network>
    </general>
    <builtin>
        <participant>
            <reader>
                <heartbeat_period>100ms</heartbeat_period>
            </reader>
        </participant>
    </builtin>
</dds>
"""
```

## 6.3 Advanced Debugging Techniques

### Custom Diagnostic Messages

Creating custom diagnostic messages for system monitoring:

```python
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Bool
import psutil
import time

class AdvancedDiagnosticNode(Node):
    def __init__(self):
        super().__init__('advanced_diagnostic_node')

        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.system_status_pub = self.create_publisher(Bool, 'system_operational', 10)

        self.diag_timer = self.create_timer(1.0, self.publish_diagnostics)

    def publish_diagnostics(self):
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # System health diagnostic
        sys_health = DiagnosticStatus()
        sys_health.name = 'System Health'

        # Check multiple system metrics
        cpu_percent = psutil.cpu_percent()
        memory_percent = psutil.virtual_memory().percent
        disk_percent = psutil.disk_usage('/').percent

        # Determine overall health
        if cpu_percent < 70 and memory_percent < 70 and disk_percent < 80:
            sys_health.level = DiagnosticStatus.OK
            sys_health.message = 'System healthy'
        elif cpu_percent < 90 or memory_percent < 90 or disk_percent < 95:
            sys_health.level = DiagnosticStatus.WARN
            sys_health.message = 'System under moderate load'
        else:
            sys_health.level = DiagnosticStatus.ERROR
            sys_health.message = 'System overloaded'

        # Add key-value pairs for detailed metrics
        sys_health.values = [
            KeyValue(key='cpu_percent', value=str(cpu_percent)),
            KeyValue(key='memory_percent', value=str(memory_percent)),
            KeyValue(key='disk_percent', value=str(disk_percent)),
            KeyValue(key='timestamp', value=str(time.time()))
        ]

        diag_array.status.append(sys_health)
        self.diag_pub.publish(diag_array)

        # Publish system operational status
        operational_msg = Bool()
        operational_msg.data = (sys_health.level != DiagnosticStatus.ERROR)
        self.system_status_pub.publish(operational_msg)

def main(args=None):
    rclpy.init(args=args)
    diag_node = AdvancedDiagnosticNode()

    try:
        rclpy.spin(diag_node)
    except KeyboardInterrupt:
        pass
    finally:
        diag_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Performance Monitoring

Implementing performance monitoring for ROS 2 systems:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32
from builtin_interfaces.msg import Time
import time
from collections import deque
import threading

class PerformanceMonitorNode(Node):
    def __init__(self):
        super().__init__('performance_monitor')

        # Publishers for performance metrics
        self.msg_rate_pub = self.create_publisher(Float64, 'perf_msg_rate', 10)
        self.cpu_usage_pub = self.create_publisher(Float64, 'perf_cpu_usage', 10)
        self.memory_usage_pub = self.create_publisher(Float64, 'perf_memory_usage', 10)

        # Performance tracking
        self.message_count = 0
        self.message_times = deque(maxlen=100)
        self.cpu_samples = deque(maxlen=50)
        self.memory_samples = deque(maxlen=50)

        # Timers
        self.rate_timer = self.create_timer(1.0, self.calculate_message_rate)
        self.perf_timer = self.create_timer(0.5, self.update_performance_metrics)

        # Thread for background monitoring
        self.monitoring_thread = threading.Thread(target=self.background_monitoring)
        self.monitoring_thread.daemon = True
        self.monitoring_thread.start()

    def background_monitoring(self):
        """Background thread for performance monitoring"""
        import psutil
        while rclpy.ok():
            # Monitor CPU usage
            cpu_percent = psutil.cpu_percent(interval=0.1)
            self.cpu_samples.append(cpu_percent)

            # Monitor memory usage
            memory_percent = psutil.virtual_memory().percent
            self.memory_samples.append(memory_percent)

            time.sleep(0.1)

    def record_message(self):
        """Call this method when a message is processed"""
        self.message_count += 1
        self.message_times.append(time.time())

    def calculate_message_rate(self):
        """Calculate and publish message processing rate"""
        if len(self.message_times) > 1:
            time_span = self.message_times[-1] - self.message_times[0]
            if time_span > 0:
                rate = len(self.message_times) / time_span
                rate_msg = Float64()
                rate_msg.data = rate
                self.msg_rate_pub.publish(rate_msg)

        # Reset counter for next interval
        self.message_count = 0

    def update_performance_metrics(self):
        """Update and publish performance metrics"""
        if self.cpu_samples:
            avg_cpu = sum(self.cpu_samples) / len(self.cpu_samples)
            cpu_msg = Float64()
            cpu_msg.data = avg_cpu
            self.cpu_usage_pub.publish(cpu_msg)

        if self.memory_samples:
            avg_memory = sum(self.memory_samples) / len(self.memory_samples)
            memory_msg = Float64()
            memory_msg.data = avg_memory
            self.memory_usage_pub.publish(memory_msg)

def main(args=None):
    rclpy.init(args=args)
    perf_node = PerformanceMonitorNode()

    try:
        rclpy.spin(perf_node)
    except KeyboardInterrupt:
        pass
    finally:
        perf_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 6.4 Advanced Visualization Techniques

### RViz Custom Plugins and Markers

Creating advanced visualization for robotic systems:

```python
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped, Vector3
from std_msgs.msg import ColorRGBA
import math
import numpy as np

class AdvancedVisualizationNode(Node):
    def __init__(self):
        super().__init__('advanced_visualization')

        self.marker_pub = self.create_publisher(MarkerArray, 'advanced_markers', 10)
        self.path_pub = self.create_publisher(PoseStamped, 'robot_path', 10)

        self.vis_timer = self.create_timer(0.1, self.publish_advanced_visualization)
        self.time_counter = 0.0

        # Store robot trajectory
        self.trajectory = []

    def publish_advanced_visualization(self):
        marker_array = MarkerArray()

        # Create a complex trajectory visualization
        self.time_counter += 0.05

        # Robot position marker
        robot_marker = Marker()
        robot_marker.header.frame_id = "map"
        robot_marker.header.stamp = self.get_clock().now().to_msg()
        robot_marker.ns = "robot"
        robot_marker.id = 0
        robot_marker.type = Marker.ARROW
        robot_marker.action = Marker.ADD
        robot_marker.pose.position.x = 3 * math.cos(self.time_counter)
        robot_marker.pose.position.y = 3 * math.sin(self.time_counter)
        robot_marker.pose.position.z = 0.0
        robot_marker.pose.orientation.w = 1.0
        robot_marker.scale = Vector3(x=0.5, y=0.2, z=0.2)
        robot_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        marker_array.markers.append(robot_marker)

        # Add to trajectory
        self.trajectory.append((robot_marker.pose.position.x, robot_marker.pose.position.y))
        if len(self.trajectory) > 200:  # Keep only last 200 points
            self.trajectory.pop(0)

        # Trajectory path
        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = "trajectory"
        path_marker.id = 1
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.pose.orientation.w = 1.0
        path_marker.scale = Vector3(x=0.05, y=0.05, z=0.05)
        path_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)

        for x, y in self.trajectory:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.05  # Slightly above ground
            path_marker.points.append(point)

        marker_array.markers.append(path_marker)

        # Create a safety zone around robot
        safety_marker = Marker()
        safety_marker.header.frame_id = "map"
        safety_marker.header.stamp = self.get_clock().now().to_msg()
        safety_marker.ns = "safety_zone"
        safety_marker.id = 2
        safety_marker.type = Marker.SPHERE
        safety_marker.action = Marker.ADD
        safety_marker.pose.position.x = robot_marker.pose.position.x
        safety_marker.pose.position.y = robot_marker.pose.position.y
        safety_marker.pose.position.z = 0.0
        safety_marker.pose.orientation.w = 1.0
        safety_marker.scale = Vector3(x=1.0, y=1.0, z=0.1)  # 1m radius
        safety_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.3)
        marker_array.markers.append(safety_marker)

        # Publish marker array
        self.marker_pub.publish(marker_array)

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

## 6.5 Best Practices for ROS 2 Development

### Code Organization and Structure

```python
# Recommended project structure
"""
my_robot_package/
├── src/
│   ├── controllers/
│   │   ├── __init__.py
│   │   └── joint_controller.py
│   ├── sensors/
│   │   ├── __init__.py
│   │   └── sensor_processor.py
│   └── utils/
│       ├── __init__.py
│       └── robot_helpers.py
├── launch/
│   ├── robot_system.launch.py
│   └── simulation.launch.py
├── config/
│   ├── robot_params.yaml
│   └── simulation_params.yaml
├── test/
│   ├── test_controllers.py
│   └── test_sensors.py
└── CMakeLists.txt / setup.py
"""

# Example of well-structured node
class WellStructuredRobotNode(Node):
    def __init__(self):
        super().__init__('well_structured_robot')

        # Initialize components
        self._initialize_parameters()
        self._initialize_publishers()
        self._initialize_subscribers()
        self._initialize_services()
        self._initialize_timers()
        self._initialize_components()

    def _initialize_parameters(self):
        """Initialize all parameters"""
        self.declare_parameter('robot_id', 'robot_01')
        self.declare_parameter('control_frequency', 50)
        self.declare_parameter('safety_timeout', 5.0)

    def _initialize_publishers(self):
        """Initialize all publishers"""
        self.status_pub = self.create_publisher(String, 'status', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

    def _initialize_subscribers(self):
        """Initialize all subscribers"""
        self.cmd_sub = self.create_subscription(
            String, 'commands', self._command_callback, 10
        )

    def _initialize_services(self):
        """Initialize all services"""
        self.enable_service = self.create_service(
            SetBool, 'enable', self._enable_callback
        )

    def _initialize_timers(self):
        """Initialize all timers"""
        self.control_timer = self.create_timer(
            1.0 / self.get_parameter('control_frequency').value,
            self._control_loop
        )

    def _initialize_components(self):
        """Initialize complex components"""
        self.controller = JointController()
        self.safety_manager = SafetyManager()

    def _command_callback(self, msg):
        """Handle incoming commands"""
        try:
            # Process command with error handling
            self.controller.process_command(msg.data)
        except Exception as e:
            self.get_logger().error(f'Command processing error: {e}')

    def _control_loop(self):
        """Main control loop"""
        try:
            # Update robot state
            self.controller.update()

            # Publish state
            self._publish_state()

            # Check safety
            self.safety_manager.check_safety()

        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')

    def _publish_state(self):
        """Publish current robot state"""
        # Implementation here
        pass
```

## 6.6 Security Considerations

ROS 2 provides built-in security features for safe robot operation:

```python
# Example of secure ROS 2 node configuration
import os

# Set security environment variables
os.environ['ROS_SECURITY_ENABLE'] = 'true'
os.environ['ROS_SECURITY_STRATEGY'] = 'Enforce'
os.environ['ROS_SECURITY_KEYSTORE'] = '/path/to/security/keystore'

# Secure node implementation
class SecureRobotNode(Node):
    def __init__(self):
        # Initialize with security context
        super().__init__('secure_robot', enable_security=True)

        # Secure communication with authentication
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            # Add security policies
        )

        self.secure_pub = self.create_publisher(String, 'secure_topic', qos_profile)
        self.secure_sub = self.create_subscription(
            String, 'secure_input', self._secure_callback, qos_profile
        )

    def _secure_callback(self, msg):
        """Handle secure message with authentication"""
        # Verify message authenticity
        if self._verify_message_authenticity(msg):
            # Process message
            self.get_logger().info(f'Secure message received: {msg.data}')
        else:
            self.get_logger().warn('Unauthenticated message received')

    def _verify_message_authenticity(self, msg):
        """Verify message authenticity"""
        # Implementation of message verification
        return True  # Simplified
```

## Summary

Advanced ROS 2 concepts include sophisticated launch file configurations, node composition, distributed system communication, advanced debugging techniques, and comprehensive visualization tools. Understanding these concepts enables the development of robust, scalable, and maintainable robotic systems.

## Next Steps

In the next section, we'll explore sensor integration and perception systems in detail, building on the advanced ROS 2 concepts learned here.

<div class="alert alert-info">
  <h5>Hardware Requirement</h5>
  <div><strong>Requirement:</strong> GPU</div>
  <div><strong>Minimum:</strong> Any</div>
  <div><strong>Recommended:</strong> Any</div>
  <div><strong>Purpose:</strong> Basic computational requirements for Week 6 advanced ROS 2 concepts</div>
</div>