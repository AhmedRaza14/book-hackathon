---
title: "Week 5: ROS 2 Fundamentals Simulation"
week: 5
module: "Robotic Infrastructure"
difficulty: "intermediate"
prerequisites: ["python-basics", "linux-basics", "robotics-concepts"]
learning_objectives:
  - "Simulate ROS 2 communication patterns"
  - "Implement distributed robotics systems"
  - "Use debugging and visualization tools"
tags: ["ros2", "simulation", "debugging", "visualization"]
hardware_requirements:
  - gpu: "Any"
  - ram: "8GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "120 minutes"
---

# Week 5: ROS 2 Fundamentals Simulation

## Learning Objectives
- Simulate ROS 2 communication patterns
- Implement distributed robotics systems
- Use debugging and visualization tools
- Practice system composition and launch files

## 5.1 Simulating ROS 2 Communication Patterns

### Publisher-Subscriber Pattern Simulation

Let's create a simulation that demonstrates the publisher-subscriber pattern with realistic timing and communication characteristics:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64, Int32
from sensor_msgs.msg import JointState, Imu
import time
import threading
import numpy as np

class CommunicationSimulator(Node):
    """Simulate realistic ROS 2 communication patterns"""

    def __init__(self):
        super().__init__('communication_simulator')

        # Publishers with different QoS settings
        self.status_pub = self.create_publisher(String, 'robot_status', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu_data', 10)

        # Subscribers
        self.cmd_sub = self.create_subscription(String, 'robot_commands', self.command_callback, 10)

        # Communication timing simulation
        self.last_publish_time = time.time()
        self.message_sequence = 0

        # Robot state simulation
        self.joint_positions = [0.0, 0.0, 0.0]
        self.is_moving = False

        # Create timers for different message rates
        self.status_timer = self.create_timer(1.0, self.publish_status)  # 1 Hz
        self.joint_timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz
        self.imu_timer = self.create_timer(0.05, self.publish_imu_data)  # 20 Hz

    def command_callback(self, msg):
        """Handle incoming commands with simulated processing delay"""
        command = msg.data.lower()

        # Simulate processing delay
        time.sleep(0.05)  # 50ms processing delay

        if command == 'move':
            self.is_moving = True
            self.get_logger().info('Robot started moving')
        elif command == 'stop':
            self.is_moving = False
            self.get_logger().info('Robot stopped')
        elif command.startswith('set_joint'):
            # Parse joint position command
            try:
                parts = command.split()
                joint_idx = int(parts[1])
                position = float(parts[2])
                if 0 <= joint_idx < len(self.joint_positions):
                    self.joint_positions[joint_idx] = position
                    self.get_logger().info(f'Set joint {joint_idx} to {position}')
            except (ValueError, IndexError):
                self.get_logger().warn(f'Invalid joint command: {command}')

    def publish_status(self):
        """Publish robot status with simulated data"""
        status_msg = String()
        status_msg.data = f'Robot operational - Moving: {self.is_moving}, Joints: {self.joint_positions}'
        self.status_pub.publish(status_msg)

        # Simulate network delay effects
        if np.random.random() < 0.01:  # 1% packet loss
            self.get_logger().warn('Simulated packet loss in status message')

    def publish_joint_states(self):
        """Publish joint states with realistic data"""
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['joint1', 'joint2', 'joint3']

        # Add simulated noise to positions
        noisy_positions = [pos + np.random.normal(0, 0.001) for pos in self.joint_positions]
        joint_msg.position = noisy_positions

        # Calculate velocities if moving
        if self.is_moving:
            velocities = [0.1, 0.05, 0.02]  # Simulated velocities
        else:
            velocities = [0.0, 0.0, 0.0]
        joint_msg.velocity = velocities

        self.joint_pub.publish(joint_msg)

    def publish_imu_data(self):
        """Publish IMU data with realistic values"""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()

        # Simulate gravity and some motion
        imu_msg.linear_acceleration.x = np.random.normal(0, 0.1)
        imu_msg.linear_acceleration.y = np.random.normal(0, 0.1)
        imu_msg.linear_acceleration.z = 9.81 + np.random.normal(0, 0.2)  # Gravity + noise

        # Simulate angular velocity
        imu_msg.angular_velocity.x = np.random.normal(0, 0.01)
        imu_msg.angular_velocity.y = np.random.normal(0, 0.01)
        imu_msg.angular_velocity.z = np.random.normal(0, 0.01)

        # Simulate orientation (simplified)
        imu_msg.orientation.w = 1.0  # No rotation initially

        self.imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    simulator = CommunicationSimulator()

    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        pass
    finally:
        simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Communication Simulation

Let's simulate service-based communication with realistic response times:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool, Trigger, AddTwoInts
import time
import random

class ServiceSimulator(Node):
    """Simulate realistic ROS 2 service communication"""

    def __init__(self):
        super().__init__('service_simulator')

        # Create different types of services
        self.move_service = self.create_service(
            SetBool,
            'move_robot',
            self.move_robot_callback
        )

        self.calibrate_service = self.create_service(
            Trigger,
            'calibrate_sensors',
            self.calibrate_callback
        )

        self.compute_service = self.create_service(
            AddTwoInts,
            'compute_trajectory',
            self.compute_trajectory_callback
        )

    def move_robot_callback(self, request, response):
        """Simulate robot movement service with realistic delays"""
        self.get_logger().info(f'Received move request: {request.data}')

        # Simulate processing time based on complexity
        if request.data:
            processing_time = random.uniform(0.2, 0.8)  # 200-800ms
        else:
            processing_time = random.uniform(0.1, 0.3)  # 100-300ms

        time.sleep(processing_time)

        # Simulate success/failure based on complexity
        success_probability = 0.95  # 95% success rate
        response.success = random.random() < success_probability

        if response.success:
            response.message = f'Move command {"completed" if request.data else "stopped"} in {processing_time:.2f}s'
        else:
            response.message = 'Move command failed due to obstacle'

        self.get_logger().info(f'Service response: {response.message}')
        return response

    def calibrate_callback(self, request, response):
        """Simulate sensor calibration service"""
        self.get_logger().info('Received calibration request')

        # Simulate long calibration process
        calibration_time = random.uniform(2.0, 5.0)  # 2-5 seconds
        time.sleep(calibration_time)

        # Simulate success with some probability
        success_probability = 0.9
        response.success = random.random() < success_probability

        if response.success:
            response.message = f'Calibration completed in {calibration_time:.2f}s'
        else:
            response.message = 'Calibration failed - sensors not responding'

        self.get_logger().info(f'Calibration response: {response.message}')
        return response

    def compute_trajectory_callback(self, request, response):
        """Simulate trajectory computation service"""
        self.get_logger().info(f'Received trajectory request: {request.a}, {request.b}')

        # Simulate computation time based on complexity
        complexity_factor = abs(request.a) + abs(request.b)
        computation_time = min(2.0, 0.1 + complexity_factor * 0.05)  # Max 2 seconds
        time.sleep(computation_time)

        # Simulate trajectory computation result
        response.sum = request.a + request.b  # Simple example

        # Add some "computation" to make it realistic
        if complexity_factor > 10:  # Complex trajectory
            response.sum += random.randint(-1, 1)  # Add some error for complex paths

        self.get_logger().info(f'Trajectory computed in {computation_time:.2f}s, result: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    service_sim = ServiceSimulator()

    try:
        rclpy.spin(service_sim)
    except KeyboardInterrupt:
        pass
    finally:
        service_sim.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5.2 Distributed Robotics Systems Simulation

### Multi-Robot Communication Simulation

Let's create a simulation of multiple robots communicating with each other:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import random
import time

class RobotNode(Node):
    """Individual robot node in a distributed system"""

    def __init__(self, robot_id, neighbors):
        super().__init__(f'robot_{robot_id}')

        self.robot_id = robot_id
        self.neighbors = neighbors
        self.pose = PoseStamped()
        self.pose.pose.position.x = random.uniform(-10, 10)
        self.pose.pose.position.y = random.uniform(-10, 10)

        # Publishers for this robot
        self.pose_pub = self.create_publisher(PoseStamped, f'robot_{robot_id}/pose', 10)
        self.status_pub = self.create_publisher(String, f'robot_{robot_id}/status', 10)

        # Subscribers for neighbor robots
        self.neighbor_subscribers = {}
        for neighbor_id in neighbors:
            sub = self.create_subscription(
                PoseStamped,
                f'robot_{neighbor_id}/pose',
                lambda msg, nid=neighbor_id: self.receive_neighbor_pose(msg, nid),
                10
            )
            self.neighbor_subscribers[neighbor_id] = sub

        # Timer for periodic updates
        self.update_timer = self.create_timer(0.5, self.update_robot)

        self.get_logger().info(f'Robot {robot_id} initialized with neighbors: {neighbors}')

    def receive_neighbor_pose(self, msg, neighbor_id):
        """Receive pose from neighbor robot"""
        self.get_logger().info(f'Robot {self.robot_id} received pose from robot {neighbor_id}')

    def update_robot(self):
        """Update robot state and publish information"""
        # Update robot position (simple movement simulation)
        self.pose.pose.position.x += random.uniform(-0.1, 0.1)
        self.pose.pose.position.y += random.uniform(-0.1, 0.1)

        # Publish pose
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.pose.header.frame_id = 'map'
        self.pose_pub.publish(self.pose)

        # Publish status
        status_msg = String()
        status_msg.data = f'Robot {self.robot_id} at ({self.pose.pose.position.x:.2f}, {self.pose.pose.position.y:.2f})'
        self.status_pub.publish(status_msg)

class MultiRobotSystem:
    """Manage multiple robot nodes"""

    def __init__(self):
        rclpy.init()
        self.robots = []

        # Define robot network topology
        robot_configs = [
            {'id': 1, 'neighbors': [2, 3]},
            {'id': 2, 'neighbors': [1, 3, 4]},
            {'id': 3, 'neighbors': [1, 2, 4]},
            {'id': 4, 'neighbors': [2, 3]}
        ]

        # Create robot nodes
        for config in robot_configs:
            robot = RobotNode(config['id'], config['neighbors'])
            self.robots.append(robot)

    def run(self):
        """Run the multi-robot system"""
        executors = []
        for robot in self.robots:
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(robot)
            executors.append(executor)

        try:
            # Spin all executors
            spin_futures = [executor.spin_until_future_complete(rclpy.task.Future()) for executor in executors]
        except KeyboardInterrupt:
            pass
        finally:
            for robot in self.robots:
                robot.destroy_node()
            rclpy.shutdown()

def main():
    system = MultiRobotSystem()
    system.run()

if __name__ == '__main__':
    main()
```

## 5.3 Debugging Tools Simulation

### ROS 2 Diagnostic Node

Let's create a diagnostic node that monitors system health:

```python
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_msgs.msg import String
import psutil
import time

class DiagnosticNode(Node):
    """Monitor and report system diagnostics"""

    def __init__(self):
        super().__init__('diagnostic_node')

        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.status_timer = self.create_timer(1.0, self.publish_diagnostics)

    def publish_diagnostics(self):
        """Publish system diagnostic information"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # CPU diagnostic
        cpu_status = DiagnosticStatus()
        cpu_status.name = 'CPU Usage'
        cpu_percent = psutil.cpu_percent()
        cpu_status.level = DiagnosticStatus.OK if cpu_percent < 80 else DiagnosticStatus.WARN
        cpu_status.message = f'CPU usage: {cpu_percent}%'
        cpu_status.hardware_id = 'cpu'
        cpu_status.values = [
            {'key': 'usage_percent', 'value': str(cpu_percent)},
            {'key': 'status', 'value': 'normal' if cpu_percent < 80 else 'high'}
        ]
        diag_array.status.append(cpu_status)

        # Memory diagnostic
        mem_status = DiagnosticStatus()
        mem_status.name = 'Memory Usage'
        memory = psutil.virtual_memory()
        mem_percent = memory.percent
        mem_status.level = DiagnosticStatus.OK if mem_percent < 80 else DiagnosticStatus.WARN
        mem_status.message = f'Memory usage: {mem_percent}%'
        mem_status.hardware_id = 'memory'
        mem_status.values = [
            {'key': 'usage_percent', 'value': str(mem_percent)},
            {'key': 'available_mb', 'value': str(memory.available // (1024*1024))}
        ]
        diag_array.status.append(mem_status)

        # ROS communication diagnostic
        ros_status = DiagnosticStatus()
        ros_status.name = 'ROS Communication'
        ros_status.level = DiagnosticStatus.OK
        ros_status.message = 'ROS nodes communicating normally'
        ros_status.hardware_id = 'ros'
        ros_status.values = [
            {'key': 'node_count', 'value': str(len(self.get_node_names()))},
            {'key': 'time_since_start', 'value': str(time.time())}
        ]
        diag_array.status.append(ros_status)

        self.diag_pub.publish(diag_array)

def main(args=None):
    rclpy.init(args=args)
    diag_node = DiagnosticNode()

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

## 5.4 Visualization Tools Simulation

### RViz Integration Simulation

Let's create a node that publishes data in formats suitable for RViz visualization:

```python
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import ColorRGBA
import math

class VisualizationNode(Node):
    """Publish visualization data for RViz"""

    def __init__(self):
        super().__init__('visualization_node')

        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'robot_pose', 10)

        self.vis_timer = self.create_timer(0.1, self.publish_visualization)
        self.pose_timer = self.create_timer(0.05, self.publish_pose)

        self.time_counter = 0.0
        self.robot_path = []

    def publish_pose(self):
        """Publish robot pose"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        # Create circular motion for visualization
        radius = 3.0
        x = radius * math.cos(self.time_counter * 0.5)
        y = radius * math.sin(self.time_counter * 0.5)
        z = 0.0

        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z

        # Add to path
        self.robot_path.append((x, y, z))
        if len(self.robot_path) > 100:  # Keep only last 100 points
            self.robot_path.pop(0)

        self.pose_pub.publish(pose_msg)

    def publish_visualization(self):
        """Publish visualization markers"""
        marker_array = MarkerArray()

        # Robot marker
        robot_marker = Marker()
        robot_marker.header.frame_id = "map"
        robot_marker.header.stamp = self.get_clock().now().to_msg()
        robot_marker.ns = "robot"
        robot_marker.id = 0
        robot_marker.type = Marker.SPHERE
        robot_marker.action = Marker.ADD
        robot_marker.pose.position.x = self.robot_path[-1][0] if self.robot_path else 0
        robot_marker.pose.position.y = self.robot_path[-1][1] if self.robot_path else 0
        robot_marker.pose.position.z = self.robot_path[-1][2] if self.robot_path else 0
        robot_marker.pose.orientation.w = 1.0
        robot_marker.scale.x = 0.3
        robot_marker.scale.y = 0.3
        robot_marker.scale.z = 0.3
        robot_marker.color.r = 1.0
        robot_marker.color.g = 0.0
        robot_marker.color.b = 0.0
        robot_marker.color.a = 1.0
        marker_array.markers.append(robot_marker)

        # Path marker
        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = "path"
        path_marker.id = 1
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.pose.orientation.w = 1.0
        path_marker.scale.x = 0.05
        path_marker.color.r = 0.0
        path_marker.color.g = 1.0
        path_marker.color.b = 0.0
        path_marker.color.a = 0.8

        for x, y, z in self.robot_path:
            point = Point()
            point.x = x
            point.y = y
            point.z = z
            path_marker.points.append(point)

        marker_array.markers.append(path_marker)

        # Target marker
        target_marker = Marker()
        target_marker.header.frame_id = "map"
        target_marker.header.stamp = self.get_clock().now().to_msg()
        target_marker.ns = "target"
        target_marker.id = 2
        target_marker.type = Marker.CYLINDER
        target_marker.action = Marker.ADD
        target_marker.pose.position.x = 2.0
        target_marker.pose.position.y = 2.0
        target_marker.pose.position.z = 0.0
        target_marker.pose.orientation.w = 1.0
        target_marker.scale.x = 0.5
        target_marker.scale.y = 0.5
        target_marker.scale.z = 0.2
        target_marker.color.r = 0.0
        target_marker.color.g = 0.0
        target_marker.color.b = 1.0
        target_marker.color.a = 0.8
        marker_array.markers.append(target_marker)

        self.marker_pub.publish(marker_array)
        self.time_counter += 0.05

def main(args=None):
    rclpy.init(args=args)
    vis_node = VisualizationNode()

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

## 5.5 Launch File Simulation

Let's create a comprehensive launch file that demonstrates system composition:

```python
# launch/complete_robot_system_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessIO
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_namespace = LaunchConfiguration('robot_namespace', default='robot1')
    enable_diagnostics = LaunchConfiguration('enable_diagnostics', default='true')
    enable_visualization = LaunchConfiguration('enable_visualization', default='true')

    # Robot controller node
    robot_controller = Node(
        package='my_robot_package',
        executable='robot_controller',
        name='robot_controller',
        namespace=robot_namespace,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_id': robot_namespace},
            {'control_frequency': 50}
        ],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    # Sensor processor node
    sensor_processor = Node(
        package='my_robot_package',
        executable='sensor_processor',
        name='sensor_processor',
        namespace=robot_namespace,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'sensor_timeout': 1.0}
        ],
        output='screen'
    )

    # Diagnostic node (conditional)
    diagnostic_node = Node(
        package='my_robot_package',
        executable='diagnostic_node',
        name='diagnostic_node',
        namespace=robot_namespace,
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        condition=IfCondition(enable_diagnostics)
    )

    # Visualization node (conditional)
    visualization_node = Node(
        package='my_robot_package',
        executable='visualization_node',
        name='visualization_node',
        namespace=robot_namespace,
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        condition=IfCondition(enable_visualization)
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    ))

    ld.add_action(DeclareLaunchArgument(
        'robot_namespace',
        default_value='robot1',
        description='Robot namespace'
    ))

    ld.add_action(DeclareLaunchArgument(
        'enable_diagnostics',
        default_value='true',
        description='Enable diagnostic node'
    ))

    ld.add_action(DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable visualization node'
    ))

    # Add nodes
    ld.add_action(robot_controller)
    ld.add_action(sensor_processor)
    ld.add_action(diagnostic_node)
    ld.add_action(visualization_node)

    # Add event handlers for logging
    ld.add_action(RegisterEventHandler(
        OnProcessStart(
            target_action=robot_controller,
            on_start=lambda event, context: print(f"Robot controller started for {robot_namespace.perform(context)}")
        )
    ))

    return ld
```

## 5.6 Performance Monitoring Simulation

Let's create a performance monitoring tool:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32
import time
import collections

class PerformanceMonitor(Node):
    """Monitor ROS 2 system performance"""

    def __init__(self):
        super().__init__('performance_monitor')

        self.message_count_pub = self.create_publisher(Int32, 'perf_msg_count', 10)
        self.latency_pub = self.create_publisher(Float64, 'perf_latency', 10)
        self.cpu_usage_pub = self.create_publisher(Float64, 'perf_cpu_usage', 10)

        # Performance tracking
        self.message_count = 0
        self.latency_samples = collections.deque(maxlen=100)
        self.cpu_samples = collections.deque(maxlen=100)

        # Timer for performance updates
        self.perf_timer = self.create_timer(1.0, self.update_performance_metrics)

    def update_performance_metrics(self):
        """Update and publish performance metrics"""
        # Publish message count
        count_msg = Int32()
        count_msg.data = self.message_count
        self.message_count_pub.publish(count_msg)

        # Calculate and publish average latency
        if self.latency_samples:
            avg_latency = sum(self.latency_samples) / len(self.latency_samples)
            latency_msg = Float64()
            latency_msg.data = avg_latency
            self.latency_pub.publish(latency_msg)

        # Calculate and publish CPU usage
        # This is a simplified simulation - in real systems you'd use psutil
        import random
        cpu_usage = random.uniform(10, 80)  # Simulated CPU usage
        cpu_msg = Float64()
        cpu_msg.data = cpu_usage
        self.cpu_usage_pub.publish(cpu_msg)

        # Reset message counter
        self.message_count = 0

    def record_message(self):
        """Record a message for performance tracking"""
        self.message_count += 1

    def record_latency(self, latency):
        """Record latency measurement"""
        self.latency_samples.append(latency)

def main(args=None):
    rclpy.init(args=args)
    perf_monitor = PerformanceMonitor()

    try:
        rclpy.spin(perf_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        perf_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5.7 Practical Exercise

Create a complete distributed robotics simulation that:
1. Implements multiple robot nodes with communication
2. Includes diagnostic and performance monitoring
3. Provides visualization capabilities
4. Uses launch files for system composition
5. Implements proper error handling and recovery

```python
# Student exercise - Complete implementation
class CompleteRobotSystem:
    """Student implementation of a complete robot system"""

    def __init__(self):
        """Initialize the complete robot system"""
        # TODO: Create multiple robot nodes
        # TODO: Implement communication between robots
        # TODO: Add diagnostic capabilities
        # TODO: Add performance monitoring
        # TODO: Add visualization support
        # TODO: Create launch files
        # TODO: Implement error handling
        pass

    def create_robot_nodes(self):
        """Create multiple robot nodes"""
        # TODO: Complete implementation
        pass

    def implement_communication(self):
        """Implement robot-to-robot communication"""
        # TODO: Complete implementation
        pass

    def add_diagnostics(self):
        """Add diagnostic capabilities"""
        # TODO: Complete implementation
        pass

    def add_visualization(self):
        """Add visualization support"""
        # TODO: Complete implementation
        pass

    def create_launch_files(self):
        """Create launch files for system composition"""
        # TODO: Complete implementation
        pass

print("Student Exercise: Implement a complete robot system")
print("Requirements:")
print("1. Multiple robot nodes with communication")
print("2. Diagnostic and performance monitoring")
print("3. Visualization capabilities")
print("4. Launch files for system composition")
print("5. Error handling and recovery mechanisms")
```

## 5.8 Best Practices for ROS 2 Simulation

### Resource Management
- Use appropriate QoS settings for different data types
- Implement proper cleanup when nodes are destroyed
- Monitor system resources to prevent overload

### Communication Patterns
- Use latching for static data that new subscribers need
- Use appropriate message rates to avoid network congestion
- Implement timeouts for service calls and action clients

### Error Handling
- Implement retry mechanisms for failed communications
- Use diagnostics to monitor system health
- Gracefully handle node failures and restarts

## Summary

ROS 2 simulation provides powerful tools for developing and testing robotic systems before deployment. Understanding communication patterns, distributed systems, debugging tools, and visualization capabilities is essential for building robust robotic applications. Proper use of launch files and system composition enables scalable and maintainable robotic systems.

<div class="alert alert-info">
  <h5>Hardware Requirement</h5>
  <div><strong>Requirement:</strong> GPU</div>
  <div><strong>Minimum:</strong> Any</div>
  <div><strong>Recommended:</strong> Any</div>
  <div><strong>Purpose:</strong> Basic computational requirements for Week 5 ROS 2 simulation</div>
</div>