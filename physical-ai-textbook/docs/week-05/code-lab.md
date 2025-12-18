---
title: "Week 5: ROS 2 Fundamentals Code Lab"
week: 5
module: "Robotic Infrastructure"
difficulty: "intermediate"
prerequisites: ["python-basics", "linux-basics", "robotics-concepts"]
learning_objectives:
  - "Create basic ROS 2 publisher/subscriber nodes"
  - "Implement simple ROS 2 services"
  - "Understand ROS 2 architecture concepts"
tags: ["ros2", "python", "nodes", "services", "rclpy"]
hardware_requirements:
  - gpu: "Any"
  - ram: "8GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "120 minutes"
---

# Week 5: ROS 2 Fundamentals Code Lab

## Learning Objectives
- Create basic ROS 2 publisher/subscriber nodes
- Implement simple ROS 2 services
- Understand ROS 2 architecture concepts
- Practice basic ROS 2 development workflows

## 5.1 Setting Up Your ROS 2 Environment

First, let's ensure your ROS 2 environment is properly set up:

```bash
# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Create a workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Install Python dependencies
pip3 install numpy matplotlib
```

## 5.2 Creating Your First Publisher Node

Let's create a basic publisher node that publishes messages to a topic:

```python
# publisher_member_function.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5.3 Creating Your First Subscriber Node

Now let's create a subscriber node that listens to the topic:

```python
# subscriber_member_function.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5.4 Creating a Service Server

Let's create a simple service that adds two integers:

```python
# service_member_function.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a}, b: {request.b}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5.5 Creating a Service Client

And a client to call the service:

```python
# client_member_function.py
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()
    minimal_client = MinimalClient()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        f'Result of add_two_ints: for {sys.argv[1]} + {sys.argv[2]} = {response.sum}')
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5.6 Running the Examples

To run these examples:

1. **Terminal 1** - Start the publisher:
```bash
python3 publisher_member_function.py
```

2. **Terminal 2** - Start the subscriber:
```bash
python3 subscriber_member_function.py
```

3. **Terminal 3** - Start the service server:
```bash
python3 service_member_function.py
```

4. **Terminal 4** - Call the service:
```bash
python3 client_member_function.py 1 2
```

## 5.7 Understanding ROS 2 Concepts

### Nodes
Nodes are independent processes that perform computation. Each node has a unique name within the ROS graph. Nodes communicate with each other through topics, services, and actions.

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')

        # Create publishers, subscribers, services, etc.
        self.publisher = self.create_publisher(String, 'my_topic', 10)
        self.subscription = self.create_subscription(String, 'other_topic', self.callback, 10)

        # Create a timer
        self.timer = self.create_timer(0.1, self.timer_callback)

    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from my node'
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

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

### Topics
Topics provide unidirectional communication using a publisher-subscriber pattern. Multiple publishers and subscribers can exist for the same topic.

```python
from std_msgs.msg import String, Int32, Float64
from sensor_msgs.msg import JointState, LaserScan, Image

class TopicNode(Node):
    def __init__(self):
        super().__init__('topic_node')

        # Different types of publishers
        self.string_pub = self.create_publisher(String, 'string_topic', 10)
        self.int_pub = self.create_publisher(Int32, 'int_topic', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Different types of subscribers
        self.string_sub = self.create_subscription(String, 'string_topic', self.string_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)

    def string_callback(self, msg):
        self.get_logger().info(f'String message: {msg.data}')

    def laser_callback(self, msg):
        self.get_logger().info(f'Laser scan has {len(msg.ranges)} ranges')
```

### Services
Services provide bidirectional communication using a request-response pattern. They are synchronous communication.

```python
from example_interfaces.srv import SetBool, Trigger
import rclpy

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')

        # Create different types of services
        self.bool_service = self.create_service(
            SetBool,
            'set_bool_service',
            self.set_bool_callback
        )

        self.trigger_service = self.create_service(
            Trigger,
            'trigger_service',
            self.trigger_callback
        )

    def set_bool_callback(self, request, response):
        response.success = True
        response.message = f'Set bool to {request.data}'
        return response

    def trigger_callback(self, request, response):
        response.success = True
        response.message = 'Triggered successfully'
        return response
```

### Parameters
Parameters allow nodes to be configured at runtime:

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'my_robot')
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('use_camera', True)

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.use_camera = self.get_parameter('use_camera').value

        self.get_logger().info(f'Robot name: {self.robot_name}')
        self.get_logger().info(f'Max velocity: {self.max_velocity}')
        self.get_logger().info(f'Use camera: {self.use_camera}')

        # Create parameter callback to handle dynamic reconfiguration
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_velocity' and param.type_ == Parameter.Type.PARAMETER_DOUBLE:
                self.max_velocity = param.value
                self.get_logger().info(f'Updated max velocity: {self.max_velocity}')
        return SetParametersResult(successful=True)
```

## 5.8 Quality of Service (QoS) Patterns

QoS settings control message delivery characteristics:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class QoSNode(Node):
    def __init__(self):
        super().__init__('qos_node')

        # Create different QoS profiles
        # Reliable communication - all messages delivered
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Best effort - no guarantee of delivery (for sensor data)
        best_effort_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publisher with reliable QoS
        self.reliable_pub = self.create_publisher(String, 'reliable_topic', reliable_qos)

        # Publisher with best effort QoS (for sensor data)
        self.sensor_pub = self.create_publisher(LaserScan, 'scan', best_effort_qos)

        # Subscriber with matching QoS
        self.sub = self.create_subscription(String, 'reliable_topic', self.callback, reliable_qos)

    def callback(self, msg):
        self.get_logger().info(f'Received with QoS: {msg.data}')
```

## 5.9 Creating a Complex Node with Multiple Components

Let's create a more complex node that combines publishers, subscribers, services, and parameters:

```python
#!/usr/bin/env python3
# complex_robot_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from example_interfaces.srv import SetBool
from rclpy.qos import QoSProfile
import time

class ComplexRobotNode(Node):
    def __init__(self):
        super().__init__('complex_robot_node')

        # Declare parameters
        self.declare_parameter('robot_id', 'robot_01')
        self.declare_parameter('control_frequency', 50)
        self.declare_parameter('enable_logging', True)

        # Get parameter values
        self.robot_id = self.get_parameter('robot_id').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.enable_logging = self.get_parameter('enable_logging').value

        # Publishers
        self.status_pub = self.create_publisher(String, 'robot_status', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_commands', 10)

        # Subscribers
        self.cmd_sub = self.create_subscription(String, 'command', self.command_callback, 10)
        self.emergency_sub = self.create_subscription(Bool, 'emergency_stop', self.emergency_callback, 10)

        # Services
        self.enable_service = self.create_service(SetBool, 'enable_robot', self.enable_callback)

        # Internal state
        self.is_enabled = True
        self.joint_positions = [0.0, 0.0, 0.0]  # Example joint positions

        # Timer for periodic updates
        self.timer = self.create_timer(1.0/self.control_frequency, self.control_loop)

        self.get_logger().info(f'Complex robot node {self.robot_id} initialized')

    def command_callback(self, msg):
        """Handle incoming commands"""
        command = msg.data.lower()

        if command == 'move_forward':
            self.joint_positions[0] += 0.1
            self.get_logger().info('Moving forward')
        elif command == 'move_backward':
            self.joint_positions[0] -= 0.1
            self.get_logger().info('Moving backward')
        elif command == 'home':
            self.joint_positions = [0.0, 0.0, 0.0]
            self.get_logger().info('Homing robot')

        # Publish joint commands
        self.publish_joint_commands()

    def emergency_callback(self, msg):
        """Handle emergency stop"""
        if msg.data:
            self.is_enabled = False
            self.get_logger().warn('Emergency stop activated!')
        else:
            self.is_enabled = True
            self.get_logger().info('Robot re-enabled')

    def enable_callback(self, request, response):
        """Handle enable/disable service"""
        self.is_enabled = request.data
        response.success = True
        response.message = f'Robot {"enabled" if self.is_enabled else "disabled"}'

        if self.is_enabled:
            self.get_logger().info('Robot enabled via service')
        else:
            self.get_logger().info('Robot disabled via service')

        return response

    def control_loop(self):
        """Main control loop"""
        if not self.is_enabled:
            return

        # Publish status
        status_msg = String()
        status_msg.data = f'Robot {self.robot_id} - Enabled: {self.is_enabled}, Joints: {self.joint_positions}'
        self.status_pub.publish(status_msg)

        # Publish joint commands
        self.publish_joint_commands()

    def publish_joint_commands(self):
        """Publish joint state commands"""
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['joint1', 'joint2', 'joint3']
        joint_msg.position = self.joint_positions
        self.joint_pub.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ComplexRobotNode()

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

## 5.10 Creating Launch Files

Let's create a launch file to start multiple nodes at once:

```python
# launch/robot_system_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    robot_namespace = LaunchConfiguration('robot_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='robot1',
            description='Robot namespace'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # Robot controller node
        Node(
            package='my_robot_package',
            executable='complex_robot_node',
            name='robot_controller',
            namespace=robot_namespace,
            parameters=[
                {'robot_id': 'my_robot'},
                {'control_frequency': 50},
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),

        # Diagnostic node
        Node(
            package='my_robot_package',
            executable='diagnostic_node',
            name='diagnostic_node',
            namespace=robot_namespace,
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        )
    ])
```

## 5.11 Testing Your Nodes

Create a simple test script to verify your nodes work correctly:

```python
#!/usr/bin/env python3
# test_robot_nodes.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import SetBool
import time

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')

        # Publisher to send commands
        self.cmd_publisher = self.create_publisher(String, 'command', 10)

        # Client for enable service
        self.enable_client = self.create_client(SetBool, 'enable_robot')

        # Wait for service
        while not self.enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for enable_robot service...')

        # Schedule test sequence
        self.timer = self.create_timer(2.0, self.run_test_sequence)
        self.test_step = 0

    def run_test_sequence(self):
        """Run a sequence of tests"""
        if self.test_step == 0:
            self.get_logger().info('Test 1: Enabling robot')
            self.enable_robot(True)
        elif self.test_step == 1:
            self.get_logger().info('Test 2: Sending move command')
            self.send_command('move_forward')
        elif self.test_step == 2:
            self.get_logger().info('Test 3: Sending home command')
            self.send_command('home')
        elif self.test_step == 3:
            self.get_logger().info('Test 4: Disabling robot')
            self.enable_robot(False)
        elif self.test_step == 4:
            self.get_logger().info('All tests completed')
            self.timer.cancel()

        self.test_step += 1

    def enable_robot(self, enable):
        """Call the enable service"""
        request = SetBool.Request()
        request.data = enable

        future = self.enable_client.call_async(request)
        future.add_done_callback(self.enable_response_callback)

    def enable_response_callback(self, future):
        """Handle enable service response"""
        try:
            response = future.result()
            self.get_logger().info(f'Enable service response: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def send_command(self, command):
        """Send a command to the robot"""
        msg = String()
        msg.data = command
        self.cmd_publisher.publish(msg)
        self.get_logger().info(f'Sent command: {command}')

def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()

    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5.12 Practical Exercise

Create a ROS 2 node that:
1. Publishes robot sensor data (joint states, IMU, etc.)
2. Subscribes to control commands
3. Provides services for robot configuration
4. Uses parameters for configuration
5. Implements proper error handling

```python
# Student exercise - Complete implementation
class StudentRobotNode(Node):
    """Student implementation of a robot node"""

    def __init__(self):
        """Initialize the robot node"""
        # TODO: Initialize node with proper name
        # TODO: Create publishers for sensor data
        # TODO: Create subscribers for commands
        # TODO: Create services for configuration
        # TODO: Declare and use parameters
        # TODO: Implement error handling
        pass

    def publish_sensor_data(self):
        """Publish sensor data"""
        # TODO: Complete implementation
        pass

    def handle_commands(self):
        """Handle incoming commands"""
        # TODO: Complete implementation
        pass

    def configure_robot(self):
        """Handle configuration requests"""
        # TODO: Complete implementation
        pass

print("Student Exercise: Implement a complete robot node")
print("Requirements:")
print("1. Publish sensor data (joint states, IMU, etc.)")
print("2. Subscribe to control commands")
print("3. Provide services for robot configuration")
print("4. Use parameters for configuration")
print("5. Implement proper error handling")
```

## Summary

In this lab, you've learned the fundamentals of ROS 2 by creating publisher/subscriber nodes, implementing services, and understanding the core concepts of the ROS 2 architecture. You've also learned about parameters, QoS settings, and how to structure more complex robotic applications.

<div class="alert alert-info">
  <h5>Hardware Requirement</h5>
  <div><strong>Requirement:</strong> GPU</div>
  <div><strong>Minimum:</strong> Any</div>
  <div><strong>Recommended:</strong> Any</div>
  <div><strong>Purpose:</strong> Basic computational requirements for Week 5 ROS 2 lab</div>
</div>