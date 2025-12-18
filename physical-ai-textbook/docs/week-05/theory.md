---
title: "Week 5: ROS 2 Fundamentals"
week: 5
module: "Robotic Infrastructure"
difficulty: "intermediate"
prerequisites: ["python-basics", "linux-basics", "robotics-concepts"]
learning_objectives:
  - "Understand ROS 2 architecture"
  - "Implement basic nodes and topics"
  - "Use rclpy for Python development"
tags: ["ros2", "rclpy", "nodes", "topics", "services"]
hardware_requirements:
  - gpu: "Any"
  - ram: "8GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "90 minutes"
---

# Week 5: ROS 2 Fundamentals

## Learning Objectives
- Understand ROS 2 architecture
- Implement basic nodes and topics
- Use rclpy for Python development
- Practice basic ROS 2 development workflows

## 5.1 Introduction to ROS 2

Robot Operating System 2 (ROS 2) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

### Key Features of ROS 2
- **Distributed Computing**: Nodes can run on different machines
- **Language Independence**: Support for multiple programming languages
- **Real-time Support**: Deterministic behavior for time-critical applications
- **Security**: Built-in security features for safe robot operation
- **Middleware**: DDS (Data Distribution Service) for communication

### ROS 2 vs ROS 1
ROS 2 addresses several limitations of ROS 1:
- **Real-time Support**: ROS 2 provides real-time capabilities
- **Multi-machine Communication**: Improved distributed system support
- **Security**: Built-in security mechanisms
- **Middleware Flexibility**: Pluggable middleware architecture
- **Lifecycle Management**: Better node lifecycle management

## 5.2 ROS 2 Architecture

### Nodes
A node is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of ROS 2 applications.

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Minimal node created')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()

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

### Topics and Messages
Topics are named buses over which nodes exchange messages. Messages are data structures exchanged between nodes.

```python
from std_msgs.msg import String  # Message type

# Publisher example
publisher = node.create_publisher(String, 'topic_name', 10)

# Subscriber example
subscriber = node.create_subscription(
    String,
    'topic_name',
    callback_function,
    10
)
```

### Services
Services provide a request-response communication pattern between nodes.

```python
from example_interfaces.srv import AddTwoInts

# Service server
service = node.create_service(AddTwoInts, 'add_two_ints', callback_function)

# Service client
client = node.create_client(AddTwoInts, 'add_two_ints')
```

### Actions
Actions are used for long-running tasks with feedback and goal management.

```python
from example_interfaces.action import Fibonacci
import rclpy.action

# Action server
action_server = rclpy.action.ActionServer(
    node,
    Fibonacci,
    'fibonacci',
    execute_callback
)

# Action client
action_client = rclpy.action.ActionClient(node, Fibonacci, 'fibonacci')
```

## 5.3 rclpy: Python Client Library

rclpy is the Python client library for ROS 2. It provides the Python API for ROS 2.

### Creating a Publisher Node

```python
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

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Subscriber Node

```python
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

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5.4 Parameters and Configuration

ROS 2 nodes can use parameters to configure their behavior at runtime.

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters
        self.declare_parameter('my_parameter', 'default_value')
        self.declare_parameter('threshold', 0.5)

        # Get parameter values
        param_value = self.get_parameter('my_parameter').value
        threshold = self.get_parameter('threshold').value

        self.get_logger().info(f'Parameter value: {param_value}')
        self.get_logger().info(f'Threshold: {threshold}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()

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

## 5.5 Launch Files and System Composition

Launch files allow you to start multiple nodes with a single command.

### Python Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_py',
            executable='talker',
            name='publisher_node'
        ),
        Node(
            package='demo_nodes_py',
            executable='listener',
            name='subscriber_node'
        )
    ])
```

### Composition
Node composition allows multiple nodes to run in the same process for better performance.

```python
from rclpy.node import Node
from rclpy import executors
from example_nodes import MinimalPublisher, MinimalSubscriber

class ComposedNode(Node):
    def __init__(self):
        super().__init__('composed_node')
        self.publisher = MinimalPublisher()
        self.subscriber = MinimalSubscriber()

        # Create executor and add nodes
        self.executor = executors.MultiThreadedExecutor()
        self.executor.add_node(self.publisher)
        self.executor.add_node(self.subscriber)

def main(args=None):
    rclpy.init(args=args)
    composed_node = ComposedNode()

    try:
        composed_node.executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        composed_node.executor.shutdown()
        rclpy.shutdown()
```

## 5.6 Quality of Service (QoS) Settings

QoS settings control how messages are delivered and handled.

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Create QoS profile
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

# Use in publisher
publisher = self.create_publisher(String, 'topic', qos_profile)

# Use in subscriber
subscriber = self.create_subscription(
    String,
    'topic',
    callback,
    qos_profile
)
```

## 5.7 Working with Custom Messages

### Creating Custom Message Types

Create a file `msg/Num.msg`:
```
int64 num
```

### Using Custom Messages

```python
from my_package.msg import Num  # Custom message

class CustomMessageNode(Node):
    def __init__(self):
        super().__init__('custom_message_node')
        self.publisher = self.create_publisher(Num, 'custom_topic', 10)

    def publish_custom_message(self, number):
        msg = Num()
        msg.num = number
        self.publisher.publish(msg)
```

## 5.8 ROS 2 Tools and Commands

### Essential ROS 2 Commands

```bash
# List all nodes
ros2 node list

# List topics
ros2 topic list

# Echo a topic
ros2 topic echo /topic_name

# Call a service
ros2 service call /service_name service_type "{request_field: value}"

# List parameters
ros2 param list

# Get parameter value
ros2 param get /node_name parameter_name

# Set parameter value
ros2 param set /node_name parameter_name value
```

## 5.9 Best Practices for ROS 2 Development

1. **Node Design**: Keep nodes focused on single responsibilities
2. **Topic Naming**: Use descriptive, consistent naming conventions
3. **Error Handling**: Implement proper error handling and logging
4. **Resource Management**: Properly clean up resources when nodes are destroyed
5. **Testing**: Write unit tests for your nodes and components
6. **Documentation**: Document your custom messages, services, and nodes

## Summary

ROS 2 provides a robust framework for robot software development with improved architecture over ROS 1. Understanding nodes, topics, services, and other core concepts is essential for building distributed robotic systems. The rclpy library provides Python developers with powerful tools to create sophisticated robotic applications.

## Next Steps

In the next section, we'll explore advanced ROS 2 concepts including launch files, system composition, and distributed robotics systems.

<div class="alert alert-info">
  <h5>Hardware Requirement</h5>
  <div><strong>Requirement:</strong> GPU</div>
  <div><strong>Minimum:</strong> Any</div>
  <div><strong>Recommended:</strong> Any</div>
  <div><strong>Purpose:</strong> Basic computational requirements for Week 5 ROS 2 fundamentals</div>
</div>