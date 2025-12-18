---
title: "Week 1: ROS 2 Python Code Lab"
week: 1
module: "Foundations of Physical AI"
difficulty: "beginner"
prerequisites: ["python-basics", "ros2-basics"]
learning_objectives:
  - "Create basic ROS 2 publisher/subscriber nodes"
  - "Implement simple ROS 2 services"
  - "Understand ROS 2 architecture concepts"
tags: ["ros2", "python", "nodes", "services"]
hardware_requirements:
  - gpu: "Any"
  - ram: "8GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "120 minutes"
---

# Week 1: ROS 2 Python Code Lab

## Learning Objectives
- Create basic ROS 2 publisher/subscriber nodes
- Implement simple ROS 2 services
- Understand ROS 2 architecture concepts
- Practice basic ROS 2 development workflows

## 2.1 Setting Up Your ROS 2 Environment

First, ensure you have ROS 2 Humble Hawksbill installed on Ubuntu 22.04 LTS. If you haven't installed ROS 2 yet, follow the official installation guide.

### Creating a Workspace

```bash
# Create a workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

## 2.2 Creating a Simple Publisher Node

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

## 2.3 Creating a Simple Subscriber Node

Now, let's create a subscriber node that listens to the topic:

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

## 2.4 Creating a Service Server

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

## 2.5 Creating a Service Client

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

## 2.6 Running the Examples

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

## 2.7 Understanding ROS 2 Concepts

### Nodes
- Independent processes that perform computation
- Each node has a unique name within the ROS graph
- Nodes communicate with each other through topics, services, and actions

### Topics
- Unidirectional communication
- Publisher-subscriber pattern
- Multiple publishers and subscribers can exist for the same topic

### Services
- Bidirectional communication
- Request-response pattern
- Synchronous communication

### Parameters
- Configuration values that can be set at runtime
- Each node has its own parameter server

## 2.8 Practical Exercise

Create a ROS 2 node that:
1. Publishes the current time to a topic every second
2. Subscribes to a topic that receives string messages
3. Prints the received messages to the console
4. Implements a service that returns the length of a string

```python
# Student code goes here
```

## Summary

In this lab, you've learned the basics of ROS 2 programming with Python. You've created publisher and subscriber nodes, implemented services, and practiced the fundamental ROS 2 communication patterns.

<div class="alert alert-info">
  <h5>Hardware Requirement</h5>
  <div><strong>Requirement:</strong> GPU</div>
  <div><strong>Minimum:</strong> Any</div>
  <div><strong>Recommended:</strong> Any</div>
  <div><strong>Purpose:</strong> Basic computational requirements for Week 1 ROS 2 lab</div>
</div>