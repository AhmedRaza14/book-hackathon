---
title: "Week 10: Vision-Language Integration - Code Lab"
week: 10
module: "AI-Robot Integration"
difficulty: "advanced"
prerequisites: ["ros2-fundamentals", "sensor-integration", "ai-fundamentals", "nvidia-isaac"]
learning_objectives:
  - "Implement vision-language model integration"
  - "Create voice-to-action pipeline"
  - "Build multimodal perception system"
  - "Test natural language command interpretation"
tags: ["vision-language", "llm", "vlm", "whisper", "multimodal", "natural-language-processing", "robot-control"]
hardware_requirements:
  - gpu: "RTX 4070 or higher"
  - ram: "32GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "120 minutes"
---

# Week 10: Vision-Language Integration - Code Lab

## Learning Objectives
- Implement vision-language model integration
- Create voice-to-action pipeline
- Build multimodal perception system
- Test natural language command interpretation

## Prerequisites
- ROS 2 Humble Hawksbill installed
- Python 3.8+ with pip
- CUDA-compatible GPU with RTX 4070 or higher
- Microphone and camera for voice and vision input
- Basic understanding of PyTorch and Transformers

## Setup Environment

### Install Required Dependencies
```bash
# Create a new workspace
mkdir -p ~/vision_language_ws/src
cd ~/vision_language_ws

# Install Python dependencies
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip3 install transformers openai-whisper sentence-transformers
pip3 install opencv-python numpy matplotlib
pip3 install speechrecognition pyttsx3
```

### ROS 2 Package Structure
```bash
# Create ROS 2 packages
cd ~/vision_language_ws/src
ros2 pkg create --build-type ament_python vision_language_interfaces
ros2 pkg create --build-type ament_python vision_language_perception
ros2 pkg create --build-type ament_python vision_language_control
```

## Part 1: Vision Model Integration

### Task 1.1: Implement CLIP-based Object Recognition
Create a Python node that uses CLIP for zero-shot object recognition:

```python
#!/usr/bin/env python3
# vision_language_perception/vision_language_perception/clip_recognizer.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import torch
from transformers import CLIPProcessor, CLIPModel
import numpy as np
import cv2

class CLIPRecognizer(Node):
    def __init__(self):
        super().__init__('clip_recognizer')

        # Initialize CLIP model
        self.model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32")
        self.processor = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")

        # ROS 2 setup
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.result_pub = self.create_publisher(String, 'clip_recognition_result', 10)

        self.get_logger().info('CLIP Recognizer node initialized')

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Prepare image and text inputs for CLIP
            image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Define potential object categories
            text_inputs = [
                "a photo of a person", "a photo of a robot",
                "a photo of a cup", "a photo of a book",
                "a photo of a table", "a photo of a chair"
            ]

            # Process with CLIP
            inputs = self.processor(text=text_inputs, images=image, return_tensors="pt", padding=True)
            outputs = self.model(**inputs)
            logits_per_image = outputs.logits_per_image
            probs = logits_per_image.softmax(dim=1)

            # Get the most likely category
            best_match_idx = probs.argmax().item()
            best_match = text_inputs[best_match_idx]
            confidence = probs[0][best_match_idx].item()

            # Publish result
            result_msg = String()
            result_msg.data = f"{best_match} (confidence: {confidence:.2f})"
            self.result_pub.publish(result_msg)

            self.get_logger().info(f'Recognition result: {result_msg.data}')

        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = CLIPRecognizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Task 1.2: Set up the package configuration
Create the setup.py for the perception package:

```python
# vision_language_perception/setup.py
from setuptools import find_packages, setup

package_name = 'vision_language_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Vision-Language perception package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'clip_recognizer = vision_language_perception.clip_recognizer:main',
        ],
    },
)
```

## Part 2: Voice-to-Action Pipeline

### Task 2.1: Implement Voice Recognition Node
Create a node that processes voice commands using Whisper:

```python
#!/usr/bin/env python3
# vision_language_perception/vision_language_perception/voice_processor.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import speech_recognition as sr
import threading
import queue

class VoiceProcessor(Node):
    def __init__(self):
        super().__init__('voice_processor')

        # Initialize Whisper model
        self.model = whisper.load_model("base")

        # Setup speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # ROS 2 setup
        self.command_pub = self.create_publisher(String, 'voice_command', 10)
        self.result_pub = self.create_publisher(String, 'transcription_result', 10)

        # Setup for continuous listening
        self.listening_queue = queue.Queue()
        self.get_logger().info('Voice Processor node initialized')

        # Start listening thread
        self.listening_thread = threading.Thread(target=self.listen_continuously)
        self.listening_thread.daemon = True
        self.listening_thread.start()

    def listen_continuously(self):
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        self.get_logger().info("Listening for voice commands...")

        while rclpy.ok():
            try:
                with self.microphone as source:
                    audio = self.recognizer.listen(source, timeout=1.0, phrase_time_limit=5)

                # Save audio to temporary file for Whisper
                with open('/tmp/temp_audio.wav', 'wb') as f:
                    f.write(audio.get_wav_data())

                # Transcribe with Whisper
                result = self.model.transcribe('/tmp/temp_audio.wav')
                text = result['text'].strip()

                if text:
                    self.get_logger().info(f'Transcribed: {text}')

                    # Publish transcription
                    result_msg = String()
                    result_msg.data = text
                    self.result_pub.publish(result_msg)

                    # Publish command for processing
                    cmd_msg = String()
                    cmd_msg.data = text
                    self.command_pub.publish(cmd_msg)

            except sr.WaitTimeoutError:
                # This is normal, just continue listening
                continue
            except Exception as e:
                self.get_logger().error(f'Error in voice processing: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = VoiceProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 3: Natural Language Command Interpretation

### Task 3.1: Create Command Parser Node
Implement a node that parses natural language commands:

```python
#!/usr/bin/env python3
# vision_language_control/vision_language_control/command_parser.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_language_interfaces.msg import RobotCommand
import re

class CommandParser(Node):
    def __init__(self):
        super().__init__('command_parser')

        # ROS 2 setup
        self.command_sub = self.create_subscription(
            String, 'voice_command', self.command_callback, 10)
        self.robot_cmd_pub = self.create_publisher(
            RobotCommand, 'robot_command_parsed', 10)
        self.feedback_pub = self.create_publisher(String, 'command_feedback', 10)

        self.get_logger().info('Command Parser node initialized')

    def command_callback(self, msg):
        command_text = msg.data.lower()
        self.get_logger().info(f'Received command: {command_text}')

        # Parse the command
        parsed_command = self.parse_command(command_text)

        if parsed_command:
            # Publish parsed command
            robot_cmd_msg = RobotCommand()
            robot_cmd_msg.command_type = parsed_command['type']
            robot_cmd_msg.parameters = str(parsed_command['params'])
            robot_cmd_msg.target_object = parsed_command.get('target', '')
            self.robot_cmd_pub.publish(robot_cmd_msg)

            feedback_msg = String()
            feedback_msg.data = f"Command parsed: {parsed_command['type']} with params {parsed_command['params']}"
            self.feedback_pub.publish(feedback_msg)
        else:
            feedback_msg = String()
            feedback_msg.data = f"Could not parse command: {command_text}"
            self.feedback_pub.publish(feedback_msg)

    def parse_command(self, command):
        """Parse natural language command into robot action"""

        # Look for navigation commands
        nav_patterns = [
            (r'move to (.+)', 'NAVIGATE_TO'),
            (r'go to (.+)', 'NAVIGATE_TO'),
            (r'go (.+)', 'NAVIGATE_TO'),
            (r'approach (.+)', 'NAVIGATE_TO'),
        ]

        for pattern, cmd_type in nav_patterns:
            match = re.search(pattern, command)
            if match:
                return {
                    'type': cmd_type,
                    'params': {'location': match.group(1)},
                    'target': match.group(1)
                }

        # Look for manipulation commands
        manip_patterns = [
            (r'pick up (.+)', 'PICK_UP'),
            (r'grasp (.+)', 'PICK_UP'),
            (r'get (.+)', 'PICK_UP'),
            (r'lift (.+)', 'PICK_UP'),
            (r'put (.+) on (.+)', 'PLACE'),
            (r'place (.+) on (.+)', 'PLACE'),
            (r'move (.+) to (.+)', 'MOVE_OBJECT'),
        ]

        for pattern, cmd_type in manip_patterns:
            match = re.search(pattern, command)
            if match:
                if cmd_type == 'PLACE':
                    return {
                        'type': cmd_type,
                        'params': {'object': match.group(1), 'destination': match.group(2)},
                        'target': match.group(1)
                    }
                else:
                    return {
                        'type': cmd_type,
                        'params': {'object': match.group(1)},
                        'target': match.group(1)
                    }

        # Look for simple actions
        if 'stop' in command:
            return {'type': 'STOP', 'params': {}}
        elif 'help' in command:
            return {'type': 'HELP', 'params': {}}
        elif 'wait' in command:
            return {'type': 'WAIT', 'params': {}}

        return None  # Could not parse

def main(args=None):
    rclpy.init(args=args)
    node = CommandParser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Task 3.2: Create the interfaces package
Create the message definition for robot commands:

```xml
<!-- vision_language_interfaces/package.xml -->
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>vision_language_interfaces</name>
  <version>0.0.0</version>
  <description>Interfaces for vision-language integration</description>
  <maintainer email="your_email@example.com">your_name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>

  <depend>builtin_interfaces</depend>
  <depend>std_msgs</depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

Create the message definition:

```bash
# Create the msg directory and file
mkdir -p ~/vision_language_ws/src/vision_language_interfaces/msg
```

```# vision_language_interfaces/msg/RobotCommand.msg
string command_type
string parameters
string target_object
builtin_interfaces/Time timestamp
```

Update the setup.py for interfaces:

```python
# vision_language_interfaces/setup.py
from setuptools import find_packages, setup
from glob import glob

package_name = 'vision_language_interfaces'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Interfaces for vision-language integration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
```

Add to package.xml dependencies:

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

## Part 4: Integration and Testing

### Task 4.1: Create a launch file
Create a launch file to start all nodes:

```python
# vision_language_perception/launch/vision_language_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision_language_perception',
            executable='clip_recognizer',
            name='clip_recognizer',
            output='screen',
        ),
        Node(
            package='vision_language_perception',
            executable='voice_processor',
            name='voice_processor',
            output='screen',
        ),
        Node(
            package='vision_language_control',
            executable='command_parser',
            name='command_parser',
            output='screen',
        ),
    ])

```

### Task 4.2: Build and run the system
```bash
# Build the workspace
cd ~/vision_language_ws
colcon build --packages-select vision_language_interfaces vision_language_perception vision_language_control

# Source the workspace
source install/setup.bash

# Run the system
ros2 launch vision_language_perception vision_language_system.launch.py
```

## Part 5: Testing and Evaluation

### Task 5.1: Create a test script
Create a simple test script to evaluate the system:

```python
#!/usr/bin/env python3
# test_vision_language.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_language_interfaces.msg import RobotCommand
import time

class VisionLanguageTester(Node):
    def __init__(self):
        super().__init__('vision_language_tester')

        self.command_pub = self.create_publisher(String, 'voice_command', 10)
        self.result_sub = self.create_subscription(
            String, 'command_feedback', self.feedback_callback, 10)

        self.test_commands = [
            "pick up the red cup",
            "move to the table",
            "go to the kitchen",
            "place the book on the shelf"
        ]

        self.test_idx = 0
        self.timer = self.create_timer(5.0, self.run_test)

        self.get_logger().info('Vision-Language Tester initialized')

    def feedback_callback(self, msg):
        self.get_logger().info(f'Test feedback: {msg.data}')

    def run_test(self):
        if self.test_idx < len(self.test_commands):
            command = self.test_commands[self.test_idx]
            self.get_logger().info(f'Sending test command: {command}')

            cmd_msg = String()
            cmd_msg.data = command
            self.command_pub.publish(cmd_msg)

            self.test_idx += 1
        else:
            self.get_logger().info('All tests completed')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    tester = VisionLanguageTester()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercises

### Exercise 1: Extend Command Vocabulary
Modify the command parser to recognize additional commands:
- "turn left/right" for rotation
- "move forward/backward" for translation
- "look at [object]" for visual attention

### Exercise 2: Improve Object Recognition
Enhance the CLIP recognizer to:
- Recognize more object categories
- Provide bounding box information
- Track objects across frames

### Exercise 3: Add Voice Feedback
Implement a text-to-speech node that provides voice feedback:
- Confirm command recognition
- Report task completion
- Ask for clarification when needed

## Summary

In this lab, you implemented a complete vision-language integration system with:
1. CLIP-based object recognition
2. Whisper-based voice processing
3. Natural language command parsing
4. ROS 2 integration for real-time operation

The system demonstrates how modern AI models can be integrated with robotic platforms to enable natural human-robot interaction.

## Next Steps

- Integrate with a physical robot platform
- Add more sophisticated planning capabilities
- Implement multimodal grounding
- Test in real-world environments