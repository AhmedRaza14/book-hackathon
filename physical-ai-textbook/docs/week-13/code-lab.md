---
title: "Week 13: Human-Robot Interaction - Code Lab"
week: 13
module: "Applications and Projects"
difficulty: "intermediate"
prerequisites: ["advanced-ai", "cognitive-planning", "vision-language", "safety-systems"]
learning_objectives:
  - "Implement social robotics interaction systems"
  - "Create natural language interfaces"
  - "Design safety protocols for HRI"
  - "Build human-robot collaboration frameworks"
tags: ["human-robot-interaction", "social-robotics", "natural-language", "ethics", "safety", "collaboration"]
hardware_requirements:
  - gpu: "RTX 4070 or higher"
  - ram: "32GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "120 minutes"
---

# Week 13: Human-Robot Interaction - Code Lab

## Learning Objectives
- Implement social robotics interaction systems
- Create natural language interfaces
- Design safety protocols for HRI
- Build human-robot collaboration frameworks

## Prerequisites
- ROS 2 Humble Hawksbill installed
- Python 3.8+ with pip
- Microphone and camera for voice and vision input
- Basic understanding of speech recognition and NLP
- Completed Week 12 materials

## Setup Environment

### Install Required Dependencies
```bash
# Create a new workspace
mkdir -p ~/hri_ws/src
cd ~/hri_ws

# Install Python dependencies for HRI
pip3 install speechrecognition pyttsx3
pip3 install transformers torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip3 install numpy scipy matplotlib
pip3 install opencv-python
pip3 install scikit-learn
pip3 install nltk spacy  # For NLP processing
python3 -m spacy download en_core_web_sm  # Download English model
```

### ROS 2 Package Structure
```bash
# Create ROS 2 packages
cd ~/hri_ws/src
ros2 pkg create --build-type ament_python hri_interfaces
ros2 pkg create --build-type ament_python social_interaction
ros2 pkg create --build-type ament_python natural_language_interface
ros2 pkg create --build-type ament_python safety_manager
```

## Part 1: Social Interaction System

### Task 1.1: Create a Social Behavior Manager
Implement a system that manages social behaviors for the robot:

```python
#!/usr/bin/env python3
# social_interaction/social_interaction/social_behavior_manager.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Pose
from hri_interfaces.msg import SocialInteraction, RobotState
import cv2
import numpy as np
from cv_bridge import CvBridge
import time

class SocialBehaviorManager(Node):
    def __init__(self):
        super().__init__('social_behavior_manager')

        # Publishers and subscribers
        self.interaction_pub = self.create_publisher(SocialInteraction, 'social_interaction', 10)
        self.robot_state_pub = self.create_publisher(RobotState, 'robot_state', 10)
        self.face_sub = self.create_subscription(
            Image, 'camera/image_raw', self.face_callback, 10)
        self.gesture_sub = self.create_subscription(
            String, 'gesture_detected', self.gesture_callback, 10)
        self.proximity_sub = self.create_subscription(
            Point, 'human_proximity', self.proximity_callback, 10)

        # Internal state
        self.bridge = CvBridge()
        self.current_interaction = None
        self.human_proximity = None
        self.last_gesture = None
        self.robot_state = RobotState()
        self.robot_state.is_interacting = False
        self.robot_state.attention_target = Point()
        self.robot_state.emotional_state = "neutral"

        # Social parameters
        self.social_distance_threshold = 2.0  # meters
        self.interaction_timeout = 30.0  # seconds

        # Timer for behavior updates
        self.behavior_timer = self.create_timer(1.0, self.update_behavior)

        self.get_logger().info('Social Behavior Manager initialized')

    def face_callback(self, msg):
        """Process face detection from camera"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Simple face detection using OpenCV (in practice, use a more sophisticated detector)
            # For this example, we'll just detect if there's a person in the frame
            # based on skin color detection
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_skin = np.array([0, 20, 70], dtype=np.uint8)
            upper_skin = np.array([20, 255, 255], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower_skin, upper_skin)

            # Check if there's a significant amount of skin-colored pixels
            if cv2.countNonZero(mask) > 1000:  # Arbitrary threshold
                self.get_logger().info('Human detected in camera view')
                self.initiate_social_interaction()
        except Exception as e:
            self.get_logger().error(f'Error in face callback: {str(e)}')

    def gesture_callback(self, msg):
        """Process gesture recognition"""
        self.last_gesture = msg.data
        self.get_logger().info(f'Gesture detected: {msg.data}')

        if msg.data == 'wave':
            self.respond_to_gesture('wave')
        elif msg.data == 'point':
            self.respond_to_gesture('point')

    def proximity_callback(self, msg):
        """Process human proximity information"""
        self.human_proximity = msg
        distance = np.sqrt(msg.x**2 + msg.y**2 + msg.z**2)

        if distance < self.social_distance_threshold:
            self.get_logger().info(f'Human in social distance: {distance:.2f}m')
            if not self.robot_state.is_interacting:
                self.initiate_social_interaction()
        else:
            if self.robot_state.is_interacting and distance > self.social_distance_threshold * 1.5:
                self.end_interaction()

    def initiate_social_interaction(self):
        """Start a social interaction with the human"""
        if not self.robot_state.is_interacting:
            self.get_logger().info('Initiating social interaction')

            interaction_msg = SocialInteraction()
            interaction_msg.type = 'greeting'
            interaction_msg.content = 'Hello! How can I help you today?'
            interaction_msg.timestamp = self.get_clock().now().to_msg()
            interaction_msg.priority = 1

            self.interaction_pub.publish(interaction_msg)

            self.robot_state.is_interacting = True
            self.robot_state.start_time = self.get_clock().now().seconds_nanoseconds()[0]
            self.robot_state.emotional_state = 'friendly'

            self.robot_state_pub.publish(self.robot_state)

    def respond_to_gesture(self, gesture):
        """Respond to a detected gesture"""
        if self.robot_state.is_interacting:
            response_msg = SocialInteraction()

            if gesture == 'wave':
                response_msg.type = 'acknowledgment'
                response_msg.content = 'Hello! Nice to meet you!'
            elif gesture == 'point':
                response_msg.type = 'acknowledgment'
                response_msg.content = 'I see what you mean!'

            response_msg.timestamp = self.get_clock().now().to_msg()
            response_msg.priority = 2

            self.interaction_pub.publish(response_msg)

    def end_interaction(self):
        """End the current interaction"""
        if self.robot_state.is_interacting:
            self.get_logger().info('Ending social interaction')

            interaction_msg = SocialInteraction()
            interaction_msg.type = 'farewell'
            interaction_msg.content = 'Thank you for interacting with me!'
            interaction_msg.timestamp = self.get_clock().now().to_msg()
            interaction_msg.priority = 1

            self.interaction_pub.publish(interaction_msg)

            self.robot_state.is_interacting = False
            self.robot_state.emotional_state = 'neutral'
            self.robot_state_pub.publish(self.robot_state)

    def update_behavior(self):
        """Update robot behavior based on current state"""
        if self.robot_state.is_interacting:
            # Check if interaction has timed out
            current_time = self.get_clock().now().seconds_nanoseconds()[0]
            if current_time - self.robot_state.start_time > self.interaction_timeout:
                self.get_logger().info('Interaction timeout, ending interaction')
                self.end_interaction()

def main(args=None):
    rclpy.init(args=args)
    node = SocialBehaviorManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 2: Natural Language Interface

### Task 2.1: Create a Natural Language Processing Node
Implement a system that processes natural language input and generates responses:

```python
#!/usr/bin/env python3
# natural_language_interface/natural_language_interface/nlp_processor.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from hri_interfaces.msg import SocialInteraction
import speech_recognition as sr
import pyttsx3
import nltk
import spacy
from transformers import pipeline
import threading
import queue
import re

class NLPProcessor(Node):
    def __init__(self):
        super().__init__('nlp_processor')

        # Publishers and subscribers
        self.speech_sub = self.create_subscription(
            String, 'voice_input', self.speech_callback, 10)
        self.interaction_pub = self.create_publisher(SocialInteraction, 'social_interaction', 10)
        self.response_pub = self.create_publisher(String, 'text_to_speech', 10)

        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Initialize text-to-speech
        self.tts_engine = pyttsx3.init()
        self.tts_engine.setProperty('rate', 150)  # Speed of speech
        self.tts_engine.setProperty('volume', 0.9)  # Volume level

        # Initialize NLP models
        try:
            self.nlp = spacy.load("en_core_web_sm")
        except OSError:
            self.get_logger().error("spaCy model not found. Run: python -m spacy download en_core_web_sm")
            self.nlp = None

        # Initialize transformer for intent classification (simplified)
        # In practice, you'd train a custom model or use a pre-trained one
        self.intent_classifier = None  # Using rule-based approach for this example

        # Internal state
        self.conversation_history = []
        self.user_context = {}

        # Start listening thread
        self.listening_thread = threading.Thread(target=self.listen_continuously, daemon=True)
        self.listening_thread.start()

        self.get_logger().info('NLP Processor initialized')

    def listen_continuously(self):
        """Continuously listen for voice input"""
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        while rclpy.ok():
            try:
                with self.microphone as source:
                    audio = self.recognizer.listen(source, timeout=1.0, phrase_time_limit=5)

                # Transcribe speech to text
                text = self.recognizer.recognize_google(audio)
                self.get_logger().info(f'Speech recognized: {text}')

                # Process the text
                self.process_text(text)

            except sr.WaitTimeoutError:
                # This is normal, just continue listening
                continue
            except sr.UnknownValueError:
                self.get_logger().warn('Could not understand audio')
            except sr.RequestError as e:
                self.get_logger().error(f'Could not request results from speech recognition service; {e}')
            except Exception as e:
                self.get_logger().error(f'Error in speech recognition: {str(e)}')

    def speech_callback(self, msg):
        """Process text input (alternative to speech recognition)"""
        self.process_text(msg.data)

    def process_text(self, text):
        """Process natural language text and generate response"""
        # Add to conversation history
        self.conversation_history.append({'role': 'user', 'text': text, 'timestamp': self.get_clock().now().seconds_nanoseconds()[0]})

        # Extract intent and entities
        intent, entities = self.extract_intent_and_entities(text)

        # Generate response based on intent
        response = self.generate_response(intent, entities, text)

        # Add to conversation history
        self.conversation_history.append({'role': 'assistant', 'text': response, 'timestamp': self.get_clock().now().seconds_nanoseconds()[0]})

        # Publish response
        response_msg = SocialInteraction()
        response_msg.type = 'response'
        response_msg.content = response
        response_msg.timestamp = self.get_clock().now().to_msg()
        response_msg.priority = 2

        self.interaction_pub.publish(response_msg)

        # Publish for text-to-speech
        tts_msg = String()
        tts_msg.data = response
        self.response_pub.publish(tts_msg)

        # Speak the response
        self.speak(response)

    def extract_intent_and_entities(self, text):
        """Extract intent and entities from text"""
        # Convert to lowercase for easier processing
        text_lower = text.lower()

        # Define simple intent patterns
        intent_patterns = {
            'greeting': [
                r'hello', r'hi', r'hey', r'good morning', r'good afternoon', r'good evening'
            ],
            'farewell': [
                r'goodbye', r'bye', r'see you', r'thank you', r'thanks'
            ],
            'question': [
                r'what', r'how', r'where', r'when', r'why', r'who', r'can you', r'could you'
            ],
            'command': [
                r'please', r'can you', r'could you', r'would you', r'go to', r'move to', r'pick up', r'bring me'
            ],
            'help': [
                r'help', r'assist', r'need help', r'what can you do'
            ],
            'information': [
                r'tell me', r'information', r'about', r'know', r'describe'
            ]
        }

        # Find the most likely intent
        detected_intent = 'unknown'
        for intent, patterns in intent_patterns.items():
            for pattern in patterns:
                if re.search(pattern, text_lower):
                    detected_intent = intent
                    break
            if detected_intent != 'unknown':
                break

        # Extract entities using spaCy if available
        entities = []
        if self.nlp:
            doc = self.nlp(text)
            for ent in doc.ents:
                entities.append({'text': ent.text, 'label': ent.label_})

        return detected_intent, entities

    def generate_response(self, intent, entities, original_text):
        """Generate appropriate response based on intent and context"""
        if intent == 'greeting':
            return "Hello! It's nice to meet you. How can I assist you today?"
        elif intent == 'farewell':
            return "Thank you for talking with me. Have a great day!"
        elif intent == 'question':
            # Check for specific types of questions
            text_lower = original_text.lower()
            if 'time' in text_lower or 'what time' in text_lower:
                from datetime import datetime
                current_time = datetime.now().strftime("%H:%M")
                return f"The current time is {current_time}."
            elif 'weather' in text_lower:
                return "I don't have access to real-time weather data, but I hope it's nice outside!"
            elif 'name' in text_lower:
                return "I'm your friendly robot assistant. You can call me ARRIA - Autonomous Robot for Responsive Intelligent Assistance."
            else:
                return "That's an interesting question. Could you tell me more about what you're looking for?"
        elif intent == 'command':
            # Extract command details
            if 'go to' in original_text.lower() or 'move to' in original_text.lower():
                return "I can help with navigation. Please specify a location, and I'll plan a path there."
            elif 'pick up' in original_text.lower() or 'bring me' in original_text.lower():
                return "I can assist with object manipulation. Please specify the object you'd like me to handle."
            else:
                return "I can help with various tasks. Could you be more specific about what you'd like me to do?"
        elif intent == 'help':
            return "I can assist with navigation, object manipulation, providing information, and engaging in conversation. What would you like help with?"
        elif intent == 'information':
            return "I can provide information about my capabilities and surroundings. What would you like to know?"
        else:
            # Default response for unknown intents
            return "I understand you're saying: " + original_text + ". How else can I assist you?"

    def speak(self, text):
        """Convert text to speech"""
        try:
            self.tts_engine.say(text)
            self.tts_engine.runAndWait()
        except Exception as e:
            self.get_logger().error(f'Error in text-to-speech: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = NLPProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 3: Safety Management System

### Task 3.1: Create a Safety Manager Node
Implement a system that ensures safe human-robot interaction:

```python
#!/usr/bin/env python3
# safety_manager/safety_manager/safety_manager.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Twist, Point, Pose
from hri_interfaces.msg import SafetyAlert, RobotState
import numpy as np
from scipy.spatial.distance import cdist
import time

class SafetyManager(Node):
    def __init__(self):
        super().__init__('safety_manager')

        # Publishers and subscribers
        self.safety_alert_pub = self.create_publisher(SafetyAlert, 'safety_alert', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, 'emergency_stop', 10)
        self.safety_cmd_pub = self.create_publisher(Twist, 'safety_cmd_vel', 10)
        self.robot_state_sub = self.create_subscription(
            RobotState, 'robot_state', self.robot_state_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.human_pos_sub = self.create_subscription(
            Point, 'human_position', self.human_position_callback, 10)

        # Internal state
        self.current_scan = None
        self.human_positions = []  # Store recent human positions
        self.robot_state = None
        self.is_interacting = False
        self.last_interaction_time = 0
        self.safety_thresholds = {
            'collision': 0.5,  # meters
            'social': 1.0,     # meters
            'safe_zone': 2.0   # meters
        }

        # Safety timers
        self.safety_timer = self.create_timer(0.1, self.check_safety)

        self.get_logger().info('Safety Manager initialized')

    def robot_state_callback(self, msg):
        """Update robot state from HRI system"""
        self.robot_state = msg
        self.is_interacting = msg.is_interacting

    def scan_callback(self, msg):
        """Process laser scan for obstacle detection"""
        self.current_scan = msg

    def human_position_callback(self, msg):
        """Update human position"""
        self.human_positions.append({
            'position': msg,
            'timestamp': time.time()
        })

        # Keep only recent positions (last 5 seconds)
        current_time = time.time()
        self.human_positions = [
            pos for pos in self.human_positions
            if current_time - pos['timestamp'] < 5.0
        ]

    def check_safety(self):
        """Check safety conditions and take appropriate action"""
        if not self.current_scan or not self.human_positions:
            return

        # Check for immediate collision risks
        collision_risk = self.check_collision_risk()
        if collision_risk:
            self.trigger_emergency_stop()
            return

        # Check social distance during interaction
        if self.is_interacting:
            social_violation = self.check_social_distance()
            if social_violation:
                self.slow_down_robot()
                return

        # Check general safety zones
        safety_zone_violation = self.check_safety_zone()
        if safety_zone_violation:
            self.adjust_robot_behavior()

    def check_collision_risk(self):
        """Check for immediate collision risks"""
        if not self.current_scan:
            return False

        # Check distances in front of robot
        front_ranges = self.current_scan.ranges[
            len(self.current_scan.ranges)//2 - 30 : len(self.current_scan.ranges)//2 + 30
        ]

        min_distance = min([r for r in front_ranges if not (r > self.current_scan.range_max or r < self.current_scan.range_min)], default=float('inf'))

        if min_distance < self.safety_thresholds['collision']:
            self.get_logger().warn(f'Collision risk detected: {min_distance:.2f}m')
            self.publish_safety_alert('collision_risk', f'Obstacle at {min_distance:.2f}m ahead')
            return True

        return False

    def check_social_distance(self):
        """Check if robot respects social distance during interaction"""
        if not self.human_positions:
            return False

        # Get the most recent human position
        latest_human_pos = self.human_positions[-1]['position']

        # Calculate distance to human
        # Assuming robot is at origin for simplicity (in practice, use TF)
        distance_to_human = np.sqrt(
            latest_human_pos.x**2 + latest_human_pos.y**2 + latest_human_pos.z**2
        )

        if distance_to_human < self.safety_thresholds['social']:
            self.get_logger().warn(f'Social distance violation: {distance_to_human:.2f}m')
            self.publish_safety_alert('social_distance_violation', f'Too close to human: {distance_to_human:.2f}m')
            return True

        return False

    def check_safety_zone(self):
        """Check if robot is in safe operating zone"""
        # This would typically check against a predefined safe area
        # For this example, we'll just check if there are any obstacles nearby
        if not self.current_scan:
            return False

        # Check for obstacles in the immediate vicinity
        close_obstacles = [r for r in self.current_scan.ranges
                          if self.current_scan.range_min < r < self.safety_thresholds['safe_zone']]

        if len(close_obstacles) > len(self.current_scan.ranges) * 0.1:  # More than 10% of readings are close
            self.get_logger().info('Operating in constrained space')
            return True

        return False

    def trigger_emergency_stop(self):
        """Trigger emergency stop"""
        self.get_logger().error('EMERGENCY STOP TRIGGERED')

        stop_msg = Bool()
        stop_msg.data = True
        self.emergency_stop_pub.publish(stop_msg)

        # Publish zero velocity command
        zero_cmd = Twist()
        self.safety_cmd_pub.publish(zero_cmd)

        self.publish_safety_alert('emergency_stop', 'Collision risk detected, stopping immediately')

    def slow_down_robot(self):
        """Reduce robot speed for safety"""
        # This would typically involve reducing the velocity commands
        # For this example, we'll just publish a safety alert
        self.publish_safety_alert('speed_reduction', 'Reducing speed for safety')

    def adjust_robot_behavior(self):
        """Adjust robot behavior based on safety conditions"""
        # This would typically involve modifying navigation parameters
        # For this example, we'll just log the adjustment
        self.get_logger().info('Adjusting robot behavior for safety')

    def publish_safety_alert(self, alert_type, description):
        """Publish safety alert"""
        alert_msg = SafetyAlert()
        alert_msg.type = alert_type
        alert_msg.description = description
        alert_msg.timestamp = self.get_clock().now().to_msg()
        alert_msg.severity = 2  # Warning level

        self.safety_alert_pub.publish(alert_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 4: Human-Robot Collaboration Framework

### Task 4.1: Create a Collaboration Manager
Implement a system that manages human-robot collaboration:

```python
#!/usr/bin/env python3
# social_interaction/social_interaction/collaboration_manager.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, Point
from hri_interfaces.msg import CollaborationTask, RobotState
from sensor_msgs.msg import LaserScan
import time
import json
from enum import Enum

class TaskState(Enum):
    IDLE = "idle"
    PLANNING = "planning"
    EXECUTING = "executing"
    COMPLETED = "completed"
    FAILED = "failed"

class CollaborationManager(Node):
    def __init__(self):
        super().__init__('collaboration_manager')

        # Publishers and subscribers
        self.task_pub = self.create_publisher(CollaborationTask, 'collaboration_task', 10)
        self.task_request_sub = self.create_subscription(
            String, 'task_request', self.task_request_callback, 10)
        self.robot_state_sub = self.create_subscription(
            RobotState, 'robot_state', self.robot_state_callback, 10)
        self.human_action_sub = self.create_subscription(
            String, 'human_action', self.human_action_callback, 10)
        self.task_status_sub = self.create_subscription(
            String, 'task_status', self.task_status_callback, 10)

        # Internal state
        self.robot_state = None
        self.current_task = None
        self.task_state = TaskState.IDLE
        self.collaboration_history = []
        self.task_queue = []

        # Collaboration parameters
        self.collision_buffer = 0.5  # meters
        self.communication_timeout = 10.0  # seconds

        # Timer for collaboration updates
        self.collaboration_timer = self.create_timer(1.0, self.update_collaboration)

        self.get_logger().info('Collaboration Manager initialized')

    def robot_state_callback(self, msg):
        """Update robot state"""
        self.robot_state = msg

    def task_request_callback(self, msg):
        """Process task request from user or system"""
        try:
            task_data = json.loads(msg.data)
            task = CollaborationTask()
            task.id = task_data.get('id', f'task_{int(time.time())}')
            task.type = task_data.get('type', 'unknown')
            task.description = task_data.get('description', '')
            task.priority = task_data.get('priority', 1)
            task.assigned_to = task_data.get('assigned_to', 'robot')
            task.parameters = json.dumps(task_data.get('parameters', {}))
            task.timestamp = self.get_clock().now().to_msg()

            # Add to task queue
            self.task_queue.append(task)
            self.get_logger().info(f'New task received: {task.description}')

            # Process the task if we're idle
            if self.task_state == TaskState.IDLE:
                self.process_next_task()

        except json.JSONDecodeError:
            self.get_logger().error('Invalid task request format')
        except Exception as e:
            self.get_logger().error(f'Error processing task request: {str(e)}')

    def human_action_callback(self, msg):
        """Process human actions that affect collaboration"""
        if self.current_task:
            action_data = json.loads(msg.data) if msg.data.startswith('{') else {'action': msg.data}

            self.get_logger().info(f'Human action detected: {action_data}')

            # Update task based on human action
            if action_data.get('action') == 'ready' and self.task_state == TaskState.PLANNING:
                self.task_state = TaskState.EXECUTING
                self.execute_task()
            elif action_data.get('action') == 'stop':
                self.abort_current_task()

    def task_status_callback(self, msg):
        """Process task status updates from other nodes"""
        try:
            status_data = json.loads(msg.data)
            task_id = status_data.get('task_id')
            status = status_data.get('status')

            if self.current_task and self.current_task.id == task_id:
                if status == 'completed':
                    self.task_state = TaskState.COMPLETED
                    self.complete_task()
                elif status == 'failed':
                    self.task_state = TaskState.FAILED
                    self.fail_task()
        except json.JSONDecodeError:
            self.get_logger().error('Invalid task status format')

    def process_next_task(self):
        """Process the next task in the queue"""
        if self.task_queue and self.task_state == TaskState.IDLE:
            self.current_task = self.task_queue.pop(0)
            self.task_state = TaskState.PLANNING

            self.get_logger().info(f'Processing task: {self.current_task.description}')

            # Plan the task (in a real system, this would involve complex planning)
            self.plan_task()

    def plan_task(self):
        """Plan the current task"""
        self.get_logger().info(f'Planning task: {self.current_task.description}')

        # In a real system, this would involve:
        # - Path planning avoiding human areas
        # - Task decomposition
        # - Resource allocation
        # - Safety verification

        # For this example, we'll just transition to executing after a short delay
        time.sleep(1)  # Simulate planning time
        self.task_state = TaskState.EXECUTING
        self.execute_task()

    def execute_task(self):
        """Execute the current task"""
        self.get_logger().info(f'Executing task: {self.current_task.description}')

        # In a real system, this would involve:
        # - Sending commands to robot
        # - Monitoring execution
        # - Handling exceptions

        # Publish task for execution
        self.task_pub.publish(self.current_task)

    def complete_task(self):
        """Complete the current task"""
        if self.current_task:
            self.get_logger().info(f'Task completed: {self.current_task.description}')

            # Add to collaboration history
            self.collaboration_history.append({
                'task_id': self.current_task.id,
                'completion_time': time.time(),
                'success': True
            })

            # Reset for next task
            self.current_task = None
            self.task_state = TaskState.IDLE

            # Process next task if available
            if self.task_queue:
                self.process_next_task()

    def fail_task(self):
        """Handle task failure"""
        if self.current_task:
            self.get_logger().error(f'Task failed: {self.current_task.description}')

            # Add to collaboration history
            self.collaboration_history.append({
                'task_id': self.current_task.id,
                'completion_time': time.time(),
                'success': False
            })

            # Reset for next task
            self.current_task = None
            self.task_state = TaskState.IDLE

    def abort_current_task(self):
        """Abort the current task"""
        if self.current_task:
            self.get_logger().info(f'Aborting task: {self.current_task.description}')
            self.task_state = TaskState.IDLE
            self.current_task = None

    def update_collaboration(self):
        """Update collaboration state"""
        if self.task_state == TaskState.EXECUTING:
            # Check if task is taking too long (timeout)
            # In a real system, this would check actual task progress
            pass

def main(args=None):
    rclpy.init(args=args)
    node = CollaborationManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 5: Complete HRI System Integration

### Task 5.1: Create Launch File
Create a launch file to start all HRI components:

```python
# social_interaction/launch/hri_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Social Behavior Manager
        Node(
            package='social_interaction',
            executable='social_behavior_manager',
            name='social_behavior_manager',
            output='screen',
        ),
        # NLP Processor
        Node(
            package='natural_language_interface',
            executable='nlp_processor',
            name='nlp_processor',
            output='screen',
        ),
        # Safety Manager
        Node(
            package='safety_manager',
            executable='safety_manager',
            name='safety_manager',
            output='screen',
        ),
        # Collaboration Manager
        Node(
            package='social_interaction',
            executable='collaboration_manager',
            name='collaboration_manager',
            output='screen',
        ),
    ])
```

## Exercises

### Exercise 1: Implement Emotion Recognition
Extend the system with emotion recognition:
- Add facial expression analysis
- Adjust robot responses based on detected emotions
- Implement emotional state tracking

### Exercise 2: Multi-Modal Interaction
Enhance the interaction system:
- Add gesture recognition
- Implement gaze tracking
- Create multimodal fusion

### Exercise 3: Ethical Decision Making
Add ethical considerations:
- Implement ethical decision trees
- Add bias detection in responses
- Create accountability mechanisms

## Summary

In this lab, you implemented a complete Human-Robot Interaction system with:
1. Social behavior management
2. Natural language processing
3. Safety management
4. Human-robot collaboration framework

The system demonstrates how different HRI components can be integrated to create safe and effective human-robot interactions.

## Next Steps

- Deploy on a physical robot platform
- Add more sophisticated NLP models
- Implement advanced safety protocols
- Test with real human users