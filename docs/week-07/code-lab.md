---
title: "Week 7: Sensor Integration and Perception Systems Code Lab"
week: 7
module: "Robotic Infrastructure"
difficulty: "advanced"
prerequisites: ["ros2-fundamentals", "python-basics", "robotics-concepts", "sensor-basics"]
learning_objectives:
  - "Implement sensor integration in ROS 2"
  - "Create perception pipelines"
  - "Handle sensor data processing and fusion"
  - "Implement sensor calibration procedures"
tags: ["sensors", "perception", "camera", "lidar", "imu", "calibration", "fusion"]
hardware_requirements:
  - gpu: "RTX 4070 or higher"
  - ram: "16GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "120 minutes"
---

# Week 7: Sensor Integration and Perception Systems Code Lab

## Learning Objectives
- Implement sensor integration in ROS 2
- Create perception pipelines
- Handle sensor data processing and fusion
- Implement sensor calibration procedures

## 7.1 Setting Up the Sensor Environment

First, let's set up the environment for sensor integration:

```bash
# Install required packages for sensor processing
sudo apt update
sudo apt install ros-humble-vision-opencv ros-humble-cv-bridge
sudo apt install ros-humble-image-transport ros-humble-compressed-image-transport
sudo apt install ros-humble-camera-calibration-parsers ros-humble-image-proc
sudo apt install ros-humble-laser-geometry ros-humble-pointcloud-to-laserscan
sudo apt install ros-humble-tf2-tools ros-humble-tf2-eigen

# Install Python dependencies
pip3 install opencv-python open3d numpy scipy matplotlib
pip3 install sensor-msgs-py message-filters
```

## 7.2 Camera Integration and Processing

Let's implement a comprehensive camera sensor node:

```python
#!/usr/bin/env python3
# camera_integration.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String

class CameraIntegrationNode(Node):
    def __init__(self):
        super().__init__('camera_integration_node')

        # Create CV bridge
        self.bridge = CvBridge()

        # Publishers
        self.image_pub = self.create_publisher(Image, 'camera/processed_image', 10)
        self.feature_pub = self.create_publisher(String, 'camera/features', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/calibrated_info', 10)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, 'camera/camera_info', self.camera_info_callback, 10
        )

        # Camera parameters
        self.camera_matrix = None
        self.distortion_coeffs = None
        self.image_width = 640
        self.image_height = 480

        # Feature detection parameters
        self.feature_detector = cv2.SIFT_create(nfeatures=500)
        self.descriptor_matcher = cv2.BFMatcher()

        # Processing flags
        self.enable_feature_detection = True
        self.enable_calibration = False

        # Timer for processing
        self.process_timer = self.create_timer(0.1, self.process_features)

        self.get_logger().info('Camera integration node initialized')

    def camera_info_callback(self, msg):
        """Receive camera calibration information"""
        self.image_width = msg.width
        self.image_height = msg.height
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.d) if msg.d else np.zeros(5)

    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Undistort image if calibration parameters are available
            if self.camera_matrix is not None and self.distortion_coeffs is not None:
                cv_image = self.undistort_image(cv_image)

            # Process image based on enabled features
            processed_image = self.process_image(cv_image)

            # Publish processed image
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
            processed_msg.header = msg.header
            self.image_pub.publish(processed_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def undistort_image(self, image):
        """Undistort image using camera calibration parameters"""
        if self.camera_matrix is None or self.distortion_coeffs is None:
            return image

        h, w = image.shape[:2]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.distortion_coeffs, (w, h), 1, (w, h)
        )

        undistorted = cv2.undistort(
            image, self.camera_matrix, self.distortion_coeffs, None, new_camera_matrix
        )

        # Crop image to remove black borders
        x, y, w, h = roi
        undistorted = undistorted[y:y+h, x:x+w]

        return undistorted

    def process_image(self, image):
        """Process image with various computer vision techniques"""
        if not self.enable_feature_detection:
            return image

        # Convert to grayscale for feature detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect features using SIFT
        keypoints, descriptors = self.feature_detector.detectAndCompute(gray, None)

        # Draw keypoints on image
        processed_image = cv2.drawKeypoints(
            image, keypoints, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
        )

        # Publish feature information
        if keypoints:
            feature_info = f'Detected {len(keypoints)} features'
            feature_msg = String()
            feature_msg.data = feature_info
            self.feature_pub.publish(feature_msg)

        return processed_image

    def process_features(self):
        """Process features for debugging or analysis"""
        # This could implement feature tracking, object recognition, etc.
        pass

def main(args=None):
    rclpy.init(args=args)
    node = CameraIntegrationNode()

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

## 7.3 LiDAR Integration and Processing

Now let's implement a LiDAR processing node:

```python
#!/usr/bin/env python3
# lidar_integration.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from collections import deque

class LidarIntegrationNode(Node):
    def __init__(self):
        super().__init__('lidar_integration_node')

        # Publishers
        self.cloud_pub = self.create_publisher(PointCloud2, 'lidar/point_cloud', 10)
        self.obstacle_pub = self.create_publisher(PointStamped, 'lidar/obstacles', 10)
        self.analysis_pub = self.create_publisher(Float64MultiArray, 'lidar/analysis', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )

        # LiDAR parameters
        self.min_range = 0.1
        self.max_range = 30.0
        self.obstacle_threshold = 1.0  # Distance threshold for obstacles

        # Processing buffers
        self.scan_buffer = deque(maxlen=10)
        self.obstacle_points = []

        # Timer for analysis
        self.analysis_timer = self.create_timer(0.5, self.analyze_environment)

        self.get_logger().info('LiDAR integration node initialized')

    def scan_callback(self, msg):
        """Process incoming LiDAR scan data"""
        # Convert scan to point cloud
        point_cloud = self.scan_to_point_cloud(msg)

        # Publish point cloud
        self.cloud_pub.publish(point_cloud)

        # Detect obstacles
        obstacles = self.detect_obstacles(msg)

        # Publish nearest obstacle
        if obstacles:
            nearest_obstacle = min(obstacles, key=lambda p: np.sqrt(p[0]**2 + p[1]**2))
            obstacle_msg = PointStamped()
            obstacle_msg.header = msg.header
            obstacle_msg.point.x = nearest_obstacle[0]
            obstacle_msg.point.y = nearest_obstacle[1]
            obstacle_msg.point.z = nearest_obstacle[2]
            self.obstacle_pub.publish(obstacle_msg)

        # Store for analysis
        self.scan_buffer.append(msg)

    def scan_to_point_cloud(self, scan_msg):
        """Convert LaserScan to PointCloud2"""
        # Calculate angles
        angles = np.linspace(
            scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges)
        )

        # Filter valid ranges
        valid_indices = [
            i for i, r in enumerate(scan_msg.ranges)
            if self.min_range <= r <= self.max_range
        ]

        # Create points
        points = []
        for i in valid_indices:
            angle = angles[i]
            range_val = scan_msg.ranges[i]
            x = range_val * np.cos(angle)
            y = range_val * np.sin(angle)
            z = 0.0  # 2D scan
            points.append([x, y, z])

        # Create PointCloud2 message
        header = scan_msg.header
        fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1)
        ]

        return pc2.create_cloud(header, fields, points)

    def detect_obstacles(self, scan_msg):
        """Detect obstacles in scan data"""
        angles = np.linspace(
            scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges)
        )

        obstacles = []
        for i, range_val in enumerate(scan_msg.ranges):
            if self.min_range <= range_val <= self.obstacle_threshold:
                angle = angles[i]
                x = range_val * np.cos(angle)
                y = range_val * np.sin(angle)
                obstacles.append([x, y, 0.0])

        return obstacles

    def analyze_environment(self):
        """Analyze environment based on recent scans"""
        if not self.scan_buffer:
            return

        # Calculate statistics
        all_ranges = []
        for scan in self.scan_buffer:
            valid_ranges = [r for r in scan.ranges if self.min_range <= r <= self.max_range]
            all_ranges.extend(valid_ranges)

        if all_ranges:
            stats_msg = Float64MultiArray()
            stats_msg.data = [
                len(all_ranges),           # Total valid readings
                np.mean(all_ranges),       # Average range
                np.std(all_ranges),        # Range standard deviation
                min(all_ranges),           # Minimum range
                max(all_ranges),           # Maximum range
                len([r for r in all_ranges if r < self.obstacle_threshold])  # Obstacle count
            ]
            self.analysis_pub.publish(stats_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarIntegrationNode()

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

## 7.4 IMU Integration and Fusion

Let's implement IMU integration with sensor fusion:

```python
#!/usr/bin/env python3
# imu_integration.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Float64
from scipy.spatial.transform import Rotation as R
import numpy as np
from collections import deque

class ImuIntegrationNode(Node):
    def __init__(self):
        super().__init__('imu_integration_node')

        # Publishers
        self.orientation_pub = self.create_publisher(Quaternion, 'imu/orientation', 10)
        self.angular_velocity_pub = self.create_publisher(Vector3, 'imu/angular_velocity', 10)
        self.linear_acceleration_pub = self.create_publisher(Vector3, 'imu/linear_acceleration', 10)
        self.roll_pitch_yaw_pub = self.create_publisher(Float64, 'imu/rpy', 10)

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10
        )

        # State variables
        self.orientation = R.from_quat([0, 0, 0, 1])  # Identity rotation
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        self.linear_acceleration = np.array([0.0, 0.0, 0.0])

        # Filtering parameters
        self.filter_alpha = 0.1  # Low-pass filter coefficient
        self.integration_time = 0.0

        # Buffers for smoothing
        self.orientation_buffer = deque(maxlen=10)
        self.acceleration_buffer = deque(maxlen=10)

        # Timer for publishing filtered data
        self.publish_timer = self.create_timer(0.05, self.publish_filtered_data)

        self.get_logger().info('IMU integration node initialized')

    def imu_callback(self, msg):
        """Process incoming IMU data"""
        # Extract raw data
        accel_raw = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        gyro_raw = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        # Apply low-pass filtering
        if len(self.acceleration_buffer) > 0:
            accel_filtered = self.filter_alpha * accel_raw + (1 - self.filter_alpha) * self.acceleration_buffer[-1]
        else:
            accel_filtered = accel_raw

        # Update state
        self.linear_acceleration = accel_filtered
        self.angular_velocity = gyro_raw

        # Integrate angular velocity to update orientation
        dt = 0.01  # Assuming 100 Hz update rate
        gyro_norm = np.linalg.norm(gyro_raw)
        if gyro_norm > 1e-6:  # Avoid division by zero
            axis = gyro_raw / gyro_norm
            angle = gyro_norm * dt
            delta_rotation = R.from_rotvec(axis * angle)
            self.orientation = self.orientation * delta_rotation

        # Store in buffers
        self.acceleration_buffer.append(accel_filtered)
        self.orientation_buffer.append(self.orientation.as_quat())

    def publish_filtered_data(self):
        """Publish filtered IMU data"""
        # Publish orientation
        orient_avg = np.mean(list(self.orientation_buffer), axis=0) if self.orientation_buffer else [0, 0, 0, 1]
        orient_msg = Quaternion()
        orient_msg.x = float(orient_avg[0])
        orient_msg.y = float(orient_avg[1])
        orient_msg.z = float(orient_avg[2])
        orient_msg.w = float(orient_avg[3])
        self.orientation_pub.publish(orient_msg)

        # Publish angular velocity
        ang_vel_msg = Vector3()
        ang_vel_msg.x = float(self.angular_velocity[0])
        ang_vel_msg.y = float(self.angular_velocity[1])
        ang_vel_msg.z = float(self.angular_velocity[2])
        self.angular_velocity_pub.publish(ang_vel_msg)

        # Publish linear acceleration
        lin_acc_msg = Vector3()
        lin_acc_msg.x = float(self.linear_acceleration[0])
        lin_acc_msg.y = float(self.linear_acceleration[1])
        lin_acc_msg.z = float(self.linear_acceleration[2])
        self.linear_acceleration_pub.publish(lin_acc_msg)

        # Publish roll, pitch, yaw
        rpy = self.orientation.as_euler('xyz')
        rpy_msg = Float64()
        rpy_msg.data = float(rpy[0])  # Roll
        self.roll_pitch_yaw_pub.publish(rpy_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuIntegrationNode()

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

## 7.5 Sensor Fusion Implementation

Let's create a comprehensive sensor fusion node:

```python
#!/usr/bin/env python3
# sensor_fusion.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
from scipy.spatial.transform import Rotation as R
import threading
from collections import deque

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Publishers
        self.fused_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'fused/pose', 10)
        self.fused_twist_pub = self.create_publisher(TwistWithCovarianceStamped, 'fused/twist', 10)
        self.odom_pub = self.create_publisher(Odometry, 'fused/odom', 10)
        self.status_pub = self.create_publisher(String, 'fusion/status', 10)

        # Subscribers
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        # State variables
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.orientation = R.from_quat([0, 0, 0, 1])
        self.angular_velocity = np.array([0.0, 0.0, 0.0])

        # Covariance matrices (diagonal for simplicity)
        self.position_covariance = np.eye(3) * 0.1
        self.orientation_covariance = np.eye(3) * 0.01
        self.velocity_covariance = np.eye(3) * 0.05

        # Buffers for sensor fusion
        self.imu_buffer = deque(maxlen=10)
        self.scan_buffer = deque(maxlen=5)

        # Fusion weights
        self.imu_weight = 0.7
        self.vision_weight = 0.2
        self.lidar_weight = 0.1

        # Thread lock for thread safety
        self.state_lock = threading.Lock()

        # Timer for fusion
        self.fusion_timer = self.create_timer(0.05, self.perform_fusion)

        self.get_logger().info('Sensor fusion node initialized')

    def imu_callback(self, msg):
        """Process IMU data for fusion"""
        with self.state_lock:
            # Extract orientation from IMU
            imu_quat = np.array([
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ])

            # Low-pass filter orientation
            if len(self.imu_buffer) > 0:
                prev_quat = self.imu_buffer[-1]
                filtered_quat = self.low_pass_filter_quaternion(imu_quat, prev_quat, 0.1)
            else:
                filtered_quat = imu_quat

            self.orientation = R.from_quat(filtered_quat)

            # Extract angular velocity
            self.angular_velocity = np.array([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ])

            # Extract linear acceleration
            linear_acc = np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ])

            # Integrate acceleration to get velocity
            dt = 0.05  # 20 Hz
            self.velocity += linear_acc * dt

            # Store in buffer
            self.imu_buffer.append(filtered_quat)

    def scan_callback(self, msg):
        """Process LiDAR data for fusion"""
        with self.state_lock:
            # Simple feature extraction from scan
            valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]

            if valid_ranges:
                # Estimate position based on nearest obstacles
                min_range_idx = np.argmin(valid_ranges)
                angle = msg.angle_min + min_range_idx * msg.angle_increment

                # This is a simplified example - in practice, this would use
                # more sophisticated SLAM algorithms
                estimated_x = valid_ranges[min_range_idx] * np.cos(angle)
                estimated_y = valid_ranges[min_range_idx] * np.sin(angle)

                # Update position estimate with LiDAR data
                self.position[0] = self.lidar_weight * estimated_x + (1 - self.lidar_weight) * self.position[0]
                self.position[1] = self.lidar_weight * estimated_y + (1 - self.lidar_weight) * self.position[1]

            self.scan_buffer.append(msg)

    def low_pass_filter_quaternion(self, q1, q2, alpha):
        """Apply low-pass filter to quaternions"""
        # Use spherical linear interpolation (SLERP) for quaternions
        dot_product = np.dot(q1, q2)

        # Ensure we take the shortest path
        if dot_product < 0:
            q2 = -q2
            dot_product = -dot_product

        # Clamp dot product to avoid numerical errors
        dot_product = np.clip(dot_product, -1.0, 1.0)

        # Calculate angle between quaternions
        angle = np.arccos(dot_product)

        if np.abs(angle) > 1e-6:  # Avoid division by zero
            sin_angle = np.sin(angle)
            s1 = np.sin((1 - alpha) * angle) / sin_angle
            s2 = np.sin(alpha * angle) / sin_angle
        else:
            s1 = 1 - alpha
            s2 = alpha

        filtered_quat = s1 * q1 + s2 * q2
        return filtered_quat / np.linalg.norm(filtered_quat)

    def perform_fusion(self):
        """Perform sensor fusion"""
        with self.state_lock:
            # Create fused pose message
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'

            # Set position
            pose_msg.pose.pose.position.x = float(self.position[0])
            pose_msg.pose.pose.position.y = float(self.position[1])
            pose_msg.pose.pose.position.z = float(self.position[2])

            # Set orientation
            quat = self.orientation.as_quat()
            pose_msg.pose.pose.orientation.x = float(quat[0])
            pose_msg.pose.pose.orientation.y = float(quat[1])
            pose_msg.pose.pose.orientation.z = float(quat[2])
            pose_msg.pose.pose.orientation.w = float(quat[3])

            # Set covariance
            pose_msg.pose.covariance[:9] = self.position_covariance.flatten()
            pose_msg.pose.covariance[21:27] = self.orientation_covariance.flatten()

            self.fused_pose_pub.publish(pose_msg)

            # Create fused twist message
            twist_msg = TwistWithCovarianceStamped()
            twist_msg.header = pose_msg.header

            # Set linear velocity
            twist_msg.twist.twist.linear.x = float(self.velocity[0])
            twist_msg.twist.twist.linear.y = float(self.velocity[1])
            twist_msg.twist.twist.linear.z = float(self.velocity[2])

            # Set angular velocity
            twist_msg.twist.twist.angular.x = float(self.angular_velocity[0])
            twist_msg.twist.twist.angular.y = float(self.angular_velocity[1])
            twist_msg.twist.twist.angular.z = float(self.angular_velocity[2])

            # Set covariance
            twist_msg.twist.covariance[:9] = self.velocity_covariance.flatten()
            twist_msg.twist.covariance[21:27] = self.angular_velocity_covariance.flatten() if hasattr(self, 'angular_velocity_covariance') else np.eye(3).flatten()

            self.fused_twist_pub.publish(twist_msg)

            # Create odometry message
            odom_msg = Odometry()
            odom_msg.header = pose_msg.header
            odom_msg.child_frame_id = 'base_link'
            odom_msg.pose = pose_msg.pose
            odom_msg.twist = twist_msg.twist

            self.odom_pub.publish(odom_msg)

            # Publish status
            status_msg = String()
            status_msg.data = f'Fusion: Pos=({self.position[0]:.2f},{self.position[1]:.2f}), Vel=({self.velocity[0]:.2f},{self.velocity[1]:.2f})'
            self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()

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

## 7.6 Camera-LiDAR Calibration

Let's implement a camera-LiDAR calibration procedure:

```python
#!/usr/bin/env python3
# calibration_procedure.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize
import threading

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')

        # Create CV bridge
        self.bridge = CvBridge()

        # Publishers
        self.calibration_status_pub = self.create_publisher(Bool, 'calibration/status', 10)

        # Subscribers
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        # Calibration state
        self.chessboard_pattern = (9, 6)
        self.obj_points = []  # 3D points in real world space
        self.img_points = []  # 2D points in image plane
        self.lidar_points = []  # Corresponding 3D points from LiDAR

        # Camera calibration parameters
        self.camera_matrix = None
        self.dist_coeffs = None

        # LiDAR to camera extrinsics
        self.extrinsics = np.eye(4)  # 4x4 transformation matrix

        # Calibration flags
        self.calibration_in_progress = False
        self.calibration_completed = False

        # Processing lock
        self.calibration_lock = threading.Lock()

        # Timer for calibration
        self.calibration_timer = self.create_timer(0.1, self.calibration_loop)

        self.get_logger().info('Calibration node initialized')

    def image_callback(self, msg):
        """Process camera images for calibration"""
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Find chessboard corners
            ret, corners = self.find_chessboard_corners(image)

            if ret:
                # Add to calibration data
                self.add_calibration_data(corners, image)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def scan_callback(self, msg):
        """Process LiDAR data for calibration"""
        # Convert scan to 3D points
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        lidar_points = []

        for i, range_val in enumerate(msg.ranges):
            if msg.range_min <= range_val <= msg.range_max:
                angle = angles[i]
                x = range_val * np.cos(angle)
                y = range_val * np.sin(angle)
                z = 0.0  # Assuming 2D scan
                lidar_points.append([x, y, z])

        # Store LiDAR points for correspondence
        self.lidar_points = lidar_points

    def find_chessboard_corners(self, image):
        """Find chessboard corners in image"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(
            gray, self.chessboard_pattern, None
        )

        if ret:
            # Refine corner locations
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria
            )
            return True, corners_refined
        return False, None

    def add_calibration_data(self, corners, image):
        """Add calibration data point"""
        if self.calibration_in_progress:
            # Prepare object points (chessboard corners in 3D space)
            objp = np.zeros((self.chessboard_pattern[0] * self.chessboard_pattern[1], 3), np.float32)
            objp[:, :2] = np.mgrid[0:self.chessboard_pattern[0], 0:self.chessboard_pattern[1]].T.reshape(-1, 2)

            # Add to calibration buffers
            self.obj_points.append(objp)
            self.img_points.append(corners)

    def start_calibration(self):
        """Start the calibration process"""
        with self.calibration_lock:
            self.calibration_in_progress = True
            self.obj_points = []
            self.img_points = []
            self.lidar_points = []
            self.get_logger().info('Calibration started')

    def stop_calibration(self):
        """Stop the calibration process"""
        with self.calibration_lock:
            self.calibration_in_progress = False
            if len(self.obj_points) >= 10:
                self.perform_calibration()
            self.get_logger().info('Calibration stopped')

    def perform_calibration(self):
        """Perform camera and LiDAR calibration"""
        try:
            # Camera calibration
            if len(self.obj_points) >= 10:
                ret, self.camera_matrix, self.dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                    self.obj_points, self.img_points,
                    (640, 480), None, None
                )

                if ret:
                    self.get_logger().info('Camera calibration completed')

                    # Save calibration parameters
                    self.save_camera_calibration()

                    # Perform LiDAR to camera calibration
                    self.calibrate_lidar_camera()

                    self.calibration_completed = True
                    self.calibration_in_progress = False

                    # Publish completion status
                    status_msg = Bool()
                    status_msg.data = True
                    self.calibration_status_pub.publish(status_msg)
                else:
                    self.get_logger().error('Camera calibration failed')
            else:
                self.get_logger().warn('Not enough calibration samples')

        except Exception as e:
            self.get_logger().error(f'Calibration error: {e}')

    def calibrate_lidar_camera(self):
        """Calibrate LiDAR to camera extrinsics"""
        if not self.camera_matrix or not self.dist_coeffs:
            self.get_logger().error('Camera calibration required first')
            return

        # This is a simplified example - real implementation would be more complex
        # and would require detecting the same pattern in both sensors

        # For now, we'll use a dummy optimization
        initial_extrinsics = np.eye(4).flatten()[:-1]  # Remove last element [0,0,0,1]

        # Dummy optimization function
        def optimization_func(params):
            # Reshape parameters to 3x4 matrix
            transform = params.reshape(3, 4)

            # Calculate reprojection error (dummy implementation)
            error = 0.0
            for obj_pt, img_pt in zip(self.obj_points, self.img_points):
                # Transform 3D points to camera frame
                # Project to image plane
                # Calculate error
                error += np.random.random()  # Dummy error

            return error

        # Perform optimization
        result = minimize(
            optimization_func,
            initial_extrinsics,
            method='Powell'
        )

        # Reshape result to 4x4 matrix
        self.extrinsics = np.eye(4)
        self.extrinsics[:3, :4] = result.x.reshape(3, 4)

        self.get_logger().info('LiDAR to camera calibration completed')

        # Save extrinsics
        self.save_extrinsics()

    def save_camera_calibration(self):
        """Save camera calibration parameters"""
        import yaml
        calibration_data = {
            'camera_matrix': self.camera_matrix.tolist(),
            'distortion_coefficients': self.dist_coeffs.tolist()
        }

        with open('/tmp/camera_calibration.yaml', 'w') as f:
            yaml.dump(calibration_data, f)

        self.get_logger().info('Camera calibration saved to /tmp/camera_calibration.yaml')

    def save_extrinsics(self):
        """Save LiDAR to camera extrinsics"""
        import yaml
        extrinsics_data = {
            'extrinsics_matrix': self.extrinsics.tolist()
        }

        with open('/tmp/lidar_camera_extrinsics.yaml', 'w') as f:
            yaml.dump(extrinsics_data, f)

        self.get_logger().info('LiDAR to camera extrinsics saved to /tmp/lidar_camera_extrinsics.yaml')

    def calibration_loop(self):
        """Calibration processing loop"""
        # This could implement automatic calibration triggers
        # or manual calibration sequences
        pass

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()

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

## 7.7 Multi-Sensor Perception Pipeline

Let's create a complete perception pipeline:

```python
#!/usr/bin/env python3
# perception_pipeline.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu, PointCloud2
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import String, Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial.transform import Rotation as R
import threading
from collections import deque

class PerceptionPipelineNode(Node):
    def __init__(self):
        super().__init__('perception_pipeline_node')

        # Create CV bridge
        self.bridge = CvBridge()

        # Publishers
        self.object_pub = self.create_publisher(String, 'perception/objects', 10)
        self.tracking_pub = self.create_publisher(Float64MultiArray, 'perception/tracking', 10)
        self.status_pub = self.create_publisher(String, 'perception/status', 10)

        # Subscribers
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)

        # Internal state
        self.current_image = None
        self.current_scan = None
        self.current_imu = None
        self.objects = []  # Detected objects
        self.tracks = {}   # Object tracks

        # Processing parameters
        self.feature_detector = cv2.SIFT_create(nfeatures=500)
        self.descriptor_matcher = cv2.BFMatcher()
        self.min_track_length = 5

        # Buffers
        self.image_buffer = deque(maxlen=10)
        self.scan_buffer = deque(maxlen=10)

        # Thread locks
        self.data_lock = threading.Lock()

        # Timer for processing
        self.process_timer = self.create_timer(0.1, self.process_pipeline)

        self.get_logger().info('Perception pipeline node initialized')

    def image_callback(self, msg):
        """Process incoming image"""
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self.data_lock:
                self.current_image = image
                self.image_buffer.append(image)
        except Exception as e:
            self.get_logger().error(f'Image callback error: {e}')

    def scan_callback(self, msg):
        """Process incoming scan"""
        with self.data_lock:
            self.current_scan = msg
            self.scan_buffer.append(msg)

    def imu_callback(self, msg):
        """Process incoming IMU data"""
        with self.data_lock:
            self.current_imu = msg

    def process_pipeline(self):
        """Main perception pipeline"""
        with self.data_lock:
            if not all([self.current_image, self.current_scan, self.current_imu]):
                return

            # Step 1: Visual object detection
            visual_objects = self.detect_visual_objects(self.current_image)

            # Step 2: LiDAR object detection
            lidar_objects = self.detect_lidar_objects(self.current_scan)

            # Step 3: Sensor fusion
            fused_objects = self.fuse_objects(visual_objects, lidar_objects)

            # Step 4: Object tracking
            tracked_objects = self.track_objects(fused_objects)

            # Step 5: Publish results
            self.publish_results(tracked_objects)

    def detect_visual_objects(self, image):
        """Detect objects in image"""
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect features
        keypoints, descriptors = self.feature_detector.detectAndCompute(gray, None)

        # Simple object detection based on keypoint density
        objects = []
        if keypoints:
            # Group nearby keypoints as potential objects
            for kp in keypoints:
                x, y = int(kp.pt[0]), int(kp.pt[1])
                objects.append({
                    'type': 'feature_point',
                    'position': (x, y),
                    'confidence': 0.5
                })

        return objects

    def detect_lidar_objects(self, scan):
        """Detect objects in LiDAR scan"""
        # Convert scan to point cloud
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        points = []

        for i, range_val in enumerate(scan.ranges):
            if scan.range_min <= range_val <= scan.range_max:
                angle = angles[i]
                x = range_val * np.cos(angle)
                y = range_val * np.sin(angle)
                points.append([x, y])

        if len(points) < 2:
            return []

        # Simple clustering for object detection
        points = np.array(points)
        labels = self.simple_cluster(points)

        objects = []
        for label in set(labels):
            if label == -1:  # Noise
                continue

            cluster_points = points[labels == label]
            center = np.mean(cluster_points, axis=0)
            size = np.std(cluster_points, axis=0)

            objects.append({
                'type': 'lidar_cluster',
                'position': center,
                'size': size,
                'confidence': 0.7
            })

        return objects

    def simple_cluster(self, points, eps=0.5, min_points=3):
        """Simple clustering algorithm"""
        if len(points) < 2:
            return np.array([])

        labels = np.full(len(points), -1)
        cluster_id = 0

        for i, point in enumerate(points):
            if labels[i] != -1:
                continue

            # Find neighbors
            distances = np.linalg.norm(points - point, axis=1)
            neighbors = np.where(distances < eps)[0]

            if len(neighbors) >= min_points:
                labels[neighbors] = cluster_id
                cluster_id += 1

        return labels

    def fuse_objects(self, visual_objects, lidar_objects):
        """Fuse objects from different sensors"""
        fused_objects = []

        # For each visual object, try to associate with LiDAR object
        for v_obj in visual_objects:
            best_association = None
            best_distance = float('inf')

            # Convert visual object to world coordinates (simplified)
            v_world = self.image_to_world(v_obj['position'])

            for l_obj in lidar_objects:
                distance = np.linalg.norm(
                    np.array(v_world[:2]) - l_obj['position'][:2]
                )
                if distance < 1.0 and distance < best_distance:  # 1m threshold
                    best_association = l_obj
                    best_distance = distance

            if best_association:
                # Create fused object
                fused_obj = {
                    'type': 'fused_object',
                    'position': best_association['position'],
                    'confidence': (v_obj['confidence'] + best_association['confidence']) / 2,
                    'visual_data': v_obj,
                    'lidar_data': best_association
                }
                fused_objects.append(fused_obj)
            else:
                # Add visual-only object
                fused_objects.append({
                    'type': 'visual_only',
                    'position': v_world[:2],
                    'confidence': v_obj['confidence']
                })

        return fused_objects

    def image_to_world(self, image_coords):
        """Convert image coordinates to world coordinates (simplified)"""
        # This would use camera calibration in practice
        # For now, return a simplified conversion
        x = (image_coords[0] - 320) * 0.01  # Approximate conversion
        y = (image_coords[1] - 240) * 0.01
        z = 1.0  # Assume 1m distance
        return [x, y, z]

    def track_objects(self, objects):
        """Track objects across frames"""
        # Simple tracking by position association
        for obj in objects:
            obj_id = self.find_nearest_existing_object(obj)
            if obj_id is not None:
                # Update existing track
                if obj_id not in self.tracks:
                    self.tracks[obj_id] = {'positions': [], 'velocity': None}

                self.tracks[obj_id]['positions'].append(obj['position'])

                # Calculate velocity if enough positions
                if len(self.tracks[obj_id]['positions']) > 1:
                    pos_list = self.tracks[obj_id]['positions']
                    if len(pos_list) >= 2:
                        dt = 0.1  # 10 Hz
                        displacement = np.array(pos_list[-1]) - np.array(pos_list[-2])
                        velocity = displacement / dt
                        self.tracks[obj_id]['velocity'] = velocity
            else:
                # Create new track
                new_id = len(self.tracks)
                self.tracks[new_id] = {
                    'positions': [obj['position']],
                    'velocity': None,
                    'confidence': obj['confidence']
                }

        # Clean up old tracks
        self.cleanup_tracks()

        return self.tracks

    def find_nearest_existing_object(self, new_object, threshold=1.0):
        """Find nearest existing object for association"""
        for track_id, track_data in self.tracks.items():
            if track_data['positions']:
                last_pos = track_data['positions'][-1]
                distance = np.linalg.norm(
                    np.array(new_object['position']) - np.array(last_pos)
                )
                if distance < threshold:
                    return track_id
        return None

    def cleanup_tracks(self):
        """Remove old tracks"""
        # Remove tracks with no recent updates or too few positions
        valid_tracks = {}
        for track_id, track_data in self.tracks.items():
            if (len(track_data['positions']) >= self.min_track_length and
                len(track_data['positions']) > 0):
                valid_tracks[track_id] = track_data
        self.tracks = valid_tracks

    def publish_results(self, tracks):
        """Publish perception results"""
        # Publish object information
        obj_msg = String()
        obj_msg.data = f'Detected {len(tracks)} objects'
        self.object_pub.publish(obj_msg)

        # Publish tracking information
        tracking_msg = Float64MultiArray()
        tracking_data = []
        for track_id, track_data in tracks.items():
            tracking_data.extend([
                track_id,
                len(track_data['positions']),
                track_data['confidence'] if 'confidence' in track_data else 0.0
            ])
            if track_data['velocity'] is not None:
                tracking_data.extend(track_data['velocity'])
            else:
                tracking_data.extend([0.0, 0.0])

        tracking_msg.data = tracking_data
        self.tracking_pub.publish(tracking_msg)

        # Publish status
        status_msg = String()
        status_msg.data = f'Perception: {len(tracks)} tracks active'
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionPipelineNode()

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

## 7.8 Practical Exercise

Create a complete sensor integration system that:
1. Implements camera, LiDAR, and IMU integration
2. Performs sensor fusion with Kalman filtering
3. Implements calibration procedures
4. Creates a perception pipeline
5. Validates sensor data quality

```python
# Student exercise - Complete implementation
class CompleteSensorSystem:
    """Student implementation of a complete sensor system"""

    def __init__(self):
        """Initialize the complete sensor system"""
        # TODO: Implement camera integration
        # TODO: Implement LiDAR integration
        # TODO: Implement IMU integration
        # TODO: Implement sensor fusion
        # TODO: Implement calibration procedures
        # TODO: Implement perception pipeline
        # TODO: Implement data quality validation
        pass

    def implement_camera_integration(self):
        """Implement camera sensor integration"""
        # TODO: Complete implementation
        pass

    def implement_lidar_integration(self):
        """Implement LiDAR sensor integration"""
        # TODO: Complete implementation
        pass

    def implement_imu_integration(self):
        """Implement IMU sensor integration"""
        # TODO: Complete implementation
        pass

    def implement_sensor_fusion(self):
        """Implement sensor fusion with Kalman filtering"""
        # TODO: Complete implementation
        pass

    def implement_calibration(self):
        """Implement calibration procedures"""
        # TODO: Complete implementation
        pass

    def implement_perception_pipeline(self):
        """Implement perception pipeline"""
        # TODO: Complete implementation
        pass

    def implement_validation(self):
        """Implement data quality validation"""
        # TODO: Complete implementation
        pass

print("Student Exercise: Implement a complete sensor system")
print("Requirements:")
print("1. Camera, LiDAR, and IMU integration")
print("2. Sensor fusion with Kalman filtering")
print("3. Calibration procedures")
print("4. Perception pipeline")
print("5. Data quality validation")
print("6. Real-time processing capabilities")
```

## Summary

In this lab, we've implemented a comprehensive sensor integration system with camera, LiDAR, and IMU processing, sensor fusion, calibration procedures, and perception pipelines. These components form the foundation of robotic perception systems that enable robots to understand and interact with their environment.

<div class="alert alert-warning">
  <h5>Hardware Requirement</h5>
  <div><strong>Requirement:</strong> GPU</div>
  <div><strong>Minimum:</strong> RTX 4070</div>
  <div><strong>Recommended:</strong> RTX 4080</div>
  <div><strong>Purpose:</strong> Sensor processing and perception algorithms require significant computational resources</div>
</div>