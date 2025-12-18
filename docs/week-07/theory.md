---
title: "Week 7: Sensor Integration and Perception Systems"
week: 7
module: "Robotic Infrastructure"
difficulty: "advanced"
prerequisites: ["ros2-fundamentals", "python-basics", "robotics-concepts", "sensor-basics"]
learning_objectives:
  - "Integrate various robotic sensors"
  - "Implement perception pipelines"
  - "Handle sensor data synchronization"
tags: ["sensors", "perception", "camera", "lidar", "imu", "calibration"]
hardware_requirements:
  - gpu: "RTX 4070 or higher"
  - ram: "16GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "90 minutes"
---

# Week 7: Sensor Integration and Perception Systems

## Learning Objectives
- Integrate various robotic sensors
- Implement perception pipelines
- Handle sensor data synchronization
- Understand sensor calibration procedures

## 7.1 Introduction to Robotic Sensors

Robotic sensors are critical components that enable robots to perceive and interact with their environment. They provide the robot with information about its internal state and external surroundings.

### Types of Robotic Sensors

1. **Proprioceptive Sensors**: Measure internal robot state
   - Joint encoders
   - Motor current sensors
   - Internal temperature sensors

2. **Exteroceptive Sensors**: Measure external environment
   - Cameras (RGB, stereo, depth)
   - LiDAR (2D, 3D)
   - IMU (accelerometer, gyroscope, magnetometer)
   - GPS
   - Force/torque sensors
   - Tactile sensors

3. **Range Sensors**: Measure distances to objects
   - Ultrasonic sensors
   - Infrared sensors
   - Time-of-flight sensors

### Sensor Characteristics

Each sensor type has specific characteristics that affect its use:

- **Accuracy**: How close measurements are to true values
- **Precision**: Consistency of repeated measurements
- **Resolution**: Smallest detectable change
- **Range**: Minimum and maximum measurable values
- **Update Rate**: Frequency of measurements
- **Noise**: Random variations in measurements
- **Latency**: Delay between measurement and output

## 7.2 Camera Systems and Computer Vision

### Camera Types and Configurations

```python
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

class CameraSensorNode(Node):
    def __init__(self):
        super().__init__('camera_sensor_node')

        # Publishers for camera data
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)

        # CV bridge for image conversion
        self.bridge = CvBridge()

        # Camera parameters
        self.camera_matrix = np.array([
            [615.0, 0.0, 320.0],
            [0.0, 615.0, 240.0],
            [0.0, 0.0, 1.0]
        ])

        # Initialize camera
        self.cap = cv2.VideoCapture(0)  # Default camera
        if not self.cap.isOpened():
            self.get_logger().error('Could not open camera')
            return

        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # Timer for capturing images
        self.timer = self.create_timer(0.033, self.capture_image)  # ~30 FPS

    def capture_image(self):
        """Capture and publish camera image"""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture image')
            return

        # Convert to ROS image message
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = 'camera_optical_frame'

        self.image_pub.publish(image_msg)

        # Publish camera info
        self.publish_camera_info()

    def publish_camera_info(self):
        """Publish camera calibration information"""
        info_msg = CameraInfo()
        info_msg.header.stamp = self.get_clock().now().to_msg()
        info_msg.header.frame_id = 'camera_optical_frame'
        info_msg.width = 640
        info_msg.height = 480
        info_msg.k = self.camera_matrix.flatten().tolist()

        self.info_pub.publish(info_msg)

    def __del__(self):
        """Clean up camera resource"""
        if hasattr(self, 'cap'):
            self.cap.release()
```

### Stereo Vision and Depth Estimation

```python
import cv2
import numpy as np

class StereoVisionProcessor:
    def __init__(self, left_camera_matrix, right_camera_matrix,
                 left_dist_coeffs, right_dist_coeffs, baseline):
        # Camera matrices and distortion coefficients
        self.left_camera_matrix = left_camera_matrix
        self.right_camera_matrix = right_camera_matrix
        self.left_dist_coeffs = left_dist_coeffs
        self.right_dist_coeffs = right_dist_coeffs
        self.baseline = baseline  # Distance between cameras

        # Stereo rectification parameters
        self.R1, self.R2, self.P1, self.P2, self.Q = None, None, None, None, None

    def rectify_images(self, left_img, right_img):
        """Rectify stereo image pair"""
        # Compute rectification parameters
        R1, R2, P1, P2, Q = cv2.stereoRectify(
            self.left_camera_matrix, self.left_dist_coeffs,
            self.right_camera_matrix, self.right_dist_coeffs,
            left_img.shape[::-1], np.eye(3), np.eye(3)
        )

        # Create undistortion maps
        map1x, map1y = cv2.initUndistortRectifyMap(
            self.left_camera_matrix, self.left_dist_coeffs,
            R1, self.P1, left_img.shape[::-1], cv2.CV_32FC1
        )
        map2x, map2y = cv2.initUndistortRectifyMap(
            self.right_camera_matrix, self.right_dist_coeffs,
            R2, self.P2, right_img.shape[::-1], cv2.CV_32FC1
        )

        # Apply rectification
        left_rectified = cv2.remap(left_img, map1x, map1y, cv2.INTER_LINEAR)
        right_rectified = cv2.remap(right_img, map2x, map2y, cv2.INTER_LINEAR)

        return left_rectified, right_rectified

    def compute_depth_map(self, left_img, right_img):
        """Compute depth map from stereo images"""
        # Convert to grayscale
        left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

        # Create stereo matcher
        stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=64,
            blockSize=11,
            P1=8 * 3 * 11**2,
            P2=32 * 3 * 11**2,
            disp12MaxDiff=1,
            uniquenessRatio=15,
            speckleWindowSize=0,
            speckleRange=2,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )

        # Compute disparity
        disparity = stereo.compute(left_gray, right_gray).astype(np.float32) / 16.0

        # Convert disparity to depth
        depth_map = (self.baseline * self.left_camera_matrix[0, 0]) / (disparity + 1e-6)

        return depth_map
```

## 7.3 LiDAR and 3D Perception

### LiDAR Sensor Integration

```python
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import rclpy
from rclpy.node import Node

class LidarSensorNode(Node):
    def __init__(self):
        super().__init__('lidar_sensor_node')

        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, 'point_cloud', 10)

        # LiDAR parameters
        self.angle_min = -np.pi
        self.angle_max = np.pi
        self.angle_increment = np.pi / 360  # 0.5 degree resolution
        self.range_min = 0.1
        self.range_max = 30.0
        self.scan_time = 0.1  # 10 Hz

        # Timer for LiDAR simulation
        self.timer = self.create_timer(self.scan_time, self.publish_scan)

    def publish_scan(self):
        """Publish simulated LiDAR scan data"""
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser_frame'
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = self.scan_time
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max

        # Generate simulated ranges (with some obstacles)
        num_points = int((self.angle_max - self.angle_min) / self.angle_increment) + 1
        ranges = np.full(num_points, self.range_max)

        # Add some obstacles
        angles = np.linspace(self.angle_min, self.angle_max, num_points)
        for i, angle in enumerate(angles):
            # Simulate a circular obstacle at (2, 0) with radius 0.5
            obstacle_x, obstacle_y, obstacle_r = 2.0, 0.0, 0.5
            distance_to_obstacle = np.sqrt((obstacle_x - 2*np.cos(angle))**2 +
                                         (obstacle_y - 2*np.sin(angle))**2) - obstacle_r

            if 0.1 < distance_to_obstacle < self.range_max:
                ranges[i] = min(ranges[i], distance_to_obstacle)

        # Add noise
        noise = np.random.normal(0, 0.01, len(ranges))
        ranges = np.maximum(self.range_min, ranges + noise)

        scan_msg.ranges = ranges.tolist()
        scan_msg.intensities = []  # Optional intensities

        self.scan_pub.publish(scan_msg)

        # Also publish as point cloud
        self.publish_point_cloud(ranges, angles)

    def publish_point_cloud(self, ranges, angles):
        """Convert scan data to point cloud"""
        points = []
        for angle, range_val in zip(angles, ranges):
            if self.range_min <= range_val <= self.range_max:
                x = range_val * np.cos(angle)
                y = range_val * np.sin(angle)
                z = 0.0  # 2D scan, so z=0
                points.append([x, y, z])

        # Create PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'laser_frame'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        cloud_msg = pc2.create_cloud(header, fields, points)
        self.cloud_pub.publish(cloud_msg)
```

### 3D Point Cloud Processing

```python
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudProcessor:
    def __init__(self):
        self.voxel_size = 0.05  # 5cm voxel size

    def convert_ros_to_o3d(self, cloud_msg):
        """Convert ROS PointCloud2 to Open3D point cloud"""
        points = []
        for point in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([point[0], point[1], point[2]])

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
        return pcd

    def downsample_point_cloud(self, pcd):
        """Downsample point cloud using voxel grid filter"""
        return pcd.voxel_down_sample(voxel_size=self.voxel_size)

    def estimate_normals(self, pcd):
        """Estimate normals for point cloud"""
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
        )
        return pcd

    def segment_planes(self, pcd):
        """Segment planar surfaces using RANSAC"""
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=0.01,
            ransac_n=3,
            num_iterations=1000
        )
        [a, b, c, d] = plane_model
        self.get_logger().info(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

        # Extract plane and non-plane points
        inlier_cloud = pcd.select_by_index(inliers)
        outlier_cloud = pcd.select_by_index(inliers, invert=True)

        return inlier_cloud, outlier_cloud

    def cluster_objects(self, pcd):
        """Cluster objects using DBSCAN"""
        labels = np.array(pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=False))
        max_label = labels.max()
        self.get_logger().info(f'Number of clusters: {max_label + 1}')

        return labels
```

## 7.4 IMU and Inertial Sensing

### IMU Integration and Fusion

```python
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

class ImuSensorNode(Node):
    def __init__(self):
        super().__init__('imu_sensor_node')

        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)

        # IMU parameters
        self.linear_acceleration_variance = 0.01**2
        self.angular_velocity_variance = 0.001**2
        self.orientation_variance = 0.01**2

        # State variables
        self.orientation = R.from_quat([0, 0, 0, 1])  # Identity rotation
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        self.linear_acceleration = np.array([0.0, 0.0, -9.81])  # Gravity

        # Timer for IMU simulation
        self.timer = self.create_timer(0.01, self.publish_imu_data)  # 100 Hz

    def publish_imu_data(self):
        """Publish simulated IMU data"""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Simulate sensor noise
        accel_noise = np.random.normal(0, np.sqrt(self.linear_acceleration_variance), 3)
        gyro_noise = np.random.normal(0, np.sqrt(self.angular_velocity_variance), 3)

        # Publish linear acceleration (with noise)
        imu_msg.linear_acceleration.x = self.linear_acceleration[0] + accel_noise[0]
        imu_msg.linear_acceleration.y = self.linear_acceleration[1] + accel_noise[1]
        imu_msg.linear_acceleration.z = self.linear_acceleration[2] + accel_noise[2]

        # Publish angular velocity (with noise)
        imu_msg.angular_velocity.x = self.angular_velocity[0] + gyro_noise[0]
        imu_msg.angular_velocity.y = self.angular_velocity[1] + gyro_noise[1]
        imu_msg.angular_velocity.z = self.angular_velocity[2] + gyro_noise[2]

        # Publish orientation (as quaternion)
        quat = self.orientation.as_quat()
        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]

        # Set covariance matrices
        imu_msg.linear_acceleration_covariance[0] = self.linear_acceleration_variance
        imu_msg.linear_acceleration_covariance[4] = self.linear_acceleration_variance
        imu_msg.linear_acceleration_covariance[8] = self.linear_acceleration_variance

        imu_msg.angular_velocity_covariance[0] = self.angular_velocity_variance
        imu_msg.angular_velocity_covariance[4] = self.angular_velocity_variance
        imu_msg.angular_velocity_covariance[8] = self.angular_velocity_variance

        imu_msg.orientation_covariance[0] = self.orientation_variance
        imu_msg.orientation_covariance[4] = self.orientation_variance
        imu_msg.orientation_covariance[8] = self.orientation_variance

        self.imu_pub.publish(imu_msg)

class ImuFusionNode(Node):
    """Node for fusing IMU data with other sensors"""

    def __init__(self):
        super().__init__('imu_fusion_node')

        # Subscribers
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Publisher for fused data
        self.fused_pub = self.create_publisher(Odometry, 'fused_odom', 10)

        # State estimation variables
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.orientation = R.from_quat([0, 0, 0, 1])

        # Kalman filter parameters
        self.process_noise = 0.1
        self.measurement_noise = 0.01

    def imu_callback(self, msg):
        """Process IMU data for state estimation"""
        # Extract IMU data
        accel = np.array([msg.linear_acceleration.x,
                         msg.linear_acceleration.y,
                         msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x,
                        msg.angular_velocity.y,
                        msg.angular_velocity.z])

        # Update orientation using gyroscope integration
        dt = 0.01  # Assuming 100 Hz
        omega_norm = np.linalg.norm(gyro)
        if omega_norm > 1e-6:  # Avoid division by zero
            axis = gyro / omega_norm
            angle = omega_norm * dt
            delta_rotation = R.from_rotvec(axis * angle)
            self.orientation = self.orientation * delta_rotation

    def odom_callback(self, msg):
        """Process odometry data for fusion"""
        # Extract position from odometry
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z

        # Extract orientation from odometry
        quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.orientation = R.from_quat(quat)

    def publish_fused_odom(self):
        """Publish fused odometry estimate"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'

        # Set position
        odom_msg.pose.pose.position.x = self.position[0]
        odom_msg.pose.pose.position.y = self.position[1]
        odom_msg.pose.pose.position.z = self.position[2]

        # Set orientation
        quat = self.orientation.as_quat()
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # Set velocity
        odom_msg.twist.twist.linear.x = self.velocity[0]
        odom_msg.twist.twist.linear.y = self.velocity[1]
        odom_msg.twist.twist.linear.z = self.velocity[2]

        self.fused_pub.publish(odom_msg)
```

## 7.5 Sensor Calibration Procedures

### Camera Calibration

```python
import cv2
import numpy as np
import yaml

class CameraCalibrator:
    def __init__(self, pattern_size=(9, 6)):
        self.pattern_size = pattern_size
        self.obj_points = []  # 3D points in real world space
        self.img_points = []  # 2D points in image plane

        # Prepare object points (e.g., chessboard corners)
        self.objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)

    def find_corners(self, img):
        """Find chessboard corners in image"""
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)

        if ret:
            # Refine corner locations
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria
            )
            return True, corners_refined
        return False, None

    def add_calibration_image(self, img):
        """Add image to calibration dataset"""
        ret, corners = self.find_corners(img)
        if ret:
            self.obj_points.append(self.objp)
            self.img_points.append(corners)
            return True
        return False

    def calibrate_camera(self):
        """Perform camera calibration"""
        if len(self.obj_points) < 10:
            raise ValueError("Need at least 10 calibration images")

        # Perform calibration
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points, self.img_points,
            (640, 480), None, None
        )

        if not ret:
            raise ValueError("Calibration failed")

        # Calculate reprojection error
        total_error = 0
        for i in range(len(self.obj_points)):
            img_points_reprojected, _ = cv2.projectPoints(
                self.obj_points[i], rvecs[i], tvecs[i],
                camera_matrix, dist_coeffs
            )
            error = cv2.norm(self.img_points[i], img_points_reprojected, cv2.NORM_L2) / len(img_points_reprojected)
            total_error += error

        avg_error = total_error / len(self.obj_points)
        self.get_logger().info(f'Calibration finished with average error: {avg_error:.4f}')

        return camera_matrix, dist_coeffs, avg_error

    def save_calibration(self, camera_matrix, dist_coeffs, filename):
        """Save calibration parameters to file"""
        calibration_data = {
            'camera_matrix': camera_matrix.tolist(),
            'distortion_coefficients': dist_coeffs.tolist()
        }

        with open(filename, 'w') as f:
            yaml.dump(calibration_data, f)

        self.get_logger().info(f'Calibration saved to {filename}')
```

### LiDAR to Camera Calibration

```python
import numpy as np
import cv2
from scipy.optimize import minimize

class LidarCameraCalibrator:
    def __init__(self):
        self.extrinsics = np.eye(4)  # 4x4 transformation matrix

    def calibrate_lidar_camera(self, lidar_points, image_points, camera_matrix, dist_coeffs):
        """Calibrate LiDAR to camera extrinsics"""
        # Initial guess for extrinsics (identity matrix)
        initial_params = self.extrinsics.flatten()[:-1]  # Remove last element (should be [0,0,0,1])

        # Optimize extrinsics to minimize reprojection error
        result = minimize(
            self._calibration_error,
            initial_params,
            args=(lidar_points, image_points, camera_matrix, dist_coeffs),
            method='Powell'
        )

        # Reshape optimized parameters back to 4x4 matrix
        self.extrinsics = np.eye(4)
        self.extrinsics[:3, :4] = result.x.reshape(3, 4)

        return self.extrinsics

    def _calibration_error(self, params, lidar_points, image_points, camera_matrix, dist_coeffs):
        """Calculate reprojection error for optimization"""
        # Reshape parameters to 3x4 transformation matrix
        transform = params.reshape(3, 4)

        # Transform LiDAR points to camera frame
        lidar_homo = np.hstack([lidar_points, np.ones((len(lidar_points), 1))])
        camera_points = (transform @ lidar_homo.T).T

        # Project to image plane
        projected_points = camera_points[:, :2] / camera_points[:, 2:3]

        # Apply distortion
        r2 = projected_points[:, 0]**2 + projected_points[:, 1]**2
        # Simplified distortion model
        distorted_points = projected_points * (1 + dist_coeffs[0] * r2)

        # Apply camera intrinsic matrix
        image_points_projected = (camera_matrix[:2, :2] @ distorted_points.T +
                                 camera_matrix[:2, 2:3]).T

        # Calculate error
        error = np.sum((image_points - image_points_projected)**2)
        return error

    def transform_lidar_to_camera(self, lidar_points):
        """Transform LiDAR points to camera coordinate frame"""
        lidar_homo = np.hstack([lidar_points, np.ones((len(lidar_points), 1))])
        camera_points = (self.extrinsics @ lidar_homo.T).T
        return camera_points[:, :3]
```

## 7.6 Data Synchronization and Time Alignment

### Sensor Synchronization

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan
from message_filters import ApproximateTimeSynchronizer, Subscriber
import threading
from collections import deque

class SensorSynchronizerNode(Node):
    def __init__(self):
        super().__init__('sensor_synchronizer')

        # Create subscribers for different sensors
        self.image_sub = Subscriber(self, Image, 'camera/image_raw')
        self.imu_sub = Subscriber(self, Imu, 'imu/data')
        self.scan_sub = Subscriber(self, LaserScan, 'scan')

        # Synchronize messages with approximate time sync
        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.imu_sub, self.scan_sub],
            queue_size=10,
            slop=0.1  # 100ms tolerance
        )
        self.sync.registerCallback(self.synchronized_callback)

        # Buffer for unsynchronized data
        self.unsync_buffer = {
            'image': deque(maxlen=100),
            'imu': deque(maxlen=1000),
            'scan': deque(maxlen=100)
        }

        # Lock for thread safety
        self.buffer_lock = threading.Lock()

    def synchronized_callback(self, image_msg, imu_msg, scan_msg):
        """Callback for synchronized sensor data"""
        self.get_logger().info(f'Synchronized data: Image {image_msg.header.stamp}, '
                              f'IMU {imu_msg.header.stamp}, '
                              f'Scan {scan_msg.header.stamp}')

        # Process synchronized data
        self.process_fusion(image_msg, imu_msg, scan_msg)

    def process_fusion(self, image_msg, imu_msg, scan_msg):
        """Process fused sensor data"""
        # Example: Combine visual and LiDAR data with IMU for pose estimation
        # This would implement a sensor fusion algorithm

        # Extract data
        image_time = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
        imu_time = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9
        scan_time = scan_msg.header.stamp.sec + scan_msg.header.stamp.nanosec * 1e-9

        # Perform fusion calculations
        # (Implementation would depend on specific application)

        self.get_logger().info(f'Fusion completed at {image_time:.3f}s')

    def image_callback(self, msg):
        """Handle unsynchronized image data"""
        with self.buffer_lock:
            self.unsync_buffer['image'].append(msg)

    def imu_callback(self, msg):
        """Handle unsynchronized IMU data"""
        with self.buffer_lock:
            self.unsync_buffer['imu'].append(msg)

    def scan_callback(self, msg):
        """Handle unsynchronized scan data"""
        with self.buffer_lock:
            self.unsync_buffer['scan'].append(msg)
```

## 7.7 Perception Pipeline Implementation

### Multi-Sensor Perception Pipeline

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, PointCloud2, Imu
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import numpy as np
import cv2
from cv_bridge import CvBridge

class PerceptionPipelineNode(Node):
    def __init__(self):
        super().__init__('perception_pipeline')

        # Initialize components
        self.bridge = CvBridge()
        self.current_image = None
        self.current_scan = None
        self.current_imu = None
        self.fusion_result = None

        # Publishers
        self.object_pub = self.create_publisher(String, 'detected_objects', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'estimated_pose', 10)

        # Subscribers
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)

        # Timer for processing pipeline
        self.process_timer = self.create_timer(0.1, self.process_perception_pipeline)

    def image_callback(self, msg):
        """Process incoming image data"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')

    def scan_callback(self, msg):
        """Process incoming scan data"""
        self.current_scan = msg

    def imu_callback(self, msg):
        """Process incoming IMU data"""
        self.current_imu = msg

    def process_perception_pipeline(self):
        """Main perception pipeline processing"""
        if not all([self.current_image, self.current_scan, self.current_imu]):
            return

        # Step 1: Visual object detection
        visual_objects = self.detect_visual_objects(self.current_image)

        # Step 2: LiDAR object detection
        lidar_objects = self.detect_lidar_objects(self.current_scan)

        # Step 3: Sensor fusion
        fused_objects = self.fuse_sensor_data(visual_objects, lidar_objects, self.current_imu)

        # Step 4: Pose estimation
        estimated_pose = self.estimate_pose(fused_objects)

        # Step 5: Publish results
        self.publish_results(fused_objects, estimated_pose)

    def detect_visual_objects(self, image):
        """Detect objects in image using computer vision"""
        # Simple color-based object detection (example)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define color ranges for detection (example: red objects)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])

        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        objects = []
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Filter small objects
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w/2
                center_y = y + h/2
                objects.append({
                    'type': 'red_object',
                    'center': (center_x, center_y),
                    'size': (w, h),
                    'confidence': 0.8
                })

        return objects

    def detect_lidar_objects(self, scan):
        """Detect objects in LiDAR scan"""
        # Simple clustering-based object detection
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        valid_indices = [i for i, r in enumerate(scan.ranges) if scan.range_min <= r <= scan.range_max]

        if len(valid_indices) < 2:
            return []

        # Convert to Cartesian coordinates
        points = []
        for i in valid_indices:
            angle = angles[i]
            range_val = scan.ranges[i]
            x = range_val * np.cos(angle)
            y = range_val * np.sin(angle)
            points.append([x, y])

        if len(points) < 2:
            return []

        # Simple clustering (DBSCAN-like)
        clusters = self.cluster_lidar_points(np.array(points))

        objects = []
        for cluster_id in set(clusters):
            if cluster_id == -1:  # Noise points
                continue

            cluster_points = np.array(points)[clusters == cluster_id]
            center = np.mean(cluster_points, axis=0)
            size = np.std(cluster_points, axis=0)

            objects.append({
                'type': 'lidar_object',
                'center': center,
                'size': size,
                'confidence': 0.7
            })

        return objects

    def cluster_lidar_points(self, points):
        """Simple clustering for LiDAR points"""
        if len(points) < 2:
            return np.array([])

        # Use DBSCAN-like approach
        labels = np.full(len(points), -1)
        cluster_id = 0

        for i, point in enumerate(points):
            if labels[i] != -1:
                continue

            # Find neighbors within radius
            distances = np.linalg.norm(points - point, axis=1)
            neighbors = np.where(distances < 0.5)[0]  # 50cm radius

            if len(neighbors) >= 3:  # Minimum points for cluster
                labels[neighbors] = cluster_id
                cluster_id += 1

        return labels

    def fuse_sensor_data(self, visual_objects, lidar_objects, imu_data):
        """Fuse data from multiple sensors"""
        # Convert visual objects to world coordinates using camera calibration
        # Convert LiDAR objects to world coordinates
        # Combine with IMU orientation data

        fused_objects = []

        # Example: Associate visual and LiDAR objects based on position
        for v_obj in visual_objects:
            # Convert image coordinates to world coordinates (simplified)
            v_world = self.image_to_world(v_obj['center'])

            # Find corresponding LiDAR object
            best_match = None
            best_distance = float('inf')

            for l_obj in lidar_objects:
                distance = np.linalg.norm(np.array(v_world[:2]) - l_obj['center'])
                if distance < 1.0 and distance < best_distance:  # 1m threshold
                    best_match = l_obj
                    best_distance = distance

            if best_match:
                # Fuse the objects
                fused_obj = {
                    'type': 'fused_object',
                    'position': l_obj['center'],
                    'confidence': (v_obj['confidence'] + l_obj['confidence']) / 2,
                    'visual_features': v_obj,
                    'lidar_features': l_obj
                }
                fused_objects.append(fused_obj)

        return fused_objects

    def image_to_world(self, image_coords):
        """Convert image coordinates to world coordinates (simplified)"""
        # This would use camera calibration parameters in practice
        # For now, return a simplified conversion
        x = (image_coords[0] - 320) * 0.01  # Approximate conversion
        y = (image_coords[1] - 240) * 0.01
        z = 1.0  # Assume 1m distance
        return [x, y, z]

    def estimate_pose(self, fused_objects):
        """Estimate robot pose based on detected objects"""
        # Example: Use detected objects to estimate position relative to known landmarks
        # This would implement a localization algorithm
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'

        # Simplified pose estimation
        if fused_objects:
            # Use first object as reference
            obj = fused_objects[0]
            pose.pose.position.x = obj['position'][0]
            pose.pose.position.y = obj['position'][1]
            pose.pose.position.z = 0.0

        return pose

    def publish_results(self, fused_objects, estimated_pose):
        """Publish perception results"""
        # Publish detected objects
        if fused_objects:
            obj_msg = String()
            obj_msg.data = f'Detected {len(fused_objects)} objects'
            self.object_pub.publish(obj_msg)

        # Publish estimated pose
        self.pose_pub.publish(estimated_pose)

def main(args=None):
    rclpy.init(args=args)
    perception_node = PerceptionPipelineNode()

    try:
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        pass
    finally:
        perception_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

Sensor integration and perception systems form the foundation of robotic awareness and decision-making. Understanding how to properly integrate various sensor types, implement perception pipelines, and handle data synchronization is crucial for building robust robotic systems. Proper calibration procedures ensure accurate sensor data, while effective fusion algorithms combine information from multiple sensors to create a comprehensive understanding of the environment.

## Next Steps

In the next section, we'll explore control systems and trajectory planning, building on the perception capabilities developed here to enable robot navigation and manipulation.

<div class="alert alert-warning">
  <h5>Hardware Requirement</h5>
  <div><strong>Requirement:</strong> GPU</div>
  <div><strong>Minimum:</strong> RTX 4070</div>
  <div><strong>Recommended:</strong> RTX 4080</div>
  <div><strong>Purpose:</strong> Sensor processing and perception algorithms require significant computational resources</div>
</div>