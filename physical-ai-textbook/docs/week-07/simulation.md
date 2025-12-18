---
title: "Week 7: Sensor Integration and Perception Systems Simulation"
week: 7
module: "Robotic Infrastructure"
difficulty: "advanced"
prerequisites: ["ros2-fundamentals", "python-basics", "robotics-concepts", "sensor-basics"]
learning_objectives:
  - "Simulate various robotic sensors"
  - "Implement perception algorithms in simulation"
  - "Test sensor fusion in virtual environments"
  - "Validate sensor data quality"
tags: ["sensors", "simulation", "perception", "camera", "lidar", "imu", "fusion"]
hardware_requirements:
  - gpu: "RTX 4070 or higher"
  - ram: "16GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "120 minutes"
---

# Week 7: Sensor Integration and Perception Systems Simulation

## Learning Objectives
- Simulate various robotic sensors
- Implement perception algorithms in simulation
- Test sensor fusion in virtual environments
- Validate sensor data quality

## 7.1 Sensor Simulation in Virtual Environments

### Camera Sensor Simulation

Let's create a comprehensive camera sensor simulation:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Header
import threading
import time

class CameraSensorSimulator(Node):
    def __init__(self):
        super().__init__('camera_sensor_simulator')

        # Create CV bridge
        self.bridge = CvBridge()

        # Publishers
        self.image_pub = self.create_publisher(Image, 'simulated_camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'simulated_camera/camera_info', 10)

        # Camera parameters
        self.width = 640
        self.height = 480
        self.fps = 30
        self.focal_length = 615.0
        self.principal_point = (320, 240)

        # Noise parameters
        self.gaussian_noise_std = 0.005
        self.poisson_lambda = 0.01
        self.uniform_range = 0.001

        # Scene parameters
        self.scene_elements = [
            {'type': 'circle', 'center': (200, 200), 'radius': 50, 'color': (255, 0, 0)},
            {'type': 'rectangle', 'top_left': (300, 150), 'bottom_right': (400, 250), 'color': (0, 255, 0)},
            {'type': 'line', 'start': (100, 300), 'end': (500, 300), 'color': (0, 0, 255)}
        ]

        # Timer for image generation
        self.timer = self.create_timer(1.0/self.fps, self.generate_image)

        # Threading for realistic timing
        self.lock = threading.Lock()

        self.get_logger().info('Camera sensor simulator initialized')

    def generate_image(self):
        """Generate synthetic camera image with realistic noise"""
        with self.lock:
            # Create synthetic image
            image = self.create_synthetic_scene()

            # Add realistic noise
            noisy_image = self.add_sensor_noise(image)

            # Convert to ROS message
            image_msg = self.bridge.cv2_to_imgmsg(noisy_image, encoding='bgr8')
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'camera_optical_frame'

            # Publish image
            self.image_pub.publish(image_msg)

            # Publish camera info
            self.publish_camera_info()

    def create_synthetic_scene(self):
        """Create a synthetic scene with various elements"""
        # Create blank image
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Add scene elements
        for element in self.scene_elements:
            if element['type'] == 'circle':
                cv2.circle(image, element['center'], element['radius'], element['color'], -1)
            elif element['type'] == 'rectangle':
                cv2.rectangle(image, element['top_left'], element['bottom_right'], element['color'], -1)
            elif element['type'] == 'line':
                cv2.line(image, element['start'], element['end'], element['color'], 2)

        # Add some dynamic elements (moving objects)
        moving_x = int(320 + 100 * np.sin(time.time() * 0.5))
        cv2.circle(image, (moving_x, 100), 30, (255, 255, 0), -1)

        return image

    def add_sensor_noise(self, image):
        """Add realistic sensor noise to image"""
        # Convert to float for processing
        float_image = image.astype(np.float32)

        # Add Gaussian noise
        gaussian_noise = np.random.normal(0, self.gaussian_noise_std * 255, image.shape)
        noisy_image = float_image + gaussian_noise

        # Add Poisson noise (photon noise)
        poisson_noise = np.random.poisson(self.poisson_lambda * float_image) - self.poisson_lambda * float_image
        noisy_image = noisy_image + poisson_noise

        # Add uniform noise
        uniform_noise = np.random.uniform(-self.uniform_range * 255, self.uniform_range * 255, image.shape)
        noisy_image = noisy_image + uniform_noise

        # Ensure values are in valid range
        noisy_image = np.clip(noisy_image, 0, 255)

        return noisy_image.astype(np.uint8)

    def publish_camera_info(self):
        """Publish camera calibration information"""
        info_msg = CameraInfo()
        info_msg.header.stamp = self.get_clock().now().to_msg()
        info_msg.header.frame_id = 'camera_optical_frame'
        info_msg.width = self.width
        info_msg.height = self.height

        # Camera matrix
        info_msg.k = [
            self.focal_length, 0.0, self.principal_point[0],
            0.0, self.focal_length, self.principal_point[1],
            0.0, 0.0, 1.0
        ]

        # Distortion coefficients (simplified)
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion for simplicity

        # Projection matrix
        info_msg.p = [
            self.focal_length, 0.0, self.principal_point[0], 0.0,
            0.0, self.focal_length, self.principal_point[1], 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        self.info_pub.publish(info_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraSensorSimulator()

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

### LiDAR Sensor Simulation

Now let's create a LiDAR sensor simulation:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

class LidarSensorSimulator(Node):
    def __init__(self):
        super().__init__('lidar_sensor_simulator')

        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, 'simulated_scan', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, 'simulated_point_cloud', 10)

        # LiDAR parameters
        self.angle_min = -np.pi
        self.angle_max = np.pi
        self.angle_increment = np.pi / 180  # 1 degree resolution
        self.range_min = 0.1
        self.range_max = 30.0
        self.scan_time = 0.1  # 10 Hz
        self.time_increment = 0.0  # No time per beam

        # Noise parameters
        self.range_noise_std = 0.01  # 1cm standard deviation
        self.angular_noise_std = 0.001  # ~0.057 degrees

        # Scene definition (simple environment with obstacles)
        self.obstacles = [
            # Wall at x=2m
            {'type': 'wall', 'position': (2.0, 0.0), 'size': (0.1, 5.0)},
            # Circular obstacle at (4, 1)
            {'type': 'circle', 'position': (4.0, 1.0), 'radius': 0.5},
            # Rectangular obstacle at (3, -1)
            {'type': 'rectangle', 'position': (3.0, -1.0), 'size': (0.8, 0.6)}
        ]

        # Timer for LiDAR simulation
        self.timer = self.create_timer(self.scan_time, self.generate_scan)

        self.get_logger().info('LiDAR sensor simulator initialized')

    def generate_scan(self):
        """Generate simulated LiDAR scan data"""
        # Calculate number of points
        num_points = int((self.angle_max - self.angle_min) / self.angle_increment) + 1

        # Calculate angles
        angles = np.linspace(self.angle_min, self.angle_max, num_points)

        # Calculate ranges to obstacles
        ranges = np.full(num_points, self.range_max)

        for i, angle in enumerate(angles):
            # Calculate ray direction
            ray_x = np.cos(angle)
            ray_y = np.sin(angle)

            # Check intersection with obstacles
            min_range = self.range_max
            for obstacle in self.obstacles:
                if obstacle['type'] == 'circle':
                    range_to_obstacle = self.intersect_circle(
                        (0, 0), (ray_x, ray_y), obstacle['position'], obstacle['radius']
                    )
                elif obstacle['type'] == 'wall':
                    range_to_obstacle = self.intersect_rectangle(
                        (0, 0), (ray_x, ray_y), obstacle['position'], obstacle['size']
                    )
                elif obstacle['type'] == 'rectangle':
                    range_to_obstacle = self.intersect_rectangle(
                        (0, 0), (ray_x, ray_y), obstacle['position'], obstacle['size']
                    )
                else:
                    continue

                if range_to_obstacle is not None and range_to_obstacle < min_range:
                    min_range = range_to_obstacle

            ranges[i] = min_range

        # Add noise to ranges
        noise = np.random.normal(0, self.range_noise_std, len(ranges))
        noisy_ranges = np.maximum(self.range_min, ranges + noise)

        # Create LaserScan message
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser_frame'
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.time_increment = self.time_increment
        scan_msg.scan_time = self.scan_time
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max
        scan_msg.ranges = noisy_ranges.tolist()

        # Publish scan
        self.scan_pub.publish(scan_msg)

        # Also publish as point cloud
        self.publish_point_cloud(angles, noisy_ranges)

    def intersect_circle(self, ray_start, ray_dir, circle_center, radius):
        """Calculate intersection of ray with circle"""
        # Ray equation: P = ray_start + t * ray_dir
        # Circle equation: ||P - circle_center|| = radius

        # Vector from ray start to circle center
        oc = np.array(circle_center) - np.array(ray_start)

        # Quadratic equation coefficients: at^2 + bt + c = 0
        a = np.dot(ray_dir, ray_dir)
        b = 2 * np.dot(oc, ray_dir)
        c = np.dot(oc, oc) - radius**2

        discriminant = b**2 - 4*a*c

        if discriminant < 0:
            return None  # No intersection

        sqrt_discriminant = np.sqrt(discriminant)
        t1 = (-b - sqrt_discriminant) / (2*a)
        t2 = (-b + sqrt_discriminant) / (2*a)

        # Return the closest positive intersection
        if t1 > 0:
            return t1
        elif t2 > 0:
            return t2
        else:
            return None

    def intersect_rectangle(self, ray_start, ray_dir, rect_center, rect_size):
        """Calculate intersection of ray with rectangle"""
        # Rectangle boundaries
        half_w, half_h = rect_size[0] / 2, rect_size[1] / 2
        min_x, max_x = rect_center[0] - half_w, rect_center[0] + half_w
        min_y, max_y = rect_center[1] - half_h, rect_center[1] + half_h

        # Calculate intersection with rectangle boundaries
        t_min = float('-inf')
        t_max = float('inf')

        # Check intersection with x boundaries
        if ray_dir[0] != 0:
            t1 = (min_x - ray_start[0]) / ray_dir[0]
            t2 = (max_x - ray_start[0]) / ray_dir[0]

            t_near = min(t1, t2)
            t_far = max(t1, t2)

            if t_near > t_min:
                t_min = t_near
            if t_far < t_max:
                t_max = t_far

            if t_min > t_max or t_max < 0:
                return None

        # Check intersection with y boundaries
        if ray_dir[1] != 0:
            t1 = (min_y - ray_start[1]) / ray_dir[1]
            t2 = (max_y - ray_start[1]) / ray_dir[1]

            t_near = min(t1, t2)
            t_far = max(t1, t2)

            if t_near > t_min:
                t_min = t_near
            if t_far < t_max:
                t_max = t_far

            if t_min > t_max or t_max < 0:
                return None

        if t_min >= 0:
            return t_min
        elif t_max >= 0:
            return t_max
        else:
            return None

    def publish_point_cloud(self, angles, ranges):
        """Convert scan data to point cloud and publish"""
        points = []
        for angle, range_val in zip(angles, ranges):
            if self.range_min <= range_val <= self.range_max:
                x = range_val * np.cos(angle)
                y = range_val * np.sin(angle)
                z = 0.0  # 2D scan
                points.append([x, y, z])

        # Create PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'laser_frame'

        fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1)
        ]

        cloud_msg = pc2.create_cloud(header, fields, points)
        self.cloud_pub.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarSensorSimulator()

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

### IMU Sensor Simulation

Let's create an IMU sensor simulation:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Header
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

class ImuSensorSimulator(Node):
    def __init__(self):
        super().__init__('imu_sensor_simulator')

        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'simulated_imu', 10)

        # IMU parameters
        self.accel_noise_std = 0.01  # 0.01 m/s²
        self.gyro_noise_std = 0.001  # 0.001 rad/s
        self.mag_noise_std = 0.1     # 0.1 µT

        # Bias parameters
        self.accel_bias = np.random.normal(0, 0.005, 3)  # Small initial bias
        self.gyro_bias = np.random.normal(0, 0.0005, 3)  # Small initial bias

        # True values (will be simulated)
        self.true_orientation = R.from_quat([0, 0, 0, 1])  # Identity
        self.true_angular_velocity = np.array([0.0, 0.0, 0.0])
        self.true_linear_acceleration = np.array([0.0, 0.0, -9.81])  # Gravity

        # Drift parameters
        self.bias_drift_rate = 1e-6  # Very slow drift

        # Timer for IMU simulation
        self.timer = self.create_timer(0.01, self.generate_imu_data)  # 100 Hz

        self.get_logger().info('IMU sensor simulator initialized')

    def generate_imu_data(self):
        """Generate simulated IMU data with realistic noise and bias"""
        current_time = time.time()

        # Update true motion (simple simulation)
        self.simulate_true_motion()

        # Add bias and noise to true values
        measured_accel = self.add_imu_noise(
            self.true_linear_acceleration,
            self.accel_noise_std,
            self.accel_bias,
            current_time
        )

        measured_gyro = self.add_imu_noise(
            self.true_angular_velocity,
            self.gyro_noise_std,
            self.gyro_bias,
            current_time
        )

        # Magnetic field (Earth's magnetic field, simplified)
        true_mag = np.array([25.0, 0.0, -45.0])  # Approximate Earth's field
        measured_mag = self.add_imu_noise(
            true_mag,
            self.mag_noise_std,
            np.zeros(3),  # No bias for magnetometer
            current_time
        )

        # Create IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Set orientation (if available from simulation)
        quat = self.true_orientation.as_quat()
        imu_msg.orientation.x = float(quat[0])
        imu_msg.orientation.y = float(quat[1])
        imu_msg.orientation.z = float(quat[2])
        imu_msg.orientation.w = float(quat[3])

        # Set angular velocity
        imu_msg.angular_velocity.x = float(measured_gyro[0])
        imu_msg.angular_velocity.y = float(measured_gyro[1])
        imu_msg.angular_velocity.z = float(measured_gyro[2])

        # Set linear acceleration
        imu_msg.linear_acceleration.x = float(measured_accel[0])
        imu_msg.linear_acceleration.y = float(measured_accel[1])
        imu_msg.linear_acceleration.z = float(measured_accel[2])

        # Set covariance (indicates uncertainty)
        # Acceleration covariance
        imu_msg.linear_acceleration_covariance[0] = self.accel_noise_std**2
        imu_msg.linear_acceleration_covariance[4] = self.accel_noise_std**2
        imu_msg.linear_acceleration_covariance[8] = self.accel_noise_std**2

        # Angular velocity covariance
        imu_msg.angular_velocity_covariance[0] = self.gyro_noise_std**2
        imu_msg.angular_velocity_covariance[4] = self.gyro_noise_std**2
        imu_msg.angular_velocity_covariance[8] = self.gyro_noise_std**2

        # Orientation covariance (unknown initially)
        imu_msg.orientation_covariance[0] = -1  # Indicates unknown

        # Publish IMU data
        self.imu_pub.publish(imu_msg)

    def add_imu_noise(self, true_value, noise_std, bias, timestamp):
        """Add realistic noise and bias to IMU measurements"""
        # Add white noise
        noise = np.random.normal(0, noise_std, 3)

        # Add bias (with slow drift)
        drift = np.random.normal(0, self.bias_drift_rate, 3)
        bias = bias + drift

        # Apply noise and bias
        measured_value = true_value + bias + noise

        return measured_value

    def simulate_true_motion(self):
        """Simulate realistic robot motion for testing"""
        current_time = time.time()

        # Simple motion simulation: oscillating motion
        amplitude = 0.1
        frequency = 0.5  # Hz

        # Update angular velocity (oscillating)
        self.true_angular_velocity[2] = amplitude * np.sin(2 * np.pi * frequency * current_time)

        # Update orientation by integrating angular velocity
        dt = 0.01  # 100 Hz
        omega_norm = np.linalg.norm(self.true_angular_velocity)
        if omega_norm > 1e-6:  # Avoid division by zero
            axis = self.true_angular_velocity / omega_norm
            angle = omega_norm * dt
            delta_rotation = R.from_rotvec(axis * angle)
            self.true_orientation = self.true_orientation * delta_rotation

        # Update linear acceleration (with gravity and motion)
        # For simplicity, we'll keep it constant for now
        # In a real simulation, this would depend on robot motion
        self.true_linear_acceleration = np.array([
            0.1 * np.sin(2 * np.pi * 0.3 * current_time),  # Small x acceleration
            0.05 * np.cos(2 * np.pi * 0.4 * current_time), # Small y acceleration
            -9.81  # Gravity (z-axis)
        ])

def main(args=None):
    rclpy.init(args=args)
    node = ImuSensorSimulator()

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

## 7.2 Perception Algorithm Simulation

### Feature Detection and Matching Simulation

Let's implement simulated perception algorithms:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String, Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque

class PerceptionSimulationNode(Node):
    def __init__(self):
        super().__init__('perception_simulation_node')

        # Create CV bridge
        self.bridge = CvBridge()

        # Publishers
        self.feature_pub = self.create_publisher(String, 'simulated_features', 10)
        self.object_pub = self.create_publisher(String, 'simulated_objects', 10)
        self.match_pub = self.create_publisher(Float64MultiArray, 'simulated_matches', 10)

        # Subscribers
        self.image_sub = self.create_subscription(Image, 'simulated_camera/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'simulated_scan', self.scan_callback, 10)

        # Perception parameters
        self.feature_detector = cv2.SIFT_create(nfeatures=500)
        self.descriptor_matcher = cv2.BFMatcher()
        self.min_match_count = 10

        # Tracking buffers
        self.previous_features = None
        self.feature_tracks = {}
        self.track_id_counter = 0

        # Timer for processing
        self.process_timer = self.create_timer(0.1, self.process_perception)

        self.get_logger().info('Perception simulation node initialized')

    def image_callback(self, msg):
        """Process simulated camera images"""
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Detect features
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            keypoints, descriptors = self.feature_detector.detectAndCompute(gray, None)

            # Track features
            if self.previous_features is not None:
                matches = self.descriptor_matcher.knnMatch(
                    self.previous_features[1], descriptors, k=2
                )

                # Apply Lowe's ratio test
                good_matches = []
                for match_pair in matches:
                    if len(match_pair) == 2:
                        m, n = match_pair
                        if m.distance < 0.7 * n.distance:
                            good_matches.append(m)

                # Publish match information
                if len(good_matches) > 0:
                    match_msg = Float64MultiArray()
                    match_msg.data = [len(good_matches), len(keypoints), len(descriptors)]
                    self.match_pub.publish(match_msg)

            # Store current features for next iteration
            self.previous_features = (keypoints, descriptors)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def scan_callback(self, msg):
        """Process simulated LiDAR scan"""
        # Simulate object detection from LiDAR
        valid_ranges = [
            (i, r) for i, r in enumerate(msg.ranges)
            if msg.range_min <= r <= msg.range_max
        ]

        if len(valid_ranges) > 10:  # Enough points for processing
            # Cluster points to detect objects
            clusters = self.cluster_lidar_points(valid_ranges, msg)

            # Publish object detection results
            obj_msg = String()
            obj_msg.data = f'Detected {len(clusters)} objects from LiDAR'
            self.object_pub.publish(obj_msg)

    def cluster_lidar_points(self, valid_ranges, scan_msg):
        """Cluster LiDAR points to detect objects"""
        # Convert valid ranges to Cartesian coordinates
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges))
        points = []

        for idx, range_val in valid_ranges:
            angle = angles[idx]
            x = range_val * np.cos(angle)
            y = range_val * np.sin(angle)
            points.append([x, y])

        if len(points) < 2:
            return []

        # Simple clustering using DBSCAN-like approach
        points = np.array(points)
        labels = self.simple_dbscan_clustering(points, eps=0.5, min_samples=5)

        clusters = []
        for label in set(labels):
            if label == -1:  # Noise points
                continue

            cluster_points = points[labels == label]
            center = np.mean(cluster_points, axis=0)
            size = np.std(cluster_points, axis=0)

            clusters.append({
                'center': center,
                'size': size,
                'points': len(cluster_points)
            })

        return clusters

    def simple_dbscan_clustering(self, points, eps=0.5, min_samples=5):
        """Simple DBSCAN clustering implementation"""
        if len(points) < min_samples:
            return np.full(len(points), -1)

        labels = np.full(len(points), -1)
        cluster_id = 0

        for i, point in enumerate(points):
            if labels[i] != -1:  # Already assigned
                continue

            # Find neighbors within epsilon
            distances = np.linalg.norm(points - point, axis=1)
            neighbors = np.where(distances < eps)[0]

            if len(neighbors) >= min_samples:
                # Expand cluster
                seeds = set(neighbors)
                while seeds:
                    current_seed = seeds.pop()
                    if labels[current_seed] == -1:  # Unassigned
                        labels[current_seed] = cluster_id

                    # Find neighbors of current seed
                    seed_distances = np.linalg.norm(points - points[current_seed], axis=1)
                    seed_neighbors = np.where(seed_distances < eps)[0]

                    for neighbor in seed_neighbors:
                        if labels[neighbor] == -1:  # Unassigned
                            labels[neighbor] = cluster_id
                            if neighbor not in [current_seed]:  # Don't add current seed
                                seeds.add(neighbor)

                cluster_id += 1

        return labels

    def process_perception(self):
        """Process perception data and publish results"""
        # This would implement the main perception pipeline
        # For simulation, we'll just publish status updates
        status_msg = String()
        status_msg.data = 'Perception simulation running'
        self.feature_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionSimulationNode()

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

## 7.3 Sensor Fusion Simulation

Let's implement sensor fusion in simulation:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, String
import numpy as np
from scipy.spatial.transform import Rotation as R
import threading
from collections import deque

class SensorFusionSimulator(Node):
    def __init__(self):
        super().__init__('sensor_fusion_simulator')

        # Publishers
        self.fused_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'fused_pose', 10)
        self.fused_twist_pub = self.create_publisher(TwistWithCovarianceStamped, 'fused_twist', 10)
        self.odom_pub = self.create_publisher(Odometry, 'fused_odom', 10)
        self.status_pub = self.create_publisher(String, 'fusion_status', 10)

        # Subscribers
        self.camera_sub = self.create_subscription(Image, 'simulated_camera/image_raw', self.camera_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, 'simulated_scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'simulated_imu', self.imu_callback, 10)

        # State estimation
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.orientation = R.from_quat([0, 0, 0, 1])
        self.angular_velocity = np.array([0.0, 0.0, 0.0])

        # Covariance matrices
        self.position_covariance = np.eye(3) * 0.1
        self.orientation_covariance = np.eye(3) * 0.01
        self.velocity_covariance = np.eye(3) * 0.05

        # Sensor buffers
        self.imu_buffer = deque(maxlen=10)
        self.lidar_buffer = deque(maxlen=5)
        self.camera_buffer = deque(maxlen=5)

        # Fusion weights
        self.imu_weight = 0.6
        self.lidar_weight = 0.3
        self.camera_weight = 0.1

        # Kalman filter parameters
        self.process_noise = 0.1
        self.measurement_noise = 0.05

        # State lock
        self.state_lock = threading.Lock()

        # Timer for fusion
        self.fusion_timer = self.create_timer(0.05, self.perform_fusion)  # 20 Hz

        self.get_logger().info('Sensor fusion simulator initialized')

    def camera_callback(self, msg):
        """Process camera data"""
        with self.state_lock:
            # Extract features from camera for position estimation
            # This would implement visual odometry or SLAM
            # For simulation, we'll add some visual cues to position
            if len(self.camera_buffer) > 0:
                # Simple visual motion estimation
                self.velocity[0] += np.random.normal(0, 0.01)  # Add small visual motion cue

            self.camera_buffer.append(msg)

    def lidar_callback(self, msg):
        """Process LiDAR data"""
        with self.state_lock:
            # Extract landmarks from LiDAR for position estimation
            # This would implement LiDAR odometry or SLAM
            valid_ranges = [
                (i, r) for i, r in enumerate(msg.ranges)
                if msg.range_min <= r <= msg.range_max
            ]

            if valid_ranges:
                # Use nearest points for position correction
                min_range_idx = min(valid_ranges, key=lambda x: x[1])[0]
                angle = msg.angle_min + min_range_idx * msg.angle_increment

                # Estimate position based on landmark
                landmark_x = msg.ranges[min_range_idx] * np.cos(angle)
                landmark_y = msg.ranges[min_range_idx] * np.sin(angle)

                # Update position estimate
                self.position[0] = self.lidar_weight * landmark_x + (1 - self.lidar_weight) * self.position[0]
                self.position[1] = self.lidar_weight * landmark_y + (1 - self.lidar_weight) * self.position[1]

            self.lidar_buffer.append(msg)

    def imu_callback(self, msg):
        """Process IMU data"""
        with self.state_lock:
            # Extract orientation from IMU
            imu_quat = np.array([
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ])

            # Update orientation
            self.orientation = R.from_quat(imu_quat)

            # Extract angular velocity
            self.angular_velocity = np.array([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ])

            # Extract linear acceleration and integrate
            linear_acc = np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ])

            # Remove gravity from acceleration
            gravity_vec = np.array([0, 0, 9.81])
            gravity_rotated = self.orientation.apply(gravity_vec)
            corrected_acc = linear_acc - gravity_rotated

            # Integrate acceleration to get velocity
            dt = 0.01  # 100 Hz
            self.velocity += corrected_acc * dt

            # Integrate velocity to get position
            self.position += self.velocity * dt

            self.imu_buffer.append(msg)

    def perform_fusion(self):
        """Perform sensor fusion using weighted averaging"""
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
            twist_msg.twist.covariance[21:27] = np.eye(3).flatten() * 0.01  # Angular velocity covariance

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
            status_msg.data = f'Fusion: Pos=({self.position[0]:.2f},{self.position[1]:.2f}), Vel=({self.velocity[0]:.2f},{self.velocity[1]:.2f}), Tracks={len(self.lidar_buffer)}'
            self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionSimulator()

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

## 7.4 Calibration Simulation

Let's create a calibration simulation:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.optimize import minimize
import threading

class CalibrationSimulatorNode(Node):
    def __init__(self):
        super().__init__('calibration_simulator_node')

        # Create CV bridge
        self.bridge = CvBridge()

        # Publishers
        self.calibration_status_pub = self.create_publisher(Bool, 'calibration_status', 10)
        self.extrinsics_pub = self.create_publisher(Float64MultiArray, 'calibration_extrinsics', 10)

        # Subscribers
        self.camera_sub = self.create_subscription(Image, 'simulated_camera/image_raw', self.camera_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, 'simulated_scan', self.lidar_callback, 10)

        # Calibration parameters
        self.chessboard_pattern = (9, 6)
        self.obj_points = []  # 3D points in real world space
        self.img_points = []  # 2D points in image plane
        self.lidar_points = []  # 3D points from LiDAR

        # Camera calibration matrices
        self.camera_matrix = None
        self.dist_coeffs = None

        # LiDAR to camera extrinsics
        self.extrinsics = np.eye(4)

        # Calibration state
        self.calibration_samples = []
        self.calibration_in_progress = False
        self.calibration_completed = False

        # Processing lock
        self.calibration_lock = threading.Lock()

        # Timer for calibration
        self.calibration_timer = self.create_timer(0.1, self.calibration_loop)

        self.get_logger().info('Calibration simulator node initialized')

    def camera_callback(self, msg):
        """Process camera images for calibration"""
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Find chessboard corners
            ret, corners = self.find_chessboard_corners(image)

            if ret:
                # Prepare object points
                objp = np.zeros((self.chessboard_pattern[0] * self.chessboard_pattern[1], 3), np.float32)
                objp[:, :2] = np.mgrid[0:self.chessboard_pattern[0], 0:self.chessboard_pattern[1]].T.reshape(-1, 2)

                # Store for calibration
                with self.calibration_lock:
                    self.obj_points.append(objp)
                    self.img_points.append(corners)

                self.get_logger().info(f'Found chessboard corners, total samples: {len(self.obj_points)}')

        except Exception as e:
            self.get_logger().error(f'Error processing calibration image: {e}')

    def lidar_callback(self, msg):
        """Process LiDAR data for calibration"""
        # In real calibration, we would detect the same chessboard pattern in LiDAR
        # For simulation, we'll create corresponding LiDAR points
        with self.calibration_lock:
            # Simulate detecting the same pattern in LiDAR
            # This is a simplified approach - real calibration would be more complex
            if len(self.obj_points) > len(self.lidar_points):
                # Create corresponding LiDAR points for the pattern
                # This would involve detecting the same pattern in LiDAR data
                pattern_3d = self.obj_points[-1]  # Last pattern
                # Add some simulated LiDAR points corresponding to the pattern
                simulated_lidar_points = self.simulate_lidar_pattern(pattern_3d)
                self.lidar_points.append(simulated_lidar_points)

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

    def simulate_lidar_pattern(self, pattern_3d):
        """Simulate LiDAR points corresponding to the pattern"""
        # This simulates what the LiDAR would see for the same pattern
        # In reality, this would involve detecting the same pattern in LiDAR data
        lidar_points = []

        # Simulate LiDAR points near the pattern locations
        for pt in pattern_3d:
            # Add some noise to simulate real LiDAR measurements
            noise = np.random.normal(0, 0.01, 3)
            lidar_point = pt + noise
            lidar_points.append(lidar_point)

        return np.array(lidar_points)

    def start_calibration(self):
        """Start the calibration process"""
        with self.calibration_lock:
            self.calibration_in_progress = True
            self.obj_points = []
            self.img_points = []
            self.lidar_points = []
            self.get_logger().info('Calibration started - collecting samples...')

    def stop_calibration(self):
        """Stop the calibration process"""
        with self.calibration_lock:
            self.calibration_in_progress = False
            if len(self.obj_points) >= 10:
                self.perform_calibration()
            self.get_logger().info('Calibration stopped')

    def perform_calibration(self):
        """Perform camera and sensor calibration"""
        try:
            if len(self.obj_points) >= 10:
                # Camera calibration
                ret, self.camera_matrix, self.dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                    self.obj_points, self.img_points,
                    (640, 480), None, None
                )

                if ret:
                    self.get_logger().info('Camera calibration completed successfully')

                    # Save camera calibration
                    self.save_camera_calibration()

                    # Perform LiDAR to camera calibration
                    self.calibrate_lidar_camera()

                    # Publish completion
                    status_msg = Bool()
                    status_msg.data = True
                    self.calibration_status_pub.publish(status_msg)

                    self.calibration_completed = True
                    self.get_logger().info('Full calibration completed')
                else:
                    self.get_logger().error('Camera calibration failed')
            else:
                self.get_logger().warn('Insufficient calibration samples')

        except Exception as e:
            self.get_logger().error(f'Calibration error: {e}')

    def calibrate_lidar_camera(self):
        """Calibrate LiDAR to camera extrinsics"""
        if not self.camera_matrix or not self.dist_coeffs:
            self.get_logger().error('Camera calibration required first')
            return

        # Initialize extrinsics matrix
        initial_extrinsics = np.eye(4).flatten()[:-1]  # Remove last element [0,0,0,1]

        # Define objective function for optimization
        def objective_function(params):
            # Reshape parameters to 3x4 transformation matrix
            transform = params.reshape(3, 4)

            total_error = 0.0

            # For each calibration sample
            for i in range(min(len(self.obj_points), len(self.lidar_points))):
                obj_pts = self.obj_points[i]
                img_pts = self.img_points[i]
                lidar_pts = self.lidar_points[i]

                # Transform LiDAR points to camera frame
                lidar_homo = np.hstack([lidar_pts, np.ones((len(lidar_pts), 1))])
                camera_pts = (transform @ lidar_homo.T).T

                # Project to image plane
                if len(camera_pts) > 0 and camera_pts[0, 2] != 0:  # Avoid division by zero
                    u = camera_pts[:, 0] / camera_pts[:, 2]  # x/z
                    v = camera_pts[:, 1] / camera_pts[:, 2]  # y/z

                    # Apply camera intrinsic matrix
                    projected_u = self.camera_matrix[0, 0] * u + self.camera_matrix[0, 2]
                    projected_v = self.camera_matrix[1, 1] * v + self.camera_matrix[1, 2]

                    # Calculate reprojection error
                    if len(projected_u) == len(img_pts):
                        error_u = (projected_u - img_pts[:, 0, 0])**2
                        error_v = (projected_v - img_pts[:, 0, 1])**2
                        total_error += np.sum(error_u + error_v)

            return total_error

        # Perform optimization
        result = minimize(
            objective_function,
            initial_extrinsics,
            method='Powell'
        )

        # Reshape result to 4x4 matrix
        self.extrinsics = np.eye(4)
        self.extrinsics[:3, :4] = result.x.reshape(3, 4)

        self.get_logger().info('LiDAR to camera calibration completed')

        # Publish extrinsics
        extrinsics_msg = Float64MultiArray()
        extrinsics_msg.data = self.extrinsics.flatten().tolist()
        self.extrinsics_pub.publish(extrinsics_msg)

        # Save extrinsics
        self.save_extrinsics()

    def save_camera_calibration(self):
        """Save camera calibration parameters"""
        import yaml
        calibration_data = {
            'camera_matrix': self.camera_matrix.tolist(),
            'distortion_coefficients': self.dist_coeffs.tolist()
        }

        with open('/tmp/simulated_camera_calibration.yaml', 'w') as f:
            yaml.dump(calibration_data, f)

        self.get_logger().info('Camera calibration saved to /tmp/simulated_camera_calibration.yaml')

    def save_extrinsics(self):
        """Save LiDAR to camera extrinsics"""
        import yaml
        extrinsics_data = {
            'extrinsics_matrix': self.extrinsics.tolist()
        }

        with open('/tmp/simulated_lidar_camera_extrinsics.yaml', 'w') as f:
            yaml.dump(extrinsics_data, f)

        self.get_logger().info('LiDAR to camera extrinsics saved to /tmp/simulated_lidar_camera_extrinsics.yaml')

    def calibration_loop(self):
        """Calibration processing loop"""
        # This could implement automatic calibration triggers
        # or manual calibration sequences
        if self.calibration_in_progress and len(self.obj_points) >= 20:
            self.stop_calibration()

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationSimulatorNode()

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

## 7.5 Quality Validation Simulation

Let's implement sensor data quality validation:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
from std_msgs.msg import String, Float64
import numpy as np
from cv_bridge import CvBridge
import cv2
from collections import deque

class QualityValidationSimulator(Node):
    def __init__(self):
        super().__init__('quality_validation_simulator')

        # Create CV bridge
        self.bridge = CvBridge()

        # Publishers
        self.quality_status_pub = self.create_publisher(String, 'sensor_quality_status', 10)
        self.image_quality_pub = self.create_publisher(Float64, 'image_quality_score', 10)
        self.lidar_quality_pub = self.create_publisher(Float64, 'lidar_quality_score', 10)
        self.imu_quality_pub = self.create_publisher(Float64, 'imu_quality_score', 10)

        # Subscribers
        self.image_sub = self.create_subscription(Image, 'simulated_camera/image_raw', self.image_quality_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, 'simulated_scan', self.lidar_quality_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'simulated_imu', self.imu_quality_callback, 10)

        # Quality assessment parameters
        self.min_brightness = 30
        self.max_brightness = 220
        self.min_contrast = 0.1
        self.max_noise = 0.1
        self.min_lidar_points = 50
        self.imu_stability_threshold = 0.01

        # Quality tracking
        self.image_quality_history = deque(maxlen=100)
        self.lidar_quality_history = deque(maxlen=100)
        self.imu_quality_history = deque(maxlen=100)

        # Timer for quality assessment
        self.quality_timer = self.create_timer(1.0, self.assess_overall_quality)

        self.get_logger().info('Quality validation simulator initialized')

    def image_quality_callback(self, msg):
        """Assess image quality"""
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Calculate quality metrics
            brightness = np.mean(gray)
            contrast = np.std(gray)

            # Calculate sharpness using Laplacian variance
            laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()

            # Calculate noise level
            noise = self.estimate_image_noise(gray)

            # Normalize metrics to 0-1 scale
            brightness_score = self.normalize_brightness(brightness)
            contrast_score = self.normalize_contrast(contrast)
            sharpness_score = self.normalize_sharpness(laplacian_var)
            noise_score = self.normalize_noise(noise)

            # Overall image quality score (weighted average)
            image_quality = (
                0.3 * brightness_score +
                0.2 * contrast_score +
                0.3 * sharpness_score +
                0.2 * (1 - noise_score)  # Lower noise is better
            )

            # Store in history
            self.image_quality_history.append(image_quality)

            # Publish quality score
            quality_msg = Float64()
            quality_msg.data = image_quality
            self.image_quality_pub.publish(quality_msg)

            # Log if quality is poor
            if image_quality < 0.5:
                self.get_logger().warn(f'Poor image quality detected: {image_quality:.3f}')

        except Exception as e:
            self.get_logger().error(f'Error assessing image quality: {e}')

    def lidar_quality_callback(self, msg):
        """Assess LiDAR quality"""
        # Count valid ranges
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        valid_ratio = len(valid_ranges) / len(msg.ranges) if msg.ranges else 0

        # Calculate range consistency
        if len(valid_ranges) > 1:
            range_std = np.std(valid_ranges)
            range_consistency = 1.0 / (1.0 + range_std)  # Lower std = higher consistency
        else:
            range_consistency = 0.0

        # Overall LiDAR quality score
        lidar_quality = (
            0.6 * min(1.0, len(valid_ranges) / self.min_lidar_points) +
            0.4 * range_consistency
        )

        # Store in history
        self.lidar_quality_history.append(lidar_quality)

        # Publish quality score
        quality_msg = Float64()
        quality_msg.data = lidar_quality
        self.lidar_quality_pub.publish(quality_msg)

        # Log if quality is poor
        if lidar_quality < 0.5:
            self.get_logger().warn(f'Poor LiDAR quality detected: {lidar_quality:.3f}')

    def imu_quality_callback(self, msg):
        """Assess IMU quality"""
        # Check for NaN values
        linear_acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        angular_vel = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        has_nan = np.any(np.isnan(linear_acc)) or np.any(np.isnan(angular_vel))

        if has_nan:
            imu_quality = 0.0
        else:
            # Check for unrealistic values
            acc_magnitude = np.linalg.norm(linear_acc)
            gyro_magnitude = np.linalg.norm(angular_vel)

            acc_reasonable = 1.0 if abs(acc_magnitude - 9.81) < 5.0 else 0.5  # Allow for motion
            gyro_reasonable = 1.0 if gyro_magnitude < 10.0 else 0.0  # 10 rad/s threshold

            imu_quality = (acc_reasonable + gyro_reasonable) / 2.0

        # Store in history
        self.imu_quality_history.append(imu_quality)

        # Publish quality score
        quality_msg = Float64()
        quality_msg.data = imu_quality
        self.imu_quality_pub.publish(quality_msg)

        # Log if quality is poor
        if imu_quality < 0.5:
            self.get_logger().warn(f'Poor IMU quality detected: {imu_quality:.3f}')

    def normalize_brightness(self, brightness):
        """Normalize brightness to quality score"""
        if self.min_brightness <= brightness <= self.max_brightness:
            return 1.0
        else:
            # Calculate distance from acceptable range
            dist_to_range = min(abs(brightness - self.min_brightness), abs(brightness - self.max_brightness))
            penalty = dist_to_range / 50.0  # Normalize penalty
            return max(0.0, 1.0 - penalty)

    def normalize_contrast(self, contrast):
        """Normalize contrast to quality score"""
        if contrast >= self.min_contrast:
            return min(1.0, contrast / 50.0)  # Cap at 1.0
        else:
            return contrast / self.min_contrast

    def normalize_sharpness(self, sharpness):
        """Normalize sharpness to quality score"""
        # Sharpness values can vary widely, normalize based on expected range
        normalized = sharpness / 1000.0  # Adjust based on typical values
        return min(1.0, normalized)

    def normalize_noise(self, noise):
        """Normalize noise to quality score"""
        if noise <= self.max_noise:
            return noise / self.max_noise
        else:
            return 1.0

    def estimate_image_noise(self, gray_image):
        """Estimate image noise using Laplacian method"""
        # Calculate local variance in small windows
        window_size = 3
        padded = np.pad(gray_image, window_size//2, mode='edge')
        noise_estimate = 0

        for i in range(gray_image.shape[0]):
            for j in range(gray_image.shape[1]):
                window = padded[i:i+window_size, j:j+window_size]
                window_var = np.var(window)
                noise_estimate += window_var

        return noise_estimate / (gray_image.shape[0] * gray_image.shape[1])

    def assess_overall_quality(self):
        """Assess overall sensor quality"""
        # Calculate average quality scores
        avg_image_quality = np.mean(self.image_quality_history) if self.image_quality_history else 0.0
        avg_lidar_quality = np.mean(self.lidar_quality_history) if self.lidar_quality_history else 0.0
        avg_imu_quality = np.mean(self.imu_quality_history) if self.imu_quality_history else 0.0

        # Overall system quality
        overall_quality = (avg_image_quality + avg_lidar_quality + avg_imu_quality) / 3.0

        # Publish status
        status_msg = String()
        status_msg.data = f'Sensor Quality - Cam: {avg_image_quality:.3f}, LiDAR: {avg_lidar_quality:.3f}, IMU: {avg_imu_quality:.3f}, Overall: {overall_quality:.3f}'
        self.quality_status_pub.publish(status_msg)

        # Log quality status
        if overall_quality > 0.8:
            self.get_logger().info(f'Excellent sensor quality: {overall_quality:.3f}')
        elif overall_quality > 0.6:
            self.get_logger().info(f'Good sensor quality: {overall_quality:.3f}')
        elif overall_quality > 0.4:
            self.get_logger().warn(f'Moderate sensor quality: {overall_quality:.3f}')
        else:
            self.get_logger().error(f'Poor sensor quality: {overall_quality:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = QualityValidationSimulator()

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

## 7.6 Practical Exercise

Create a complete sensor simulation system that:
1. Implements camera, LiDAR, and IMU simulation
2. Performs sensor fusion with quality validation
3. Implements calibration procedures
4. Validates sensor data quality in simulation
5. Provides realistic noise models

```python
# Student exercise - Complete implementation
class CompleteSensorSimulationSystem:
    """Student implementation of a complete sensor simulation system"""

    def __init__(self):
        """Initialize the complete sensor simulation system"""
        # TODO: Implement camera sensor simulation
        # TODO: Implement LiDAR sensor simulation
        # TODO: Implement IMU sensor simulation
        # TODO: Implement sensor fusion in simulation
        # TODO: Implement calibration procedures
        # TODO: Implement quality validation
        # TODO: Implement realistic noise models
        pass

    def implement_camera_simulation(self):
        """Implement realistic camera sensor simulation"""
        # TODO: Complete implementation
        pass

    def implement_lidar_simulation(self):
        """Implement realistic LiDAR sensor simulation"""
        # TODO: Complete implementation
        pass

    def implement_imu_simulation(self):
        """Implement realistic IMU sensor simulation"""
        # TODO: Complete implementation
        pass

    def implement_sensor_fusion(self):
        """Implement sensor fusion in simulation"""
        # TODO: Complete implementation
        pass

    def implement_calibration(self):
        """Implement calibration procedures"""
        # TODO: Complete implementation
        pass

    def implement_quality_validation(self):
        """Implement sensor data quality validation"""
        # TODO: Complete implementation
        pass

    def implement_noise_models(self):
        """Implement realistic noise models"""
        # TODO: Complete implementation
        pass

print("Student Exercise: Implement a complete sensor simulation system")
print("Requirements:")
print("1. Realistic camera, LiDAR, and IMU simulation")
print("2. Sensor fusion with quality validation")
print("3. Calibration procedures")
print("4. Sensor data quality validation")
print("5. Realistic noise models")
print("6. Performance optimization")
```

## Summary

Sensor simulation and perception systems in virtual environments provide a safe and cost-effective way to test and validate robotic algorithms before deployment on real hardware. Understanding how to simulate realistic sensor data with appropriate noise models, implement perception algorithms, and validate sensor quality is crucial for developing robust robotic systems.

<div class="alert alert-warning">
  <h5>Hardware Requirement</h5>
  <div><strong>Requirement:</strong> GPU</div>
  <div><strong>Minimum:</strong> RTX 4070</div>
  <div><strong>Recommended:</strong> RTX 4080</div>
  <div><strong>Purpose:</strong> Sensor simulation and perception algorithms require significant computational resources</div>
</div>