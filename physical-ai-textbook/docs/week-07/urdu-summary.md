---
title: "ہفتہ 7: سینسر انٹیگریشن اور ت Percption سسٹم"
week: 7
module: "روبوٹک انفراسٹرکچر"
difficulty: "اعلیٰ"
prerequisites: ["ros2-fundamentals", "python-basics", "robotics-concepts", "sensor-basics"]
learning_objectives:
  - "روبوٹک سینسرز کو انضمام کریں"
  - "ادراک پائپ لائنز نافذ کریں"
  - "سینسر ڈیٹا ہم وقت سازی کا انتظام کریں"
tags: ["sensors", "perception", "camera", "lidar", "imu", "calibration", "urdu"]
hardware_requirements:
  - gpu: "RTX 4070 یا اس سے زیادہ"
  - ram: "16GB کم از کم"
  - os: "Ubuntu 22.04 LTS"
duration: "90 منٹ"
---

# ہفتہ 7: سینسر انٹیگریشن اور ادراک سسٹم

## سیکھنے کے اہداف
- روبوٹک سینسرز کو انضمام کریں
- ادراک پائپ لائنز نافذ کریں
- سینسر ڈیٹا ہم وقت سازی کا انتظام کریں
- سینسر کیلیبریشن طریقہ کار سمجھیں

## 7.1 روبوٹک سینسرز کا تعارف

روبوٹک سینسرز اہم اجزاء ہیں جو روبوٹس کو اپنے ماحول کا ادراک کرنے اور اس کے ساتھ تعامل کرنے کے قابل بناتے ہیں۔ وہ روبوٹ کو اس کی داخلی حالت اور بیرونی ماحول کے بارے میں معلومات فراہم کرتے ہیں۔

### روبوٹک سینسرز کی اقسام

1. **پروپریوسیف سینسرز**: داخلی روبوٹ حالت کو ناپتے ہیں
   - جوڑی انکوڈرز
   - موتور کرنٹ سینسرز
   - داخلی درجہ حرارت سینسرز

2. **ایکسٹریو سینسرز**: بیرونی ماحول کو ناپتے ہیں
   - کیمرے (RGB، اسٹیریو، گہرائی)
   - لائیڈار (2D، 3D)
   - IMU (ایکسلیرو میٹر، جائراسکوپ، میگنیٹومیٹر)
   - GPS
   - فورس/ٹورک سینسرز
   - ٹیکٹائل سینسرز

3. **رینج سینسرز**: اشیاء تک فاصلے ناپتے ہیں
   - الٹرا سونک سینسرز
   - انفراریڈ سینسرز
   - ٹائم آف فلائٹ سینسرز

### سینسر کی خصوصیات

ہر سینسر کی قسم کی مخصوص خصوصیات ہوتی ہیں جو اس کے استعمال کو متاثر کرتی ہیں:

- **درستگی**: پیمائش کتنا حقیقی اقدار کے قریب ہے
- **درستی**: دہرائی گئی پیمائش کی مطابقت
- **ریزولوشن**: چھوٹی سے چھوٹی قابل شناسائی تبدیلی
- **رینج**: کم از کم اور زیادہ سے زیادہ قابل پیمائش اقدار
- **اپ ڈیٹ کی شرح**: پیمائش کی تعدد
- **نوائز**: پیمائش میں بے ترتیب تبدیلیاں
- **دیری**: پیمائش اور آؤٹ پٹ کے درمیان تاخیر

## 7.2 کیمرہ سسٹم اور کمپیوٹر وژن

### کیمرہ کی اقسام اور کنفیگریشنز

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

        # کیمرہ ڈیٹا کے لیے پبلشرز
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)

        # تصویر کنورژن کے لیے CV برج
        self.bridge = CvBridge()

        # کیمرہ پیرامیٹرز
        self.camera_matrix = np.array([
            [615.0, 0.0, 320.0],
            [0.0, 615.0, 240.0],
            [0.0, 0.0, 1.0]
        ])

        # کیمرہ کو شروع کریں
        self.cap = cv2.VideoCapture(0)  # ڈیفالٹ کیمرہ
        if not self.cap.isOpened():
            self.get_logger().error('کیمرہ کھول نہیں سکا')
            return

        # کیمرہ خصوصیات سیٹ کریں
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # تصاویر کو قبضہ کرنے کے لیے ٹائمر
        self.timer = self.create_timer(0.033, self.capture_image)  # ~30 FPS

    def capture_image(self):
        """کیمرہ تصویر قبضہ کریں اور شائع کریں"""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('تصویر قبضہ کرنے میں ناکامی')
            return

        # ROS تصویر پیغام میں تبدیل کریں
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = 'camera_optical_frame'

        self.image_pub.publish(image_msg)

        # کیمرہ معلومات شائع کریں
        self.publish_camera_info()

    def publish_camera_info(self):
        """کیمرہ کیلیبریشن معلومات شائع کریں"""
        info_msg = CameraInfo()
        info_msg.header.stamp = self.get_clock().now().to_msg()
        info_msg.header.frame_id = 'camera_optical_frame'
        info_msg.width = 640
        info_msg.height = 480
        info_msg.k = self.camera_matrix.flatten().tolist()

        self.info_pub.publish(info_msg)

    def __del__(self):
        """کیمرہ وسائل کو صاف کریں"""
        if hasattr(self, 'cap'):
            self.cap.release()
```

### اسٹیریو وژن اور گہرائی کا تخمینہ

```python
import cv2
import numpy as np

class StereoVisionProcessor:
    def __init__(self, left_camera_matrix, right_camera_matrix,
                 left_dist_coeffs, right_dist_coeffs, baseline):
        # کیمرہ میٹرکس اور ڈسٹورشن کوائفیسینٹس
        self.left_camera_matrix = left_camera_matrix
        self.right_camera_matrix = right_camera_matrix
        self.left_dist_coeffs = left_dist_coeffs
        self.right_dist_coeffs = right_dist_coeffs
        self.baseline = baseline  # کیمرز کے درمیان فاصلہ

        # اسٹیریو ریکٹیفکیشن پیرامیٹرز
        self.R1, self.R2, self.P1, self.P2, self.Q = None, None, None, None, None

    def rectify_images(self, left_img, right_img):
        """اسٹیریو تصویر کا جوڑا درست کریں"""
        # ریکٹیفکیشن پیرامیٹرز کا حساب کریں
        R1, R2, P1, P2, Q = cv2.stereoRectify(
            self.left_camera_matrix, self.left_dist_coeffs,
            self.right_camera_matrix, self.right_dist_coeffs,
            left_img.shape[::-1], np.eye(3), np.eye(3)
        )

        # ڈسٹورشن میپس بنائیں
        map1x, map1y = cv2.initUndistortRectifyMap(
            self.left_camera_matrix, self.left_dist_coeffs,
            R1, self.P1, left_img.shape[::-1], cv2.CV_32FC1
        )
        map2x, map2y = cv2.initUndistortRectifyMap(
            self.right_camera_matrix, self.right_dist_coeffs,
            R2, self.P2, right_img.shape[::-1], cv2.CV_32FC1
        )

        # ریکٹیفکیشن لاگو کریں
        left_rectified = cv2.remap(left_img, map1x, map1y, cv2.INTER_LINEAR)
        right_rectified = cv2.remap(right_img, map2x, map2y, cv2.INTER_LINEAR)

        return left_rectified, right_rectified

    def compute_depth_map(self, left_img, right_img):
        """اسٹیریو تصاویر سے گہرائی کا نقشہ بنائیں"""
        # گرے سکیل میں تبدیل کریں
        left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

        # اسٹیریو میچر بنائیں
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

        # ڈسپیرٹی کا حساب کریں
        disparity = stereo.compute(left_gray, right_gray).astype(np.float32) / 16.0

        # ڈسپیرٹی کو گہرائی میں تبدیل کریں
        depth_map = (self.baseline * self.left_camera_matrix[0, 0]) / (disparity + 1e-6)

        return depth_map
```

## 7.3 لائیڈار اور 3D ادراک

### لائیڈار سینسر انٹیگریشن

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

        # پبلشرز
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, 'point_cloud', 10)

        # لائیڈار پیرامیٹرز
        self.angle_min = -np.pi
        self.angle_max = np.pi
        self.angle_increment = np.pi / 360  # 0.5 ڈگری ریزولوشن
        self.range_min = 0.1
        self.range_max = 30.0
        self.scan_time = 0.1  # 10 Hz

        # لائیڈار سیمولیشن کے لیے ٹائمر
        self.timer = self.create_timer(self.scan_time, self.publish_scan)

    def publish_scan(self):
        """سیمولیٹڈ لائیڈار اسکین ڈیٹا شائع کریں"""
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

        # جعلی رینجس بنائیں (کچھ رکاوٹوں کے ساتھ)
        num_points = int((self.angle_max - self.angle_min) / self.angle_increment) + 1
        ranges = np.full(num_points, self.range_max)

        # کچھ رکاوٹیں شامل کریں
        angles = np.linspace(self.angle_min, self.angle_max, num_points)
        for i, angle in enumerate(angles):
            # (2, 0) پر ایک گولائی رکاوٹ کا سیمولیٹ کریں جس کا رداس 0.5 ہے
            obstacle_x, obstacle_y, obstacle_r = 2.0, 0.0, 0.5
            distance_to_obstacle = np.sqrt((obstacle_x - 2*np.cos(angle))**2 +
                                         (obstacle_y - 2*np.sin(angle))**2) - obstacle_r

            if 0.1 < distance_to_obstacle < self.range_max:
                ranges[i] = min(ranges[i], distance_to_obstacle)

        # نوائز شامل کریں
        noise = np.random.normal(0, 0.01, len(ranges))
        ranges = np.maximum(self.range_min, ranges + noise)

        scan_msg.ranges = ranges.tolist()
        scan_msg.intensities = []  # اختیاری انٹینسیٹیز

        self.scan_pub.publish(scan_msg)

        # پوائنٹ کلاؤڈ کے طور پر بھی شائع کریں
        self.publish_point_cloud(ranges, angles)

    def publish_point_cloud(self, ranges, angles):
        """اسکین ڈیٹا کو پوائنٹ کلاؤڈ میں تبدیل کریں"""
        points = []
        for angle, range_val in zip(angles, ranges):
            if self.range_min <= range_val <= self.range_max:
                x = range_val * np.cos(angle)
                y = range_val * np.sin(angle)
                z = 0.0  # 2D اسکین، تو z=0
                points.append([x, y, z])

        # PointCloud2 پیغام بنائیں
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

### 3D پوائنٹ کلاؤڈ پروسیسنگ

```python
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudProcessor:
    def __init__(self):
        self.voxel_size = 0.05  # 5cm والوکس سائز

    def convert_ros_to_o3d(self, cloud_msg):
        """ROS PointCloud2 کو Open3D پوائنٹ کلاؤڈ میں تبدیل کریں"""
        points = []
        for point in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([point[0], point[1], point[2]])

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
        return pcd

    def downsample_point_cloud(self, pcd):
        """والوکس گرڈ فلٹر کا استعمال کرتے ہوئے پوائنٹ کلاؤڈ کو ڈاؤن سیمپل کریں"""
        return pcd.voxel_down_sample(voxel_size=self.voxel_size)

    def estimate_normals(self, pcd):
        """پوائنٹ کلاؤڈ کے لیے نارملس کا تخمینہ لگائیں"""
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
        )
        return pcd

    def segment_planes(self, pcd):
        """RANSAC کا استعمال کرتے ہوئے پلینر سر فیسیز کو سیگمینٹ کریں"""
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=0.01,
            ransac_n=3,
            num_iterations=1000
        )
        [a, b, c, d] = plane_model
        self.get_logger().info(f"پلین کا مساوات: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

        # پلین اور نان-پلین پوائنٹس نکالیں
        inlier_cloud = pcd.select_by_index(inliers)
        outlier_cloud = pcd.select_by_index(inliers, invert=True)

        return inlier_cloud, outlier_cloud

    def cluster_objects(self, pcd):
        """DBSCAN کا استعمال کرتے ہوئے اشیاء کو کلسٹر کریں"""
        labels = np.array(pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=False))
        max_label = labels.max()
        self.get_logger().info(f'کلسٹرز کی تعداد: {max_label + 1}')

        return labels
```

## 7.4 IMU اور انرٹیل سینسنگ

### IMU انٹیگریشن اور فیوژن

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

        # IMU پیرامیٹرز
        self.linear_acceleration_variance = 0.01**2
        self.angular_velocity_variance = 0.001**2
        self.orientation_variance = 0.01**2

        # اسٹیٹ متغیرات
        self.orientation = R.from_quat([0, 0, 0, 1])  # شناخت کا گھوماؤ
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        self.linear_acceleration = np.array([0.0, 0.0, -9.81])  # گریویٹی

        # IMU سیمولیشن کے لیے ٹائمر
        self.timer = self.create_timer(0.01, self.publish_imu_data)  # 100 Hz

    def publish_imu_data(self):
        """سیمولیٹڈ IMU ڈیٹا شائع کریں"""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # سینسر نوائز کا سیمولیٹ کریں
        accel_noise = np.random.normal(0, np.sqrt(self.linear_acceleration_variance), 3)
        gyro_noise = np.random.normal(0, np.sqrt(self.angular_velocity_variance), 3)

        # لینیئر ایکسلریشن شائع کریں (نوائز کے ساتھ)
        imu_msg.linear_acceleration.x = self.linear_acceleration[0] + accel_noise[0]
        imu_msg.linear_acceleration.y = self.linear_acceleration[1] + accel_noise[1]
        imu_msg.linear_acceleration.z = self.linear_acceleration[2] + accel_noise[2]

        # اینگولر ویلوسٹی شائع کریں (نوائز کے ساتھ)
        imu_msg.angular_velocity.x = self.angular_velocity[0] + gyro_noise[0]
        imu_msg.angular_velocity.y = self.angular_velocity[1] + gyro_noise[1]
        imu_msg.angular_velocity.z = self.angular_velocity[2] + gyro_noise[2]

        # اورینٹیشن شائع کریں (کوارٹینین کے طور پر)
        quat = self.orientation.as_quat()
        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]

        # کواریئنس میٹرکس سیٹ کریں
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
    """سینسر فیوژن کے لیے IMU ڈیٹا کو دوسرے سینسرز کے ساتھ ضم کرنے والے نوڈ"""

    def __init__(self):
        super().__init__('imu_fusion_node')

        # سبسکرائیبرز
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # فیوژن ڈیٹا کے لیے پبلشر
        self.fused_pub = self.create_publisher(Odometry, 'fused_odom', 10)

        # اسٹیٹ اسٹیمیٹنگ متغیرات
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.orientation = R.from_quat([0, 0, 0, 1])

        # کیلمین فلٹر پیرامیٹرز
        self.process_noise = 0.1
        self.measurement_noise = 0.01

    def imu_callback(self, msg):
        """اسٹیٹ اسٹیمیٹنگ کے لیے IMU ڈیٹا کو پروسیس کریں"""
        # IMU ڈیٹا نکالیں
        accel = np.array([msg.linear_acceleration.x,
                         msg.linear_acceleration.y,
                         msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x,
                        msg.angular_velocity.y,
                        msg.angular_velocity.z])

        # گیروسکوپ انٹیگریشن کا استعمال کرتے ہوئے اورینٹیشن کو اپ ڈیٹ کریں
        dt = 0.01  # 100 Hz کا فرض کریں
        omega_norm = np.linalg.norm(gyro)
        if omega_norm > 1e-6:  # صفر سے تقسیم کو روکیں
            axis = gyro / omega_norm
            angle = omega_norm * dt
            delta_rotation = R.from_rotvec(axis * angle)
            self.orientation = self.orientation * delta_rotation

    def odom_callback(self, msg):
        """فیوژن کے لیے اودومیٹری ڈیٹا کو پروسیس کریں"""
        # اودومیٹری سے پوزیشن نکالیں
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z

        # اودومیٹری سے اورینٹیشن نکالیں
        quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.orientation = R.from_quat(quat)

    def publish_fused_odom(self):
        """فیوژن والی اودومیٹری اسٹیمیٹ شائع کریں"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'

        # پوزیشن سیٹ کریں
        odom_msg.pose.pose.position.x = self.position[0]
        odom_msg.pose.pose.position.y = self.position[1]
        odom_msg.pose.pose.position.z = self.position[2]

        # اورینٹیشن سیٹ کریں
        quat = self.orientation.as_quat()
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # ویلوسٹی سیٹ کریں
        odom_msg.twist.twist.linear.x = self.velocity[0]
        odom_msg.twist.twist.linear.y = self.velocity[1]
        odom_msg.twist.twist.linear.z = self.velocity[2]

        self.fused_pub.publish(odom_msg)
```

## 7.5 سینسر کیلیبریشن طریقہ کار

### کیمرہ کیلیبریشن

```python
import cv2
import numpy as np
import yaml

class CameraCalibrator:
    def __init__(self, pattern_size=(9, 6)):
        self.pattern_size = pattern_size
        self.obj_points = []  # حقیقی دنیا کی 3D پوائنٹس
        self.img_points = []  # تصویر کے 2D پوائنٹس

        # آبجیکٹ پوائنٹس تیار کریں (مثلاً چیس بورڈ کونرز)
        self.objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)

    def find_corners(self, img):
        """تصویر میں چیس بورڈ کونرز تلاش کریں"""
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)

        if ret:
            # کونرز کی جگہوں کو بہتر بنائیں
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria
            )
            return True, corners_refined
        return False, None

    def add_calibration_image(self, img):
        """کیلیبریشن ڈیٹا سیٹ میں تصویر شامل کریں"""
        ret, corners = self.find_corners(img)
        if ret:
            self.obj_points.append(self.objp)
            self.img_points.append(corners)
            return True
        return False

    def calibrate_camera(self):
        """کیمرہ کیلیبریشن کریں"""
        if len(self.obj_points) < 10:
            raise ValueError("کم از کم 10 کیلیبریشن تصاویر کی ضرورت ہے")

        # کیلیبریشن کریں
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points, self.img_points,
            (640, 480), None, None
        )

        if not ret:
            raise ValueError("کیلیبریشن ناکام ہو گیا")

        # ریپروجیکشن کی غلطی کا حساب کریں
        total_error = 0
        for i in range(len(self.obj_points)):
            img_points_reprojected, _ = cv2.projectPoints(
                self.obj_points[i], rvecs[i], tvecs[i],
                camera_matrix, dist_coeffs
            )
            error = cv2.norm(self.img_points[i], img_points_reprojected, cv2.NORM_L2) / len(img_points_reprojected)
            total_error += error

        avg_error = total_error / len(self.obj_points)
        self.get_logger().info(f'کیلیبریشن مکمل ہو گیا اوسط غلطی: {avg_error:.4f}')

        return camera_matrix, dist_coeffs, avg_error

    def save_calibration(self, camera_matrix, dist_coeffs, filename):
        """کیلیبریشن پیرامیٹرز کو فائل میں محفوظ کریں"""
        calibration_data = {
            'camera_matrix': camera_matrix.tolist(),
            'distortion_coefficients': dist_coeffs.tolist()
        }

        with open(filename, 'w') as f:
            yaml.dump(calibration_data, f)

        self.get_logger().info(f'کیلیبریشن {filename} میں محفوظ ہو گیا')
```

### لائیڈار سے کیمرہ کیلیبریشن

```python
import numpy as np
import cv2
from scipy.optimize import minimize

class LidarCameraCalibrator:
    def __init__(self):
        self.extrinsics = np.eye(4)  # 4x4 ٹرانسفارمیشن میٹرکس

    def calibrate_lidar_camera(self, lidar_points, image_points, camera_matrix, dist_coeffs):
        """لائیڈار سے کیمرہ ایکسٹرینزکس کیلیبریٹ کریں"""
        # ایکسٹرینزکس کے لیے ابتدائی اندیشہ (شناخت میٹرکس)
        initial_params = self.extrinsics.flatten()[:-1]  # آخری عنصر ہٹا دیں ([0,0,0,1] ہونا چاہیے)

        # ایکسٹرینزکس کو بہتر بنانے کے لیے اپٹیمائز کریں
        result = minimize(
            self._calibration_error,
            initial_params,
            args=(lidar_points, image_points, camera_matrix, dist_coeffs),
            method='Powell'
        )

        # آپٹیمائز کیے گئے پیرامیٹرز کو 4x4 میٹرکس میں دوبارہ شکل دیں
        self.extrinsics = np.eye(4)
        self.extrinsics[:3, :4] = result.x.reshape(3, 4)

        return self.extrinsics

    def _calibration_error(self, params, lidar_points, image_points, camera_matrix, dist_coeffs):
        """آپٹیمائزیشن کے لیے ریپروجیکشن کی غلطی کا حساب کریں"""
        # پیرامیٹرز کو 3x4 ٹرانسفارمیشن میٹرکس میں دوبارہ شکل دیں
        transform = params.reshape(3, 4)

        # لائیڈار پوائنٹس کو کیمرہ فریم میں ٹرانسفارم کریں
        lidar_homo = np.hstack([lidar_points, np.ones((len(lidar_points), 1))])
        camera_points = (transform @ lidar_homo.T).T

        # امیج پلین میں پروجیکٹ کریں
        projected_points = camera_points[:, :2] / camera_points[:, 2:3]

        # ڈسٹورشن لاگو کریں
        r2 = projected_points[:, 0]**2 + projected_points[:, 1]**2
        # مختصر ڈسٹورشن ماڈل
        distorted_points = projected_points * (1 + dist_coeffs[0] * r2)

        # کیمرہ انٹرنسک میٹرکس لاگو کریں
        image_points_projected = (camera_matrix[:2, :2] @ distorted_points.T +
                                 camera_matrix[:2, 2:3]).T

        # غلطی کا حساب کریں
        error = np.sum((image_points - image_points_projected)**2)
        return error

    def transform_lidar_to_camera(self, lidar_points):
        """لائیڈار پوائنٹس کو کیمرہ کوآرڈینیٹ فریم میں ٹرانسفارم کریں"""
        lidar_homo = np.hstack([lidar_points, np.ones((len(lidar_points), 1))])
        camera_points = (self.extrinsics @ lidar_homo.T).T
        return camera_points[:, :3]
```

## 7.6 ڈیٹا ہم وقت سازی اور ٹائم الائمنٹ

### سینسر ہم وقت سازی

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

        # مختلف سینسرز کے لیے سبسکرائیبرز بنائیں
        self.image_sub = Subscriber(self, Image, 'camera/image_raw')
        self.imu_sub = Subscriber(self, Imu, 'imu/data')
        self.scan_sub = Subscriber(self, LaserScan, 'scan')

        # تقریب وقت ہم وقت سازی کے ساتھ پیغامات کو ہم وقت کریں
        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.imu_sub, self.scan_sub],
            queue_size=10,
            slop=0.1  # 100ms ٹولرنس
        )
        self.sync.registerCallback(self.synchronized_callback)

        # غیر ہم وقت ڈیٹا کے لیے بفر
        self.unsync_buffer = {
            'image': deque(maxlen=100),
            'imu': deque(maxlen=1000),
            'scan': deque(maxlen=100)
        }

        # تھریڈ سیفٹی کے لیے لاک
        self.buffer_lock = threading.Lock()

    def synchronized_callback(self, image_msg, imu_msg, scan_msg):
        """ہم وقت شدہ سینسر ڈیٹا کے لیے کال بیک"""
        self.get_logger().info(f'ہم وقت شدہ ڈیٹا: تصویر {image_msg.header.stamp}, '
                              f'IMU {imu_msg.header.stamp}, '
                              f'اسکین {scan_msg.header.stamp}')

        # ہم وقت شدہ ڈیٹا کو پروسیس کریں
        self.process_fusion(image_msg, imu_msg, scan_msg)

    def process_fusion(self, image_msg, imu_msg, scan_msg):
        """فیوژن سینسر ڈیٹا کو پروسیس کریں"""
        # مثال: وژول اور لائیڈار ڈیٹا کو IMU کے ساتھ جوڑیں تاکہ پوز اسٹیمیٹ کیا جا سکے
        # یہ ایک سینسر فیوژن الگورتھم نافذ کرے گا

        # ڈیٹا نکالیں
        image_time = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
        imu_time = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9
        scan_time = scan_msg.header.stamp.sec + scan_msg.header.stamp.nanosec * 1e-9

        # فیوژن کے حسابات انجام دیں
        # (نفاذ کسی مخصوص ایپلی کیشن پر منحصر ہوگا)

        self.get_logger().info(f'فیوژن {image_time:.3f}s پر مکمل ہو گیا')
```

## 7.7 ادراک پائپ لائن نفاذ

### ملٹی سینسر ادراک پائپ لائن

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

        # اجزاء کو شروع کریں
        self.bridge = CvBridge()
        self.current_image = None
        self.current_scan = None
        self.current_imu = None
        self.fusion_result = None

        # پبلشرز
        self.object_pub = self.create_publisher(String, 'detected_objects', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'estimated_pose', 10)

        # سبسکرائیبرز
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)

        # پروسیسنگ پائپ لائن کے لیے ٹائمر
        self.process_timer = self.create_timer(0.1, self.process_perception_pipeline)

    def image_callback(self, msg):
        """ان کمنگ تصویر ڈیٹا کو پروسیس کریں"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'تصویر کنورژن کی غلطی: {e}')

    def scan_callback(self, msg):
        """ان کمنگ اسکین ڈیٹا کو پروسیس کریں"""
        self.current_scan = msg

    def imu_callback(self, msg):
        """ان کمنگ IMU ڈیٹا کو پروسیس کریں"""
        self.current_imu = msg

    def process_perception_pipeline(self):
        """مرکزی ادراک پائپ لائن پروسیسنگ"""
        if not all([self.current_image, self.current_scan, self.current_imu]):
            return

        # مرحلہ 1: وژول اوبجیکٹ ڈیٹیکشن
        visual_objects = self.detect_visual_objects(self.current_image)

        # مرحلہ 2: لائیڈار اوبجیکٹ ڈیٹیکشن
        lidar_objects = self.detect_lidar_objects(self.current_scan)

        # مرحلہ 3: سینسر فیوژن
        fused_objects = self.fuse_sensor_data(visual_objects, lidar_objects, self.current_imu)

        # مرحلہ 4: پوز اسٹیمیٹ
        estimated_pose = self.estimate_pose(fused_objects)

        # مرحلہ 5: نتائج شائع کریں
        self.publish_results(fused_objects, estimated_pose)

    def detect_visual_objects(self, image):
        """کمپیوٹر وژن کا استعمال کرتے ہوئے تصویر میں اشیاء تلاش کریں"""
        # سادہ رنگ کی بنیاد پر اوبجیکٹ ڈیٹیکشن (مثال)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # ڈیٹیکشن کے لیے رنگ کی حدیں مقرر کریں (مثال: لال اشیاء)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])

        mask = cv2.inRange(hsv, lower_red, upper_red)

        # کونٹورز تلاش کریں
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        objects = []
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # چھوٹی اشیاء کو فلٹر کریں
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
        """لائیڈار اسکین میں اشیاء تلاش کریں"""
        # سادہ کلسٹرنگ بیسڈ اوبجیکٹ ڈیٹیکشن
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        valid_indices = [i for i, r in enumerate(scan.ranges) if scan.range_min <= r <= scan.range_max]

        if len(valid_indices) < 2:
            return []

        # کارٹیزین کوآرڈینیٹس میں تبدیل کریں
        points = []
        for i in valid_indices:
            angle = angles[i]
            range_val = scan.ranges[i]
            x = range_val * np.cos(angle)
            y = range_val * np.sin(angle)
            points.append([x, y])

        if len(points) < 2:
            return []

        # سادہ کلسٹرنگ (DBSCAN جیسی)
        clusters = self.cluster_lidar_points(np.array(points))

        objects = []
        for cluster_id in set(clusters):
            if cluster_id == -1:  # نوائز پوائنٹس
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
        """لائیڈار پوائنٹس کے لیے سادہ کلسٹرنگ"""
        if len(points) < 2:
            return np.array([])

        # DBSCAN جیسے کے قریب کا استعمال کریں
        labels = np.full(len(points), -1)
        cluster_id = 0

        for i, point in enumerate(points):
            if labels[i] != -1:
                continue

            # ارد گرد کے پوائنٹس تلاش کریں
            distances = np.linalg.norm(points - point, axis=1)
            neighbors = np.where(distances < 0.5)[0]  # 50cm رداس

            if len(neighbors) >= 3:  # کلسٹر کے لیے کم از کم پوائنٹس
                labels[neighbors] = cluster_id
                cluster_id += 1

        return labels

    def fuse_sensor_data(self, visual_objects, lidar_objects, imu_data):
        """متعدد سینسرز سے ڈیٹا ضم کریں"""
        # وژول اشیاء کو دنیا کے کوآرڈینیٹس میں تبدیل کریں کیمرہ کیلیبریشن کا استعمال کرتے ہوئے
        # لائیڈار اشیاء کو دنیا کے کوآرڈینیٹس میں تبدیل کریں
        # IMU اورینٹیشن ڈیٹا کے ساتھ ضم کریں

        fused_objects = []

        # مثال: پوزیشن کی بنیاد پر وژول اور لائیڈار اشیاء کو منسلک کریں
        for v_obj in visual_objects:
            # تصویر کے کوآرڈینیٹس کو دنیا کے کوآرڈینیٹس میں تبدیل کریں (سادہ کردہ)
            v_world = self.image_to_world(v_obj['center'])

            # مطابقت رکھنے والی لائیڈار اوبجیکٹ تلاش کریں
            best_match = None
            best_distance = float('inf')

            for l_obj in lidar_objects:
                distance = np.linalg.norm(np.array(v_world[:2]) - l_obj['center'])
                if distance < 1.0 and distance < best_distance:  # 1m تھریشولڈ
                    best_match = l_obj
                    best_distance = distance

            if best_match:
                # اشیاء کو ضم کریں
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
        """تصویر کے کوآرڈینیٹس کو دنیا کے کوآرڈینیٹس میں تبدیل کریں (سادہ کردہ)"""
        # یہ کیمرہ کیلیبریشن پیرامیٹرز کا استعمال کرے گا
        # ابھی، ایک سادہ تبدیلی کو انجام دیں
        x = (image_coords[0] - 320) * 0.01  # تقریبی تبدیلی
        y = (image_coords[1] - 240) * 0.01
        z = 1.0  # 1میٹر فاصلہ فرض کریں
        return [x, y, z]

    def estimate_pose(self, fused_objects):
        """تشخیص شدہ اشیاء کی بنیاد پر روبوٹ پوز کا تخمینہ لگائیں"""
        # مثال: نمایاں اشیاء کا استعمال کرتے ہوئے جانے ہوئے نقوش کے رشتے میں پوز کا تخمینہ لگائیں
        # یہ ایک لوکلائزیشن الگورتھم نافذ کرے گا
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'

        # سادہ کردہ پوز اسٹیمیٹ
        if fused_objects:
            # پہلی اوبجیکٹ کو حوالہ کے طور پر استعمال کریں
            obj = fused_objects[0]
            pose.pose.position.x = obj['position'][0]
            pose.pose.position.y = obj['position'][1]
            pose.pose.position.z = 0.0

        return pose

    def publish_results(self, fused_objects, estimated_pose):
        """ادراک نتائج شائع کریں"""
        # تشخیص شدہ اشیاء شائع کریں
        if fused_objects:
            obj_msg = String()
            obj_msg.data = f'{len(fused_objects)} اشیاء کا پتہ چلا'
            self.object_pub.publish(obj_msg)

        # تخمینہ شدہ پوز شائع کریں
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

## خلاصہ

سینسر انٹیگریشن اور ادراک سسٹم روبوٹک آگہی اور فیصلہ سازی کی بنیاد ہیں۔ مختلف سینسر اقسام کو صحیح طریقے سے انضمام کرنا، ادراک پائپ لائنز نافذ کرنا، اور ڈیٹا ہم وقت سازی کا انتظام کرنا روبوٹک سسٹم تیار کرنے کے لیے ضروری ہے۔ مناسب کیلیبریشن طریقہ کار اچھے سینسر ڈیٹا کو یقینی بناتے ہیں، جبکہ مؤثر فیوژن الگورتھمز متعدد سینسرز سے معلومات کو جمع کر کے ماحول کا جامع تصور فراہم کرتے ہیں۔

## اگلے اقدامات

اگلے حصے میں، ہم کنٹرول سسٹم اور ٹریجکٹری پلاننگ کا جائزہ لیں گے، یہاں سیکھے گئے ادراک کے تصورات کو بنیاد بنا کر روبوٹ نیویگیشن اور مینیپولیشن کو فعال کرنا۔

<div class="alert alert-warning" dir="rtl">
  <h5>ہارڈ ویئر کی ضرورت</h5>
  <div><strong>ضرورت:</strong> GPU</div>
  <div><strong>کم از کم:</strong> RTX 4070</div>
  <div><strong>تجویز کردہ:</strong> RTX 4080</div>
  <div><strong>مقصد:</strong> سینسر پروسیسنگ اور ادراک الگورتھمز کو کافی کمپیوٹیشنل وسائل کی ضرورت ہوتی ہے</div>
</div>