---
title: "ہفتہ 6: اعلیٰ ROS 2 تصورات"
week: 6
module: "روبوٹک انفراسٹرکچر"
difficulty: "اعلیٰ"
prerequisites: ["ros2-fundamentals", "python-basics", "robotics-concepts"]
learning_objectives:
  - "لاؤنچ فائلز اور سسٹم کمپوزیشن کے ساتھ کام کریں"
  - "تقسیم شدہ روبوٹکس سسٹم نافذ کریں"
  - "ڈیبگنگ اور وژولائزیشن ٹولز استعمال کریں"
tags: ["ros2", "launch", "composition", "debugging", "visualization", "urdu"]
hardware_requirements:
  - gpu: "کوئی بھی"
  - ram: "8GB کم از کم"
  - os: "Ubuntu 22.04 LTS"
duration: "90 منٹ"
---

# ہفتہ 6: اعلیٰ ROS 2 تصورات

## سیکھنے کے اہداف
- لاؤنچ فائلز اور سسٹم کمپوزیشن کے ساتھ کام کریں
- تقسیم شدہ روبوٹکس سسٹم نافذ کریں
- ڈیبگنگ اور وژولائزیشن ٹولز استعمال کریں
- ROS 2 ڈیولپمنٹ کے بہترین طریقے سمجھیں

## 6.1 اعلیٰ لاؤنچ فائلز اور سسٹم کمپوزیشن

### جامع لاؤنچ فائل سٹرکچر

ROS 2 میں لاؤنچ فائلز ایک ہی وقت میں متعدد نوڈز کو شروع کرنے اور تشکیل دینے کے لیے طاقتور میکانزم فراہم کرتے ہیں۔ اعلیٰ لاؤنچ فائلز میں مشروط ایکسیکیوشن، پیرامیٹر مینجمنٹ، اور جامع نوڈ کنفیگریشن شامل ہو سکتی ہے۔

```python
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.descriptions import ParameterFile

def generate_launch_description():
    # لاؤنچ آرگومنٹس کا اعلان کریں
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_namespace = LaunchConfiguration('robot_namespace')
    enable_logging = LaunchConfiguration('enable_logging')

    return LaunchDescription([
        # لاؤنچ آرگومنٹس کا اعلان کریں
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='سیمولیشن ٹائم استعمال کریں'
        ),
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='robot1',
            description='روبوٹ نیم اسپیس'
        ),
        DeclareLaunchArgument(
            'enable_logging',
            default_value='true',
            description='تفصیلی لاگنگ فعال کریں'
        ),

        # عالمی پیرامیٹر سیٹ کریں
        SetParameter(name='use_sim_time', value=use_sim_time),

        # مشروط ایکسیکیوشن کے ساتھ نوڈز گروپ کریں
        GroupAction(
            condition=IfCondition(enable_logging),
            actions=[
                # تشخیصی نوڈ
                Node(
                    package='diagnostics',
                    executable='diagnostic_aggregator',
                    name='diagnostic_aggregator',
                    parameters=[{'use_sim_time': use_sim_time}]
                ),
                # کارکردگی مانیٹر
                Node(
                    package='performance_monitor',
                    executable='perf_monitor',
                    name='performance_monitor',
                    parameters=[{'use_sim_time': use_sim_time}]
                )
            ]
        ),

        # روبوٹ کنٹرولر نوڈ
        Node(
            package='my_robot_package',
            executable='robot_controller',
            name='robot_controller',
            namespace=robot_namespace,
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_id': robot_namespace},
                {'control_frequency': 50},
                {'max_velocity': 1.0}
            ],
            remappings=[
                ('/cmd_vel', 'cmd_vel'),
                ('/odom', 'odom'),
                ('/scan', 'scan')
            ],
            respawn=True,
            respawn_delay=2.0,
            output='screen'
        )
    ])
```

### کمپوزیبل نوڈز اور کمپوننٹس

نوڈ کمپوزیشن متعدد نوڈز کو ایک ہی عمل میں چلنے کی اجازت دیتا ہے، مواصلت کے اخراجات کو کم کرتا ہے اور کارکردگی کو بہتر بناتا ہے۔

```python
# کمپوزیبل نوڈ مثال
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String

class ComposableNode(Node):
    """کمپوزیبل نوڈ کی مثال جسے کمپوننٹ کے طور پر لوڈ کیا جا سکتا ہے"""

    def __init__(self):
        super().__init__('composable_node')

        # پیرامیٹر کا اعلان کریں
        self.declare_parameter('publish_rate', 1.0, ParameterDescriptor(
            description='پیغامات شائع کرنے کی شرح'
        ))

        # پبلشر اور سبسکرائیب بنائیں
        self.publisher = self.create_publisher(String, 'composed_topic', 10)
        self.subscription = self.create_subscription(
            String, 'composed_input', self.subscription_callback, 10
        )

        # ٹائمر بنائیں
        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0/rate, self.timer_callback)

    def subscription_callback(self, msg):
        self.get_logger().info(f'کمپوزیٹ نوڈ میں موصول ہوا: {msg.data}')

    def timer_callback(self):
        msg = String()
        msg.data = f'کمپوزیٹ نوڈ پیغام {self.get_clock().now().nanoseconds} پر'
        self.publisher.publish(msg)
```

### لائف سائیکل نوڈز

لائف سائیکل نوڈز نوڈ اسٹیٹ مینجمنٹ اور سسٹم ابتدائی کارروائی کے لیے بہتر کنٹرول فراہم کرتے ہیں۔

```python
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn
from std_msgs.msg import String

class LifecycleManagerNode(LifecycleNode):
    """لائف سائیکل نوڈ کی مثال اسٹیٹ مینجمنٹ کے ساتھ"""

    def __init__(self):
        super().__init__('lifecycle_node')
        self.publisher = None

    def on_configure(self, state):
        """جب نوڈ CONFIGURING اسٹیٹ میں جاتا ہے کال کیا جاتا ہے"""
        self.get_logger().info('لائف سائیکل نوڈ تشکیل دیا جا رہا ہے')

        # پبلشر بنائیں
        self.publisher = self.create_publisher(String, 'lifecycle_topic', 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        """جب نوڈ ACTIVATING اسٹیٹ میں جاتا ہے کال کیا جاتا ہے"""
        self.get_logger().info('لائف سائیکل نوڈ فعال کیا جا رہا ہے')

        # پبلشر فعال کریں
        self.publisher.on_activate()

        # ٹائمر بنائیں
        self.timer = self.create_timer(1.0, self.timer_callback)

        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        msg = String()
        msg.data = f'لائف سائیکل نوڈ {self.get_clock().now().nanoseconds} پر فعال ہے'
        self.publisher.publish(msg)
```

## 6.2 تقسیم شدہ روبوٹکس سسٹم

### متعدد مشین کمیونیکیشن

ROS 2 کا DDS (ڈیٹا ڈسٹری بیوشن سروس) مڈل ویئر مشینوں کے درمیان مضبوط تقسیم شدہ کمیونیکیشن کو فعال کرتا ہے۔

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket

class DistributedNode(Node):
    def __init__(self):
        super().__init__('distributed_node')

        # اس مشین کا IP حاصل کریں
        self.host_ip = self.get_host_ip()
        self.get_logger().info(f'نوڈ IP: {self.host_ip} پر چل رہا ہے')

        # تقسیم شدہ کمیونیکیشن کے لیے پبلشر
        self.dist_publisher = self.create_publisher(String, 'distributed_topic', 10)

        # ٹائمر مسلسل لوکیشن کی معلومات شائع کرنے کے لیے
        self.timer = self.create_timer(5.0, self.publish_location_info)

    def get_host_ip(self):
        """ہوسٹ مشین کا IP ایڈریس حاصل کریں"""
        try:
            # مقامی IP کا تعین کرنے کے لیے ایک ریموٹ سرور سے رابطہ کریں
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                return s.getsockname()[0]
        except Exception:
            return "127.0.0.1"

    def publish_location_info(self):
        """اس نوڈ کی لوکیشن کے بارے میں معلومات شائع کریں"""
        msg = String()
        msg.data = f'نوڈ {self.get_name()} ہوسٹ {self.host_ip} پر {self.get_clock().now().nanoseconds} پر'
        self.dist_publisher.publish(msg)
```

## 6.3 اعلیٰ ڈیبگنگ تکنیک

### حسب ضرورت تشخیصی پیغامات

سسٹم مانیٹرنگ کے لیے حسب ضرورت تشخیصی پیغامات تیار کرنا:

```python
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Bool
import psutil
import time

class AdvancedDiagnosticNode(Node):
    def __init__(self):
        super().__init__('advanced_diagnostic_node')

        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.system_status_pub = self.create_publisher(Bool, 'system_operational', 10)

        self.diag_timer = self.create_timer(1.0, self.publish_diagnostics)

    def publish_diagnostics(self):
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # سسٹم صحت تشخیص
        sys_health = DiagnosticStatus()
        sys_health.name = 'سسٹم صحت'

        # متعدد سسٹم میٹرکس چیک کریں
        cpu_percent = psutil.cpu_percent()
        memory_percent = psutil.virtual_memory().percent
        disk_percent = psutil.disk_usage('/').percent

        # کل صحت کا تعین کریں
        if cpu_percent < 70 and memory_percent < 70 and disk_percent < 80:
            sys_health.level = DiagnosticStatus.OK
            sys_health.message = 'سسٹم صحت مند'
        elif cpu_percent < 90 or memory_percent < 90 or disk_percent < 95:
            sys_health.level = DiagnosticStatus.WARN
            sys_health.message = 'سسٹم معمولی بوجھ میں'
        else:
            sys_health.level = DiagnosticStatus.ERROR
            sys_health.message = 'سسٹم بوجھ میں'

        # تفصیلی میٹرکس کے لیے کلید-ویلیو جوڑیں
        sys_health.values = [
            KeyValue(key='cpu_percent', value=str(cpu_percent)),
            KeyValue(key='memory_percent', value=str(memory_percent)),
            KeyValue(key='disk_percent', value=str(disk_percent)),
            KeyValue(key='timestamp', value=str(time.time()))
        ]

        diag_array.status.append(sys_health)
        self.diag_pub.publish(diag_array)

        # سسٹم آپریشنل اسٹیٹش پبلش کریں
        operational_msg = Bool()
        operational_msg.data = (sys_health.level != DiagnosticStatus.ERROR)
        self.system_status_pub.publish(operational_msg)
```

## 6.4 اعلیٰ وژولائزیشن تکنیک

### RViz کسٹم پلگ انز اور مارکرز

روبوٹک سسٹم کے لیے اعلیٰ وژولائزیشن تیار کرنا:

```python
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped, Vector3
from std_msgs.msg import ColorRGBA
import math
import numpy as np

class AdvancedVisualizationNode(Node):
    def __init__(self):
        super().__init__('advanced_visualization')

        self.marker_pub = self.create_publisher(MarkerArray, 'advanced_markers', 10)
        self.path_pub = self.create_publisher(PoseStamped, 'robot_path', 10)

        self.vis_timer = self.create_timer(0.1, self.publish_advanced_visualization)
        self.time_counter = 0.0

        # روبوٹ ٹریجکٹری ذخیرہ کریں
        self.trajectory = []

    def publish_advanced_visualization(self):
        marker_array = MarkerArray()

        # ایک جامع ٹریجکٹری وژولائزیشن بنائیں
        self.time_counter += 0.05

        # روبوٹ پوزیشن مارکر
        robot_marker = Marker()
        robot_marker.header.frame_id = "map"
        robot_marker.header.stamp = self.get_clock().now().to_msg()
        robot_marker.ns = "robot"
        robot_marker.id = 0
        robot_marker.type = Marker.ARROW
        robot_marker.action = Marker.ADD
        robot_marker.pose.position.x = 3 * math.cos(self.time_counter)
        robot_marker.pose.position.y = 3 * math.sin(self.time_counter)
        robot_marker.pose.position.z = 0.0
        robot_marker.pose.orientation.w = 1.0
        robot_marker.scale = Vector3(x=0.5, y=0.2, z=0.2)
        robot_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        marker_array.markers.append(robot_marker)

        # ٹریجکٹری میں شامل کریں
        self.trajectory.append((robot_marker.pose.position.x, robot_marker.pose.position.y))
        if len(self.trajectory) > 200:  # صرف آخری 200 پوائنٹس رکھیں
            self.trajectory.pop(0)

        # ٹریجکٹری پاتھ
        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = "trajectory"
        path_marker.id = 1
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.pose.orientation.w = 1.0
        path_marker.scale = Vector3(x=0.05, y=0.05, z=0.05)
        path_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)

        for x, y in self.trajectory:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.05  # ذرا زمین سے اوپر
            path_marker.points.append(point)

        marker_array.markers.append(path_marker)
```

## 6.5 ROS 2 ڈیولپمنٹ کے لیے بہترین طریقے

### کوڈ کی تنظیم اور سٹرکچر

```python
# تجویز کردہ پروجیکٹ سٹرکچر
"""
my_robot_package/
├── src/
│   ├── controllers/
│   │   ├── __init__.py
│   │   └── joint_controller.py
│   ├── sensors/
│   │   ├── __init__.py
│   │   └── sensor_processor.py
│   └── utils/
│       ├── __init__.py
│       └── robot_helpers.py
├── launch/
│   ├── robot_system.launch.py
│   └── simulation.launch.py
├── config/
│   ├── robot_params.yaml
│   └── simulation_params.yaml
├── test/
│   ├── test_controllers.py
│   └── test_sensors.py
└── CMakeLists.txt / setup.py
"""

# اچھی طرح سے سٹرکچر والے نوڈ کی مثال
class WellStructuredRobotNode(Node):
    def __init__(self):
        super().__init__('well_structured_robot')

        # اجزاء کی ابتدائی کارروائی
        self._initialize_parameters()
        self._initialize_publishers()
        self._initialize_subscribers()
        self._initialize_services()
        self._initialize_timers()
        self._initialize_components()

    def _initialize_parameters(self):
        """تمام پیرامیٹر کی ابتدائی کارروائی"""
        self.declare_parameter('robot_id', 'robot_01')
        self.declare_parameter('control_frequency', 50)
        self.declare_parameter('safety_timeout', 5.0)

    def _initialize_publishers(self):
        """تمام پبلشر کی ابتدائی کارروائی"""
        self.status_pub = self.create_publisher(String, 'status', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

    def _initialize_subscribers(self):
        """تمام سبسکرائیب کی ابتدائی کارروائی"""
        self.cmd_sub = self.create_subscription(
            String, 'commands', self._command_callback, 10
        )

    def _initialize_services(self):
        """تمام سروسز کی ابتدائی کارروائی"""
        self.enable_service = self.create_service(
            SetBool, 'enable', self._enable_callback
        )

    def _initialize_timers(self):
        """تمام ٹائمرز کی ابتدائی کارروائی"""
        self.control_timer = self.create_timer(
            1.0 / self.get_parameter('control_frequency').value,
            self._control_loop
        )

    def _initialize_components(self):
        """جامع اجزاء کی ابتدائی کارروائی"""
        self.controller = JointController()
        self.safety_manager = SafetyManager()

    def _command_callback(self, msg):
        """آنے والے کمانڈز کو ہینڈل کریں"""
        try:
            # کمانڈ کی پروسیسنگ میں خرابی کو ہینڈل کریں
            self.controller.process_command(msg.data)
        except Exception as e:
            self.get_logger().error(f'کمانڈ پروسیسنگ خرابی: {e}')

    def _control_loop(self):
        """مرکزی کنٹرول لوپ"""
        try:
            # روبوٹ اسٹیٹ اپ ڈیٹ کریں
            self.controller.update()

            # اسٹیٹ شائع کریں
            self._publish_state()

            # سیفٹی چیک کریں
            self.safety_manager.check_safety()

        except Exception as e:
            self.get_logger().error(f'کنٹرول لوپ خرابی: {e}')

    def _publish_state(self):
        """موجودہ روبوٹ اسٹیٹ شائع کریں"""
        # یہاں نفاذ
        pass
```

## خلاصہ

اعلیٰ ROS 2 تصورات میں جامع لاؤنچ فائل کنفیگریشنز، نوڈ کمپوزیشن، تقسیم شدہ سسٹم کمیونیکیشن، اعلیٰ ڈیبگنگ تکنیک، اور جامع وژولائزیشن ٹولز شامل ہیں۔ ان تصورات کو سمجھنا مضبوط، قابل توسیع، اور برقرار رکھنے والے روبوٹک سسٹم تیار کرنے کے لیے ضروری ہے۔

## اگلے اقدامات

اگلے حصے میں، ہم سینسر انٹیگریشن اور پرچیپشن سسٹم کا تفصیل سے جائزہ لیں گے، یہاں سیکھے گئے اعلیٰ ROS 2 تصورات کو بنیاد بنا کر۔

<div class="alert alert-info" dir="rtl">
  <h5>ہارڈ ویئر کی ضرورت</h5>
  <div><strong>ضرورت:</strong> GPU</div>
  <div><strong>کم از کم:</strong> کوئی بھی</div>
  <div><strong>تجویز کردہ:</strong> کوئی بھی</div>
  <div><strong>مقصد:</strong> ہفتہ 6 اعلیٰ ROS 2 تصورات کے لیے بنیادی کمپیوٹیشنل ضروریات</div>
</div>