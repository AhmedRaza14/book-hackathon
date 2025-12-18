---
title: "ہفتہ 5: ROS 2 کی بنیادیں"
week: 5
module: "روبوٹک انفراسٹرکچر"
difficulty: "متوسط"
prerequisites: ["python-basics", "linux-basics", "robotics-concepts"]
learning_objectives:
  - "ROS 2 آرکیٹیکچر کو سمجھیں"
  - "بنیادی نوڈز اور ٹاپکس نافذ کریں"
  - "پائی تھون کے لیے rclpy استعمال کریں"
tags: ["ros2", "rclpy", "nodes", "topics", "services", "urdu"]
hardware_requirements:
  - gpu: "کوئی بھی"
  - ram: "8GB کم از کم"
  - os: "Ubuntu 22.04 LTS"
duration: "90 منٹ"
---

# ہفتہ 5: ROS 2 کی بنیادیں

## سیکھنے کے اہداف
- ROS 2 آرکیٹیکچر کو سمجھیں
- بنیادی نوڈز اور ٹاپکس نافذ کریں
- پائی تھون کے لیے rclpy استعمال کریں
- ROS 2 کے بنیادی ڈیولپمنٹ ورک فلو کا مشق کریں

## 5.1 ROS 2 کا تعارف

روبوٹ آپریٹنگ سسٹم 2 (ROS 2) روبوٹ سافٹ ویئر لکھنے کے لیے ایک لچکدار فریم ورک ہے۔ یہ ٹولز، لائبریریز، اور رواج کا ایک مجموعہ ہے جو کہ مختلف روبوٹ پلیٹ فارم پر پیچیدہ اور مضبوط روبوٹ کے رویے کو تیار کرنے کے کام کو آسان بنانے کا مقصد رکھتا ہے۔

### ROS 2 کی کلیدی خصوصیات
- **تقسیم شدہ کمپیوٹنگ**: نوڈز مختلف مشینوں پر چل سکتے ہیں
- **زبان کی آزادی**: متعدد پروگرامنگ زبانوں کی حمایت
- **ریل ٹائم کی حمایت**: وقت کے اہم ایپلی کیشنز کے لیے متعینہ رویہ
- **سیکورٹی**: محفوظ روبوٹ آپریشن کے لیے بلٹ ان سیکورٹی خصوصیات
- **مڈل ویئر**: کمیونیکیشن کے لیے DDS (ڈیٹا ڈسٹری بیوشن سروس)

### ROS 2 بمقابلہ ROS 1
ROS 2 ROS 1 کی کئی حدود کو حل کرتا ہے:
- **ریل ٹائم کی حمایت**: ROS 2 ریل ٹائم کی صلاحیات فراہم کرتا ہے
- **متعدد مشین کمیونیکیشن**: بہتر تقسیم شدہ سسٹم کی حمایت
- **سیکورٹی**: بلٹ ان سیکورٹی میکانزم
- **مڈل ویئر لچک**: منسلک ہونے والے مڈل ویئر کی ترتیب
- **لائف سائیکل مینجمنٹ**: بہتر نوڈ لائف سائیکل مینجمنٹ

## 5.2 ROS 2 آرکیٹیکچر

### نوڈز
ایک نوڈ ایک ایکسیکیوٹیبل ہے جو دوسرے نوڈز کے ساتھ مواصلت کے لیے ROS 2 کا استعمال کرتا ہے۔ نوڈز ROS 2 ایپلی کیشنز کے بنیادی عناصر ہیں۔

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('کم از کم نوڈ تیار کیا گیا')

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

### ٹاپکس اور میسج
ٹاپکس نامزد بس ہیں جن کے ذریعے نوڈز میسج کا تبادلہ کرتے ہیں۔ میسج ڈیٹا سٹرکچر ہیں جو نوڈز کے درمیان تبدیل کیے جاتے ہیں۔

```python
from std_msgs.msg import String  # میسج ٹائپ

# پبلشر مثال
publisher = node.create_publisher(String, 'topic_name', 10)

# سبسکرائیب مثال
subscriber = node.create_subscription(
    String,
    'topic_name',
    callback_function,
    10
)
```

### سروسز
سروسز نوڈز کے درمیان درخواست-جواب کمیونیکیشن پیٹرن فراہم کرتے ہیں۔

```python
from example_interfaces.srv import AddTwoInts

# سروس سرور
service = node.create_service(AddTwoInts, 'add_two_ints', callback_function)

# سروس کلائنٹ
client = node.create_client(AddTwoInts, 'add_two_ints')
```

### ایکشنز
ایکشنز طویل چلنے والے کاموں کے لیے استعمال ہوتے ہیں جن میں فیڈ بیک اور گول مینجمنٹ ہوتا ہے۔

```python
from example_interfaces.action import Fibonacci
import rclpy.action

# ایکشن سرور
action_server = rclpy.action.ActionServer(
    node,
    Fibonacci,
    'fibonacci',
    execute_callback
)

# ایکشن کلائنٹ
action_client = rclpy.action.ActionClient(node, Fibonacci, 'fibonacci')
```

## 5.3 rclpy: پائی تھون کلائنٹ لائبریری

rclpy ROS 2 کے لیے پائی تھون کلائنٹ لائبریری ہے۔ یہ ROS 2 کے لیے پائی تھون API فراہم کرتا ہے۔

### ایک پبلشر نوڈ تیار کرنا

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # سیکنڈ
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'ہیلو ورلڈ: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'پبلش کر رہا ہے: "{msg.data}"')
        self.i += 1
```

### ایک سبسکرائیب نوڈ تیار کرنا

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
        self.subscription  # غیر استعمال شدہ متغیر کی چیتavari rukhna

    def listener_callback(self, msg):
        self.get_logger().info(f'میں نے سنا: "{msg.data}"')
```

## 5.4 پیرامیٹرز اور کنفیگریشن

ROS 2 نوڈز پیرامیٹر استعمال کر سکتے ہیں تاکہ ان کا رویہ رن ٹائم پر تشکیل دیا جا سکے۔

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # پیرامیٹر کا اعلان کریں
        self.declare_parameter('my_parameter', 'default_value')
        self.declare_parameter('threshold', 0.5)

        # پیرامیٹر ویلیو حاصل کریں
        param_value = self.get_parameter('my_parameter').value
        threshold = self.get_parameter('threshold').value

        self.get_logger().info(f'پیرامیٹر ویلیو: {param_value}')
        self.get_logger().info(f'تھریشولڈ: {threshold}')
```

## 5.5 لانچ فائلز اور سسٹم کمپوزیشن

لانچ فائلز آپ کو ایک کمانڈ کے ساتھ متعدد نوڈز شروع کرنے کی اجازت دیتی ہیں۔

### پائی تھون لانچ فائل

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

### کمپوزیشن
نوڈ کمپوزیشن متعدد نوڈز کو بہتر کارکردگی کے لیے ایک ہی عمل میں چلنے کی اجازت دیتا ہے۔

```python
from rclpy.node import Node
from rclpy import executors
from example_nodes import MinimalPublisher, MinimalSubscriber

class ComposedNode(Node):
    def __init__(self):
        super().__init__('composed_node')
        self.publisher = MinimalPublisher()
        self.subscriber = MinimalSubscriber()

        # ایگزیکیوٹر بنائیں اور نوڈز شامل کریں
        self.executor = executors.MultiThreadedExecutor()
        self.executor.add_node(self.publisher)
        self.executor.add_node(self.subscriber)
```

## 5.6 کوالٹی آف سروس (QoS) ترتیبات

QoS ترتیبات کنٹرول کرتی ہیں کہ میسج کیسے ڈیلیور اور ہینڈل کیے جاتے ہیں۔

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# QoS پروفائل بنائیں
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

# پبلشر میں استعمال کریں
publisher = self.create_publisher(String, 'topic', qos_profile)

# سبسکرائیب میں استعمال کریں
subscriber = self.create_subscription(
    String,
    'topic',
    callback,
    qos_profile
)
```

## 5.7 حسب ضرورت میسج کے ساتھ کام کرنا

### حسب ضرورت میسج ٹائپس تیار کرنا

ایک فائل `msg/Num.msg` تیار کریں:
```
int64 num
```

### حسب ضرورت میسج استعمال کرنا

```python
from my_package.msg import Num  # حسب ضرورت میسج

class CustomMessageNode(Node):
    def __init__(self):
        super().__init__('custom_message_node')
        self.publisher = self.create_publisher(Num, 'custom_topic', 10)

    def publish_custom_message(self, number):
        msg = Num()
        msg.num = number
        self.publisher.publish(msg)
```

## 5.8 ROS 2 ٹولز اور کمانڈز

### ضروری ROS 2 کمانڈز

```bash
# تمام نوڈز کی فہرست
ros2 node list

# ٹاپکس کی فہرست
ros2 topic list

# ٹاپک کو ایکو کریں
ros2 topic echo /topic_name

# سروس کال کریں
ros2 service call /service_name service_type "{request_field: value}"

# پیرامیٹر کی فہرست
ros2 param list

# پیرامیٹر ویلیو حاصل کریں
ros2 param get /node_name parameter_name

# پیرامیٹر ویلیو سیٹ کریں
ros2 param set /node_name parameter_name value
```

## 5.9 ROS 2 ڈیولپمنٹ کے لیے بہترین طریقے

1. **نوڈ ڈیزائن**: نوڈز کو واحد ذمہ داریوں پر مرکوز رکھیں
2. **ٹاپکس کا نام**: تفصیلی، مسلسل نامزد کونوینشن استعمال کریں
3. **خرابی کا سامنا**: مناسب خرابی کا سامنا اور لاگنگ نافذ کریں
4. **ریسورس مینجمنٹ**: نوڈز کو تباہ کرتے وقت وسائل کی مناسب صفائی کریں
5. **ٹیسٹنگ**: اپنے نوڈز اور اجزاء کے لیے یونٹ ٹیسٹ لکھیں
6. **دستاویزات**: اپنے حسب ضرورت میسج، سروسز، اور نوڈز کو دستاویز کریں

## خلاصہ

ROS 2 روبوٹ سافٹ ویئر ڈیولپمنٹ کے لیے ایک مضبوط فریم ورک فراہم کرتا ہے جس میں ROS 1 کے مقابلے بہتر آرکیٹیکچر ہے۔ نوڈز، ٹاپکس، سروسز، اور دیگر بنیادی تصورات کو سمجھنا تقسیم شدہ روبوٹک سسٹم تیار کرنے کے لیے ضروری ہے۔ rclpy لائبریری پائی تھون ڈیولپرز کو پیچیدہ روبوٹک ایپلی کیشنز تیار کرنے کے لیے طاقتور ٹولز فراہم کرتا ہے۔

## اگلے اقدامات

اگلے حصے میں، ہم ROS 2 کے اعلیٰ تصورات کا جائزہ لیں گے بشمول لانچ فائلز، سسٹم کمپوزیشن، اور تقسیم شدہ روبوٹکس سسٹم۔

<div class="alert alert-info" dir="rtl">
  <h5>ہارڈ ویئر کی ضرورت</h5>
  <div><strong>ضرورت:</strong> GPU</div>
  <div><strong>کم از کم:</strong> کوئی بھی</div>
  <div><strong>تجویز کردہ:</strong> کوئی بھی</div>
  <div><strong>مقصد:</strong> ہفتہ 5 ROS 2 بنیاد کے لیے بنیادی کمپیوٹیشنل ضروریات</div>
</div>