---
title: "ہفتہ 3: فزکس سیمولیشن اور ڈیجیٹل ٹوئنز"
week: 3
module: "فزیکل ای آئی کی بنیاد"
difficulty: "متوسط"
prerequisites: ["kinematics", "python-basics", "physics-basics"]
learning_objectives:
  - "سیمولیشن ماحول کو سیٹ اپ اور تشکیل دیں"
  - "روبوٹ کے ماڈلز کا URDF بنائیں"
  - "فزکس سیمولیشن کے اصول سمجھیں"
tags: ["gazebo", "urdf", "simulation", "physics", "digital-twins", "urdu"]
hardware_requirements:
  - gpu: "RTX 4070 یا اس سے زیادہ"
  - ram: "16GB کم از کم"
  - os: "Ubuntu 22.04 LTS"
duration: "90 منٹ"
---

# ہفتہ 3: فزکس سیمولیشن اور ڈیجیٹل ٹوئنز

## سیکھنے کے اہداف
- سیمولیشن ماحول کو سیٹ اپ اور تشکیل دیں
- روبوٹ کے ماڈلز کا URDF بنائیں
- فزکس سیمولیشن کے اصول سمجھیں
- ورچوئل ماحول میں سینسر سیمولیشن نافذ کریں

## 3.1 فزکس سیمولیشن کا تعارف

فزکس سیمولیشن فزیکل ای آئی کی ترقی کا ایک ستون ہے، جو حقیقی ہارڈ ویئر پر عمل کرنے سے پہلے الگورتھم کو ٹیسٹ کرنے کے لیے ایک محفوظ، قیمت کے لحاظ سے مؤثر ماحول فراہم کرتا ہے۔ یہ فراہم کرتا ہے:

- **الگورتھم کی توثیق**: کنٹرول اور منصوبہ بندی کے الگورتھم کو ایک کنٹرول ماحول میں ٹیسٹ کریں
- **ڈیٹا جنریشن**: مشین لرننگ ماڈلز کے لیے تربیتی ڈیٹا سیٹ بنائیں
- **سیفٹی ٹیسٹنگ**: ہارڈ ویئر یا انسانوں کے لیے رسک کے بغیر سیفٹی کریٹیکل سسٹم کی توثیق کریں
- **کارکردگی کی بہتری**: حقیقی دنیا کے نفاذ سے پہلے پیرامیٹر کو فائن ٹیون کریں

### ڈیجیٹل ٹوئن کی تصور

ایک ڈیجیٹل ٹوئن ایک جسمانی سسٹم کی ورچوئل نمائندگی ہے جو اس کی خصوصیات، حالتیں، اور سلوک کو حقیقی وقت میں عکاس کرتا ہے۔ روبوٹکس میں، ڈیجیٹل ٹوئنز فراہم کرتے ہیں:

- **ورچوئل ٹیسٹنگ**: ہارڈ ویئر کے نفاذ سے پہلے الگورتھم کی توثیق
- **پریڈکٹو مینٹیننس**: سسٹم کے سلوک کو مانیٹر اور پریڈکٹ کریں
- **آپٹیمائزیشن**: حقیقی دنیا کے نفاذ سے پہلے پیرامیٹر کو فائن ٹیون کریں
- **ٹریننگ**: ہارڈ ویئر کی پابندیوں کے بغیر ای آئی ماڈلز کو ترقی دیں اور ٹیسٹ کریں

## 3.2 سیمولیشن ماحول کا جائزہ

### گیزبو بمقابلہ دیگر پلیٹ فارم

**گیزبو** (اب اگنیشن گیزبو):
- اوپن سورس فزکس سیمولیٹر
- ROS کی مضبوط انضمام
- حقیقی فزکس انجن (ODE، بُلیٹ، سیم بอดی)
- وسیع ماڈل ڈیٹا بیس

**یونیٹی**:
- گیم انجن کو روبوٹکس کے لیے اڈاپٹ کیا گیا
- اعلیٰ معیار کے گرافکس
- کراس پلیٹ فارم سپورٹ
- NVIDIA Isaac انضمام

**پائی بُلیٹ**:
- پائی تھون کے دوست فزکس انجن
- حقیقی وقت کی سیمولیشن صلاحیات
- ریفورسمنٹ لرننگ کے لیے اچھا
- کراس پلیٹ فارم

## 3.3 URDF (یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ)

URDF ایک XML-مبنی فارمیٹ ہے جو روبوٹ ماڈلز کی نمائندگی کے لیے استعمال ہوتا ہے۔ یہ روبوٹ کی جسمانی اور وژول خصوصیات کو بیان کرتا ہے۔

### URDF کی ساخت

ایک عام URDF فائل میں یہ شامل ہوتا ہے:

```xml
<?xml version="1.0"?>
<robot name="example_robot">
  <!-- لنکس جڑی جسم کی نمائندگی کرتے ہیں -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- جوڑیاں لنکس کو جوڑتی ہیں -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="1000.0" velocity="1.0"/>
  </joint>
</robot>
```

### URDF کے اہم جز

1. **لنکس**: جڑی جسم کی نمائندگی کرتے ہیں وژول، کولیژن، اور انیشل خصوصیات کے ساتھ
2. **جوڑیاں**: لنکس کے درمیان کنکشن کی وضاحت کرتے ہیں مخصوص ڈگریز آف فریڈم کے ساتھ
3. **میٹریلز**: وژول ظہور کی وضاحت کرتے ہیں
4. **ٹرانسمیشنز**: وضاحت کرتے ہیں کہ ایکٹو ایٹرز جوڑیوں سے کیسے منسلک ہوتے ہیں

## 3.4 فزکس سیمولیشن کے اصول

### ڈائی نامکس اور کنیمیٹکس

فزکس سیمولیشن کنیمیٹک اور ڈائی نامکس کے حسابات کو جوڑتا ہے:

- **کنیمیٹکس**: جیومیٹرک تعلقات (پوزیشن، رفتار) بروئے قوت کے بغیر
- **ڈائی نامکس**: قوتوں اور ٹورک کے اثر کے تحت موشن

### انٹیگریشن طریقے

سیمولیشن انجن مساوات حرکت کو حل کرنے کے لیے عددی انٹیگریشن استعمال کرتے ہیں:

- **ایولر میتھڈ**: سادہ لیکن کم درست
- **رُنگ کُٹا**: زیادہ درست لیکن مہنگا کمپیوٹیشنل
- **ورلیٹ انٹیگریشن**: فزکس سیمولیشن میں استحکام کے لیے اچھا

### کولیژن ڈیٹیکشن

کولیژن ڈیٹیکشن سسٹم شناخت کرتے ہیں جب اشیاء میں ٹکراؤ ہوتا ہے:

- **براڈ فیز**: غیر جڑنے والے جوڑوں کو جلدی ہٹا دیتے ہیں
- **نیرو فیز**: درست کولیژن ڈیٹیکشن
- **کنٹینیوئس کولیژن ڈیٹیکشن**: زیادہ رفتار پر اشیاء کے ایک دوسرے کو کاٹنے سے روکتا ہے

## 3.5 فزکس سیمولیشن کو سیٹ کرنا

### گیزبو (اگنیشن) کا انسٹال کرنا

```bash
# Ubuntu 22.04 کے لیے
sudo apt update
sudo apt install ignition-harmonic
# یا ROS 2 ہمبل کے لیے
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
```

### بنیادی گیزبو لانچ

```bash
# خالی دنیا کے ساتھ گیزبو لانچ کریں
ign gazebo -r empty.sdf
# یا ROS 2 کے ساتھ
ros2 launch gazebo_ros empty_world.launch.py
```

## 3.6 پیچیدہ روبوٹ ماڈلز بنانا

### ملٹی لنک روبوٹ URDF

```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <!-- بیس لنک -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.15"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <!-- شولڈر جوڑی اور لنک -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="1000" velocity="1"/>
  </joint>

  <link name="shoulder_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.2"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- البو جوڑی اور لنک -->
  <joint name="elbow_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="elbow_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="1000" velocity="1"/>
  </joint>

  <link name="elbow_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0005"/>
    </inertial>
  </link>
</robot>
```

## 3.7 سینسر سیمولیشن

### سیمولیشن میں سینسر کی اقسام

1. **کیمرہ سینسرز**: وژول ادراک
2. **لائیڈار**: 3D میپنگ اور رکاوٹ کا پتہ لگانا
3. **IMU**: اندرونی پیمائش
4. **فورس/ٹورک سینسرز**: رابطہ کی قوتیں
5. **GPS**: پوزیشن کا تخمینہ

### URDF میں سینسرز شامل کرنا

```xml
<!-- کیمرہ سینسر -->
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <always_on>1</always_on>
    <update_rate>30</update_rate>
    <visualize>true</visualize>
  </sensor>
</gazebo>

<!-- IMU سینسر -->
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>1</always_on>
    <update_rate>100</update_rate>
    <visualize>false</visualize>
  </sensor>
</gazebo>
```

## 3.8 فزکس پیرامیٹرز اور ٹیوننگ

### گریویٹی اور ماحولیاتی ترتیبات

گریویٹی کو دنیا کی فائل میں مختلف ماحول کی شبیہہ کاری کے لیے تبدیل کیا جا سکتا ہے:

```xml
<world name="custom_gravity">
  <gravity>0 0 -1.62</gravity>  <!-- چاند کی گریویٹی -->
  <physics type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
  </physics>
</world>
```

### میٹریل خصوصیات

میٹریل کی خصوصیات کولیژن کے سلوک کو متاثر کرتی ہیں:

```xml
<gazebo reference="link_name">
  <mu1>0.2</mu1>  <!-- فریکشن کوائفیسینٹ -->
  <mu2>0.2</mu2>  <!-- فریکشن کوائفیسینٹ (دوسرا سمت) -->
  <kp>1000000.0</kp>  <!-- سپرنگ سٹفنس -->
  <kd>1.0</kd>        <!-- ڈیمپنگ کوائفیسینٹ -->
</gazebo>
```

## 3.9 سیمولیشن کنٹرول کے لیے پائی تھون نافذ کاری

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

class SimulationController(Node):
    """سیمولیشن روبوٹ کا کنٹرولر"""

    def __init__(self):
        super().__init__('simulation_controller')

        # جوڑی کمانڈز کے لیے پبلشر
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # مسلسل کنٹرول اپ ڈیٹس کے لیے ٹائمر
        self.timer = self.create_timer(0.1, self.control_loop)

        # ہمارے روبوٹ کے جوڑی کے نام
        self.joint_names = ['shoulder_joint', 'elbow_joint']

        # مطلوبہ جوڑی کی پوزیشنز
        self.desired_positions = [0.0, 0.0]

        # ٹریجکٹری جنریشن کے لیے ٹائم کاؤنٹر
        self.time_counter = 0.0

    def control_loop(self):
        """مرکزی کنٹرول لوپ"""
        # ایک سادہ اوسیلیٹنگ ٹریجکٹری جنریٹ کریں
        self.time_counter += 0.1

        # ایک ہموار اوسیلیٹنگ موشن بنائیں
        shoulder_pos = 0.5 * np.sin(self.time_counter * 0.5)
        elbow_pos = 0.3 * np.cos(self.time_counter * 0.3)

        # ٹریجکٹری میسج بنائیں
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [shoulder_pos, elbow_pos]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000  # 0.1 سیکنڈ

        traj_msg.points = [point]

        # ٹریجکٹری کو پبلش کریں
        self.joint_pub.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)

    controller = SimulationController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3.10 سیمولیشن کی توثیق اور ٹیسٹنگ

### سیمولیشن کی درستگی کی جانچ

سیمولیشن کی درستگی کی توثیق کرنے کے لیے:

1. **تحلیلی ماڈلز سے موازنہ کریں**: یقینی بنائیں کہ کنیمیٹک حل ریاضی کے حساب سے مماثل ہیں
2. **کنسرویشن لازم کی چیک کریں**: یقینی بنائیں کہ توانائی اور تابکاری مناسب طریقے سے محفوظ ہیں
3. **سینسر ماڈلز کی توثیق کریں**: موازنہ کریں کہ سیمولیٹڈ سینسر ڈیٹا متوقع ویلیو سے مماثل ہے
4. **ایج کیسز کو ٹیسٹ کریں**: جوڑی کی حدود اور کولیژن منظار ناموں کے لیے سلوک کی توثیق کریں

### کارکردگی کے خیالات

- **ٹائم سٹیپ**: چھوٹے اسٹیپس درستگی میں اضافہ کرتے ہیں لیکن کارکردگی کم کرتے ہیں
- **ریل ٹائم فیکٹر**: سیمولیشن ٹائم کا حقیقی ٹائم سے تناسب
- **سالور پیرامیٹرز**: استحکام اور کمپیوٹیشنل لاگت کے درمیان توازن

## 3.11 "سیم ٹو ریئل" ٹرانسفر چیلنجز

### ریئلٹی گیپ

"سیم ٹو ریئل" گیپ سیمولیشن اور حقیقت کے درمیان فرق کو اشارہ کرتا ہے:

- **ماڈل کی نا درستی**: حقیقی روبوٹس کے پاس غیر ماڈل شدہ ڈائی نامکس ہیں
- **سینسر نوائز**: سیمولیٹڈ سینسر اکثر بہت صاف ہوتے ہیں
- **ماحولیاتی عوامل**: لائٹنگ، سطح کی خصوصیات، وغیرہ
- **ایکٹو ایٹر ڈائی نامکس**: حقیقی ایکٹو ایٹر میں تاخیر اور پابندیاں ہوتی ہیں

### گیپ کو پُر کرنا

1. **ڈومین رینڈمائزیشن**: تربیت کے دوران سیمولیشن پیرامیٹر کو رینڈمائز کریں
2. **سسٹم آئیڈنٹیفکیشن**: حقیقی روبوٹ سے مماثل کرنے کے لیے سیمولیشن پیرامیٹر کو کیلیبریٹ کریں
3. **مستحکم کنٹرول**: عدم یقینی کو سنبھالنے کے لیے کنٹرولز ڈیزائن کریں
4. **پیش رفت ٹرانسفر**: تدریج سے سیمولیشن سے حقیقت کی طرف بڑھیں

## خلاصہ

فزکس سیمولیشن اور ڈیجیٹل ٹوئنز فزیکل ای آئی کی ترقی کے لیے اہم ٹولز ہیں۔ وہ حقیقی ہارڈ ویئر پر نفاذ سے پہلے الگورتھم کو ٹیسٹ اور توثیق کرنے کے لیے ایک محفوظ، کارآمد ماحول فراہم کرتے ہیں۔ URDF، فزکس کے اصول، اور سیمولیشن پیرامیٹر کو سمجھنا سیمولیشن اور حقیقت کے درمیان گیپ کو پُر کرنے کے لیے مؤثر ڈیجیٹل ٹوئنز بنانے کے لیے اہم ہے۔

## اگلے اقدامات

اگلے حصے میں، ہم ڈیجیٹل ٹوئنز کو مزید تفصیل سے دیکھیں گے، بشمول "ڈیجیٹل ٹوئن ورک اسٹیشن" بمقابلہ "ایج کٹ" تصورات اور ہارڈ ویئر سیمولیشن کے لیے یونیٹی انضمام۔

<div class="alert alert-warning" dir="rtl">
  <h5>ہارڈ ویئر کی ضرورت</h5>
  <div><strong>ضرورت:</strong> GPU</div>
  <div><strong>کم از کم:</strong> RTX 4070</div>
  <div><strong>تجویز کردہ:</strong> RTX 4080</div>
  <div><strong>مقصد:</strong> فزکس سیمولیشن کو کافی کمپیوٹیشنل وسائل کی ضرورت ہوتی ہے</div>
</div>