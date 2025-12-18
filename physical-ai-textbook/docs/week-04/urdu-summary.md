---
title: "ہفتہ 4: ڈیجیٹل ٹوئنز اور ایج کمپیوٹنگ"
week: 4
module: "فزیکل ای آئی کی بنیاد"
difficulty: "متوسط"
prerequisites: ["physics-simulation", "urdf", "ros2-basics"]
learning_objectives:
  - "'ڈیجیٹل ٹوئن ورک اسٹیشن' اور 'ایج کٹ' میں تمیز کریں"
  - "سینسر سیمولیشن نافذ کریں"
  - "'سیم ٹو ریئل' ٹرانسفر چیلنجز سمجھیں"
tags: ["digital-twins", "edge-computing", "jetson", "unity", "simulation", "urdu"]
hardware_requirements:
  - gpu: "RTX 4070 یا Jetson Orin Nano"
  - ram: "16GB کم از کم"
  - os: "Ubuntu 22.04 LTS"
duration: "90 منٹ"
---

# ہفتہ 4: ڈیجیٹل ٹوئنز اور ایج کمپیوٹنگ

## سیکھنے کے اہداف
- 'ڈیجیٹل ٹوئن ورک اسٹیشن' اور 'ایج کٹ' (Jetson Orin Nano) میں تمیز کریں
- سینسر سیمولیشن نافذ کریں
- 'سیم ٹو ریئل' ٹرانسفر چیلنجز اور حل سمجھیں
- ہارڈ ویئر سیمولیشن کے لیے یونیٹی کا جائزہ لیں

## 4.1 ڈیجیٹل ٹوئن آرکیٹیکچر: ورک اسٹیشن بمقابلہ ایج

روبوٹکس میں ڈیجیٹل ٹوئنز کو مختلف آرکیٹیکچر پیٹرنز میں نافذ کیا جا سکتا ہے جس کی بنیاد کمپیوٹیشنل ضروریات اور ریل ٹائم کنٹرولز پر ہوتی ہے۔

### ڈیجیٹل ٹوئن ورک اسٹیشن

ورک اسٹیشن کا نقطہ نظر طاقتور کمپیوٹنگ وسائل کا استعمال کرتا ہے:

**افضیات:**
- زیادہ معیار کی فزکس سیمولیشن
- پیچیدہ سینسر ماڈلنگ
- اعلیٰ کوالٹی رینڈرنگ صلاحیات
- وسیع کمپیوٹیشنل وسائل
- متعدد ہم وقت سیمولیشن کی حمایت

**نقصانات:**
- زیادہ لاگت
- نیٹ ورک میں تاخیر کے مسائل
- کم قابل نقل
- بجلی کی کھپت

### ایج کٹ (Jetson Orin Nano)

ایج کمپیوٹنگ ڈیجیٹل ٹوئن کی صلاحیات کو جسمانی روبوٹ کے قریب لاتا ہے:

**افضیات:**
- کم تاخیر
- کم بینڈ وڈتھ کی ضرورت
- ریل ٹائم پروسیسنگ
- قابل نقل
- لاگت مؤثر

**نقصانات:**
- محدود کمپیوٹیشنل وسائل
- کم سیمولیشن معیار
- حرارتی پابندیاں
- میموری کی حدود

### آرکیٹیکچر موازنہ

```python
class DigitalTwinArchitecture:
    def __init__(self, architecture_type):
        self.type = architecture_type
        self.computational_power = self._get_computational_power()
        self.latency = self._get_latency()
        self.power_consumption = self._get_power_consumption()

    def _get_computational_power(self):
        if self.type == "workstation":
            return "زیادہ (RTX 4070+)"
        elif self.type == "edge":
            return "متوسط (Jetson Orin Nano)"

    def _get_latency(self):
        if self.type == "workstation":
            return "متوسط سے زیادہ (نیٹ ورک پر منحصر)"
        elif self.type == "edge":
            return "کم (مقامی پروسیسنگ)"

    def _get_power_consumption(self):
        if self.type == "workstation":
            return "زیادہ (500W+)"
        elif self.type == "edge":
            return "کم (15-25W)"
```

## 4.2 Jetson Orin Nano برائے ایج نفاذ

### ہارڈ ویئر تفصیلات

Jetson Orin Nano ایج ای آئی ایپلی کیشنز کے لیے ڈیزائن کیا گیا ہے:

- **GPU**: 1024-core NVIDIA Ampere architecture GPU
- **CPU**: 6-core ARM Cortex-A78AE v8.2 64-bit CPU
- **میموری**: 4GB یا 8GB LPDDR5
- **بجلی**: 15W سے 25W TDP
- **کنیکٹیویٹی**: گیگابٹ ایتھر نیٹ، M.2 Key M slot for NVMe storage

### Jetson کو روبوٹکس کے لیے سیٹ کرنا

```bash
# سسٹم اپ ڈیٹ کریں
sudo apt update && sudo apt upgrade

# Jetson کے لیے ROS 2 Humble انسٹال کریں
sudo apt install ros-humble-ros-base
sudo apt install ros-humble-gazebo-ros-pkgs

# NVIDIA لائبریریز انسٹال کریں
sudo apt install nvidia-jetpack

# روبوٹکس لائبریریز انسٹال کریں
pip3 install numpy scipy matplotlib
```

### ایج کے لیے بہترین سیمولیشن

```python
import numpy as np
import time

class EdgeOptimizedSimulator:
    """Jetson Orin Nano کے لیے ہلکی سیمولیشن"""

    def __init__(self):
        self.dt = 0.01  # کارکردگی کے لیے بڑا ٹائم سٹیپ
        self.max_joints = 6  # پیچیدگی میں کمی
        self.use_approximate_physics = True

    def forward_kinematics(self, joint_angles):
        """ایج کمپیوٹنگ کے لیے بہترین فارورڈ کنیمیٹکس"""
        # سادہ کنیمیٹکس حساب
        positions = []
        cumulative_angle = 0

        for i, (angle, length) in enumerate(zip(joint_angles, self.link_lengths)):
            cumulative_angle += angle
            x = sum([l * np.cos(sum(joint_angles[:j+1])) for j, l in enumerate(self.link_lengths[:i+1])])
            y = sum([l * np.sin(sum(joint_angles[:j+1])) for j, l in enumerate(self.link_lengths[:i+1])])
            positions.append([x, y])

        return positions

    def run_simulation_step(self):
        """ایج کے لیے ایک سیمولیشن سٹیپ چلائیں"""
        start_time = time.time()

        # سادہ فزکس حسابات انجام دیں
        self.update_positions()
        self.check_collisions_approximate()

        # یقینی بنائیں کہ ہم ٹائمنگ کی پابندیوں سے تجاوز نہ کریں
        elapsed = time.time() - start_time
        if elapsed < self.dt:
            time.sleep(self.dt - elapsed)
```

## 4.3 ہارڈ ویئر سیمولیشن کے لیے یونیٹی

یونیٹی روبوٹکس کے لیے زیادہ معیار کے گرافکس اور فزکس سیمولیشن کی صلاحیات فراہم کرتا ہے:

### NVIDIA Isaac Unity انضمام

NVIDIA Isaac Unity یونیٹی کی رینڈرنگ صلاحیات کو روبوٹکس سیمولیشن کے ساتھ جوڑتا ہے:

- **فوٹو ریلیسٹک رینڈرنگ**: زیادہ معیار کی وژول سیمولیشن
- **فزکس انجن**: PhysX برائے حقیقی فزکس
- **ROS 2 برج**: ROS 2 کے ساتھ بے داغ انضمام
- **AI تربیت کا ماحول**: ڈومین رینڈمائزیشن کی حمایت

## 4.4 ڈیجیٹل ٹوئنز میں سینسر سیمولیشن

### حقیقی سینسر ماڈلنگ

سینسر سیمولیشن کو حقیقی دنیا کی نا مکمل معلومات کا خیال رکھنا چاہیے:

```python
import numpy as np

class SensorSimulator:
    """ناصر ماڈلز کے ساتھ حقیقی سینسر سیمولیشن"""

    def __init__(self):
        self.camera_noise_params = {
            'gaussian_std': 0.005,
            'poisson_lambda': 0.01,
            'uniform_range': 0.001
        }

        self.lidar_noise_params = {
            'range_std': 0.01,  # 1cm معیاری انحراف
            'angular_std': 0.001,  # 0.057 ڈگری
            'dropout_rate': 0.001  # 0.1% ڈراپ آؤٹ
        }

    def simulate_camera(self, ideal_image):
        """ناصر کے ساتھ حقیقی کیمرہ سینسر کی شبیہ کاری کریں"""
        # گاؤسین نوائز شامل کریں
        gaussian_noise = np.random.normal(0, self.camera_noise_params['gaussian_std'], ideal_image.shape)

        # پوائسن نوائز شامل کریں (فوٹون نوائز)
        poisson_noise = np.random.poisson(self.camera_noise_params['poisson_lambda'], ideal_image.shape)

        # یونیفارم نوائز شامل کریں
        uniform_noise = np.random.uniform(-self.camera_noise_params['uniform_range'],
                                        self.camera_noise_params['uniform_range'],
                                        ideal_image.shape)

        noisy_image = ideal_image + gaussian_noise + poisson_noise + uniform_noise

        # یقینی بنائیں کہ ویلیو موزوں حد میں ہیں
        noisy_image = np.clip(noisy_image, 0, 255)

        return noisy_image.astype(np.uint8)

    def simulate_imu(self, true_acceleration, true_angular_velocity):
        """بائس اور نوائز کے ساتھ حقیقی IMU کی شبیہ کاری کریں"""
        # ایکسیلرومیٹر بائس اور نوائز
        accel_bias = np.random.normal(0, 0.01, 3)  # 0.01 m/s² بائس
        accel_noise = np.random.normal(0, 0.02, 3)  # 0.02 m/s² نوائز

        # جائیرو اسکوپ بائس اور نوائز
        gyro_bias = np.random.normal(0, 0.001, 3)  # 0.001 rad/s بائس
        gyro_noise = np.random.normal(0, 0.002, 3)  # 0.002 rad/s نوائز

        measured_accel = true_acceleration + accel_bias + accel_noise
        measured_gyro = true_angular_velocity + gyro_bias + gyro_noise

        return measured_accel, measured_gyro
```

## 4.5 VSLAM (وژول سیمولٹینیس لوکلائزیشن اینڈ میپنگ)

### ڈیجیٹل ٹوئنز میں VSLAM

وژول SLAM خود مختار نیویگیشن اور ماحول کی تفہیم کے لیے اہم ہے:

```python
import cv2
import numpy as np
from collections import deque

class VSLAMSimulator:
    """ڈیجیٹل ٹوئن ماحول میں وژول SLAM سیمولیشن"""

    def __init__(self):
        self.feature_detector = cv2.SIFT_create()
        self.matcher = cv2.BFMatcher()
        self.camera_matrix = None
        self.r_t_history = deque(maxlen=100)  # پوز ہسٹری سٹور کریں
        self.map_points = {}  # 3D میپ پوائنٹس سٹور کریں
```

## 4.6 "سیم ٹو ریئل" ٹرانسفر چیلنجز

### ریئلٹی گیپ مسئلہ

"سیم ٹو ریئل" گیپ سیمولیشن اور حقیقی دنیا کی کارکردگی کے درمیان فرق کو ظاہر کرتا ہے:

**جسمانی فروق:**
- بالکل روبوٹ ڈائی نامکس ماڈلنگ
- سطح کا اصطکاک تغیر
- ایکٹو ایٹر ردعمل کے اوقات
- سینسر نوائز کی خصوصیات

**ماحولیاتی فروق:**
- لائٹنگ کی حالتیں
- سطح کے ڈھنگ
- درجہ حرارت کے تغیرات
- بیرونی متزلزل

### ڈومین رینڈمائزیشن

ڈومین رینڈمائزیشن ریئلٹی گیپ کو پُر کرنے میں مدد کرتا ہے:

```python
import random

class DomainRandomizer:
    """ڈومین رینڈمائزیشن کے لیے سیمولیشن پیرامیٹر رینڈمائز کریں"""

    def __init__(self):
        self.param_ranges = {
            'friction': (0.1, 0.8),
            'mass_multiplier': (0.8, 1.2),
            'inertia_multiplier': (0.8, 1.2),
            'gravity_multiplier': (0.9, 1.1),
            'camera_noise_std': (0.001, 0.01),
            'lidar_noise_std': (0.005, 0.02)
        }

    def randomize_environment(self):
        """ماحولیاتی پیرامیٹر رینڈمائز کریں"""
        randomized_params = {}
        for param, (min_val, max_val) in self.param_ranges.items():
            randomized_params[param] = random.uniform(min_val, max_val)
        return randomized_params
```

## 4.7 ایج بمقابلہ کلاؤڈ آرکیٹیکچر پیٹرنز

### ہائبرڈ آرکیٹیکچر

ہائبرڈ نقطہ نظر ایج اور کلاؤڈ کمپیوٹنگ کے فوائد کو جوڑتا ہے:

```python
class HybridDigitalTwin:
    """ایج اور کلاؤڈ کو جوڑنے والی ہائبرڈ ڈیجیٹل ٹوئن آرکیٹیکچر"""

    def __init__(self):
        self.edge_twin = EdgeDigitalTwin()
        self.cloud_twin = CloudDigitalTwin()
        self.sync_threshold = 0.1  # جب خامی حد سے تجاوز کرے تو سینک کریں
```

## 4.8 عملی نفاذ کے خیالات

### ایج کے لیے کارکردگی کی بہتری

```python
class OptimizedEdgeTwin:
    """ایج نفاذ کے لیے بہترین ڈیجیٹل ٹوئن"""

    def __init__(self):
        self.use_approximate_models = True
        self.reduced_visual_fidelity = True
        self.lower_physics_accuracy = True
        self.optimized_meshes = True
```

## 4.9 Jetson Orin Nano مخصوص بہتری

### Jetson کے لیے CUDA بہتری

```python
import cupy as cp  # Jetson پر GPU تیزی کے لیے CuPy استعمال کریں

class JetsonOptimizedSimulator:
    """Jetson Orin Nano GPU کے لیے بہترین سیمولیٹر"""

    def __init__(self):
        # GPU ارےز کو متعارف کرائیں
        self.joint_positions_gpu = cp.zeros(6, dtype=cp.float32)
        self.link_lengths_gpu = cp.array([0.5, 0.4, 0.3], dtype=cp.float32)
        self.transform_matrices_gpu = cp.zeros((4, 4, 6), dtype=cp.float32)
```

## خلاصہ

ڈیجیٹل ٹوئنز فزیکل ای آئی سسٹم کا ایک اہم جزو ہیں، جو سیمولیشن اور حقیقت کے درمیان گیپ کو پُر کرتے ہیں۔ ورک اسٹیشن اور ایج آرکیٹیکچر کے انتخاب کی بنیاد مخصوص درخواست کی ضروریات، کمپیوٹیشنل پابندیاں، اور ریل ٹائم کارکردگی کی ضروریات پر ہوتی ہے۔ "سیم ٹو ریئل" ٹرانسفر چیلنجز کو سمجھنا اور مناسب ڈومین رینڈمائزیشن تکنیکیں نافذ کرنا روبوٹکس سسٹم کے مربوط نفاذ کے لیے اہم ہے۔

## اگلے اقدامات

اگلے موڈیول میں، ہم ROS 2 کے بنیادیات کا جائزہ لیں گے اور ڈیجیٹل ٹوئن آرکیٹیکچر کیسے روبوٹکس میڈیاٹر سسٹم کے ساتھ انضمام کرتے ہیں۔

<div class="alert alert-warning" dir="rtl">
  <h5>ہارڈ ویئر کی ضرورت</h5>
  <div><strong>ضرورت:</strong> GPU</div>
  <div><strong>کم از کم:</strong> RTX 4070 یا Jetson Orin Nano</div>
  <div><strong>تجویز کردہ:</strong> RTX 4080 یا Jetson Orin Nano (اعلیٰ طاقت کا موڈ)</div>
  <div><strong>مقصد:</strong> ڈیجیٹل ٹوئن سیمولیشن کو کافی کمپیوٹیشنل وسائل کی ضرورت ہوتی ہے، خاص طور پر ریل ٹائم کارکردگی کے لیے</div>
</div>