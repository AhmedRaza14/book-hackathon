---
title: "ہفتہ 8: کنٹرول سسٹم اور ٹریجکٹری پلاننگ"
week: 8
module: "روبوٹک انفراسٹرکچر"
difficulty: "اعلیٰ"
prerequisites: ["ros2-fundamentals", "python-basics", "robotics-concepts", "kinematics", "sensors"]
learning_objectives:
  - "روبوٹ کنٹرول سسٹم نافذ کریں"
  - "ٹریجکٹری پلاننگ الگورتھم ڈیزائن کریں"
  - "PID کنٹرولرز ٹیون کریں"
  - "ٹریجکٹریز منصوبہ بندی اور انجام دیں"
tags: ["control", "pid", "trajectory", "planning", "navigation", "motion", "urdu"]
hardware_requirements:
  - gpu: "کوئی بھی"
  - ram: "8GB کم از کم"
  - os: "Ubuntu 22.04 LTS"
duration: "90 منٹ"
---

# ہفتہ 8: کنٹرول سسٹم اور ٹریجکٹری پلاننگ

## سیکھنے کے اہداف
- روبوٹ کنٹرول سسٹم نافذ کریں
- ٹریجکٹری پلاننگ الگورتھم ڈیزائن کریں
- PID کنٹرولرز ٹیون کریں
- ٹریجکٹریز منصوبہ بندی اور انجام دیں
- سیفٹی سسٹم اور ایمرجنسی اسٹاپس کو سمجھیں

## 8.1 روبوٹ کنٹرول سسٹم کا تعارف

روبوٹ کنٹرول سسٹم روبوٹک سسٹم کے لیے بنیادی ہیں جو ہائی لیول کمانڈز کو مصدقہ موٹر ایکشنز میں تبدیل کرتے ہیں۔ وہ یقینی بناتے ہیں کہ روبوٹ اپنے ماحول میں درست اور محفوظ طریقے سے حرکت کرے۔

### کنٹرول سسٹم آرکیٹیکچر

ایک عام روبوٹ کنٹرول سسٹم میں کئی لیئرز ہوتے ہیں:

1. **ہائی لیول پلینر**: ڈیزائرڈ ٹریجکٹریز جنریٹ کرتا ہے
2. **موشن کنٹرولر**: ٹریجکٹریز کو جوائنٹ کمانڈز میں تبدیل کرتا ہے
3. **لو لیول کنٹرولر**: انفرادی جوائنٹ موٹرز کا انتظام کرتا ہے
4. **سیفٹی سسٹم**: مانیٹر کرتا ہے اور سیفٹی کنٹرینٹس کو نافذ کرتا ہے

### کنٹرول سسٹم کی اقسام

1. **پن لوپ کنٹرول**: کمانڈز بنا فیڈ بیک بھیجی جاتی ہیں
2. **کلوزڈ لوپ کنٹرول**: فیڈ بیک کا استعمال کرتے ہوئے خامیوں کی اصلاح کرتا ہے
3. **فیڈ فارورڈ کنٹرول**: ماڈل کی بنیاد پر پیڈکٹو کنٹرول
4. **اڈاپٹیو کنٹرول**: تبدیل ہوتی حالت کے مطابق پیرامیٹر ایڈجسٹ کرتا ہے

## 8.2 PID کنٹرولرز

پروپورشل-انٹیگرل-ڈیریویٹو (PID) کنٹرولرز روبوٹ کنٹرول سسٹم میں بنیادی ہیں۔

### PID کنٹرولر تھیوری

PID کنٹرولر کا آؤٹ پٹ یوں حساب کیا جاتا ہے:

$$u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}$$

جہاں:
- $u(t)$: کنٹرولر آؤٹ پٹ
- $e(t)$: خامی کا سگنل (مطلوبہ - اصل)
- $K_p$: پروپورشل گین
- $K_i$: انٹیگرل گین
- $K_d$: ڈیریویٹو گین

### PID نفاذ

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np
import time
from collections import deque

class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(-np.inf, np.inf)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits

        self.reset()

    def reset(self):
        """PID کنٹرولر کی حالت کو ری سیٹ کریں"""
        self.last_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.last_time = None

    def compute(self, setpoint, measured_value, dt=None):
        """PID آؤٹ پٹ کا حساب لگائیں"""
        current_time = time.time()

        if dt is None:
            if self.last_time is None:
                dt = 0.01  # ڈیفالٹ ٹائم سٹیپ
            else:
                dt = current_time - self.last_time
        self.last_time = current_time

        # خامی کا حساب لگائیں
        error = setpoint - measured_value

        # پروپورشل ٹرم
        proportional = self.kp * error

        # انٹیگرل ٹرم
        self.integral += error * dt
        integral = self.ki * self.integral

        # ڈیریویٹو ٹرم
        if dt > 0:
            self.derivative = (error - self.last_error) / dt
        derivative = self.kd * self.derivative

        # آؤٹ پٹ کا حساب لگائیں
        output = proportional + integral + derivative

        # آؤٹ پٹ کی حدیں لاگو کریں
        output = np.clip(output, self.output_limits[0], self.output_limits[1])

        # اگلی اسٹیپ کے لیے سٹور کریں
        self.last_error = error

        return output

class JointControllerNode(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # پبلشرز
        self.joint_cmd_pub = self.create_publisher(JointState, 'joint_commands', 10)
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )

        # PID کنٹرولرز کے لیے
        self.pid_controllers = {}
        self.joint_positions = {}
        self.joint_velocities = {}
        self.joint_targets = {}

        # ہر جوائنٹ کے لیے PID کنٹرولرز شروع کریں
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        for joint_name in joint_names:
            self.pid_controllers[joint_name] = PIDController(
                kp=10.0, ki=0.1, kd=0.5, output_limits=(-10.0, 10.0)
            )
            self.joint_targets[joint_name] = 0.0

        # کنٹرول ٹائمر
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100 Hz

    def joint_state_callback(self, msg):
        """جوائنٹ اسٹیٹ اپ ڈیٹ کریں"""
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            self.joint_positions[name] = pos
            self.joint_velocities[name] = vel

    def control_loop(self):
        """مرکزی کنٹرول لوپ"""
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.name = list(self.joint_targets.keys())

        for joint_name in self.joint_targets.keys():
            if joint_name in self.joint_positions:
                current_pos = self.joint_positions[joint_name]
                target_pos = self.joint_targets[joint_name]

                # PID آؤٹ پٹ کا حساب لگائیں
                control_output = self.pid_controllers[joint_name].compute(
                    target_pos, current_pos
                )

                cmd_msg.effort.append(control_output)

        self.joint_cmd_pub.publish(cmd_msg)
```

## 8.3 اعلیٰ کنٹرول تکنیکیں

### LQR (لینئر کویڈریٹک ریگولیٹر) کنٹرول

```python
import numpy as np
from scipy.linalg import solve_continuous_are

class LQRController:
    """LQR کنٹرولر کا نفاذ"""

    def __init__(self, A, B, Q, R):
        """
        LQR کنٹرولر کے لیے A, B, Q, R میٹرکس کے ساتھ شروع کریں
        A: سسٹم میٹرکس
        B: ان پٹ میٹرکس
        Q: سٹیٹ کوسٹ میٹرکس
        R: کنٹرول کوسٹ میٹرکس
        """
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R

        # LQR گین کا حساب لگائیں
        self.P = solve_continuous_are(self.A, self.B, self.Q, self.R)
        self.K = np.linalg.inv(self.R) @ self.B.T @ self.P

    def compute_control(self, state_error):
        """سٹیٹ خامی کے مطابق کنٹرول آؤٹ پٹ کا حساب لگائیں"""
        # LQR کنٹرول لاء: u = -K * x_error
        control_input = -self.K @ state_error
        return control_input

class ModelPredictiveController:
    """ماڈل پریڈکٹو کنٹرول کا نفاذ"""

    def __init__(self, prediction_horizon=10, control_horizon=5):
        self.prediction_horizon = prediction_horizon
        self.control_horizon = control_horizon

        # MPC پیرامیٹرز
        self.Q = np.eye(2) * 10  # سٹیٹ کوسٹ
        self.R = np.eye(1) * 0.1  # کنٹرول کوسٹ
        self.P = np.eye(2) * 5  # ٹرمنل کوسٹ

    def solve_mpc(self, current_state, reference_trajectory):
        """MPC کے مسئلے کو حل کریں"""
        # یہ ایک سادہ MPC نفاذ ہے
        # حقیقی نفاذ میں کویڈریٹک پروگرامنگ استعمال ہوتی ہے

        # ٹریجکٹری کے حوالے سے ٹریکنگ کا حساب لگائیں
        predicted_states = []
        control_sequence = []

        current_x = current_state.copy()
        for k in range(self.control_horizon):
            # سادہ پیش گوئی ماڈل (انٹیگریٹر)
            if k < len(reference_trajectory):
                reference = reference_trajectory[k]
                error = reference - current_x
                control = -0.5 * error  # سادہ فیڈ بیک
            else:
                control = 0.0

            # کنٹرول کو لاگو کریں اور اگلا سٹیٹ تلاش کریں
            current_x += control * 0.01  # dt = 0.01s
            predicted_states.append(current_x.copy())
            control_sequence.append(control)

        return control_sequence[0] if control_sequence else 0.0  # پہلا کنٹرول واپس کریں
```

## 8.4 ٹریجکٹری پلاننگ

### جوائنٹ اسپیس ٹریجکٹری پلاننگ

```python
import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

class TrajectoryPlanner:
    """جوائنٹ اسپیس میں ٹریجکٹریز منصوبہ بندی کریں"""

    def __init__(self):
        self.current_trajectory = None
        self.trajectory_time = 0.0

    def plan_polynomial_trajectory(self, start_positions, end_positions, duration):
        """کوئنٹک پولینومئل کا استعمال کرتے ہوئے ٹریجکٹری منصوبہ بندی کریں"""
        if len(start_positions) != len(end_positions):
            raise ValueError("شروع اور اختتام کی پوزیشنز کا ایک جیسا لمبائی ہونی چاہیے")

        trajectory = JointTrajectory()
        trajectory.joint_names = [f'joint_{i}' for i in range(len(start_positions))]

        # ڈیوریشن کے مطابق نمبر آف پوائنٹس (100 Hz)
        num_points = int(duration * 100)
        dt = duration / num_points

        for i in range(num_points + 1):
            t = i * dt
            fraction = min(1.0, t / duration)

            # کوئنٹک پولینومئل برائے ہموار ٹریجکٹری (0, 0, 0 شروع اور اختتام کی حالتیں)
            # s = 10*f^3 - 15*f^4 + 6*f^5 (جہاں f فریکشن ہے)
            s = 10 * fraction**3 - 15 * fraction**4 + 6 * fraction**5

            point = JointTrajectoryPoint()
            point.positions = []
            point.velocities = []
            point.accelerations = []

            for start_pos, end_pos in zip(start_positions, end_positions):
                pos = start_pos + s * (end_pos - start_pos)
                point.positions.append(pos)

                # رفتار کا حساب لگائیں (پوزیشن کا ڈیریویٹو)
                ds_dt = (30 * fraction**2 - 60 * fraction**3 + 30 * fraction**4) / duration
                vel = (end_pos - start_pos) * ds_dt
                point.velocities.append(vel)

                # ایکسلریشن کا حساب لگائیں (رفتار کا ڈیریویٹو)
                d2s_dt2 = (60 * fraction - 180 * fraction**2 + 120 * fraction**3) / (duration**2)
                acc = (end_pos - start_pos) * d2s_dt2
                point.accelerations.append(acc)

            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            trajectory.points.append(point)

        return trajectory

    def plan_spline_trajectory(self, waypoints, times):
        """کیوبک سپلائن کا استعمال کرتے ہوئے ٹریجکٹری منصوبہ بندی کریں"""
        if len(waypoints) != len(times):
            raise ValueError("ویزپوائنٹس اور ٹائمز کا ایک جیسا لمبائی ہونا چاہیے")

        trajectory = JointTrajectory()
        trajectory.joint_names = [f'joint_{i}' for i in range(len(waypoints[0]))]

        # ہر جوائنٹ کے لیے سپلائن بنائیں
        num_joints = len(waypoints[0])
        splines = []

        for j in range(num_joints):
            joint_waypoints = [wp[j] for wp in waypoints]
            spline = CubicSpline(times, joint_waypoints, bc_type='natural')
            splines.append(spline)

        # ٹریجکٹری پوائنٹس جنریٹ کریں
        total_time = times[-1]
        num_points = int(total_time * 100)  # 100 Hz

        for i in range(num_points + 1):
            t = i * (total_time / num_points)

            point = JointTrajectoryPoint()
            point.positions = [spline(t) for spline in splines]
            point.velocities = [spline(t, 1) for spline in splines]  # 1st ڈیریویٹو
            point.accelerations = [spline(t, 2) for spline in splines]  # 2nd ڈیریویٹو

            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            trajectory.points.append(point)

        return trajectory

    def plan_trapezoidal_trajectory(self, start_pos, end_pos, duration, max_vel=1.0):
        """ٹریپیزoidal ٹریجکٹری منصوبہ بندی کریں"""
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint_0']

        distance = abs(end_pos - start_pos)
        direction = 1 if end_pos > start_pos else -1

        # ایکسلریشن اور ڈیسلریشن ٹائم کا حساب لگائیں
        # متناسب ٹریپیزoidal کے لیے: acc_time = dec_time
        # کل فاصلہ = acc_distance + const_distance + dec_distance
        # acc_distance = dec_distance = 0.5 * max_vel * acc_time
        # const_distance = max_vel * const_time
        # total_distance = max_vel * acc_time + max_vel * const_time = max_vel * (acc_time + const_time)

        acc_time = max_vel / 2.0  # 2.0 rad/s^2 ایکسلریشن فرض کریں
        min_duration_for_triangle = 2 * acc_time  # ٹرائی اینگل پروفائل کے لیے کم از کم

        if duration < min_duration_for_triangle:
            # ٹرائی اینگل پروفائل (کبھی زیادہ سے زیادہ رفتار نہیں پہنچتا)
            acc_time = duration / 2.0
            actual_max_vel = max_vel * (duration / min_duration_for_triangle)
        else:
            actual_max_vel = max_vel

        const_time = duration - 2 * acc_time

        num_points = int(duration * 100)
        dt = duration / num_points

        for i in range(num_points + 1):
            t = i * dt

            if t <= acc_time:
                # ایکسلریشن فیز
                pos_fraction = 0.5 * (actual_max_vel / acc_time) * t**2
                vel = (actual_max_vel / acc_time) * t
                acc = actual_max_vel / acc_time
            elif t <= acc_time + const_time:
                # مستقل رفتار فیز
                pos_fraction = 0.5 * actual_max_vel * acc_time + \
                              actual_max_vel * (t - acc_time)
                vel = actual_max_vel
                acc = 0.0
            else:
                # ڈیسلریشن فیز
                time_in_dec = t - (acc_time + const_time)
                pos_fraction = 0.5 * actual_max_vel * acc_time + \
                              actual_max_vel * const_time + \
                              actual_max_vel * time_in_dec - \
                              0.5 * (actual_max_vel / acc_time) * time_in_dec**2
                vel = actual_max_vel - (actual_max_vel / acc_time) * time_in_dec
                acc = -actual_max_vel / acc_time

            pos = start_pos + direction * pos_fraction

            point = JointTrajectoryPoint()
            point.positions = [pos]
            point.velocities = [direction * vel]
            point.accelerations = [direction * acc]
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            trajectory.points.append(point)

        return trajectory
```

## 8.5 کارٹیزین اسپیس کنٹرول

### کارٹیزین ٹریجکٹری پلاننگ

```python
from scipy.spatial.transform import Rotation as R

class CartesianTrajectoryPlanner:
    """کارٹیزین اسپیس میں ٹریجکٹریز منصوبہ بندی کریں"""

    def plan_linear_trajectory(self, start_pose, end_pose, duration):
        """کارٹیزین اسپیس میں لکیری ٹریجکٹری منصوبہ بندی کریں"""
        # پوزیشن اور اورینٹیشن نکالیں
        start_pos = np.array(start_pose[:3])
        end_pos = np.array(end_pose[:3])

        start_rot = R.from_quat(start_pose[3:])  # [x, y, z, w]
        end_rot = R.from_quat(end_pose[3:])

        # پوزیشن ٹریجکٹری منصوبہ بندی کریں
        pos_trajectory_func = self.plan_polynomial_trajectory(
            start_pos, end_pos, duration
        )

        # اورینٹیشن ٹریجکٹری منصوبہ بندی کریں (SLERP - سpherical linear interpolation)
        def rot_trajectory_func(t):
            t_norm = np.clip(t / duration, 0, 1)
            interp_rot = start_rot.slerp(end_rot, t_norm)
            return interp_rot.as_quat()

        def trajectory_func(t):
            pos, vel, acc = pos_trajectory_func(t)
            rot = rot_trajectory_func(t)
            return pos, rot, vel, acc

        return trajectory_func

    def plan_circular_trajectory(self, center, radius, start_angle, end_angle, duration):
        """2D میں سرکولر ٹریجکٹری منصوبہ بندی کریں"""
        def trajectory_func(t):
            t_norm = np.clip(t / duration, 0, 1)
            angle = start_angle + t_norm * (end_angle - start_angle)

            x = center[0] + radius * np.cos(angle)
            y = center[1] + radius * np.sin(angle)
            z = center[2]  # مستقل اونچائی

            pos = np.array([x, y, z])

            # رفتار کا حساب لگائیں (سکرکل کا مماس)
            vel_x = -radius * np.sin(angle) * (end_angle - start_angle) / duration
            vel_y = radius * np.cos(angle) * (end_angle - start_angle) / duration
            vel_z = 0.0
            vel = np.array([vel_x, vel_y, vel_z])

            # ایکسلریشن کا حساب لگائیں (سینٹریپیٹل)
            acc_x = -radius * np.cos(angle) * ((end_angle - start_angle) / duration)**2
            acc_y = -radius * np.sin(angle) * ((end_angle - start_angle) / duration)**2
            acc_z = 0.0
            acc = np.array([acc_x, acc_y, acc_z])

            # شناخت کا گھوماؤ
            rot = np.array([0, 0, 0, 1])  # [x, y, z, w]

            return pos, rot, vel, acc

        return trajectory_func
```

## 8.6 PID ٹیوننگ کے طریقے

### زیگر نکوچ ٹیوننگ

```python
class PIDTuner:
    """PID کنٹرولرز کو ٹیون کرنے کے لیے طریقے فراہم کرتا ہے"""

    def __init__(self):
        self.method = 'ziegler_nichols'

    def ziegler_nichols_tuning(self, plant_gain, critical_period):
        """
        زیگر نکوچ ٹیوننگ کا استعمال کرتے ہوئے PID گین کا حساب لگائیں
        plant_gain: سسٹم کا گین جب اوسیلیشن شروع ہوتا ہے
        critical_period: اوسیلیشن کا دورانیہ
        """
        # زیگر نکوچ کے پی سی ایل کے لیے فارمولے
        ku = plant_gain
        tu = critical_period

        # P کنٹرولر
        kp_p = 0.5 * ku

        # PI کنٹرولر
        kp_pi = 0.45 * ku
        ti_pi = 0.8 * tu
        ki_pi = kp_pi / ti_pi

        # PID کنٹرولر
        kp_pid = 0.6 * ku
        ti_pid = 0.5 * tu
        td_pid = 0.125 * tu
        ki_pid = kp_pid / ti_pid
        kd_pid = kp_pid * td_pid

        return {
            'P': {'kp': kp_p, 'ki': 0.0, 'kd': 0.0},
            'PI': {'kp': kp_pi, 'ki': ki_pi, 'kd': 0.0},
            'PID': {'kp': kp_pid, 'ki': ki_pid, 'kd': kd_pid}
        }

    def chien_hrones_reswick_tuning(self, plant_gain, critical_period):
        """
        چین-ہرونز-ریس وک ٹیوننگ کا استعمال کرتے ہوئے PID گین کا حساب لگائیں
        """
        ku = plant_gain
        tu = critical_period

        # P کنٹرولر (0% اوور شوٹ)
        kp_p_no_overshoot = 0.3 * ku

        # PI کنٹرولر (0% اوور شوٹ)
        kp_pi_no_overshoot = 0.35 * ku
        ti_pi_no_overshoot = 1.2 * tu
        ki_pi_no_overshoot = kp_pi_no_overshoot / ti_pi_no_overshoot

        # PID کنٹرولر (0% اوور شوٹ)
        kp_pid_no_overshoot = 0.5 * ku
        ti_pid_no_overshoot = 1.4 * tu
        td_pid_no_overshoot = 0.45 * tu
        ki_pid_no_overshoot = kp_pid_no_overshoot / ti_pid_no_overshoot
        kd_pid_no_overshoot = kp_pid_no_overshoot * td_pid_no_overshoot

        return {
            'P_no_overshoot': {'kp': kp_p_no_overshoot, 'ki': 0.0, 'kd': 0.0},
            'PI_no_overshoot': {'kp': kp_pi_no_overshoot, 'ki': ki_pi_no_overshoot, 'kd': 0.0},
            'PID_no_overshoot': {
                'kp': kp_pid_no_overshoot,
                'ki': ki_pid_no_overshoot,
                'kd': kd_pid_no_overshoot
            }
        }

    def relay_auto_tuning(self, controller, plant_system):
        """
        ریلے کا استعمال کرتے ہوئے PID گینز کو خودکار طور پر ٹیون کریں
        """
        # ریلے ٹیسٹ کا نفاذ
        relay_output = 1.0  # ریلے کا آؤٹ پٹ
        oscillation_amplitude = 0.0
        oscillation_period = 0.0

        # ریلے ٹیسٹ کو چلائیں
        # یہ ایک نمونہ ہے - حقیقی نفاذ پلانٹ سسٹم کے ساتھ کام کرے گا
        # اوسیلیشن کا پتہ لگائیں اور اس کا امپلی ٹیوڈ اور دورانیہ نوٹ کریں

        # کریٹکل گین اور دورانیہ کا حساب لگائیں
        critical_gain = 4 * relay_output / (np.pi * oscillation_amplitude)
        critical_period = oscillation_period

        # زیگر نکوچ کا استعمال کرتے ہوئے گینز کا حساب لگائیں
        tuned_gains = self.ziegler_nichols_tuning(critical_gain, critical_period)
        return tuned_gains
```

## 8.7 کنٹرولر کارکردگی کی جانچ

### کارکردگی کے اشاریہ جات

```python
class PerformanceEvaluator:
    """کنٹرولر کارکردگی کی جانچ کے لیے اشاریہ جات کا حساب لگاتا ہے"""

    def __init__(self):
        self.error_history = []
        self.settling_time_threshold = 0.02  # 2% کا میٹرک
        self.steady_state_error_threshold = 0.01  # 1% کا میٹرک

    def calculate_performance_metrics(self, reference_signal, actual_signal):
        """
        کنٹرولر کارکردگی کے اشاریہ جات کا حساب لگائیں
        """
        errors = reference_signal - actual_signal

        # RMSE (Root Mean Square Error)
        rmse = np.sqrt(np.mean(errors**2))

        # MAE (Mean Absolute Error)
        mae = np.mean(np.abs(errors))

        # IAE (Integral Absolute Error)
        iae = np.trapz(np.abs(errors))

        # ISE (Integral Square Error)
        ise = np.trapz(errors**2)

        # ITAE (Integral of Time-weighted Absolute Error)
        time_vector = np.linspace(0, len(errors)/100, len(errors))  # 100 Hz فرض کریں
        itae = np.trapz(time_vector * np.abs(errors))

        # ITSE (Integral of Time-weighted Square Error)
        itse = np.trapz(time_vector * errors**2)

        # اوور شوٹ کا حساب لگائیں
        max_response = np.max(actual_signal)
        final_value = actual_signal[-1] if len(actual_signal) > 0 else 0
        overshoot = max(0, (max_response - final_value) / abs(final_value)) if final_value != 0 else 0

        # سیٹلنگ ٹائم کا حساب لگائیں
        settling_time = self.calculate_settling_time(actual_signal, final_value)

        # رائس ٹائم کا حساب لگائیں
        rise_time = self.calculate_rise_time(actual_signal, final_value)

        metrics = {
            'rmse': rmse,
            'mae': mae,
            'iae': iae,
            'ise': ise,
            'itae': itae,
            'itse': itse,
            'overshoot': overshoot,
            'settling_time': settling_time,
            'rise_time': rise_time
        }

        return metrics

    def calculate_settling_time(self, response, final_value):
        """سیٹلنگ ٹائم کا حساب لگائیں"""
        if len(response) == 0:
            return 0

        threshold = self.settling_time_threshold * abs(final_value)
        settling_start = -1

        # آخری 10% سیمپلز کو چھوڑ دیں کیونکہ وہ سٹیڈی اسٹیٹ ہو سکتے ہیں
        for i in range(len(response) - int(0.1 * len(response))):
            if abs(response[i] - final_value) <= threshold:
                if settling_start == -1:
                    settling_start = i
            else:
                settling_start = -1  # ری سیٹ اگر خامی حد سے اوپر چلی جائے

        if settling_start != -1:
            settling_samples = len(response) - settling_start
            settling_time = settling_samples * 0.01  # 100 Hz فرض کریں
            return settling_time
        else:
            return len(response) * 0.01  # ونڈو کے اندر سیٹل نہیں ہوا

    def calculate_rise_time(self, response, final_value):
        """رائس ٹائم کا حساب لگائیں"""
        if len(response) == 0 or final_value == 0:
            return 0

        # 10% سے 90% تک پہنچنے کا وقت
        ten_percent = 0.1 * final_value
        ninety_percent = 0.9 * final_value

        ten_percent_index = -1
        ninety_percent_index = -1

        for i, val in enumerate(response):
            if ten_percent_index == -1 and val >= ten_percent:
                ten_percent_index = i
            if val >= ninety_percent:
                ninety_percent_index = i
                break

        if ten_percent_index != -1 and ninety_percent_index != -1:
            rise_samples = ninety_percent_index - ten_percent_index
            rise_time = rise_samples * 0.01  # 100 Hz فرض کریں
            return rise_time
        else:
            return len(response) * 0.01  # 90% تک نہیں پہنچا

    def evaluate_trajectory_tracking(self, desired_trajectory, actual_trajectory):
        """ٹریجکٹری ٹریکنگ کارکردگی کی جانچ کریں"""
        if len(desired_trajectory) != len(actual_trajectory):
            raise ValueError("Desired and actual trajectories must have same length")

        # 3D error کا حساب لگائیں (اگر کارٹیزین ہے)
        if len(desired_trajectory[0]) == 3:  # [x, y, z]
            position_errors = np.array([
                np.linalg.norm(des_pos - act_pos)
                for des_pos, act_pos in zip(desired_trajectory, actual_trajectory)
            ])
        else:  # جوائنٹ اسپیس
            position_errors = np.array([
                np.mean(np.abs(des_pos - act_pos))
                for des_pos, act_pos in zip(desired_trajectory, actual_trajectory)
            ])

        # کارکردگی کے اشاریہ جات کا حساب لگائیں
        rmse = np.sqrt(np.mean(position_errors**2))
        max_error = np.max(position_errors)
        mean_error = np.mean(position_errors)

        # ٹریکنگ ایکویٹی کا اسکور (0-1 کے درمیان، 1 بہترین)
        max_acceptable_error = 0.01  # 1cm یا 0.01 rad
        tracking_score = max(0, 1 - (mean_error / max_acceptable_error))

        return {
            'rmse': rmse,
            'max_error': max_error,
            'mean_error': mean_error,
            'tracking_score': tracking_score,
            'success_rate': np.mean(position_errors < max_acceptable_error)
        }
```

## 8.8 سیفٹی سسٹم اور ایمرجنسی اسٹاپس

### کنٹرول سسٹم میں سیفٹی کا نفاذ

```python
class SafetySystem:
    """کنٹرول سسٹم کے لیے سیفٹی سسٹم کا نفاذ"""

    def __init__(self):
        self.emergency_active = False
        self.safety_violations = []
        self.joint_limits = {}
        self.velocity_limits = {}
        self.acceleration_limits = {}
        self.torque_limits = {}

        # سیفٹی پیرامیٹرز
        self.violation_threshold = 0.1  # حد کا 10%
        self.emergency_threshold = 0.5  # حد کا 50%

        # ایمرجنسی کے بعد ری سیٹ کے لیے ٹائمر
        self.emergency_reset_timer = None

    def set_joint_limits(self, joint_limits_dict):
        """جوائنٹ کی حدیں سیٹ کریں"""
        self.joint_limits = joint_limits_dict

    def set_velocity_limits(self, velocity_limits_dict):
        """رفتار کی حدیں سیٹ کریں"""
        self.velocity_limits = velocity_limits_dict

    def set_torque_limits(self, torque_limits_dict):
        """ٹورک کی حدیں سیٹ کریں"""
        self.torque_limits = torque_limits_dict

    def check_safety_conditions(self, joint_states):
        """سیفٹی کی حالتیں چیک کریں"""
        violations = []

        for i, joint_name in enumerate(joint_states.name):
            if joint_name in self.joint_limits:
                # پوزیشن کی چیک
                pos = joint_states.position[i]
                min_pos, max_pos = self.joint_limits[joint_name]

                pos_violation_ratio = max(
                    abs(pos - min_pos) / (max_pos - min_pos),
                    abs(pos - max_pos) / (max_pos - min_pos)
                )

                if pos_violation_ratio > self.emergency_threshold:
                    violations.append(f'EMERGENCY: Position limit exceeded for {joint_name}')
                    self.trigger_emergency_stop()
                elif pos_violation_ratio > self.violation_threshold:
                    violations.append(f'WARNING: Position near limit for {joint_name}')

            if joint_name in self.velocity_limits:
                # رفتار کی چیک
                vel = abs(joint_states.velocity[i])
                max_vel = self.velocity_limits[joint_name]

                vel_violation_ratio = vel / max_vel

                if vel_violation_ratio > self.emergency_threshold:
                    violations.append(f'EMERGENCY: Velocity limit exceeded for {joint_name}')
                    self.trigger_emergency_stop()
                elif vel_violation_ratio > self.violation_threshold:
                    violations.append(f'WARNING: Velocity near limit for {joint_name}')

            if joint_name in self.torque_limits:
                # ٹورک کی چیک
                torque = abs(joint_states.effort[i])
                max_torque = self.torque_limits[joint_name]

                torque_violation_ratio = torque / max_torque

                if torque_violation_ratio > self.emergency_threshold:
                    violations.append(f'EMERGENCY: Torque limit exceeded for {joint_name}')
                    self.trigger_emergency_stop()
                elif torque_violation_ratio > self.violation_threshold:
                    violations.append(f'WARNING: Torque near limit for {joint_name}')

        # سیفٹی وائلیشنز کو اسٹور کریں
        self.safety_violations.extend(violations)

        return len(violations) == 0  # کوئی وائلیشن نہیں ہے تو سیف ہے

    def trigger_emergency_stop(self):
        """ایمرجنسی اسٹاپ ٹریگر کریں"""
        if not self.emergency_active:
            self.emergency_active = True
            self.get_logger().error('EMERGENCY STOP ACTIVATED - SHUTTING DOWN SAFELY')

            # صفر کمانڈز شائع کریں تاکہ تمام حرکت رک جائے
            self.publish_zero_commands()

            # ایمرجنسی کے بعد ری سیٹ کے لیے ٹائمر شروع کریں
            self.emergency_reset_timer = time.time()

    def publish_zero_commands(self):
        """صفر کمانڈز شائع کریں"""
        # یہ اصل نفاذ میں تمام کنٹرولرز کو صفر کمانڈز بھیجے گا
        pass

    def can_reset_emergency(self):
        """چیک کریں کہ ایمرجنسی ری سیٹ کی جا سکتی ہے یا نہیں"""
        if not self.emergency_active:
            return False

        # کم از کم 5 سیکنڈ انتظار کریں اور سب چیک کریں
        if time.time() - self.emergency_reset_timer < 5.0:
            return False

        # تمام سیفٹی کنڈیشنز کو دوبارہ چیک کریں
        # اگر سب ٹھیک ہے تو ری سیٹ کی اجازت دیں
        return True

    def reset_emergency(self):
        """ایمرجنسی ری سیٹ کریں"""
        if self.emergency_active and self.can_reset_emergency():
            self.emergency_active = False
            self.get_logger().info('Emergency stop reset - system ready')
            return True
        return False

class AdvancedControllerNode(Node):
    def __init__(self):
        super().__init__('advanced_controller')

        # سیفٹی سسٹم شروع کریں
        self.safety_system = SafetySystem()

        # کنٹرولر کے دیگر اجزاء شروع کریں
        self.joint_cmd_pub = self.create_publisher(JointState, 'joint_commands', 10)
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )

        # سیفٹی چیک کے لیے ٹائمر
        self.safety_timer = self.create_timer(0.01, self.safety_check)  # 100 Hz

        # اندرونی حالت
        self.current_joint_states = JointState()

    def joint_state_callback(self, msg):
        """جوائنٹ اسٹیٹ اپ ڈیٹ کریں"""
        self.current_joint_states = msg

    def safety_check(self):
        """سیفٹی کی چیک کریں"""
        if not self.safety_system.check_safety_conditions(self.current_joint_states):
            # کوئی وائلیشن ہے، ایکشن لیں
            pass

    def publish_control_commands(self, commands):
        """کنٹرول کمانڈز شائع کریں (سیفٹی چیک کے بعد)"""
        if not self.safety_system.emergency_active:
            self.joint_cmd_pub.publish(commands)
        else:
            # ایمرجنسی کے دوران کوئی کمانڈ نہیں بھیجیں
            self.publish_zero_commands()
```

## 8.9 عملی مشق

کنٹرول سسٹم کا مکمل نفاذ کریں جو:
1. PID، LQR، اور اڈاپٹیو کنٹرولرز نافذ کرتا ہے
2. ٹریجکٹریز کو منصوبہ بندی اور انجام دیتا ہے
3. سیفٹی سسٹم کو نافذ کرتا ہے
4. کارکردگی کی جانچ کرتا ہے
5. ایمرجنسی طریقہ کار کو نافذ کرتا ہے

```python
# طالب علم کی مشق - مکمل کنٹرول سسٹم کا نفاذ
class CompleteControlSystem:
    """طالب علم کا مکمل کنٹرول سسٹم کا نفاذ"""

    def __init__(self):
        """مکمل کنٹرول سسٹم شروع کریں"""
        # TODO: PID کنٹرولرز نافذ کریں
        # TODO: LQR کنٹرولر نافذ کریں
        # TODO: اڈاپٹیو کنٹرولر نافذ کریں
        # TODO: ٹریجکٹری پلاننگ الگورتھم نافذ کریں
        # TODO: سیفٹی سسٹم نافذ کریں
        # TODO: کارکردگی کی جانچ نافذ کریں
        # TODO: ایمرجنسی طریقہ کار نافذ کریں
        pass

    def implement_pid_controller(self):
        """PID کنٹرولر نافذ کریں"""
        # TODO: مکمل نفاذ مکمل کریں
        pass

    def implement_lqr_controller(self):
        """LQR کنٹرولر نافذ کریں"""
        # TODO: مکمل نفاذ مکمل کریں
        pass

    def implement_adaptive_controller(self):
        """اڈاپٹیو کنٹرولر نافذ کریں"""
        # TODO: مکمل نفاذ مکمل کریں
        pass

    def implement_trajectory_planning(self):
        """ٹریجکٹری پلاننگ نافذ کریں"""
        # TODO: مکمل نفاذ مکمل کریں
        pass

    def implement_safety_system(self):
        """سیفٹی سسٹم نافذ کریں"""
        # TODO: مکمل نفاذ مکمل کریں
        pass

    def implement_performance_evaluation(self):
        """کارکردگی کی جانچ نافذ کریں"""
        # TODO: مکمل نفاذ مکمل کریں
        pass

    def implement_emergency_procedures(self):
        """ایمرجنسی طریقہ کار نافذ کریں"""
        # TODO: مکمل نفاذ مکمل کریں
        pass

print("طالب علم کی مشق: ایک مکمل کنٹرول سسٹم نافذ کریں")
print("ضروریات:")
print("1. PID، LQR، اور اڈاپٹیو کنٹرولرز")
print("2. ٹریجکٹریز کی منصوبہ بندی اور انجام")
print("3. سیفٹی سسٹم نفاذ")
print("4. کارکردگی کی جانچ")
print("5. ایمرجنسی طریقہ کار")
print("6. ریل ٹائم نفاذ")
```

## خلاصہ

کنٹرول سسٹم اور ٹریجکٹری پلاننگ روبوٹکس کی بنیاد ہیں جو روبوٹس کو درست اور محفوظ طریقے سے حرکت کرنے کے قابل بناتے ہیں۔ PID کنٹرولرز، ٹریجکٹری پلاننگ الگورتھم، سیفٹی سسٹم، اور کارکردگی کی جانچ کو سمجھنا روبوٹک سسٹم کی ترقی کے لیے ضروری ہے۔

## اگلے اقدامات

اگلے حصے میں، ہم نیویگیشن اور موشن پلاننگ کا جائزہ لیں گے، جہاں ہم یہاں سیکھے گئے کنٹرول کے تصورات کو روبوٹ کی نیویگیشن اور مینیپولیشن کو فعال کرنے کے لیے استعمال کریں گے۔

<div class="alert alert-info" dir="rtl">
  <h5>ہارڈ ویئر کی ضرورت</h5>
  <div><strong>ضرورت:</strong> GPU</div>
  <div><strong>کم از کم:</strong> کوئی بھی</div>
  <div><strong>تجویز کردہ:</strong> کوئی بھی</div>
  <div><strong>مقصد:</strong> ہفتہ 8 کنٹرول سسٹم اور ٹریجکٹری پلاننگ کے لیے بنیادی کمپیوٹیشنل ضروریات</div>
</div>