---
title: "Week 6: Advanced ROS 2 Concepts Simulation"
week: 6
module: "Robotic Infrastructure"
difficulty: "advanced"
prerequisites: ["ros2-fundamentals", "python-basics", "robotics-concepts"]
learning_objectives:
  - "Simulate complex launch file scenarios"
  - "Implement distributed system communication"
  - "Use advanced debugging and visualization tools"
tags: ["ros2", "simulation", "launch", "distributed", "debugging"]
hardware_requirements:
  - gpu: "Any"
  - ram: "8GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "120 minutes"
---

# Week 6: Advanced ROS 2 Concepts Simulation

## Learning Objectives
- Simulate complex launch file scenarios
- Implement distributed system communication
- Use advanced debugging and visualization tools
- Practice system composition and monitoring

## 6.1 Complex Launch File Simulation

### Multi-Robot Launch Simulation

Let's simulate complex launch scenarios with multiple robots and system components:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import time
import threading
from collections import defaultdict

class LaunchSimulatorNode(Node):
    """Simulate complex launch file scenarios"""

    def __init__(self):
        super().__init__('launch_simulator')

        # Publishers for system status
        self.system_status_pub = self.create_publisher(String, 'system_status', 10)
        self.robot_status_pub = self.create_publisher(String, 'robot_status', 10)

        # Internal state tracking
        self.active_nodes = set()
        self.node_status = defaultdict(bool)  # {node_name: is_running}
        self.robot_count = 0
        self.system_health = True

        # Simulate system startup
        self.startup_timer = self.create_timer(2.0, self._simulate_system_startup)
        self.status_timer = self.create_timer(1.0, self._publish_system_status)

    def _simulate_system_startup(self):
        """Simulate complex system startup sequence"""
        self.get_logger().info('Starting system simulation...')

        # Simulate node startup with dependencies
        startup_sequence = [
            'parameter_server',
            'tf2_server',
            'robot_state_publisher',
            'joint_state_publisher',
            'robot_controller',
            'sensor_processor',
            'navigation_stack',
            'visualization'
        ]

        for i, node_name in enumerate(startup_sequence):
            # Simulate startup delay
            time.sleep(0.5)

            # Add node to active nodes
            self.active_nodes.add(node_name)
            self.node_status[node_name] = True

            self.get_logger().info(f'Started node: {node_name} ({i+1}/{len(startup_sequence)})')

            # Publish status update
            status_msg = String()
            status_msg.data = f'Started: {node_name}, Total active: {len(self.active_nodes)}'
            self.system_status_pub.publish(status_msg)

        self.get_logger().info('System startup simulation completed')

    def _publish_system_status(self):
        """Publish current system status"""
        status_msg = String()
        status_msg.data = f'System operational: {len(self.active_nodes)} nodes active, Health: {"GOOD" if self.system_health else "ISSUES"}'
        self.system_status_pub.publish(status_msg)

class MultiRobotLaunchSimulator:
    """Simulate multi-robot launch scenarios"""

    def __init__(self):
        rclpy.init()
        self.robots = []
        self.launch_configurations = []

    def create_robot_launch_config(self, robot_id, config_type='standard'):
        """Create launch configuration for a robot"""
        config = {
            'robot_id': robot_id,
            'namespace': f'robot_{robot_id}',
            'config_type': config_type,
            'nodes': [
                'robot_state_publisher',
                'joint_state_publisher',
                'robot_controller',
                'sensor_processor',
                'localization',
                'planning',
                'control'
            ],
            'parameters': {
                'use_sim_time': True,
                'robot_radius': 0.3,
                'max_velocity': 1.0,
                'control_frequency': 50
            }
        }
        return config

    def simulate_launch_sequence(self, num_robots=3):
        """Simulate launching multiple robots"""
        self.get_logger().info(f'Simulating launch of {num_robots} robots')

        for i in range(num_robots):
            config = self.create_robot_launch_config(i+1)
            self.launch_configurations.append(config)

            # Simulate launch process
            self._launch_robot_simulation(config)

    def _launch_robot_simulation(self, config):
        """Simulate launching a single robot"""
        robot_id = config['robot_id']
        namespace = config['namespace']

        self.get_logger().info(f'Launching robot {robot_id} in namespace {namespace}')

        # Simulate node startup with timing
        for node in config['nodes']:
            time.sleep(0.2)  # Simulate startup delay
            self.get_logger().info(f'  Started {node} for robot {robot_id}')

        # Simulate parameter loading
        time.sleep(0.1)
        self.get_logger().info(f'  Loaded parameters for robot {robot_id}')

        # Add to active robots
        self.robots.append(robot_id)
        self.get_logger().info(f'Robot {robot_id} launch completed')

    def simulate_dynamic_reconfiguration(self):
        """Simulate dynamic system reconfiguration"""
        self.get_logger().info('Simulating dynamic reconfiguration...')

        # Simulate adding a new robot during operation
        new_robot_config = self.create_robot_launch_config(len(self.robots) + 1, 'specialized')
        self.launch_configurations.append(new_robot_config)

        self._launch_robot_simulation(new_robot_config)
        self.get_logger().info('Dynamic reconfiguration completed')

    def run_simulation(self):
        """Run the complete simulation"""
        try:
            # Initial launch
            self.simulate_launch_sequence(3)

            # Wait for systems to stabilize
            time.sleep(2.0)

            # Dynamic reconfiguration
            self.simulate_dynamic_reconfiguration()

            # Run for some time
            time.sleep(5.0)

        except KeyboardInterrupt:
            self.get_logger().info('Simulation interrupted')
        finally:
            self.cleanup()

    def cleanup(self):
        """Clean up simulation"""
        self.get_logger().info('Cleaning up simulation...')
        rclpy.shutdown()

def main():
    simulator = MultiRobotLaunchSimulator()
    simulator.run_simulation()

if __name__ == '__main__':
    main()
```

## 6.2 Distributed System Communication Simulation

### Multi-Machine Communication Simulation

Let's simulate distributed communication between multiple machines:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped
import socket
import threading
import json
import time
from collections import defaultdict

class DistributedCommunicationSimulator(Node):
    """Simulate distributed communication between multiple machines"""

    def __init__(self):
        super().__init__('distributed_comm_sim')

        # Publishers
        self.status_pub = self.create_publisher(String, 'comm_status', 10)
        self.data_pub = self.create_publisher(String, 'distributed_data', 10)

        # Internal state
        self.local_ip = self._get_local_ip()
        self.machine_id = self._generate_machine_id()
        self.connected_machines = {}  # {machine_id: {'ip': ip, 'last_seen': time, 'status': status}}
        self.message_queue = []
        self.message_counter = 0

        # Network simulation parameters
        self.network_latency = 0.05  # 50ms average latency
        self.packet_loss_rate = 0.01  # 1% packet loss
        self.jitter = 0.01  # 10ms jitter

        # Timers
        self.heartbeat_timer = self.create_timer(1.0, self._send_heartbeat)
        self.status_timer = self.create_timer(2.0, self._publish_status)

        # Start network simulation threads
        self.network_thread = threading.Thread(target=self._network_simulation_worker)
        self.network_thread.daemon = True
        self.network_thread.start()

        self.get_logger().info(f'Distributed communication simulator started on {self.local_ip} as {self.machine_id}')

    def _get_local_ip(self):
        """Get local machine IP"""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                return s.getsockname()[0]
        except Exception:
            return "127.0.0.1"

    def _generate_machine_id(self):
        """Generate unique machine identifier"""
        import uuid
        return f"machine_{uuid.uuid4().hex[:8]}_{int(time.time()) % 1000}"

    def _send_heartbeat(self):
        """Send heartbeat to maintain network presence"""
        heartbeat_msg = {
            'type': 'heartbeat',
            'machine_id': self.machine_id,
            'ip_address': self.local_ip,
            'timestamp': time.time(),
            'status': 'active',
            'connected_count': len(self.connected_machines)
        }

        self._send_distributed_message(heartbeat_msg)

    def _send_distributed_message(self, message_data):
        """Send message to distributed system with simulated network effects"""
        # Simulate network effects
        if self._should_drop_packet():
            self.get_logger().warn(f'Packet dropped (simulated): {message_data.get("type", "unknown")}')
            return

        # Simulate latency
        latency = self.network_latency + (self.jitter * (2 * (time.time() % 1) - 1))  # Add jitter
        time.sleep(latency)

        # Create ROS message
        ros_msg = String()
        ros_msg.data = json.dumps(message_data)
        self.data_pub.publish(ros_msg)

        # Update message counter
        self.message_counter += 1

    def _should_drop_packet(self):
        """Determine if packet should be dropped based on loss rate"""
        return (time.time() * 1000) % 100 < (self.packet_loss_rate * 100)

    def _publish_status(self):
        """Publish communication status"""
        status_msg = String()
        status_msg.data = f'Machine {self.machine_id}: {len(self.connected_machines)} connected, {self.message_counter} messages sent'
        self.status_pub.publish(status_msg)

    def _network_simulation_worker(self):
        """Background worker for network simulation"""
        while rclpy.ok():
            # Simulate receiving messages from other machines
            self._simulate_message_reception()

            # Clean up stale connections
            current_time = time.time()
            stale_machines = []
            for machine_id, info in self.connected_machines.items():
                if current_time - info['last_seen'] > 5.0:  # 5 second timeout
                    stale_machines.append(machine_id)

            for machine_id in stale_machines:
                del self.connected_machines[machine_id]
                self.get_logger().info(f'Removed stale machine: {machine_id}')

            time.sleep(0.1)

    def _simulate_message_reception(self):
        """Simulate receiving messages from other machines"""
        # This would normally receive from actual network
        # For simulation, we'll generate periodic status updates
        if len(self.connected_machines) < 3:  # Simulate discovery of other machines
            # Simulate discovery of a new machine
            new_machine_id = f"simulated_machine_{len(self.connected_machines) + 1}"
            self.connected_machines[new_machine_id] = {
                'ip': f"192.168.1.{10 + len(self.connected_machines)}",
                'last_seen': time.time(),
                'status': 'active'
            }
            self.get_logger().info(f'Discovered simulated machine: {new_machine_id}')

    def send_command_to_all(self, command, data=None):
        """Send command to all connected machines"""
        command_msg = {
            'type': 'command',
            'machine_id': self.machine_id,
            'command': command,
            'data': data,
            'timestamp': time.time()
        }

        self._send_distributed_message(command_msg)

    def send_data_sync(self, data_type, data):
        """Send data for synchronization"""
        sync_msg = {
            'type': 'sync',
            'machine_id': self.machine_id,
            'data_type': data_type,
            'data': data,
            'timestamp': time.time()
        }

        self._send_distributed_message(sync_msg)

class MultiMachineSystemSimulator:
    """Simulate a multi-machine distributed system"""

    def __init__(self, num_machines=3):
        self.num_machines = num_machines
        self.machines = []
        self.simulation_active = False

    def create_machine_simulator(self, machine_num):
        """Create a distributed communication simulator for a machine"""
        # This would normally create a separate ROS context for each machine
        # For simulation, we'll create separate nodes
        rclpy.init()

        node = DistributedCommunicationSimulator()
        node.get_logger().info(f'Created machine simulator {machine_num}')

        return node

    def simulate_system_startup(self):
        """Simulate startup of multi-machine system"""
        self.get_logger().info(f'Starting simulation of {self.num_machines} machines')

        # Create machine simulators
        for i in range(self.num_machines):
            machine = self.create_machine_simulator(i+1)
            self.machines.append(machine)

        self.simulation_active = True

    def run_distributed_simulation(self):
        """Run the distributed simulation"""
        self.simulate_system_startup()

        # Simulate coordinated activities
        for i in range(10):  # Run for 10 cycles
            if not self.simulation_active:
                break

            # Simulate synchronized activity
            self._simulate_coordinated_activity()

            time.sleep(1.0)

    def _simulate_coordinated_activity(self):
        """Simulate coordinated activity between machines"""
        self.get_logger().info('Simulating coordinated activity...')

        # Simulate sending commands to all machines
        for machine in self.machines:
            machine.send_command_to_all('move_to_position', {'x': 1.0, 'y': 2.0})
            time.sleep(0.1)  # Stagger commands

        # Simulate data synchronization
        for machine in self.machines:
            machine.send_data_sync('sensor_data', {'timestamp': time.time(), 'values': [1, 2, 3]})

    def cleanup(self):
        """Clean up the simulation"""
        self.simulation_active = False
        for machine in self.machines:
            machine.destroy_node()
        rclpy.shutdown()

def main():
    simulator = MultiMachineSystemSimulator(num_machines=3)
    try:
        simulator.run_distributed_simulation()
    except KeyboardInterrupt:
        pass
    finally:
        simulator.cleanup()

if __name__ == '__main__':
    main()
```

## 6.3 Advanced Debugging Simulation

### Comprehensive Diagnostic System Simulation

Let's create a comprehensive debugging and diagnostic simulation:

```python
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Float64, String, Bool
from rclpy.qos import QoSProfile
import psutil
import time
from collections import deque
import threading
import random
from datetime import datetime

class AdvancedDiagnosticSimulator(Node):
    """Simulate advanced diagnostic and debugging capabilities"""

    def __init__(self):
        super().__init__('advanced_diagnostic_sim')

        # Publishers
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.system_metrics_pub = self.create_publisher(String, 'system_metrics', 10)
        self.error_pub = self.create_publisher(String, 'error_log', 10)
        self.performance_pub = self.create_publisher(Float64, 'performance_score', 10)

        # Internal state
        self.system_metrics = {
            'cpu_usage': 0.0,
            'memory_usage': 0.0,
            'disk_usage': 0.0,
            'network_usage': 0.0,
            'temperature': 0.0
        }

        self.error_log = deque(maxlen=100)
        self.warning_log = deque(maxlen=100)
        self.performance_history = deque(maxlen=50)

        self.simulation_mode = 'normal'  # 'normal', 'stress', 'error'
        self.anomaly_counter = 0

        # Timers
        self.diag_timer = self.create_timer(1.0, self._publish_diagnostics)
        self.metrics_timer = self.create_timer(0.5, self._update_metrics)
        self.simulation_timer = self.create_timer(10.0, self._update_simulation_mode)

        # Start monitoring threads
        self.monitoring_thread = threading.Thread(target=self._continuous_monitoring)
        self.monitoring_thread.daemon = True
        self.monitoring_thread.start()

        self.get_logger().info('Advanced diagnostic simulator started')

    def _update_metrics(self):
        """Update system metrics with simulated data"""
        # Simulate realistic system metrics
        base_cpu = random.uniform(10, 30)
        base_memory = random.uniform(20, 40)
        base_disk = random.uniform(15, 25)

        # Add variation based on simulation mode
        if self.simulation_mode == 'stress':
            base_cpu += random.uniform(40, 70)
            base_memory += random.uniform(30, 50)
        elif self.simulation_mode == 'error':
            base_cpu += random.uniform(70, 90)
            base_memory += random.uniform(60, 80)

        self.system_metrics['cpu_usage'] = min(base_cpu, 100)
        self.system_metrics['memory_usage'] = min(base_memory, 100)
        self.system_metrics['disk_usage'] = min(base_disk, 100)
        self.system_metrics['network_usage'] = random.uniform(0, 20)
        self.system_metrics['temperature'] = 30 + (self.system_metrics['cpu_usage'] * 0.3)

    def _update_simulation_mode(self):
        """Randomly change simulation mode to test diagnostic system"""
        modes = ['normal', 'stress', 'error']
        # Bias towards normal mode but occasionally introduce stress/error
        weights = [0.7, 0.2, 0.1]
        self.simulation_mode = random.choices(modes, weights=weights)[0]

        self.get_logger().info(f'Simulation mode changed to: {self.simulation_mode}')

    def _continuous_monitoring(self):
        """Background monitoring thread"""
        while rclpy.ok():
            try:
                # Simulate periodic events
                if random.random() < 0.1:  # 10% chance per second
                    self._generate_random_event()

                # Check for anomalies
                self._check_for_anomalies()

                time.sleep(1.0)
            except Exception as e:
                self.get_logger().error(f'Monitoring thread error: {e}')
                time.sleep(1.0)

    def _generate_random_event(self):
        """Generate random system events for testing"""
        event_types = ['normal', 'warning', 'error']
        event_weights = [0.8, 0.15, 0.05]
        event_type = random.choices(event_types, weights=event_weights)[0]

        if event_type == 'warning':
            warning_msg = f'Warning: High CPU usage detected at {datetime.now()}'
            self._log_warning(warning_msg)
        elif event_type == 'error':
            error_msg = f'Error: Sensor timeout at {datetime.now()}'
            self._log_error(error_msg)

    def _check_for_anomalies(self):
        """Check for system anomalies"""
        # Check for CPU spikes
        if self.system_metrics['cpu_usage'] > 85:
            self.anomaly_counter += 1
            if self.anomaly_counter > 3:  # Persistent high CPU
                self._log_warning(f'Anomaly: Persistent high CPU usage ({self.system_metrics["cpu_usage"]:.1f}%)')
                self.anomaly_counter = 0
        else:
            self.anomaly_counter = max(0, self.anomaly_counter - 1)

        # Check for memory issues
        if self.system_metrics['memory_usage'] > 80:
            self._log_warning(f'Anomaly: High memory usage ({self.system_metrics["memory_usage"]:.1f}%)')

    def _log_error(self, message):
        """Log an error message"""
        error_entry = {
            'timestamp': time.time(),
            'message': message,
            'severity': 'error'
        }
        self.error_log.append(error_entry)

        log_msg = String()
        log_msg.data = f'ERROR: {message}'
        self.error_pub.publish(log_msg)

        self.get_logger().error(message)

    def _log_warning(self, message):
        """Log a warning message"""
        warning_entry = {
            'timestamp': time.time(),
            'message': message,
            'severity': 'warning'
        }
        self.warning_log.append(warning_entry)

        log_msg = String()
        log_msg.data = f'WARNING: {message}'
        self.error_pub.publish(log_msg)

        self.get_logger().warn(message)

    def _publish_diagnostics(self):
        """Publish comprehensive diagnostic information"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # System health diagnostic
        sys_diag = DiagnosticStatus()
        sys_diag.name = 'System Health'
        sys_diag.hardware_id = self.get_name()

        # Calculate health score
        health_score = (100 - self.system_metrics['cpu_usage'] +
                       100 - self.system_metrics['memory_usage'] +
                       100 - self.system_metrics['disk_usage']) / 3

        if health_score > 70:
            sys_diag.level = DiagnosticStatus.OK
            sys_diag.message = f'System healthy (score: {health_score:.1f})'
        elif health_score > 40:
            sys_diag.level = DiagnosticStatus.WARN
            sys_diag.message = f'System under load (score: {health_score:.1f})'
        else:
            sys_diag.level = DiagnosticStatus.ERROR
            sys_diag.message = f'System overloaded (score: {health_score:.1f})'

        # Add detailed metrics
        sys_diag.values = [
            KeyValue(key='cpu_percent', value=f'{self.system_metrics["cpu_usage"]:.1f}%'),
            KeyValue(key='memory_percent', value=f'{self.system_metrics["memory_usage"]:.1f}%'),
            KeyValue(key='disk_percent', value=f'{self.system_metrics["disk_usage"]:.1f}%'),
            KeyValue(key='temperature_c', value=f'{self.system_metrics["temperature"]:.1f}'),
            KeyValue(key='simulation_mode', value=self.simulation_mode),
            KeyValue(key='error_count', value=str(len(self.error_log))),
            KeyValue(key='warning_count', value=str(len(self.warning_log)))
        ]

        diag_array.status.append(sys_diag)

        # Component diagnostics
        components = ['cpu', 'memory', 'disk', 'network']
        for component in components:
            comp_diag = DiagnosticStatus()
            comp_diag.name = f'{component.capitalize()} Usage'
            comp_diag.hardware_id = f'{self.get_name()}_{component}'

            usage = self.system_metrics[f'{component}_usage']

            if usage < 70:
                comp_diag.level = DiagnosticStatus.OK
                comp_diag.message = f'{component.capitalize()} usage normal'
            elif usage < 90:
                comp_diag.level = DiagnosticStatus.WARN
                comp_diag.message = f'{component.capitalize()} usage high'
            else:
                comp_diag.level = DiagnosticStatus.ERROR
                comp_diag.message = f'{component.capitalize()} usage critical'

            comp_diag.values = [
                KeyValue(key='usage_percent', value=f'{usage:.1f}%'),
                KeyValue(key='status', value='normal' if usage < 70 else 'elevated')
            ]

            diag_array.status.append(comp_diag)

        # Publish diagnostics
        self.diag_pub.publish(diag_array)

        # Publish performance score
        perf_score = Float64()
        perf_score.data = health_score / 100.0  # Normalize to 0-1 range
        self.performance_pub.publish(perf_score)

        # Publish system metrics summary
        metrics_msg = String()
        metrics_msg.data = f'CPU: {self.system_metrics["cpu_usage"]:.1f}%, Memory: {self.system_metrics["memory_usage"]:.1f}%, Disk: {self.system_metrics["disk_usage"]:.1f}%, Errors: {len(self.error_log)}, Warnings: {len(self.warning_log)}'
        self.system_metrics_pub.publish(metrics_msg)

def main(args=None):
    rclpy.init(args=args)
    diag_sim = AdvancedDiagnosticSimulator()

    try:
        rclpy.spin(diag_sim)
    except KeyboardInterrupt:
        pass
    finally:
        diag_sim.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 6.4 Advanced Visualization Simulation

### Multi-Component Visualization System

Let's create a comprehensive visualization simulation:

```python
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped, Vector3
from std_msgs.msg import ColorRGBA, String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
import math
import numpy as np
from collections import deque
import random

class AdvancedVisualizationSimulator(Node):
    """Simulate advanced visualization capabilities"""

    def __init__(self):
        super().__init__('advanced_vis_sim')

        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, 'advanced_markers', 10)
        self.path_pub = self.create_publisher(Path, 'simulated_path', 10)
        self.status_pub = self.create_publisher(String, 'vis_status', 10)

        # Internal state
        self.robots = {}  # {robot_id: position}
        self.paths = {}  # {robot_id: deque of positions}
        self.obstacles = []
        self.scan_points = []
        self.visualization_mode = 'normal'  # 'normal', 'debug', 'analysis'
        self.time_counter = 0.0

        # Initialize robots
        self._initialize_robots()
        self._initialize_environment()

        # Timers
        self.vis_timer = self.create_timer(0.05, self._publish_visualization)  # 20 Hz
        self.update_timer = self.create_timer(0.1, self._update_simulation)

    def _initialize_robots(self):
        """Initialize simulated robots"""
        for i in range(3):  # 3 robots
            robot_id = f'robot_{i+1}'
            # Random starting positions
            x = random.uniform(-5, 5)
            y = random.uniform(-5, 5)
            z = 0.0

            self.robots[robot_id] = (x, y, z)
            self.paths[robot_id] = deque(maxlen=200)  # Keep last 200 positions

    def _initialize_environment(self):
        """Initialize simulated environment"""
        # Create random obstacles
        for _ in range(10):
            x = random.uniform(-8, 8)
            y = random.uniform(-8, 8)
            self.obstacles.append((x, y, 0.5))  # (x, y, radius)

    def _update_simulation(self):
        """Update simulation state"""
        self.time_counter += 0.1

        # Move robots in patterns
        for robot_id, (x, y, z) in self.robots.items():
            # Circular motion for robot 1
            if robot_id == 'robot_1':
                new_x = 3 * math.cos(self.time_counter * 0.3)
                new_y = 3 * math.sin(self.time_counter * 0.3)
            # Square pattern for robot 2
            elif robot_id == 'robot_2':
                side = 4
                period = 8  # seconds per side
                t = (self.time_counter % period) / period
                if t < 0.25:
                    new_x = -side/2 + (t * 4) * side
                    new_y = -side/2
                elif t < 0.5:
                    new_x = side/2
                    new_y = -side/2 + ((t - 0.25) * 4) * side
                elif t < 0.75:
                    new_x = side/2 - ((t - 0.5) * 4) * side
                    new_y = side/2
                else:
                    new_x = -side/2
                    new_y = side/2 - ((t - 0.75) * 4) * side
            # Random walk for robot 3
            else:
                new_x = x + random.uniform(-0.2, 0.2)
                new_y = y + random.uniform(-0.2, 0.2)
                # Keep within bounds
                new_x = max(-10, min(10, new_x))
                new_y = max(-10, min(10, new_y))

            # Update robot position
            self.robots[robot_id] = (new_x, new_y, z)

            # Add to path
            self.paths[robot_id].append((new_x, new_y, z))

    def _publish_visualization(self):
        """Publish all visualization elements"""
        marker_array = MarkerArray()

        # Create markers for each visualization component
        marker_id = 0

        # Robot markers
        for robot_id, (x, y, z) in self.robots.items():
            robot_marker = self._create_robot_marker(robot_id, (x, y, z), marker_id)
            marker_array.markers.append(robot_marker)
            marker_id += 1

        # Robot paths
        for robot_id, path in self.paths.items():
            if len(path) > 1:
                path_marker = self._create_path_marker(robot_id, path, marker_id)
                marker_array.markers.append(path_marker)
                marker_id += 1

        # Obstacles
        for i, (x, y, radius) in enumerate(self.obstacles):
            obstacle_marker = self._create_obstacle_marker((x, y, 0), radius, marker_id + i)
            marker_array.markers.append(obstacle_marker)

        # Environment bounds
        bounds_marker = self._create_bounds_marker(marker_id + len(self.obstacles))
        marker_array.markers.append(bounds_marker)

        # Safety zones around robots
        safety_zones = self._create_safety_zones(marker_id + len(self.obstacles) + 1)
        marker_array.markers.extend(safety_zones)

        # Publish markers
        self.marker_pub.publish(marker_array)

        # Publish path for each robot
        self._publish_robot_paths()

        # Publish status
        status_msg = String()
        status_msg.data = f'Visualizing {len(self.robots)} robots, {len(self.obstacles)} obstacles'
        self.status_pub.publish(status_msg)

    def _create_robot_marker(self, robot_id, position, marker_id):
        """Create robot visualization marker"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robots"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2] + 0.25  # Half cylinder height

        # Orientation
        marker.pose.orientation.w = 1.0

        # Scale
        marker.scale = Vector3(x=0.3, y=0.3, z=0.5)

        # Color based on robot ID
        colors = {
            'robot_1': (1.0, 0.0, 0.0),  # Red
            'robot_2': (0.0, 1.0, 0.0),  # Green
            'robot_3': (0.0, 0.0, 1.0)   # Blue
        }
        color = colors.get(robot_id, (1.0, 1.0, 1.0))  # White default
        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.8)

        return marker

    def _create_path_marker(self, robot_id, path, marker_id):
        """Create path visualization marker"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"paths_{robot_id}"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Scale
        marker.scale = Vector3(x=0.03, y=0.03, z=0.03)

        # Color based on robot
        base_colors = {
            'robot_1': (1.0, 0.0, 0.0),
            'robot_2': (0.0, 1.0, 0.0),
            'robot_3': (0.0, 0.0, 1.0)
        }
        color = base_colors.get(robot_id, (1.0, 1.0, 1.0))
        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.6)

        # Points
        for x, y, z in path:
            point = Point()
            point.x = x
            point.y = y
            point.z = z + 0.05  # Slightly above ground
            marker.points.append(point)

        return marker

    def _create_obstacle_marker(self, position, radius, marker_id):
        """Create obstacle visualization marker"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "obstacles"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2] + radius

        # Orientation
        marker.pose.orientation.w = 1.0

        # Scale
        marker.scale = Vector3(x=radius * 2, y=radius * 2, z=radius * 2)

        # Color (orange for obstacles)
        marker.color = ColorRGBA(r=1.0, g=0.65, b=0.0, a=0.7)

        return marker

    def _create_bounds_marker(self, marker_id):
        """Create environment bounds marker"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "bounds"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Scale
        marker.scale = Vector3(x=0.1, y=0.1, z=0.1)

        # Color
        marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.8)

        # Square bounds (-10 to 10 in x and y)
        bounds = [
            (-10, -10, 0.1), (10, -10, 0.1),
            (10, 10, 0.1), (-10, 10, 0.1), (-10, -10, 0.1)
        ]

        for x, y, z in bounds:
            point = Point()
            point.x = x
            point.y = y
            point.z = z
            marker.points.append(point)

        return marker

    def _create_safety_zones(self, start_id):
        """Create safety zones around robots"""
        markers = []

        for i, (robot_id, (x, y, z)) in enumerate(self.robots.items()):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "safety_zones"
            marker.id = start_id + i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Position
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z + 0.1

            # Orientation
            marker.pose.orientation.w = 1.0

            # Scale (1.5m radius safety zone)
            marker.scale = Vector3(x=3.0, y=3.0, z=0.2)

            # Color (semi-transparent yellow)
            marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.2)

            markers.append(marker)

        return markers

    def _publish_robot_paths(self):
        """Publish individual robot paths"""
        for robot_id, path in self.paths.items():
            if len(path) < 2:
                continue

            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = self.get_clock().now().to_msg()

            for x, y, z in path:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = z
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)

            # Publish to robot-specific topic
            pub_name = f'path_{robot_id}'
            if not hasattr(self, pub_name):
                setattr(self, pub_name, self.create_publisher(Path, f'path_{robot_id}', 10))

            getattr(self, pub_name).publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    vis_sim = AdvancedVisualizationSimulator()

    try:
        rclpy.spin(vis_sim)
    except KeyboardInterrupt:
        pass
    finally:
        vis_sim.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 6.5 System Composition and Monitoring Simulation

### Complete System Integration Simulation

Let's create a simulation that integrates all advanced concepts:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float64
from diagnostic_msgs.msg import DiagnosticArray
from visualization_msgs.msg import MarkerArray
import time
import threading
from collections import defaultdict

class SystemIntegrationSimulator(Node):
    """Simulate complete system integration with all advanced concepts"""

    def __init__(self):
        super().__init__('system_integration_sim')

        # Publishers for system monitoring
        self.system_status_pub = self.create_publisher(String, 'system_status', 10)
        self.health_score_pub = self.create_publisher(Float64, 'system_health_score', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # Internal system state
        self.components = {
            'launch_system': {'status': 'active', 'health': 1.0},
            'distributed_comm': {'status': 'active', 'health': 1.0},
            'debugging_tools': {'status': 'active', 'health': 1.0},
            'visualization': {'status': 'active', 'health': 1.0},
            'safety_system': {'status': 'active', 'health': 1.0}
        }

        self.system_metrics = {
            'uptime': 0.0,
            'message_rate': 0.0,
            'cpu_usage': 0.0,
            'memory_usage': 0.0
        }

        self.start_time = time.time()

        # Timers for system simulation
        self.status_timer = self.create_timer(1.0, self._publish_system_status)
        self.health_timer = self.create_timer(0.5, self._update_health_scores)
        self.metrics_timer = self.create_timer(2.0, self._update_system_metrics)

        # Start background processes
        self.background_thread = threading.Thread(target=self._background_processes)
        self.background_thread.daemon = True
        self.background_thread.start()

        self.get_logger().info('System integration simulator started')

    def _update_health_scores(self):
        """Update health scores for all system components"""
        for component, data in self.components.items():
            # Simulate health fluctuations
            base_health = 1.0
            if component == 'safety_system':
                # Safety system should remain very healthy
                fluctuation = random.uniform(-0.05, 0.05)
            else:
                # Other components can have more variation
                fluctuation = random.uniform(-0.2, 0.1)

            new_health = max(0.0, min(1.0, base_health + fluctuation))
            data['health'] = new_health

            # Update status based on health
            if new_health > 0.8:
                data['status'] = 'healthy'
            elif new_health > 0.5:
                data['status'] = 'degraded'
            else:
                data['status'] = 'critical'

    def _update_system_metrics(self):
        """Update system-level metrics"""
        current_time = time.time()
        self.system_metrics['uptime'] = current_time - self.start_time

        # Simulate realistic metrics
        self.system_metrics['message_rate'] = random.uniform(50, 200)  # Hz
        self.system_metrics['cpu_usage'] = random.uniform(10, 40)     # %
        self.system_metrics['memory_usage'] = random.uniform(20, 60)  # %

    def _publish_system_status(self):
        """Publish comprehensive system status"""
        # Calculate overall system health
        total_health = sum(data['health'] for data in self.components.values())
        avg_health = total_health / len(self.components) if self.components else 0.0

        # Publish health score
        health_msg = Float64()
        health_msg.data = avg_health
        self.health_score_pub.publish(health_msg)

        # Publish detailed status
        status_msg = String()
        status_parts = [f'System Health: {avg_health:.2f}']

        for name, data in self.components.items():
            status_parts.append(f'{name}: {data["status"]}({data["health"]:.2f})')

        status_parts.append(f'Uptime: {self.system_metrics["uptime"]:.0f}s')
        status_parts.append(f'Msg Rate: {self.system_metrics["message_rate"]:.0f}Hz')

        status_msg.data = ', '.join(status_parts)
        self.system_status_pub.publish(status_msg)

        # Publish diagnostic array
        self._publish_diagnostics(avg_health)

    def _publish_diagnostics(self, overall_health):
        """Publish diagnostic information"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # Overall system diagnostic
        overall_diag = DiagnosticStatus()
        overall_diag.name = 'Overall System Health'
        overall_diag.hardware_id = 'system_integration_sim'
        overall_diag.level = 0 if overall_health > 0.7 else (1 if overall_health > 0.4 else 2)
        overall_diag.message = f'Overall health: {overall_health:.2f}'

        overall_diag.values = [
            {'key': 'uptime_seconds', 'value': f'{self.system_metrics["uptime"]:.1f}'},
            {'key': 'message_rate_hz', 'value': f'{self.system_metrics["message_rate"]:.1f}'},
            {'key': 'cpu_percent', 'value': f'{self.system_metrics["cpu_usage"]:.1f}'},
            {'key': 'memory_percent', 'value': f'{self.system_metrics["memory_usage"]:.1f}'}
        ]

        diag_array.status.append(overall_diag)

        # Individual component diagnostics
        for name, data in self.components.items():
            comp_diag = DiagnosticStatus()
            comp_diag.name = f'Component: {name}'
            comp_diag.hardware_id = f'system_integration_sim_{name}'
            comp_diag.level = 0 if data['health'] > 0.7 else (1 if data['health'] > 0.4 else 2)
            comp_diag.message = f'{data["status"]} - Health: {data["health"]:.2f}'

            comp_diag.values = [
                {'key': 'status', 'value': data['status']},
                {'key': 'health_score', 'value': f'{data["health"]:.2f}'}
            ]

            diag_array.status.append(comp_diag)

        self.diag_pub.publish(diag_array)

    def _background_processes(self):
        """Simulate background system processes"""
        while rclpy.ok():
            try:
                # Simulate periodic system events
                self._simulate_system_event()

                # Simulate component interactions
                self._simulate_component_interaction()

                time.sleep(0.5)
            except Exception as e:
                self.get_logger().error(f'Background process error: {e}')
                time.sleep(1.0)

    def _simulate_system_event(self):
        """Simulate system events"""
        # Randomly trigger system events
        if random.random() < 0.05:  # 5% chance per 0.5 seconds
            event_type = random.choice(['component_restart', 'parameter_update', 'diagnostic_check'])

            if event_type == 'component_restart':
                component = random.choice(list(self.components.keys()))
                self.get_logger().info(f'Simulated restart of {component}')
                # Simulate temporary degradation
                self.components[component]['health'] = max(0.3, self.components[component]['health'] - 0.2)

            elif event_type == 'parameter_update':
                self.get_logger().info('Simulated parameter update')

            elif event_type == 'diagnostic_check':
                self.get_logger().info('Simulated diagnostic check')

    def _simulate_component_interaction(self):
        """Simulate interactions between system components"""
        # Simulate launch system affecting other components
        if self.components['launch_system']['health'] < 0.5:
            # Degraded launch system affects others
            for name in ['distributed_comm', 'debugging_tools', 'visualization']:
                if name in self.components:
                    self.components[name]['health'] = max(0.2, self.components[name]['health'] - 0.1)

        # Safety system can override other statuses
        if self.components['safety_system']['health'] < 0.3:
            # Critical safety issues affect everything
            for name, data in self.components.items():
                if name != 'safety_system':
                    data['health'] = max(0.1, data['health'] - 0.15)

def main(args=None):
    rclpy.init(args=args)
    sys_sim = SystemIntegrationSimulator()

    try:
        rclpy.spin(sys_sim)
    except KeyboardInterrupt:
        pass
    finally:
        sys_sim.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 6.6 Practical Exercise

Create a complete simulation environment that:
1. Implements complex launch file scenarios
2. Simulates distributed system communication
3. Includes advanced debugging and diagnostics
4. Provides comprehensive visualization
5. Integrates all system components

```python
# Student exercise - Complete implementation
class CompleteSimulationEnvironment:
    """Student implementation of a complete simulation environment"""

    def __init__(self):
        """Initialize the complete simulation environment"""
        # TODO: Implement complex launch scenarios
        # TODO: Simulate distributed communication
        # TODO: Add advanced debugging tools
        # TODO: Implement comprehensive visualization
        # TODO: Integrate system components
        # TODO: Add monitoring and analysis tools
        pass

    def implement_launch_scenarios(self):
        """Implement complex launch file scenarios"""
        # TODO: Complete implementation
        pass

    def simulate_distributed_system(self):
        """Simulate distributed system communication"""
        # TODO: Complete implementation
        pass

    def add_debugging_tools(self):
        """Add advanced debugging and diagnostic tools"""
        # TODO: Complete implementation
        pass

    def implement_visualization(self):
        """Implement comprehensive visualization"""
        # TODO: Complete implementation
        pass

    def integrate_components(self):
        """Integrate all system components"""
        # TODO: Complete implementation
        pass

    def add_monitoring(self):
        """Add system monitoring and analysis"""
        # TODO: Complete implementation
        pass

print("Student Exercise: Implement a complete simulation environment")
print("Requirements:")
print("1. Complex launch file scenarios")
print("2. Distributed system communication simulation")
print("3. Advanced debugging and diagnostic tools")
print("4. Comprehensive visualization system")
print("5. Integrated system components")
print("6. System monitoring and analysis tools")
```

## 6.7 Best Practices for Advanced ROS 2 Simulation

### Performance Considerations
- Use appropriate QoS settings for different data types
- Implement efficient data structures for large datasets
- Consider message compression for high-bandwidth data
- Use multi-threading for I/O intensive operations

### Resource Management
- Monitor and limit memory usage in long-running simulations
- Implement proper cleanup for temporary resources
- Use connection pooling for network operations
- Implement backpressure mechanisms for high-throughput systems

### Error Handling and Recovery
- Implement retry mechanisms with exponential backoff
- Use circuit breakers for unreliable components
- Implement graceful degradation when components fail
- Maintain system state consistency during failures

## Summary

Advanced ROS 2 simulation involves complex system integration, distributed communication, comprehensive debugging tools, and sophisticated visualization capabilities. Understanding these concepts enables the development of robust and maintainable robotic systems with proper monitoring and analysis capabilities.

<div class="alert alert-info">
  <h5>Hardware Requirement</h5>
  <div><strong>Requirement:</strong> GPU</div>
  <div><strong>Minimum:</strong> Any</div>
  <div><strong>Recommended:</strong> Any</div>
  <div><strong>Purpose:</strong> Basic computational requirements for Week 6 advanced ROS 2 simulation</div>
</div>