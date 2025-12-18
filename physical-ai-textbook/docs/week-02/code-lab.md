---
title: "Week 2: Mathematical Foundations Code Lab"
week: 2
module: "Foundations of Physical AI"
difficulty: "intermediate"
prerequisites: ["python-basics", "numpy", "linear-algebra"]
learning_objectives:
  - "Implement forward and inverse kinematics algorithms"
  - "Create transformation matrices in Python"
  - "Calculate and analyze Jacobian matrices"
tags: ["kinematics", "transformations", "jacobians", "python"]
hardware_requirements:
  - gpu: "Any"
  - ram: "8GB minimum"
  - os: "Any"
duration: "120 minutes"
---

# Week 2: Mathematical Foundations Code Lab

## Learning Objectives
- Implement forward and inverse kinematics algorithms
- Create transformation matrices in Python
- Calculate and analyze Jacobian matrices
- Visualize robot kinematics in 2D and 3D

## 2.1 Setting Up the Environment

First, let's install the required packages for our mathematical computations:

```bash
pip install numpy matplotlib scipy
```

## 2.2 Implementing Transformation Matrices

Let's create a comprehensive library for handling transformations in robotics:

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Transform3D:
    """Class for handling 3D transformations in robotics"""

    @staticmethod
    def rotation_x(angle):
        """Create a rotation matrix around X-axis"""
        return np.array([
            [1, 0, 0],
            [0, np.cos(angle), -np.sin(angle)],
            [0, np.sin(angle), np.cos(angle)]
        ])

    @staticmethod
    def rotation_y(angle):
        """Create a rotation matrix around Y-axis"""
        return np.array([
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)]
        ])

    @staticmethod
    def rotation_z(angle):
        """Create a rotation matrix around Z-axis"""
        return np.array([
            [np.cos(angle), -np.sin(angle), 0],
            [np.sin(angle), np.cos(angle), 0],
            [0, 0, 1]
        ])

    @staticmethod
    def homogeneous_transform(rotation, translation):
        """Create a 4x4 homogeneous transformation matrix"""
        T = np.eye(4)
        T[:3, :3] = rotation
        T[:3, 3] = translation
        return T

    @staticmethod
    def translation(x, y, z):
        """Create a translation matrix"""
        return np.array([x, y, z])

# Example: Create a transformation matrix
R = Transform3D.rotation_z(np.pi/4)  # 45-degree rotation around Z
t = Transform3D.translation(1, 2, 3)  # Translation by (1, 2, 3)
T = Transform3D.homogeneous_transform(R, t)

print("Transformation Matrix:")
print(T)
```

## 2.3 Forward Kinematics Implementation

Let's implement forward kinematics for a 3-DOF planar manipulator:

```python
class PlanarManipulator3DOF:
    """3-DOF Planar Manipulator for forward and inverse kinematics"""

    def __init__(self, link_lengths):
        """
        Initialize the manipulator with link lengths
        link_lengths: list of [l1, l2, l3] for each link
        """
        self.l1, self.l2, self.l3 = link_lengths

    def forward_kinematics(self, joint_angles):
        """
        Calculate forward kinematics for 3-DOF planar manipulator

        Parameters:
        joint_angles: [theta1, theta2, theta3] in radians

        Returns:
        positions: list of [base_pos, joint2_pos, joint3_pos, end_effector_pos]
        """
        theta1, theta2, theta3 = joint_angles

        # Calculate positions of each joint
        # Joint 1 (base) is at origin
        joint1 = np.array([0, 0])

        # Joint 2 position
        x2 = self.l1 * np.cos(theta1)
        y2 = self.l1 * np.sin(theta1)
        joint2 = np.array([x2, y2])

        # Joint 3 position
        x3 = x2 + self.l2 * np.cos(theta1 + theta2)
        y3 = y2 + self.l2 * np.sin(theta1 + theta2)
        joint3 = np.array([x3, y3])

        # End effector position
        x_end = x3 + self.l3 * np.cos(theta1 + theta2 + theta3)
        y_end = y3 + self.l3 * np.sin(theta1 + theta2 + theta3)
        end_effector = np.array([x_end, y_end])

        return [joint1, joint2, joint3, end_effector]

    def jacobian(self, joint_angles):
        """
        Calculate the Jacobian matrix for the 3-DOF manipulator

        Returns:
        Jacobian matrix of size 2x3 (2D position, 3 joints)
        """
        theta1, theta2, theta3 = joint_angles

        # Calculate partial derivatives
        # dx/dtheta1
        dx_dt1 = -self.l1*np.sin(theta1) - self.l2*np.sin(theta1+theta2) - self.l3*np.sin(theta1+theta2+theta3)
        # dy/dtheta1
        dy_dt1 = self.l1*np.cos(theta1) + self.l2*np.cos(theta1+theta2) + self.l3*np.cos(theta1+theta2+theta3)

        # dx/dtheta2
        dx_dt2 = -self.l2*np.sin(theta1+theta2) - self.l3*np.sin(theta1+theta2+theta3)
        # dy/dtheta2
        dy_dt2 = self.l2*np.cos(theta1+theta2) + self.l3*np.cos(theta1+theta2+theta3)

        # dx/dtheta3
        dx_dt3 = -self.l3*np.sin(theta1+theta2+theta3)
        # dy/dtheta3
        dy_dt3 = self.l3*np.cos(theta1+theta2+theta3)

        J = np.array([
            [dx_dt1, dx_dt2, dx_dt3],
            [dy_dt1, dy_dt2, dy_dt3]
        ])

        return J

# Example usage
manipulator = PlanarManipulator3DOF([1.0, 0.8, 0.6])  # Link lengths
angles = [np.pi/4, np.pi/6, np.pi/3]  # Joint angles

positions = manipulator.forward_kinematics(angles)
jacobian = manipulator.jacobian(angles)

print("End effector position:", positions[-1])
print("Jacobian matrix:")
print(jacobian)
```

## 2.4 Visualization of the Manipulator

```python
def plot_planar_manipulator(positions, title="Planar Manipulator Configuration"):
    """Plot the planar manipulator configuration"""
    fig, ax = plt.subplots(figsize=(10, 8))

    # Extract x and y coordinates
    x_coords = [pos[0] for pos in positions]
    y_coords = [pos[1] for pos in positions]

    # Draw links
    ax.plot(x_coords, y_coords, 'b-', linewidth=5, label='Manipulator Links')

    # Draw joints
    ax.plot(x_coords[0], y_coords[0], 'ko', markersize=12, label='Base')
    for i in range(1, len(positions)-1):
        ax.plot(x_coords[i], y_coords[i], 'ko', markersize=10, label=f'Joint {i+1}' if i == 1 else "")
    ax.plot(x_coords[-1], y_coords[-1], 'ro', markersize=12, label='End Effector')

    # Add labels to joints
    for i, (x, y) in enumerate(zip(x_coords, y_coords)):
        ax.text(x, y, f'  J{i}', fontsize=10, verticalalignment='bottom')

    ax.set_xlim(-3, 3)
    ax.set_ylim(-1, 3)
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend()
    ax.set_title(title)

    plt.show()

# Visualize the manipulator
plot_planar_manipulator(positions, "3-DOF Planar Manipulator - Configuration 1")
```

## 2.5 Inverse Kinematics Implementation

Let's implement an inverse kinematics solver using the Jacobian pseudoinverse method:

```python
class InverseKinematics:
    """Inverse kinematics solver using Jacobian pseudoinverse"""

    def __init__(self, manipulator, max_iterations=100, tolerance=1e-4):
        self.manipulator = manipulator
        self.max_iterations = max_iterations
        self.tolerance = tolerance

    def solve(self, target_pos, initial_angles):
        """
        Solve inverse kinematics using Jacobian pseudoinverse

        Parameters:
        target_pos: [x, y] target end effector position
        initial_angles: [theta1, theta2, theta3] initial joint angles

        Returns:
        joint_angles: solved joint angles or None if no solution found
        """
        current_angles = np.array(initial_angles)

        for i in range(self.max_iterations):
            # Calculate current end effector position
            current_positions = self.manipulator.forward_kinematics(current_angles)
            current_pos = current_positions[-1]

            # Calculate error
            error = target_pos - current_pos

            # Check if we're close enough
            if np.linalg.norm(error) < self.tolerance:
                print(f"Solution found in {i+1} iterations")
                return current_angles

            # Calculate Jacobian
            J = self.manipulator.jacobian(current_angles)

            # Calculate joint angle update using pseudoinverse
            # dtheta = J^+ * dx (where J^+ is pseudoinverse of J)
            dtheta = np.linalg.pinv(J) @ error

            # Update joint angles
            current_angles = current_angles + 0.1 * dtheta  # Small step size for stability

        print(f"No solution found within {self.max_iterations} iterations")
        return None

# Example: Solve for a target position
ik_solver = InverseKinematics(manipulator)

target = np.array([1.5, 1.5])
initial_guess = [0.1, 0.1, 0.1]

solution = ik_solver.solve(target, initial_guess)

if solution is not None:
    print(f"Solution joint angles: {solution}")

    # Verify the solution
    final_positions = manipulator.forward_kinematics(solution)
    final_pos = final_positions[-1]
    print(f"Final end effector position: {final_pos}")
    print(f"Target position: {target}")
    print(f"Error: {np.linalg.norm(target - final_pos)}")

    # Plot the solution
    plot_planar_manipulator(final_positions, "3-DOF Planar Manipulator - IK Solution")
```

## 2.6 3D Manipulator Example

Let's create a simple 3D manipulator example:

```python
class SpatialManipulator:
    """Simple 3D manipulator with 3 revolute joints"""

    def __init__(self, link_lengths):
        self.link_lengths = link_lengths

    def dh_transform(self, a, alpha, d, theta):
        """Calculate Denavit-Hartenberg transformation matrix"""
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def forward_kinematics_3d(self, joint_angles):
        """Calculate forward kinematics for a simple 3D manipulator"""
        theta1, theta2, theta3 = joint_angles
        l1, l2, l3 = self.link_lengths

        # DH parameters for a simple 3-DOF arm
        # Joint 1: revolute, z-axis rotation
        T1 = self.dh_transform(0, np.pi/2, 0, theta1)

        # Joint 2: revolute, x-axis rotation, link length l1
        T2 = self.dh_transform(l1, 0, 0, theta2)

        # Joint 3: revolute, x-axis rotation, link length l2
        T3 = self.dh_transform(l2, 0, 0, theta3)

        # Total transformation
        T_total = T1 @ T2 @ T3

        # Extract end effector position
        end_effector_pos = T_total[:3, 3]

        return end_effector_pos, T_total

# Example usage
spatial_manip = SpatialManipulator([0.5, 0.4, 0.3])
angles_3d = [np.pi/4, np.pi/6, np.pi/3]

pos_3d, transform_3d = spatial_manip.forward_kinematics_3d(angles_3d)
print(f"3D End effector position: {pos_3d}")
print(f"Full transformation matrix:\n{transform_3d}")
```

## 2.7 3D Visualization

```python
def plot_3d_manipulator(link_positions):
    """Plot the 3D manipulator configuration"""
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Extract coordinates
    x_coords = [pos[0] for pos in link_positions]
    y_coords = [pos[1] for pos in link_positions]
    z_coords = [pos[2] for pos in link_positions]

    # Draw links
    ax.plot(x_coords, y_coords, z_coords, 'b-', linewidth=5, label='Manipulator Links')

    # Draw joints
    ax.scatter(x_coords[0], y_coords[0], z_coords[0], color='black', s=100, label='Base')
    for i in range(1, len(link_positions)-1):
        ax.scatter(x_coords[i], y_coords[i], z_coords[i], color='black', s=80)
    ax.scatter(x_coords[-1], y_coords[-1], z_coords[-1], color='red', s=100, label='End Effector')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Manipulator Configuration')
    ax.legend()

    plt.show()

# Example: Create 3D positions for visualization
# For simplicity, we'll create a sample set of positions
sample_positions = [
    np.array([0, 0, 0]),      # Base
    np.array([0.3, 0.2, 0.1]), # Joint 2
    np.array([0.6, 0.4, 0.2]), # Joint 3
    np.array([0.9, 0.6, 0.3])  # End effector
]

plot_3d_manipulator(sample_positions)
```

## 2.8 Jacobian Analysis and Singularities

```python
def analyze_jacobian_properties(jacobian):
    """Analyze properties of the Jacobian matrix"""
    print("Jacobian Analysis:")
    print(f"Shape: {jacobian.shape}")
    print(f"Rank: {np.linalg.matrix_rank(jacobian)}")

    # For square Jacobians, calculate determinant
    if jacobian.shape[0] == jacobian.shape[1]:
        det = np.linalg.det(jacobian)
        print(f"Determinant: {det}")
        if abs(det) < 1e-6:
            print("WARNING: Jacobian is close to singular!")

    # Calculate condition number (measure of how close to singular)
    cond_num = np.linalg.cond(jacobian)
    print(f"Condition number: {cond_num}")

    if cond_num > 1000:
        print("WARNING: Jacobian is ill-conditioned (close to singular)")

# Analyze the Jacobian from our 3-DOF manipulator
analyze_jacobian_properties(jacobian)

# Test near singularity
near_singular_angles = [0.01, np.pi, 0.01]  # This might be near a singularity
near_singular_jac = manipulator.jacobian(near_singular_angles)
print("\nNear-singularity Jacobian analysis:")
analyze_jacobian_properties(near_singular_jac)
```

## 2.9 Practical Exercise

Create a program that:
1. Implements forward kinematics for a 6-DOF robotic arm
2. Calculates the Jacobian matrix
3. Identifies potential singular configurations
4. Visualizes the manipulator in different configurations

```python
# Student exercise - Complete implementation
class SixDOFManipulator:
    """6-DOF manipulator implementation"""

    def __init__(self, dh_params):
        """
        Initialize with DH parameters
        dh_params: list of [a, alpha, d, theta] for each joint
        """
        self.dh_params = dh_params  # List of [a, alpha, d, theta] for each joint

    def dh_transform(self, a, alpha, d, theta):
        """Calculate Denavit-Hartenberg transformation matrix"""
        # TODO: Implement DH transformation matrix
        pass

    def forward_kinematics(self, joint_angles):
        """Calculate forward kinematics for 6-DOF manipulator"""
        # TODO: Implement forward kinematics
        pass

    def jacobian(self, joint_angles):
        """Calculate the geometric Jacobian for the 6-DOF manipulator"""
        # TODO: Implement Jacobian calculation
        pass

# TODO: Complete the implementation based on the concepts learned
print("Student Exercise: Complete the 6-DOF manipulator implementation")
```

## Summary

In this lab, we've implemented key mathematical concepts for robotics including transformation matrices, forward and inverse kinematics, and Jacobian calculations. These implementations form the foundation for robot control and manipulation, enabling precise positioning and motion planning.

<div class="alert alert-info">
  <h5>Hardware Requirement</h5>
  <div><strong>Requirement:</strong> GPU</div>
  <div><strong>Minimum:</strong> Any</div>
  <div><strong>Recommended:</strong> Any</div>
  <div><strong>Purpose:</strong> Basic computational requirements for Week 2 mathematical calculations</div>
</div>