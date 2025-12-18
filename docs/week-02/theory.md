---
title: "Week 2: Mathematical Foundations for Robotics"
week: 2
module: "Foundations of Physical AI"
difficulty: "intermediate"
prerequisites: ["linear-algebra", "calculus", "python-basics"]
learning_objectives:
  - "Apply linear algebra concepts to robotics"
  - "Understand kinematic transformations"
  - "Work with Jacobians and transformation matrices"
tags: ["linear-algebra", "kinematics", "jacobians", "transformations"]
hardware_requirements:
  - gpu: "Any"
  - ram: "8GB minimum"
  - os: "Any"
duration: "90 minutes"
---
# Week 2: Mathematical Foundations for Robotics

## Learning Objectives
- Apply linear algebra concepts to robotics
- Understand kinematic transformations
- Work with Jacobians and transformation matrices
- Implement forward and inverse kinematics equations

## 2.1 Linear Algebra for Robotics

Linear algebra forms the mathematical foundation for robotics, enabling us to represent and manipulate spatial relationships, transformations, and system dynamics.

### Vectors in Robotics

Vectors are used extensively in robotics to represent:
- Position and orientation in 3D space
- Velocities and accelerations
- Forces and torques
- Joint angles and configurations

### Matrices in Robotics

Matrices enable us to represent:
- Linear transformations (rotation, scaling, shearing)
- System dynamics and control matrices
- Covariance matrices for uncertainty
- Jacobian matrices for velocity relationships

## 2.2 Forward Kinematics

Forward kinematics solves the problem of determining the end-effector position and orientation given the joint angles.

### Transformation Matrices

The relationship between different coordinate frames is represented using transformation matrices. A 4x4 transformation matrix $T$ combines rotation $R$ and translation $p$:

$$T = \begin{bmatrix} R & p \\ 0^T & 1 \end{bmatrix}$$

Where $R$ is a 3x3 rotation matrix and $p$ is a 3x1 translation vector.

### Denavit-Hartenberg Convention

The Denavit-Hartenberg (DH) convention provides a systematic method for defining coordinate frames on robotic manipulators:

1. **Link length** ($a_i$): Distance along $x_i$ from $z_{i-1}$ to $z_i$
2. **Link twist** ($\alpha_i$): Angle between $z_{i-1}$ and $z_i$ about $x_i$
3. **Link offset** ($d_i$): Distance along $z_{i-1}$ from origin of frame $i-1$ to origin of frame $i$
4. **Joint angle** ($\theta_i$): Angle between $x_{i-1}$ and $x_i$ about $z_{i-1}$

### Transformation Matrix Calculation

For each joint $i$, the transformation matrix is:

$$A_i = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

The complete forward kinematics is given by:
$$T_{total} = A_1 \cdot A_2 \cdot ... \cdot A_n$$

## 2.3 Inverse Kinematics

Inverse kinematics determines the joint angles required to achieve a desired end-effector position and orientation. This is generally more challenging than forward kinematics.

### Analytical Solutions

For simple manipulators with specific geometric arrangements, analytical solutions may exist. For example, a 6-DOF manipulator with 3 intersecting wrist axes has closed-form solutions.

### Numerical Solutions

For complex manipulators, numerical methods are often used:
- Jacobian-based methods
- Gradient descent
- Pseudoinverse methods

## 2.4 Jacobians and Velocity Relationships

The Jacobian matrix $J(q)$ relates joint velocities to end-effector velocities:

$$\dot{x} = J(q)\dot{q}$$

Where:
- $\dot{x}$ is the end-effector velocity vector in Cartesian space
- $J(q)$ is the Jacobian matrix that depends on joint angles $q$
- $\dot{q}$ is the joint velocity vector

### Jacobian Calculation

The Jacobian can be computed using the geometric approach:

For a revolute joint $i$:
$$J_{v_i} = z_{i-1} \times (p_n - p_{i-1})$$
$$J_{\omega_i} = z_{i-1}$$

For a prismatic joint $i$:
$$J_{v_i} = z_{i-1}$$
$$J_{\omega_i} = 0$$

Where:
- $J_{v_i}$ is the linear velocity component
- $J_{\omega_i}$ is the angular velocity component
- $z_{i-1}$ is the axis of joint $i$ in frame $i-1$
- $p_n$ is the end-effector position
- $p_{i-1}$ is the position of joint $i-1$

## 2.5 Singularities and Manipulability

### Singularities

Singularities occur when the Jacobian loses rank, meaning the robot loses one or more degrees of freedom in Cartesian space. At singularities:
- The robot cannot move in certain directions
- Joint velocities may become very large
- The inverse Jacobian becomes undefined

### Manipulability

Manipulability measures how "well-conditioned" the Jacobian is. One common measure is:
$$m = \sqrt{\det(JJ^T)}$$

## 2.6 Python Implementation

Let's implement forward kinematics for a simple 2-DOF planar manipulator:

```python
import numpy as np
import matplotlib.pyplot as plt

def rotation_matrix(theta):
    """Create a 2D rotation matrix"""
    return np.array([[np.cos(theta), -np.sin(theta)],
                     [np.sin(theta), np.cos(theta)]])

def forward_kinematics_2dof(joint_angles, link_lengths):
    """
    Calculate forward kinematics for a 2-DOF planar manipulator

    Parameters:
    joint_angles: [theta1, theta2] in radians
    link_lengths: [l1, l2] in meters

    Returns:
    end_effector_pos: [x, y] position of end effector
    """
    theta1, theta2 = joint_angles
    l1, l2 = link_lengths

    # Position of joint 2
    x2 = l1 * np.cos(theta1)
    y2 = l1 * np.sin(theta1)

    # Position of end effector
    x_end = x2 + l2 * np.cos(theta1 + theta2)
    y_end = y2 + l2 * np.sin(theta1 + theta2)

    return np.array([x_end, y_end])

def jacobian_2dof(joint_angles, link_lengths):
    """
    Calculate the Jacobian for a 2-DOF planar manipulator
    """
    theta1, theta2 = joint_angles
    l1, l2 = link_lengths

    J = np.array([
        [-l1*np.sin(theta1) - l2*np.sin(theta1+theta2), -l2*np.sin(theta1+theta2)],
        [l1*np.cos(theta1) + l2*np.cos(theta1+theta2), l2*np.cos(theta1+theta2)]
    ])

    return J

# Example usage
theta = [np.pi/4, np.pi/6]  # Joint angles
lengths = [1.0, 0.8]        # Link lengths

end_pos = forward_kinematics_2dof(theta, lengths)
jacobian = jacobian_2dof(theta, lengths)

print(f"End effector position: {end_pos}")
print(f"Jacobian matrix:\n{jacobian}")
```

## 2.7 Visualization and Analysis

```python
def plot_robot_2dof(joint_angles, link_lengths):
    """Plot the 2-DOF manipulator configuration"""
    theta1, theta2 = joint_angles
    l1, l2 = link_lengths

    # Calculate positions
    joint2_x = l1 * np.cos(theta1)
    joint2_y = l1 * np.sin(theta1)

    end_x = joint2_x + l2 * np.cos(theta1 + theta2)
    end_y = joint2_y + l2 * np.sin(theta1 + theta2)

    # Plot
    fig, ax = plt.subplots(figsize=(8, 8))

    # Draw links
    ax.plot([0, joint2_x], [0, joint2_y], 'b-', linewidth=5, label='Link 1')
    ax.plot([joint2_x, end_x], [joint2_y, end_y], 'r-', linewidth=5, label='Link 2')

    # Draw joints
    ax.plot(0, 0, 'ko', markersize=10, label='Base')
    ax.plot(joint2_x, joint2_y, 'ko', markersize=8, label='Joint 2')
    ax.plot(end_x, end_y, 'ro', markersize=8, label='End Effector')

    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_aspect('equal')
    ax.grid(True)
    ax.legend()
    ax.set_title('2-DOF Planar Manipulator')

    plt.show()

# Visualize the robot
plot_robot_2dof(theta, lengths)
```

## Summary

Mathematical foundations are crucial for understanding and controlling robotic systems. The concepts of forward and inverse kinematics, along with Jacobian matrices, form the core mathematical tools needed for robot control and manipulation. Understanding these concepts enables precise control of robot motion and forms the basis for more advanced topics in robotics.

## Next Steps

In the next section, we'll explore physics simulation and digital twins, applying the mathematical concepts learned here to create realistic robot simulations.

<div class="alert alert-info">
  <h5>Hardware Requirement</h5>
  <div><strong>Requirement:</strong> GPU</div>
  <div><strong>Minimum:</strong> Any</div>
  <div><strong>Recommended:</strong> Any</div>
  <div><strong>Purpose:</strong> Basic computational requirements for Week 2 mathematical calculations</div>
</div>