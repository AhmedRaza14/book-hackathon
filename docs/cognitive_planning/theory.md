# Week 11: Cognitive Planning

## Learning Objectives
- Design task and motion planning systems
- Implement hierarchical planning architectures
- Handle uncertainty in decision making
- Understand reactive vs. deliberative systems

## Topics Covered

### Task and Motion Planning
Task and motion planning is a fundamental challenge in robotics that involves determining a sequence of actions to achieve a specified goal. Task planning focuses on high-level symbolic actions, while motion planning deals with geometric and kinematic constraints.

Task planning typically involves:
- Symbolic state representation
- Operator-based action models
- Heuristic search algorithms
- Planning domain definition language (PDDL)

Motion planning addresses:
- Configuration space (C-space) representation
- Collision-free path computation
- Kinodynamic constraints
- Sampling-based algorithms (RRT, PRM)
- Optimization-based methods

### Hierarchical Planning Architectures
Hierarchical planning decomposes complex problems into manageable subproblems organized across multiple levels of abstraction. Common architectures include:

- Behavior trees for modular task decomposition
- Finite state machines for reactive control
- HTN (Hierarchical Task Networks) for structured planning
- Goal-oriented action planning (GOAP)
- Multi-layer control architectures

Each layer operates at different temporal and spatial resolutions, with higher levels providing abstract goals and lower levels handling detailed execution.

### Reactive vs. Deliberative Systems
Robotic systems can be classified based on their decision-making approach:

Reactive systems:
- Respond directly to sensory input
- Fast reaction times
- Limited planning horizon
- Suitable for well-structured environments
- Examples: subsumption architecture, behavior-based robotics

Deliberative systems:
- Plan ahead using internal models
- Consider multiple alternatives
- Optimize over extended horizons
- Computationally intensive
- Better for complex, uncertain environments

Hybrid architectures combine both approaches for robust performance.

### Decision-Making Under Uncertainty
Real-world robotics involves dealing with various sources of uncertainty including:
- State uncertainty (sensor noise, occlusions)
- Action uncertainty (actuator noise, model inaccuracies)
- Environmental uncertainty (dynamic obstacles, changing conditions)

Techniques for handling uncertainty include:
- Probabilistic roadmaps
- Partially Observable Markov Decision Processes (POMDPs)
- Monte Carlo methods
- Bayesian inference
- Robust optimization

## Activities
- Implement task planner
- Design hierarchical system
- Test decision making under uncertainty

## Key Equations and Formulas
- Bellman equation for optimal value function: V*(s) = max_a Σ_s' P(s'|s,a)[R(s,a,s') + γV*(s')]
- Configuration space: C_free = C - C_obstacle, where C is total configuration space

## Assessment
- Planning system implementation
- Uncertainty handling project

## Additional Resources
- Automated Planning and Acting by Ghallab et al.
- Probabilistic Robotics by Thrun et al.
- Hierarchical Task Network Planning Literature
- Uncertainty in Robotics Research Papers