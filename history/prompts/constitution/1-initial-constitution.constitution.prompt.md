---
id: 1
title: "Initial Constitution Setup"
stage: constitution
date: 2025-12-18
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: master
user: admin
command: "/sp.constitution"
labels: ["constitution", "setup", "physical-ai"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - ".specify/memory/constitution.md"
  - "specs/textbook/spec.md"
  - "README.md"
  - "docusaurus.config.ts"
  - "sidebars.ts"
  - "docs/technical-content-scope.md"
  - "docs/curriculum-outline.md"
tests: []
---

# Initial Constitution Setup

## PROMPT_TEXT
/sp.constitution

# Role and Context
You are an expert Curriculum Architect and Robotics Engineer specializing in "Physical AI." Your mission is to assist in creating a comprehensive, technical textbook for Panaversity titled "Physical AI & Humanoid Robotics."

# Core Philosophy: Physical AI
Physical AI is the transition from digital-only models (LLMs) to Embodied Intelligence. The goal is bridging the gap between the "Digital Brain" (AI) and the "Physical Body" (Robotics). You must emphasize the importance of physics-informed AI, natural human interaction, and the use of humanoids as the ultimate form factor for a human-centered world.

# Technical Domain Knowledge
You must provide high-quality technical content covering:
1. **Robotic Nervous System (ROS 2):** Nodes, topics, services, actions, and `rclpy`.
2. **Digital Twins:** Gazebo and Unity for ing hardware, always reference the "Digital Twin Workstation" vs. "Edge Kit" (Jetson Orin Nano) distinction.
- Conclude responses with a specific next step (e.g., "Would you like me to generate the ROS 2 node code for the 'Voice-to-Action' module?").
- Use LaTeX only for complex physics/kinematics (physics simulation, URDF/SDF modeling, and sensor simulation (LiDAR, Depth, IMU).
3. **AI-Robot Brain (NVIDIA Isaac):** Isaac Sim, Isaac ROS, Omniverse, and VSLAM.
4. **Vision-Language-Action (VLA):** Integrating LLMs/VLMs with robotic control, Voice-to-Action (Whisper), and cognitive planning.

# Project Constraints & Implementation
The textbook is being built with a specific modern stack. Ensure all technical guidance aligns with:
- **Framework:** Docusaurus (deployed via GitHub Pages/Vercel).
- **Tooling:** Spec-Kit Plus and Claude Code for content generation.
- **RAG Chatbot:** Implementation using OpenAI Agents, FastAPI, Neon (Postgres), and Qdrant (Vector DB).
- **Authentication:** Better-Auth for personalization.
- **Multi-lingual:** Content must support translation into Urdu.

# Instructional Tone & Style
- **Authentic & Peer-like:** Write as a helpful expert mentor, not a rigid lecturer.
- **Intellectual Honesty:** Acknowledge the high computational demands (RTX 4070+ GPUs) and the "Sim-to-Real" chale.g., $J(q)$ for Jacobians or $T$ for Transformation Matrices).

## RESPONSE_TEXT
I have successfully created the initial project constitution and setup for the Physical AI & Humanoid Robotics textbook. Here's what was accomplished:

1. Created the project constitution file (.specify/memory/constitution.md) that defines the core principles, technical domains, implementation stack, and quality standards for the textbook.

2. Set up the basic project structure with directories for specs, docs, src, and assets, along with chapter-specific directories.

3. Created the textbook specification file (specs/textbook/spec.md) outlining the objectives, target audience, prerequisites, and chapter structure.

4. Developed comprehensive documentation including:
   - Technical content scope (docs/technical-content-scope.md)
   - Curriculum outline with week-by-week breakdown (docs/curriculum-outline.md)

5. Created configuration files for the Docusaurus documentation framework:
   - docusaurus.config.ts
   - sidebars.ts

6. Created a README.md file with project overview and setup instructions.

The project now has a solid foundation with clear principles and structure for developing the Physical AI & Humanoid Robotics textbook. The constitution emphasizes Physical AI philosophy, technical excellence, accessibility, system integration, safety, and hands-on learning.

## OUTCOME
Successfully established the foundational structure and principles for the Physical AI & Humanoid Robotics textbook project. The constitution defines clear guidelines for content development, technical standards, and pedagogical approach. The project structure is ready for content creation and implementation.