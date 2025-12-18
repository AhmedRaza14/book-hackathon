---
id: 2
title: "Textbook Technical Specification"
stage: spec
date: 2025-12-18
surface: agent
model: Claude Sonnet 4.5
feature: textbook
branch: master
user: admin
command: "/sp.specify"
labels: ["specification", "architecture", "physical-ai"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "specs/textbook/specification.md"
tests: []
---

# Textbook Technical Specification

## PROMPT_TEXT
/sp.specify
# Role
You are a Technical Architect and Lead Developer. Your task is to transform the "Physical AI & Humanoid Robotics Textbook Constitution" into a detailed technical specification (specification.md) for a full-stack AI-native textbook project.

# Input Reference
Refer strictly to the provided "Constitution.md" for the mission, tech stack (Docusaurus, ROS 2, NVIDIA Isaac, Better-Auth, FastAPI, Neon, Qdrant), and the 13-week course breakdown.

# Instructions for generating Specification.md
Please generate a comprehensive specification document covering the following sections:

## 1. Project Architecture
- Define the repository structure for a Docusaurus site integrated with a FastAPI backend.
- Detail the RAG (Retrieval-Augmented Generation) pipeline: How content from MDX files will be ingested into Qdrant using OpenAI embeddings.
- Specify the flow for the "Personalization" feature: How Better-Auth user data (hardware/software background) will influence content rendering via React components.

## 2. Content Schema & Taxonomy
- Define a standard layout for the 13 modules (Week 1 to Week 13).
- Every chapter must include: Technical Theory, Code Lab (ROS 2/Python), Simulation Guide (Gazebo/Isaac), and an Urdu Summary section.
- Define custom MDX components for: "Hardware Alerts" (RTX requirements), "Interactive Code Blocks," and "Urdu Translation Toggles."

## 3. Database & Auth Schema
- Design the Neon Postgres schema for User Profiles (storing the signup questionnaire results: GPU model, RAM, ROS experience).
- Define the Qdrant collection schema for semantic search and context-aware RAG.

## 4. API Endpoints (FastAPI)
- `/chat`: Streamed response from OpenAI Agent using Qdrant context.

## RESPONSE_TEXT
I have successfully created a comprehensive technical specification for the Physical AI & Humanoid Robotics Textbook based on the constitution. The specification document (specs/textbook/specification.md) includes:

## 1. Project Architecture
- Detailed repository structure with frontend, backend, and content organization
- System architecture with Docusaurus frontend, FastAPI backend, Neon Postgres, and Qdrant vector store
- RAG pipeline from MDX content to OpenAI embeddings to Qdrant indexing
- Personalization flow using Better-Auth user profiles to adapt content

## 2. Content Schema & Taxonomy
- Standard "TRUS" format for chapters (Technical Theory, ROS 2/Python Code Lab, Urdu Summary, Simulation Guide)
- 13-week module structure with consistent organization
- Content metadata schema with frontmatter requirements
- Custom MDX components for Hardware Alerts, Interactive Code Blocks, and Urdu Translation Toggles
- Content validation rules for code examples and mathematical notation

## 3. Database & Auth Schema
- Neon Postgres schema with user_profiles, content_metadata, and user_progress tables
- Qdrant collection schema for content vectors and user interactions
- Proper relationships and indexing for efficient queries

## 4. API Endpoints
- Authentication endpoints with Better-Auth integration
- Content endpoints for retrieving and tracking progress
- RAG chat endpoint with streamed responses from OpenAI Agent
- Simulation and lab execution endpoints
- Personalization endpoints for adaptive content delivery
- Admin endpoints for content management

The specification also includes security, privacy, performance, and scalability considerations to ensure the system is robust and maintainable.

## OUTCOME
Successfully created a comprehensive technical specification that transforms the Physical AI & Humanoid Robotics Textbook Constitution into a detailed implementation plan. The specification covers all required aspects including architecture, content schema, database design, and API endpoints, providing a complete roadmap for implementing the full-stack AI-native textbook project.