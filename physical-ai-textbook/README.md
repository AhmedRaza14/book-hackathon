# Physical AI & Humanoid Robotics Textbook

This repository contains the source materials for the "Physical AI & Humanoid Robotics" textbook for Panaversity. The textbook bridges the gap between digital AI models and embodied intelligence, focusing on the integration of AI with physical robotics systems.

## Table of Contents
- [Overview](#overview)
- [Technical Stack](#technical-stack)
- [Prerequisites](#prerequisites)
- [Getting Started](#getting-started)
- [Project Structure](#project-structure)
- [Features](#features)
- [Contributing](#contributing)

## Overview
This textbook aims to bridge the gap between digital AI models and embodied intelligence, focusing on the integration of AI with physical robotics systems. The book covers both theoretical foundations and practical implementation of humanoid robots capable of intelligent interaction with the physical world.

## Technical Stack
- **Framework:** Docusaurus (deployed via GitHub Pages/Vercel)
- **Robot Operating System:** ROS 2 Humble Hawksbill
- **Simulation:** Gazebo, NVIDIA Isaac Sim
- **AI Platform:** NVIDIA Isaac ROS, Omniverse
- **Programming Language:** Python
- **Backend:** FastAPI
- **Database:** Neon Postgres
- **Vector Store:** Qdrant
- **Authentication:** Better-Auth

## Prerequisites
- RTX 4070 or higher GPU recommended for simulation
- Ubuntu 22.04 LTS or Windows 10/11 with WSL2
- At least 32GB RAM for complex simulations
- Basic Python programming knowledge
- Understanding of linear algebra and calculus

## Getting Started

### Frontend (Docusaurus)
1. Navigate to the project directory
2. Install dependencies: `npm install`
3. Start the development server: `npm start`
4. Open your browser to http://localhost:3000

### Backend (FastAPI)
1. Navigate to the backend directory: `cd backend`
2. Create a virtual environment: `python -m venv venv`
3. Activate the virtual environment:
   - On Windows: `venv\Scripts\activate`
   - On macOS/Linux: `source venv/bin/activate`
4. Install dependencies: `pip install -r requirements.txt`
5. Set up environment variables (see `.env.example`)
6. Run the server: `uvicorn app.main:app --reload`

## Project Structure
```
physical-ai-textbook/
├── docs/                    # Textbook content (15-week curriculum)
│   ├── intro.md            # Introduction page
│   ├── week-01/            # Week 1 content
│   │   ├── theory.mdx      # Theory component
│   │   ├── code-lab.mdx    # ROS 2 Python code lab
│   │   ├── simulation.mdx  # Simulation guide
│   │   └── urdu-summary.mdx # Urdu summary
│   ├── week-02/            # Week 2 content
│   └── ...                 # Weeks 3-15
├── backend/                # FastAPI backend
│   ├── app/                # Application code
│   │   ├── main.py         # Main application
│   │   ├── models.py       # Database models
│   │   ├── schemas.py      # Pydantic schemas
│   │   ├── database.py     # Database configuration
│   │   ├── auth.py         # Authentication logic
│   │   ├── rag.py          # RAG system
│   │   └── api/            # API routes
│   └── requirements.txt    # Backend dependencies
├── src/                    # Docusaurus source files
│   ├── components/         # Custom React components
│   ├── pages/              # Static pages
│   └── css/                # Custom styles
├── assets/                 # Images, videos, and other assets
└── ...
```

## Features

### 1. Personalized Learning
- Content adapts to user's hardware specifications and experience level
- "Personalize" button for adjusting content difficulty
- Hardware-aware content filtering

### 2. Multi-language Support
- Urdu translations for broader accessibility
- "Translate to Urdu" toggle with RTL support
- Cultural appropriateness considerations

### 3. Interactive Elements
- Interactive code blocks with simulation capabilities
- Hardware requirement alerts
- AI-powered chat assistant with textbook knowledge

### 4. 15-Week Curriculum
- Comprehensive coverage from Physical AI fundamentals to capstone projects
- TRUS format: Theory, ROS 2/Python, Urdu Summary, Simulation Guide
- Progressive difficulty and complexity

### 5. RAG System
- Retrieval-Augmented Generation for context-aware responses
- Integration with textbook content
- Streaming responses for better user experience

## Custom Components

### HardwareAlert
```md
<HardwareAlert
  requirement="gpu"
  minimum="RTX 4070"
  recommended="RTX 4080"
  purpose="Simulation and AI processing"
/>
```

### InteractiveCodeBlock
```md
<InteractiveCodeBlock
  language="python"
  framework="ros2"
  environment="isaac-sim"
  solutionAvailable={true}
  hint="Use rclpy to create the node"
>
# Student code goes here
</InteractiveCodeBlock>
```

### UrduTranslationToggle
```md
<UrduTranslationToggle>
  <div className="english-content">
    {/* English content */}
  </div>
  <div className="urdu-content" dir="rtl">
    {/* Urdu translation */}
  </div>
</UrduTranslationToggle>
```

## Contributing
We welcome contributions to improve the textbook content, code examples, and exercises. Please follow the contribution guidelines in the documentation.

## License
This textbook is released under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
