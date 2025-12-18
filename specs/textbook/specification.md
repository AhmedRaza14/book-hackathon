# Physical AI & Humanoid Robotics Textbook - Technical Specification

## 1. Project Architecture

### 1.1 Repository Structure
```
physical-ai-textbook/
├── .specify/                    # SpecKit Plus configuration
│   └── memory/
│       └── constitution.md
├── specs/                       # Project specifications
│   └── textbook/
│       └── spec.md
├── docs/                        # Textbook content in Docusaurus
│   ├── introduction/
│   ├── ros2/
│   ├── digital_twins/
│   ├── ai_brain/
│   ├── vla/
│   └── projects/
├── backend/                     # FastAPI backend
│   ├── app/
│   │   ├── main.py
│   │   ├── models/
│   │   ├── schemas/
│   │   ├── database/
│   │   ├── auth/
│   │   ├── rag/
│   │   └── api/
│   └── requirements.txt
├── src/                         # Docusaurus source files
│   ├── components/              # Custom React components
│   ├── pages/                   # Static pages
│   └── css/                     # Custom styles
├── assets/                      # Images, videos, and other assets
├── history/                     # Prompt History Records
│   └── prompts/
│       ├── constitution/
│       └── textbook/
├── docusaurus.config.ts         # Docusaurus configuration
├── sidebars.ts                  # Navigation sidebar configuration
├── package.json                 # Frontend dependencies
├── pyproject.toml               # Backend dependencies
└── README.md
```

### 1.2 System Architecture
The system follows a microservices architecture with a Docusaurus frontend and FastAPI backend:

- **Frontend**: Docusaurus site with React components for interactive content
- **Backend**: FastAPI server handling authentication, RAG, and user personalization
- **Database**: Neon Postgres for user profiles and content metadata
- **Vector Store**: Qdrant for semantic search and RAG context
- **Authentication**: Better-Auth for user management and personalization
- **Content Pipeline**: MDX content → embedding extraction → Qdrant indexing

### 1.3 RAG Pipeline
1. **Content Ingestion**: MDX files from docs/ directory are processed
2. **Embedding Generation**: OpenAI embeddings created for content chunks
3. **Indexing**: Content stored in Qdrant vector database with metadata
4. **Query Processing**: User queries converted to embeddings for similarity search
5. **Context Retrieval**: Relevant content retrieved from Qdrant
6. **Response Generation**: OpenAI Agent generates responses using retrieved context

### 1.4 Personalization Flow
1. **User Profile**: Better-Auth captures hardware/software background during signup
2. **Context Adaptation**: React components adjust content based on user profile
3. **Adaptive Rendering**: Content difficulty and examples tailored to user experience level
4. **Progress Tracking**: Learning path adjusted based on user interactions

## 2. Content Schema & Taxonomy

### 2.1 Standard Chapter Layout
Each chapter follows the "TRUS" format:
- **T** - Technical Theory (core concepts and principles)
- **R** - ROS 2/Python Code Lab (practical implementation)
- **U** - Urdu Summary (localized content)
- **S** - Simulation Guide (Gazebo/Isaac practicals)

### 2.2 Module Structure (Week 1 to Week 15)
```
docs/
├── week-01/
│   ├── theory.mdx
│   ├── code-lab.mdx
│   ├── simulation.mdx
│   └── urdu-summary.mdx
├── week-02/
│   ├── theory.mdx
│   ├── code-lab.mdx
│   ├── simulation.mdx
│   └── urdu-summary.mdx
├── week-03/
│   ├── theory.mdx
│   ├── code-lab.mdx
│   ├── simulation.mdx
│   └── urdu-summary.mdx
├── week-04/
│   ├── theory.mdx
│   ├── code-lab.mdx
│   ├── simulation.mdx
│   └── urdu-summary.mdx
├── week-05/
│   ├── theory.mdx
│   ├── code-lab.mdx
│   ├── simulation.mdx
│   └── urdu-summary.mdx
├── week-06/
│   ├── theory.mdx
│   ├── code-lab.mdx
│   ├── simulation.mdx
│   └── urdu-summary.mdx
├── week-07/
│   ├── theory.mdx
│   ├── code-lab.mdx
│   ├── simulation.mdx
│   └── urdu-summary.mdx
├── week-08/
│   ├── theory.mdx
│   ├── code-lab.mdx
│   ├── simulation.mdx
│   └── urdu-summary.mdx
├── week-09/
│   ├── theory.mdx
│   ├── code-lab.mdx
│   ├── simulation.mdx
│   └── urdu-summary.mdx
├── week-10/
│   ├── theory.mdx
│   ├── code-lab.mdx
│   ├── simulation.mdx
│   └── urdu-summary.mdx
├── week-11/
│   ├── theory.mdx
│   ├── code-lab.mdx
│   ├── simulation.mdx
│   └── urdu-summary.mdx
├── week-12/
│   ├── theory.mdx
│   ├── code-lab.mdx
│   ├── simulation.mdx
│   └── urdu-summary.mdx
├── week-13/
│   ├── theory.mdx
│   ├── code-lab.mdx
│   ├── simulation.mdx
│   └── urdu-summary.mdx
├── week-14/
│   ├── theory.mdx
│   ├── code-lab.mdx
│   ├── simulation.mdx
│   └── urdu-summary.mdx
└── week-15/
    ├── theory.mdx
    ├── code-lab.mdx
    ├── simulation.mdx
    └── urdu-summary.mdx
```

### 2.3 Content Metadata Schema
Each MDX file includes frontmatter with:
```yaml
---
title: "Chapter Title"
week: 1
module: "Foundations of Physical AI"
difficulty: "intermediate"  # beginner, intermediate, advanced
prerequisites: ["linear-algebra", "python-basics"]
learning_objectives:
  - "Understand concept of Physical AI"
  - "Implement basic ROS 2 node"
  - "Simulate robot in Gazebo"
tags: ["ros2", "kinematics", "simulation"]
hardware_requirements:
  - gpu: "RTX 4070 or higher"
  - ram: "32GB minimum"
  - os: "Ubuntu 22.04 LTS"
duration: "90 minutes"  # estimated completion time
---
```

### 2.4 Custom MDX Components

#### Hardware Alerts Component
```jsx
<HardwareAlert
  requirement="gpu"
  minimum="RTX 4070"
  recommended="RTX 4080"
  purpose="Simulation and AI processing"
/>
```

#### Interactive Code Blocks
```jsx
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

#### Urdu Translation Toggle
```jsx
<UrduTranslationToggle>
  <div className="english-content">
    {/* English content */}
  </div>
  <div className="urdu-content" dir="rtl">
    {/* Urdu translation */}
  </div>
</UrduTranslationToggle>
```

### 2.5 Content Validation Rules
- All code examples must be tested and functional
- Mathematical notation uses LaTeX format (e.g., $J(q)$ for Jacobians)
- Hardware requirements clearly specified for each chapter
- URDF/SDF examples provided where applicable
- Sensor simulation examples included (LiDAR, Depth, IMU)
- Consistent 15-week structure maintained throughout (Week 1-15)

## 3. Database & Auth Schema

### 3.1 Neon Postgres Schema

#### User Profiles Table
```sql
CREATE TABLE user_profiles (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id VARCHAR(255) UNIQUE NOT NULL,
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW(),

  -- Hardware profile
  gpu_model VARCHAR(100),
  gpu_memory INTEGER,  -- in GB
  system_ram INTEGER,  -- in GB
  cpu_cores INTEGER,
  os_type VARCHAR(50),  -- 'ubuntu', 'windows', 'macos'

  -- Experience profile
  ros_experience_level VARCHAR(20),  -- 'beginner', 'intermediate', 'advanced'
  python_experience_level VARCHAR(20),
  ai_ml_experience_level VARCHAR(20),
  robotics_background BOOLEAN DEFAULT FALSE,

  -- Learning preferences
  preferred_language VARCHAR(10) DEFAULT 'en',  -- 'en', 'ur'
  learning_pace VARCHAR(20),  -- 'fast', 'moderate', 'slow'
  notification_preferences JSONB,

  -- Progress tracking
  current_module INTEGER DEFAULT 1,
  completed_modules INTEGER[] DEFAULT '{}',
  total_progress_percent INTEGER DEFAULT 0,

  FOREIGN KEY (user_id) REFERENCES accounts(id)
);
```

#### Content Metadata Table
```sql
CREATE TABLE content_metadata (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  chapter_id VARCHAR(100) UNIQUE NOT NULL,
  title VARCHAR(255) NOT NULL,
  week_number INTEGER NOT NULL,
  module_name VARCHAR(255) NOT NULL,
  difficulty VARCHAR(20) NOT NULL,  -- 'beginner', 'intermediate', 'advanced'
  duration_minutes INTEGER,
  prerequisites TEXT[],
  learning_objectives TEXT[],
  tags TEXT[],
  hardware_requirements JSONB,
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW()
);
```

#### User Progress Table
```sql
CREATE TABLE user_progress (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id VARCHAR(255) NOT NULL,
  chapter_id VARCHAR(100) NOT NULL,
  status VARCHAR(20) DEFAULT 'not-started',  -- 'not-started', 'in-progress', 'completed'
  started_at TIMESTAMP DEFAULT NOW(),
  completed_at TIMESTAMP,
  time_spent_seconds INTEGER DEFAULT 0,
  quiz_score INTEGER,  -- percentage
  lab_completion BOOLEAN DEFAULT FALSE,
  simulation_completion BOOLEAN DEFAULT FALSE,
  overall_completion_percent INTEGER DEFAULT 0,

  UNIQUE(user_id, chapter_id),
  FOREIGN KEY (user_id) REFERENCES accounts(id),
  FOREIGN KEY (chapter_id) REFERENCES content_metadata(chapter_id)
);
```

### 3.2 Qdrant Collection Schema

#### Content Vectors Collection
```json
{
  "collection_name": "textbook_content",
  "vectors_config": {
    "content_vector": {
      "size": 1536,  // OpenAI embedding size
      "distance": "Cosine"
    }
  },
  "payload_schema": {
    "chapter_id": "keyword",
    "week_number": "integer",
    "module_name": "keyword",
    "difficulty": "keyword",
    "content_type": "keyword",  // 'theory', 'code', 'simulation', 'summary'
    "tags": "keyword[]",
    "text_content": "text",
    "section_title": "keyword",
    "chunk_index": "integer",
    "total_chunks": "integer"
  }
}
```

#### User Interaction Collection
```json
{
  "collection_name": "user_interactions",
  "vectors_config": {
    "query_vector": {
      "size": 1536,
      "distance": "Cosine"
    }
  },
  "payload_schema": {
    "user_id": "keyword",
    "session_id": "keyword",
    "query_text": "text",
    "response_text": "text",
    "timestamp": "integer",
    "chapter_context": "keyword",
    "helpfulness_rating": "integer"  // 1-5 scale
  }
}
```

## 4. API Endpoints (FastAPI)

### 4.1 Authentication Endpoints (Better-Auth Integration)
```python
# POST /auth/register
# User registration with hardware/software questionnaire
{
  "email": "user@example.com",
  "password": "secure_password",
  "profile": {
    "gpu_model": "RTX 4070",
    "gpu_memory": 12,
    "system_ram": 32,
    "ros_experience_level": "beginner",
    "python_experience_level": "intermediate",
    "preferred_language": "en"
  }
}

# POST /auth/login
# Standard login with profile update capability
{
  "email": "user@example.com",
  "password": "secure_password"
}

# GET /auth/profile
# Retrieve user profile with personalization data
# Response includes hardware specs, experience levels, and learning preferences

# PUT /auth/profile
# Update user profile and preferences
```

### 4.2 Content Endpoints
```python
# GET /api/content/chapter/{chapter_id}
# Retrieve chapter content with personalization
# Query params:
# - user_id (optional, for personalization)
# - language (en/ur)
# Response includes MDX content, related resources, and difficulty adjustment

# GET /api/content/progress/{user_id}
# Get user's content progress and recommendations
# Response includes completed chapters, current module, and next recommendations

# PUT /api/content/progress/{user_id}/{chapter_id}
# Update user progress for a chapter
{
  "status": "completed",
  "time_spent": 2400,  // seconds
  "quiz_score": 85,
  "lab_completed": true
}
```

### 4.3 RAG Chat Endpoint
```python
# POST /api/chat
# Streamed response from OpenAI Agent using Qdrant context
{
  "query": "Explain inverse kinematics for a 6-DOF robot arm",
  "user_id": "user-uuid",  // optional for personalization
  "chapter_context": "week-02-theory",  // optional for focused context
  "temperature": 0.7,
  "max_tokens": 1000
}

# Response: Server-sent events stream
{
  "id": "response-id",
  "type": "token|context|error",
  "content": "explanation of inverse kinematics...",
  "retrieved_context": [
    {
      "chapter_id": "week-02-theory",
      "section": "kinematics-fundamentals",
      "relevance_score": 0.89
    }
  ],
  "sources": ["week-02-theory", "week-03-code-lab"]
}
```

### 4.4 Simulation & Lab Endpoints
```python
# POST /api/simulation/execute
# Execute simulation code and return results
{
  "chapter_id": "week-04-code-lab",
  "code": "import rclpy\n# student code...",
  "simulation_environment": "gazebo",  // or "isaac-sim"
  "parameters": {
    "robot_model": "ur5",
    "simulation_time": 10.0
  }
}

# GET /api/simulation/status/{execution_id}
# Get simulation execution status and results
# Response includes logs, metrics, and visualization data
```

### 4.5 Personalization Endpoints
```python
# GET /api/personalization/recommendations/{user_id}
# Get personalized content recommendations
# Response includes:
# - Recommended next chapters based on progress
# - Adjusted difficulty levels
# - Hardware-appropriate content
# - Language preference consideration

# GET /api/personalization/adapt-content/{chapter_id}
# Get chapter content adapted to user profile
# Query params:
# - user_id: for personalization
# - hardware_filter: adjust examples based on user's hardware
# - experience_level: adjust complexity
# - language: en/ur
```

### 4.6 Admin & Content Management Endpoints
```python
# POST /api/admin/content/ingest
# Ingest new content into the RAG system
{
  "content_path": "docs/week-05/theory.mdx",
  "force_reindex": false
}

# GET /api/admin/content/stats
# Get content statistics and indexing status
# Response includes indexed chapters, vector counts, and quality metrics
```

## 5. Security & Privacy Considerations

### 5.1 Data Protection
- User profiles encrypted at rest
- Personalization data anonymized where possible
- Hardware specs and experience levels used only for content adaptation
- Compliance with privacy regulations (GDPR, etc.)

### 5.2 API Security
- Rate limiting on all endpoints
- JWT token authentication for all user-specific operations
- Input validation and sanitization
- Secure embedding generation and storage

### 5.3 Content Security
- Code execution in sandboxed environments
- Simulation access controlled and monitored
- Content integrity checks for MDX files

## 6. Performance & Scalability

### 6.1 Caching Strategy
- CDN for static content (MDX files, images)
- Redis for user sessions and temporary data
- Qdrant for vector similarity searches
- Database query result caching

### 6.2 Load Balancing
- Multiple FastAPI instances behind load balancer
- Database read replicas for high availability
- Vector database sharding for large content collections

### 6.3 Monitoring & Observability
- Request/response logging
- Performance metrics (latency, throughput)
- Error tracking and alerting
- Content quality monitoring