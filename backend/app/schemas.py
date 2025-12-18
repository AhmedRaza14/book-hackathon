from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime

# User schemas
class UserCreate(BaseModel):
    email: str
    password: str
    profile: Optional[dict] = None  # Hardware/software questionnaire data

class UserLogin(BaseModel):
    email: str
    password: str

class UserResponse(BaseModel):
    id: str
    email: str
    created_at: datetime

    class Config:
        from_attributes = True

class UserProfileUpdate(BaseModel):
    gpu_model: Optional[str] = None
    gpu_memory: Optional[int] = None
    system_ram: Optional[int] = None
    cpu_cores: Optional[int] = None
    os_type: Optional[str] = None
    ros_experience_level: Optional[str] = None
    python_experience_level: Optional[str] = None
    ai_ml_experience_level: Optional[str] = None
    robotics_background: Optional[bool] = None
    preferred_language: Optional[str] = None
    learning_pace: Optional[str] = None
    notification_preferences: Optional[dict] = None

class UserProfileResponse(BaseModel):
    id: str
    user_id: str
    gpu_model: Optional[str] = None
    gpu_memory: Optional[int] = None
    system_ram: Optional[int] = None
    cpu_cores: Optional[int] = None
    os_type: Optional[str] = None
    ros_experience_level: Optional[str] = None
    python_experience_level: Optional[str] = None
    ai_ml_experience_level: Optional[str] = None
    robotics_background: Optional[bool] = None
    preferred_language: Optional[str] = None
    learning_pace: Optional[str] = None
    notification_preferences: Optional[dict] = None
    current_module: int
    completed_modules: List[int]
    total_progress_percent: int
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

# Authentication response
class Token(BaseModel):
    access_token: str
    token_type: str

class TokenData(BaseModel):
    user_id: str

# Content schemas
class ContentMetadataCreate(BaseModel):
    chapter_id: str
    title: str
    week_number: int
    module_name: str
    difficulty: str
    duration_minutes: Optional[int] = None
    prerequisites: Optional[List[str]] = []
    learning_objectives: Optional[List[str]] = []
    tags: Optional[List[str]] = []
    hardware_requirements: Optional[dict] = {}

class ContentMetadataResponse(BaseModel):
    id: str
    chapter_id: str
    title: str
    week_number: int
    module_name: str
    difficulty: str
    duration_minutes: Optional[int] = None
    prerequisites: List[str]
    learning_objectives: List[str]
    tags: List[str]
    hardware_requirements: Optional[dict]
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

class ChapterContent(BaseModel):
    chapter_id: str
    content: str
    language: str = "en"

class UpdateProgress(BaseModel):
    status: Optional[str] = None  # 'not-started', 'in-progress', 'completed'
    time_spent: Optional[int] = None  # seconds
    quiz_score: Optional[int] = None
    lab_completed: Optional[bool] = None
    simulation_completed: Optional[bool] = None

# Chat schemas
class ChatRequest(BaseModel):
    query: str
    user_id: Optional[str] = None
    chapter_context: Optional[str] = None
    temperature: float = 0.7
    max_tokens: int = 1000

class ChatResponse(BaseModel):
    response: str
    sources: List[str]
    retrieved_context: List[dict]