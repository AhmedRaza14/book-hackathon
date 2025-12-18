from sqlalchemy import create_engine, Column, Integer, String, DateTime, Boolean, JSON, ARRAY
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from datetime import datetime
import uuid

Base = declarative_base()

class User(Base):
    __tablename__ = "users"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    email = Column(String, unique=True, index=True)
    hashed_password = Column(String)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

class UserProfile(Base):
    __tablename__ = "user_profiles"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    user_id = Column(String, index=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

    # Hardware profile
    gpu_model = Column(String, nullable=True)
    gpu_memory = Column(Integer, nullable=True)  # in GB
    system_ram = Column(Integer, nullable=True)  # in GB
    cpu_cores = Column(Integer, nullable=True)
    os_type = Column(String, nullable=True)  # 'ubuntu', 'windows', 'macos'

    # Experience profile
    ros_experience_level = Column(String, nullable=True)  # 'beginner', 'intermediate', 'advanced'
    python_experience_level = Column(String, nullable=True)
    ai_ml_experience_level = Column(String, nullable=True)
    robotics_background = Column(Boolean, default=False)

    # Learning preferences
    preferred_language = Column(String, default='en')  # 'en', 'ur'
    learning_pace = Column(String, nullable=True)  # 'fast', 'moderate', 'slow'
    notification_preferences = Column(JSON, default=dict)

    # Progress tracking
    current_module = Column(Integer, default=1)
    completed_modules = Column(ARRAY(Integer), default=[])
    total_progress_percent = Column(Integer, default=0)

class ContentMetadata(Base):
    __tablename__ = "content_metadata"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    chapter_id = Column(String, unique=True, index=True)
    title = Column(String, nullable=False)
    week_number = Column(Integer, nullable=False)
    module_name = Column(String, nullable=False)
    difficulty = Column(String, nullable=False)  # 'beginner', 'intermediate', 'advanced'
    duration_minutes = Column(Integer)
    prerequisites = Column(ARRAY(String), default=[])
    learning_objectives = Column(ARRAY(String), default=[])
    tags = Column(ARRAY(String), default=[])
    hardware_requirements = Column(JSON)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

class UserProgress(Base):
    __tablename__ = "user_progress"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    user_id = Column(String, index=True)
    chapter_id = Column(String, index=True)
    status = Column(String, default='not-started')  # 'not-started', 'in-progress', 'completed'
    started_at = Column(DateTime, default=datetime.utcnow)
    completed_at = Column(DateTime, nullable=True)
    time_spent_seconds = Column(Integer, default=0)
    quiz_score = Column(Integer, nullable=True)  # percentage
    lab_completion = Column(Boolean, default=False)
    simulation_completion = Column(Boolean, default=False)
    overall_completion_percent = Column(Integer, default=0)

    __table_args__ = ({"sqlite_autoincrement": True},)