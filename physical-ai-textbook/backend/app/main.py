from fastapi import FastAPI, Depends
from fastapi.middleware.cors import CORSMiddleware
from .database import engine
from . import models
from .api import auth, chat, content
from .auth import get_current_user

# Create database tables
models.Base.metadata.create_all(bind=engine)

app = FastAPI(
    title="Physical AI & Humanoid Robotics API",
    description="API for the Physical AI & Humanoid Robotics textbook platform",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify your frontend domain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routers
app.include_router(auth.router, prefix="/auth", tags=["authentication"])
app.include_router(chat.router, prefix="/api", tags=["chat"])
app.include_router(content.router, prefix="/api", tags=["content"])

@app.get("/")
def read_root():
    return {"message": "Physical AI & Humanoid Robotics API"}

@app.get("/health")
def health_check():
    return {"status": "healthy"}