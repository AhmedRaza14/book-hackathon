from fastapi import APIRouter, Depends, HTTPException, Query
from sqlalchemy.orm import Session
from typing import List, Optional
import os

from ..database import get_db
from .. import models, schemas, auth

router = APIRouter()

@router.get("/content/chapter/{chapter_id}", response_model=schemas.ContentMetadataResponse)
def get_chapter_content(
    chapter_id: str,
    user_id: Optional[str] = Query(None),
    language: str = Query("en", regex="^(en|ur)$"),
    db: Session = Depends(get_db)
):
    """
    Retrieve chapter content with personalization based on user profile.
    """
    # Get content metadata
    content_metadata = db.query(models.ContentMetadata).filter(
        models.ContentMetadata.chapter_id == chapter_id
    ).first()

    if not content_metadata:
        raise HTTPException(status_code=404, detail="Chapter not found")

    # If user_id is provided, we could personalize the content
    # For now, just return the metadata
    return content_metadata

@router.get("/content/progress/{user_id}", response_model=dict)
def get_user_progress(
    user_id: str,
    db: Session = Depends(get_db)
):
    """
    Get user's content progress and recommendations.
    """
    # Get user profile to understand their background
    user_profile = db.query(models.UserProfile).filter(
        models.UserProfile.user_id == user_id
    ).first()

    # Get user progress records
    progress_records = db.query(models.UserProgress).filter(
        models.UserProgress.user_id == user_id
    ).all()

    # Calculate overall progress
    completed_count = sum(1 for record in progress_records if record.status == 'completed')
    total_chapters = db.query(models.ContentMetadata).count()

    # Get completed modules
    completed_modules = []
    for record in progress_records:
        if record.status == 'completed':
            # Extract module number from chapter_id (assuming format like "week-01-theory")
            try:
                module_num = int(record.chapter_id.split('-')[1])
                if module_num not in completed_modules:
                    completed_modules.append(module_num)
            except:
                pass

    # Get next recommendations based on current progress
    next_recommendations = []
    if completed_modules:
        next_module = max(completed_modules) + 1
        # Get all chapters for the next module
        next_chapters = db.query(models.ContentMetadata).filter(
            models.ContentMetadata.week_number == next_module
        ).all()
        next_recommendations = [chapter.chapter_id for chapter in next_chapters]
    else:
        # If no progress, recommend starting with week 1
        first_chapters = db.query(models.ContentMetadata).filter(
            models.ContentMetadata.week_number == 1
        ).all()
        next_recommendations = [chapter.chapter_id for chapter in first_chapters]

    return {
        "completed_count": completed_count,
        "total_chapters": total_chapters,
        "overall_progress_percent": (completed_count / total_chapters * 100) if total_chapters > 0 else 0,
        "completed_modules": sorted(completed_modules),
        "next_recommendations": next_recommendations,
        "current_module": max(completed_modules) + 1 if completed_modules else 1,
        "user_profile": user_profile.__dict__ if user_profile else None
    }

@router.put("/content/progress/{user_id}/{chapter_id}")
def update_chapter_progress(
    user_id: str,
    chapter_id: str,
    progress_update: schemas.UpdateProgress,
    db: Session = Depends(get_db)
):
    """
    Update user progress for a specific chapter.
    """
    # Check if progress record exists
    progress_record = db.query(models.UserProgress).filter(
        models.UserProgress.user_id == user_id,
        models.UserProgress.chapter_id == chapter_id
    ).first()

    if not progress_record:
        # Create new progress record
        progress_record = models.UserProgress(
            user_id=user_id,
            chapter_id=chapter_id
        )
        db.add(progress_record)

    # Update fields based on the request
    if progress_update.status is not None:
        progress_record.status = progress_update.status
    if progress_update.time_spent is not None:
        progress_record.time_spent_seconds = progress_update.time_spent
    if progress_update.quiz_score is not None:
        progress_record.quiz_score = progress_update.quiz_score
    if progress_update.lab_completed is not None:
        progress_record.lab_completion = progress_update.lab_completed
    if progress_update.simulation_completed is not None:
        progress_record.simulation_completion = progress_update.simulation_completed

    # Calculate overall completion percentage
    completed_parts = 0
    total_parts = 3  # quiz, lab, simulation

    if progress_record.quiz_score is not None and progress_record.quiz_score > 0:
        completed_parts += 1
    if progress_record.lab_completion:
        completed_parts += 1
    if progress_record.simulation_completion:
        completed_parts += 1

    progress_record.overall_completion_percent = int((completed_parts / total_parts) * 100)

    db.commit()

    # Update user profile with current progress
    user_profile = db.query(models.UserProfile).filter(
        models.UserProfile.user_id == user_id
    ).first()

    if user_profile:
        # Update completed modules in user profile
        completed_chapters = db.query(models.UserProgress).filter(
            models.UserProgress.user_id == user_id,
            models.UserProgress.status == 'completed'
        ).all()

        completed_modules = []
        for record in completed_chapters:
            try:
                module_num = int(record.chapter_id.split('-')[1])
                if module_num not in completed_modules:
                    completed_modules.append(module_num)
            except:
                pass

        user_profile.completed_modules = completed_modules
        user_profile.total_progress_percent = int((len(completed_chapters) / db.query(models.ContentMetadata).count()) * 100)
        db.commit()

    return {"message": "Progress updated successfully"}

@router.get("/content/metadata", response_model=List[schemas.ContentMetadataResponse])
def get_all_content_metadata(
    week_number: Optional[int] = None,
    difficulty: Optional[str] = None,
    tag: Optional[str] = None,
    db: Session = Depends(get_db)
):
    """
    Get all content metadata with optional filtering.
    """
    query = db.query(models.ContentMetadata)

    if week_number is not None:
        query = query.filter(models.ContentMetadata.week_number == week_number)
    if difficulty is not None:
        query = query.filter(models.ContentMetadata.difficulty == difficulty)
    if tag is not None:
        query = query.filter(models.ContentMetadata.tags.any(tag))

    content_list = query.all()
    return content_list

@router.get("/content/personalization/recommendations/{user_id}")
def get_personalized_recommendations(
    user_id: str,
    db: Session = Depends(get_db)
):
    """
    Get personalized content recommendations based on user profile.
    """
    # Get user profile
    user_profile = db.query(models.UserProfile).filter(
        models.UserProfile.user_id == user_id
    ).first()

    if not user_profile:
        raise HTTPException(status_code=404, detail="User profile not found")

    # Get user progress
    user_progress = db.query(models.UserProgress).filter(
        models.UserProgress.user_id == user_id
    ).all()

    completed_chapter_ids = [progress.chapter_id for progress in user_progress if progress.status == 'completed']

    # Build query based on user profile
    query = db.query(models.ContentMetadata)

    # Filter out already completed chapters
    if completed_chapter_ids:
        query = query.filter(~models.ContentMetadata.chapter_id.in_(completed_chapter_ids))

    # Apply difficulty filter based on user experience
    if user_profile.ros_experience_level == "beginner":
        query = query.filter(models.ContentMetadata.difficulty.in_(['beginner', 'intermediate']))
    elif user_profile.ros_experience_level == "advanced":
        query = query.filter(models.ContentMetadata.difficulty.in_(['intermediate', 'advanced']))

    # Apply hardware filter - if user doesn't have high-end GPU, filter out content requiring Isaac Sim
    if user_profile.gpu_model and "RTX" in user_profile.gpu_model:
        # User has RTX GPU, include Isaac Sim content
        pass
    else:
        # User doesn't have RTX GPU, exclude Isaac Sim content
        query = query.filter(~models.ContentMetadata.tags.any("isaac-sim"))

    recommended_content = query.limit(5).all()

    return {
        "recommended_content": [item.chapter_id for item in recommended_content],
        "user_profile_summary": {
            "experience_level": user_profile.ros_experience_level,
            "gpu_model": user_profile.gpu_model,
            "current_module": user_profile.current_module
        },
        "reasoning": "Content recommendations are based on your experience level and hardware specifications."
    }