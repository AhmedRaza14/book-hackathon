from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import HTTPBearer
from sqlalchemy.orm import Session
from typing import Optional
from datetime import timedelta
import os

from ..database import get_db
from .. import models, schemas, auth

router = APIRouter()

@router.post("/register", response_model=schemas.Token)
def register(user: schemas.UserCreate, db: Session = Depends(get_db)):
    # Create the user
    db_user = auth.create_user(db, user)

    # Create user profile if provided
    if user.profile:
        user_profile = models.UserProfile(
            user_id=db_user.id,
            **user.profile
        )
        db.add(user_profile)
        db.commit()

    # Create access token
    access_token_expires = timedelta(minutes=auth.ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = auth.create_access_token(
        data={"sub": db_user.id}, expires_delta=access_token_expires
    )

    return {"access_token": access_token, "token_type": "bearer"}

@router.post("/login", response_model=schemas.Token)
def login(user_credentials: schemas.UserLogin, db: Session = Depends(get_db)):
    user = auth.authenticate_user(db, user_credentials.email, user_credentials.password)
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password",
            headers={"WWW-Authenticate": "Bearer"},
        )

    access_token_expires = timedelta(minutes=auth.ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = auth.create_access_token(
        data={"sub": user.id}, expires_delta=access_token_expires
    )

    return {"access_token": access_token, "token_type": "bearer"}

@router.get("/profile", response_model=schemas.UserProfileResponse)
def get_profile(
    current_user: models.User = Depends(auth.get_current_user),
    db: Session = Depends(get_db)
):
    profile = db.query(models.UserProfile).filter(
        models.UserProfile.user_id == current_user.id
    ).first()

    if not profile:
        # Create a default profile if it doesn't exist
        profile = models.UserProfile(user_id=current_user.id)
        db.add(profile)
        db.commit()
        db.refresh(profile)

    return profile

@router.put("/profile", response_model=schemas.UserProfileResponse)
def update_profile(
    profile_update: schemas.UserProfileUpdate,
    current_user: models.User = Depends(auth.get_current_user),
    db: Session = Depends(get_db)
):
    profile = db.query(models.UserProfile).filter(
        models.UserProfile.user_id == current_user.id
    ).first()

    if not profile:
        # Create a new profile if it doesn't exist
        profile = models.UserProfile(user_id=current_user.id)
        db.add(profile)

    # Update profile fields
    for field, value in profile_update.dict(exclude_unset=True).items():
        setattr(profile, field, value)

    db.commit()
    db.refresh(profile)

    return profile