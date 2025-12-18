from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from typing import List
import logging

from ..database import get_db
from .. import models, schemas, auth, rag

router = APIRouter()

@router.post("/chat", response_model=schemas.ChatResponse)
async def chat_endpoint(chat_request: schemas.ChatRequest, db: Session = Depends(get_db)):
    """
    Chat endpoint that uses RAG to provide context-aware responses from the textbook.
    """
    try:
        # Retrieve relevant content from Qdrant based on the query
        filters = {}
        if chat_request.chapter_context:
            filters["chapter_id"] = chat_request.chapter_context
        if chat_request.user_id:
            # Get user profile to potentially filter content based on experience level
            user_profile = db.query(models.UserProfile).filter(
                models.UserProfile.user_id == chat_request.user_id
            ).first()
            if user_profile and user_profile.ros_experience_level:
                # We could filter content difficulty based on user experience
                pass

        relevant_content = rag.retrieve_relevant_content(
            query=chat_request.query,
            top_k=5,
            filters=filters
        )

        # Generate response using OpenAI with the retrieved context
        response_text = rag.generate_rag_response(
            query=chat_request.query,
            context=relevant_content,
            user_id=chat_request.user_id
        )

        # Prepare sources list from the retrieved content
        sources = list(set([item["chapter_id"] for item in relevant_content if item["chapter_id"]]))

        # Prepare the response
        response = schemas.ChatResponse(
            response=response_text,
            sources=sources,
            retrieved_context=relevant_content
        )

        return response

    except Exception as e:
        logging.error(f"Error in chat endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error during chat processing")

@router.post("/ingest-content")
async def ingest_content(content_path: str, chapter_id: str, week_number: int,
                        module_name: str, difficulty: str, tags: List[str],
                        force_reindex: bool = False):
    """
    Ingest content from MDX files into the Qdrant vector database.
    """
    try:
        # Process the MDX content file and store in Qdrant
        rag.ingest_mdx_content(
            file_path=content_path,
            chapter_id=chapter_id,
            week_number=week_number,
            module_name=module_name,
            difficulty=difficulty,
            tags=tags
        )

        return {"message": f"Successfully ingested content for chapter {chapter_id}"}

    except Exception as e:
        logging.error(f"Error in content ingestion: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error during content ingestion")