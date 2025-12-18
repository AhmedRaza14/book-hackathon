from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any
import openai
import os
from sentence_transformers import SentenceTransformer
import asyncio

# Initialize Qdrant client
QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

if QDRANT_API_KEY:
    qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
else:
    qdrant_client = QdrantClient(url=QDRANT_URL)

# Initialize OpenAI client
openai.api_key = os.getenv("OPENAI_API_KEY")

# Initialize sentence transformer for embeddings
embedding_model = SentenceTransformer('all-MiniLM-L6-v2')

# Collection names
CONTENT_COLLECTION = "textbook_content"
INTERACTION_COLLECTION = "user_interactions"

def initialize_collections():
    """Initialize Qdrant collections for the textbook content and user interactions."""

    # Create content vectors collection
    try:
        qdrant_client.get_collection(CONTENT_COLLECTION)
    except:
        qdrant_client.create_collection(
            collection_name=CONTENT_COLLECTION,
            vectors_config=models.VectorParams(size=384, distance=models.Distance.COSINE),  # Using sentence transformer embedding size
        )

        # Create payload index for faster filtering
        qdrant_client.create_payload_index(
            collection_name=CONTENT_COLLECTION,
            field_name="chapter_id",
            field_schema=models.PayloadSchemaType.KEYWORD
        )
        qdrant_client.create_payload_index(
            collection_name=CONTENT_COLLECTION,
            field_name="week_number",
            field_schema=models.PayloadSchemaType.INTEGER
        )
        qdrant_client.create_payload_index(
            collection_name=CONTENT_COLLECTION,
            field_name="difficulty",
            field_schema=models.PayloadSchemaType.KEYWORD
        )

    # Create user interaction collection
    try:
        qdrant_client.get_collection(INTERACTION_COLLECTION)
    except:
        qdrant_client.create_collection(
            collection_name=INTERACTION_COLLECTION,
            vectors_config=models.VectorParams(size=384, distance=models.Distance.COSINE),
        )

def generate_embeddings(texts: List[str]) -> List[List[float]]:
    """Generate embeddings for the given texts using sentence transformer."""
    embeddings = embedding_model.encode(texts)
    return embeddings.tolist()

def store_content_in_qdrant(
    chapter_id: str,
    content_chunks: List[Dict[str, Any]],
    week_number: int,
    module_name: str,
    difficulty: str,
    tags: List[str]
):
    """Store content chunks in Qdrant vector database."""

    points = []
    for i, chunk in enumerate(content_chunks):
        # Generate embedding for the content
        embedding = generate_embeddings([chunk["text"]])[0]

        point = models.PointStruct(
            id=len(points),  # This should be a unique ID in practice
            vector=embedding,
            payload={
                "chapter_id": chapter_id,
                "week_number": week_number,
                "module_name": module_name,
                "difficulty": difficulty,
                "content_type": chunk.get("content_type", "text"),
                "text_content": chunk["text"],
                "section_title": chunk.get("section_title", ""),
                "chunk_index": i,
                "total_chunks": len(content_chunks),
                "tags": tags
            }
        )
        points.append(point)

    # Generate unique IDs for the points
    import uuid
    for point in points:
        point.id = str(uuid.uuid4())

    # Upload points to Qdrant
    qdrant_client.upsert(
        collection_name=CONTENT_COLLECTION,
        points=points
    )

def retrieve_relevant_content(query: str, top_k: int = 5, filters: Dict = None) -> List[Dict[str, Any]]:
    """Retrieve relevant content from Qdrant based on the query."""

    # Generate embedding for the query
    query_embedding = generate_embeddings([query])[0]

    # Prepare filters if provided
    qdrant_filters = None
    if filters:
        filter_conditions = []
        for key, value in filters.items():
            if isinstance(value, list):
                # Handle array fields like tags
                filter_conditions.append(
                    models.FieldCondition(
                        key=key,
                        match=models.MatchAny(any=value)
                    )
                )
            else:
                filter_conditions.append(
                    models.FieldCondition(
                        key=key,
                        match=models.MatchValue(value=value)
                    )
                )

        if filter_conditions:
            qdrant_filters = models.Filter(
                must=filter_conditions
            )

    # Search in Qdrant
    search_results = qdrant_client.search(
        collection_name=CONTENT_COLLECTION,
        query_vector=query_embedding,
        query_filter=qdrant_filters,
        limit=top_k,
        with_payload=True
    )

    # Format results
    results = []
    for result in search_results:
        results.append({
            "chapter_id": result.payload.get("chapter_id"),
            "week_number": result.payload.get("week_number"),
            "module_name": result.payload.get("module_name"),
            "difficulty": result.payload.get("difficulty"),
            "content_type": result.payload.get("content_type"),
            "text_content": result.payload.get("text_content"),
            "section_title": result.payload.get("section_title"),
            "relevance_score": result.score,
            "tags": result.payload.get("tags", [])
        })

    return results

def generate_rag_response(query: str, context: List[Dict[str, Any]], user_id: str = None) -> str:
    """Generate a response using OpenAI based on the retrieved context."""

    # Format the context for the LLM
    context_text = "\n\n".join([
        f"Chapter: {item['chapter_id']}\nSection: {item['section_title']}\nContent: {item['text_content']}"
        for item in context
    ])

    # Prepare the prompt for the LLM
    prompt = f"""
    You are an AI assistant for the Physical AI & Humanoid Robotics textbook.
    Answer the user's question based on the provided context from the textbook.

    Context:
    {context_text}

    Question: {query}

    Please provide a helpful, accurate response based on the textbook content.
    If the context doesn't contain the information needed to answer the question,
    please say so and suggest where the user might find the information in the textbook.
    """

    # Call OpenAI API
    response = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",  # You can change this to gpt-4 if preferred
        messages=[
            {"role": "system", "content": "You are an AI assistant for the Physical AI & Humanoid Robotics textbook. Provide helpful, accurate responses based on the textbook content."},
            {"role": "user", "content": prompt}
        ],
        max_tokens=1000,
        temperature=0.7
    )

    # Store the interaction in Qdrant for future reference
    if user_id:
        interaction_embedding = generate_embeddings([query])[0]
        interaction_point = models.PointStruct(
            id=str(len(qdrant_client.scroll(INTERACTION_COLLECTION, limit=1)[0]) + 1),
            vector=interaction_embedding,
            payload={
                "user_id": user_id,
                "query_text": query,
                "response_text": response.choices[0].message.content,
                "timestamp": int(asyncio.get_event_loop().time()),
                "chapter_context": context[0].get("chapter_id") if context else None
            }
        )
        qdrant_client.upsert(
            collection_name=INTERACTION_COLLECTION,
            points=[interaction_point]
        )

    return response.choices[0].message.content

def ingest_mdx_content(file_path: str, chapter_id: str, week_number: int, module_name: str, difficulty: str, tags: List[str]):
    """Process MDX content file and store it in Qdrant."""

    import re

    # Read the MDX file
    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()

    # Extract content sections (excluding frontmatter)
    # Find the end of frontmatter
    frontmatter_end = content.find('---\n', content.find('---\n') + 1) + 4
    main_content = content[frontmatter_end:] if frontmatter_end > 3 else content

    # Split content into chunks based on headings
    chunks = []

    # Split by headings (##, ###, etc.)
    sections = re.split(r'\n(?=##\s)', main_content)

    for i, section in enumerate(sections):
        if section.strip():
            # Extract section title if it starts with a heading
            lines = section.split('\n')
            title = ''
            if lines and lines[0].startswith('## '):
                title = lines[0][3:].strip()  # Remove '## ' prefix
                section = '\n'.join(lines[1:]).strip()

            # If the section is too long, further split it
            if len(section) > 1000:  # If more than 1000 characters
                sub_chunks = [section[i:i+1000] for i in range(0, len(section), 1000)]
                for j, sub_chunk in enumerate(sub_chunks):
                    chunks.append({
                        "text": sub_chunk,
                        "content_type": "text",
                        "section_title": f"{title} (Part {j+1})" if title else f"Part {j+1}"
                    })
            else:
                chunks.append({
                    "text": section,
                    "content_type": "text",
                    "section_title": title
                })

    # Store the chunks in Qdrant
    store_content_in_qdrant(chapter_id, chunks, week_number, module_name, difficulty, tags)