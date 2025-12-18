import os
import logging
import cohere
from typing import List
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from schemas import RetrievedDocumentChunk

load_dotenv()
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_COLLECTION_NAME = "rag_embedding"
COHERE_API_KEY = os.getenv("COHERE_API_KEY")

qdrant = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY
)

co = cohere.Client(COHERE_API_KEY)

def retrieve_book_content(query: str, limit: int = 5) -> List[RetrievedDocumentChunk]:
    if not query:
        return []

    try:
        # 1️⃣ Embed query
        embedding = co.embed(
            texts=[query],
            model="embed-english-light-v3.0",
            input_type="search_query"
        ).embeddings[0]

        # 2️⃣ Search Qdrant (NEW API)
        results = qdrant.search_points(
            collection_name=QDRANT_COLLECTION_NAME,
            vector=embedding,
            limit=limit,
            with_payload=True
        )

        chunks = []
        for r in results:
            chunks.append(
                RetrievedDocumentChunk(
                    id=str(r.id),
                    text=r.payload.get("text", ""),
                    metadata=r.payload,
                    score=r.score
                )
            )

        logger.info(f"Retrieved {len(chunks)} chunks from Qdrant")
        return chunks

    except Exception as e:
        logger.error(f"Qdrant retrieval error: {e}", exc_info=True)
        return []
