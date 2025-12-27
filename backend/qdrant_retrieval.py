import os
import logging
from typing import List
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
from schemas import RetrievedDocumentChunk

load_dotenv()

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COHERE_API_KEY = os.getenv("COHERE_API_KEY")

COLLECTION_NAME = "rag_embedding"
EMBED_MODEL = "embed-english-light-v3.0"

qdrant = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY,
)

co = cohere.Client(COHERE_API_KEY)


def retrieve_book_content(query: str, limit: int = 5) -> List[RetrievedDocumentChunk]:
    if not query.strip():
        return []

    try:
        # 🔹 Embed query (SAME MODEL AS INGEST)
        query_embedding = co.embed(
            texts=[query],
            model=EMBED_MODEL,
            input_type="search_query"
        ).embeddings[0]

        # 🔹 Vector search
        results = qdrant.search_points(
            collection_name=COLLECTION_NAME,
            vector=query_embedding,
            limit=limit,
            with_payload=True
        )

        chunks = []
        for r in results:
            logger.info(f"Retrieved chunk score: {r.score}")

            chunks.append(
                RetrievedDocumentChunk(
                    id=str(r.id),
                    text=r.payload.get("text", ""),
                    metadata=r.payload,
                    score=r.score
                )
            )

        logger.info(f"Total retrieved chunks: {len(chunks)}")
        return chunks

    except Exception as e:
        logger.error(f"Retrieval failed: {e}", exc_info=True)
        return []
