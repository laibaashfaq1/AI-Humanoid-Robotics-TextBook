import time
import logging
import os

import cohere
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv

from schemas import AskRequest, AskResponse
from qdrant_retrieval import retrieve_book_content

# --------------------------------------------------
# ENV & LOGGING
# --------------------------------------------------

load_dotenv()

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# --------------------------------------------------
# COHERE CLIENT
# --------------------------------------------------

COHERE_API_KEY = os.getenv("COHERE_API_KEY")
if not COHERE_API_KEY:
    raise RuntimeError("COHERE_API_KEY not set")

co = cohere.Client(COHERE_API_KEY)

# --------------------------------------------------
# FASTAPI APP
# --------------------------------------------------

app = FastAPI(title="Book RAG Chatbot API")

# ✅ FIX: ADD CORS MIDDLEWARE (VERY IMPORTANT)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],      # allow frontend
    allow_credentials=True,
    allow_methods=["*"],      # POST, OPTIONS, etc
    allow_headers=["*"],
)

# --------------------------------------------------
# ASK ENDPOINT
# --------------------------------------------------

@app.post("/ask", response_model=AskResponse)
async def ask_question(request: AskRequest):
    start_time = time.time()

    try:
        logger.info(f"Received question: {request.question}")

        # 1️⃣ Retrieve relevant book chunks
        retrieved_chunks = retrieve_book_content(request.question)

        if not retrieved_chunks:
            return AskResponse(
                answer="I cannot answer from the book.",
                retrieved_chunks=[]
            )

        # 2️⃣ Build context (top 3 chunks only)
        context = "\n\n".join(chunk.text for chunk in retrieved_chunks[:3])

        # 3️⃣ Strict RAG prompt
        prompt = f"""
You are a book assistant.

Rules:
- Answer ONLY using the provided book content.
- Do NOT use external knowledge.
- If the answer is not in the book, say exactly:
  "I cannot answer from the book."

BOOK CONTENT:
{context}

QUESTION:
{request.question}
"""

        # 4️⃣ Cohere Chat
        response = co.chat(
            model="command-r",
            message=prompt,
            temperature=0.1
        )

        # ✅ Safe response extraction
        answer = response.message.content[0].text.strip()

        logger.info("Answer generated successfully")

        return AskResponse(
            answer=answer,
            retrieved_chunks=retrieved_chunks
        )

    except Exception as e:
        logger.error(f"Error answering question: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail="Internal server error")

    finally:
        logger.info(f"Processed in {time.time() - start_time:.2f}s")
