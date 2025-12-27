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

# ✅ CORS FIX (IMPORTANT)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# --------------------------------------------------
# ASK ENDPOINT
# --------------------------------------------------

@app.post("/ask", response_model=AskResponse)
async def ask_question(request: AskRequest):
    start_time = time.time()

    try:
        logger.info(f"Question: {request.question}")

        retrieved_chunks = retrieve_book_content(request.question)
        
        if not retrieved_chunks or all(len(c.text.strip()) < 20 for c in retrieved_chunks):
            return AskResponse(
                answer="I'm sorry, but this topic doesn't seem to be covered in the book. Please feel free to ask me anything about ROS 2, robotics systems, or specific chapters.",
                retrieved_chunks=[]
            )

        # 🔹 Build context
        context = "\n\n".join(
            f"[Source: {c.metadata.get('page_title', 'Book')}]\n{c.text}"
            for c in retrieved_chunks[:3]
        )

        # 🔹 PREAMBLE
        preamble = """
You are a helpful AI assistant for the book "AI & Humanoid Robotics". Your purpose is to provide accurate and relevant information based *only* on the content of the book.

When answering, you must follow these rules:
1.  **Cite Sources:** Always cite the source of your information (e.g., "[Source: Chapter 1]").
2.  **Stick to the Book:** Do not use any external knowledge. If the book doesn't cover a topic, say so.
3.  **Be Concise:** Keep your answers clear and to the point.
4.  **Be Conversational:** Frame your answers in a friendly, helpful way.
"""

        # 🔹 STRONG RAG PROMPT
        prompt = f"""
You are an AI assistant for the book "AI & Humanoid Robotics".

Instructions:
- Use ONLY the information from BOOK CONTENT.
- If the question is partially related, answer ONLY what the book explains.
- If the topic is not covered, clearly say the book does not explain it.

BOOK CONTENT:
{context}

QUESTION:
{request.question}

ANSWER (based only on book):
"""

        try:
            response = co.chat(message=prompt, preamble=preamble)
            if len(response.text) > 0:
                answer = response.text
            else:
                logger.error("Response from Cohere API was empty.")
                answer = "Sorry, I encountered an issue processing your request."
        except IndexError as e:
            logger.error(f"An error occurred: {e}")
            answer = "Sorry, I encountered an issue processing your request."
        except Exception as e:
            logger.error(f"An unexpected error occurred: {e}")
            answer = "Sorry, I encountered an issue processing your request."

        logger.info("Answer generated")

        return AskResponse(
            answer=answer,
            retrieved_chunks=retrieved_chunks
        )

    except Exception as e:
        logger.error(e, exc_info=True)
        raise HTTPException(status_code=500, detail="Internal server error")

    finally:
        logger.info(f"Processed in {time.time() - start_time:.2f}s")
