# RAG Agent FastAPI Backend

This project implements a Retrieval-Augmented Generation (RAG) agent using FastAPI, OpenAI Agents SDK, Qdrant for vector search, and Cohere for embeddings. The agent answers questions based on provided book content.

## Setup Instructions

1.  **Clone the repository**:
    ```bash
    git clone <repository_url>
    cd <repository_name>/backend
    ```

2.  **Create a virtual environment and install dependencies**:
    Make sure you have `uv` installed (`pip install uv`).
    ```bash
    uv venv
    uv pip install -r requirements.txt
    ```

3.  **Configure environment variables**:
    Create a `.env` file in the `backend/` directory with the following variables:
    ```
    OPENAI_API_KEY="your_openai_api_key_here"
    QDRANT_URL="your_qdrant_url_here"
    QDRANT_API_KEY="your_qdrant_api_key_here"
    COHERE_API_KEY="your_cohere_api_key_here"
    QDRANT_COLLECTION_NAME="rag_collection" # Or your specific collection name
    ```
    Replace the placeholder values with your actual API keys and Qdrant details.

## Usage

1.  **Run the FastAPI application**:
    Ensure your virtual environment is activated.
    ```bash
    uvicorn main:app --reload
    ```
    The API will be available at `http://127.0.0.1:8000`.

2.  **Access API Documentation**:
    Open your browser and navigate to `http://127.0.0.1:8000/docs` for interactive API documentation (Swagger UI).

3.  **Test the `/ask` endpoint**:
    You can use the Swagger UI to test the `/ask` endpoint, or use a tool like `curl` or `httpx`.

    Example using `httpx` (ensure `httpx` is installed: `uv pip install httpx`):
    ```python
    import httpx
    import asyncio

    async def main():
        async with httpx.AsyncClient() as client:
            response = await client.post(
                "http://127.0.0.1:8000/ask", 
                json={"question": "What is the main theme of the book?"}
            )
            print(response.json())

    asyncio.run(main())
    ```

4.  **Run local tests**:
    Ensure the FastAPI app is running (step 1).
    ```bash
    python local_test.py
    ```
    This script (`backend/local_test.py`) will send predefined questions to the running API and validate responses.

## Project Structure

```
backend/
├── main.py                 # FastAPI application entry point, /ask endpoint
├── schemas.py              # Pydantic models for request/response
├── book_agent.py           # OpenAI Agent definition
├── qdrant_retrieval.py     # Qdrant and Cohere client, content retrieval logic
├── requirements.txt        # Python dependencies
├── .env                    # Environment variables (API keys, URLs)
├── local_test.py           # Local testing script for the API
└── README.md               # This file
```