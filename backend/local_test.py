import httpx
import asyncio
from schemas import AskRequest, AskResponse, RetrievedDocumentChunk

# Assuming FastAPI app is running locally on port 8000
BASE_URL = "http://127.0.0.1:8000"

async def run_test_case(test_name: str, question: str):
    print(f"\n--- Running Test Case: {test_name} ---")
    try:
        async with httpx.AsyncClient() as client:
            response = await client.post(f"{BASE_URL}/ask", json={"question": question})
            response.raise_for_status()  # Raise an exception for HTTP errors (4xx or 5xx) 
            
            ask_response = AskResponse.model_validate(response.json())
            
            print(f"Question: {question}")
            print(f"Answer: {ask_response.answer}")
            print(f"Retrieved Chunks Count: {len(ask_response.retrieved_chunks) if ask_response.retrieved_chunks else 0}")
            
            # Basic validation
            assert ask_response.answer is not None and ask_response.answer != "", "Answer should not be empty"
            if "not answer" in ask_response.answer.lower():
                print("Validation: Agent stated inability to answer (expected for some cases).")
            else:
                assert ask_response.retrieved_chunks is not None and len(ask_response.retrieved_chunks) > 0, \
                    "Retrieved chunks should not be empty for an answered question."
            
            print(f"Test Case '{test_name}' PASSED.")

    except httpx.ConnectError as e:
        print(f"Test Case '{test_name}' FAILED: Could not connect to {BASE_URL}. Is the FastAPI app running? Error: {e}")
    except httpx.HTTPStatusError as e:
        print(f"Test Case '{test_name}' FAILED: HTTP Error {e.response.status_code} - {e.response.text}")
    except Exception as e:
        print(f"Test Case '{test_name}' FAILED: An unexpected error occurred: {e}")

async def main():
    # Example test cases
    test_cases = [
    {"name": "General Book Question", "question": "What is the main topic of the book?"},
    {"name": "Specific Content Query", "question": "Can you tell me about the robotics arm mentioned in the book?"},
    {"name": "Out of Context Question", "question": "What is the capital of France?"},
    {"name": "No Content Expected", "question": "Tell me about advanced quantum physics (assuming not in book)."},
    ]

    for case in test_cases:
        await run_test_case(case["name"], case["question"])

if __name__ == "__main__":
    asyncio.run(main())
