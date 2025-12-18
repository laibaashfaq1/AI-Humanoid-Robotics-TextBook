# Research Findings: RAG Agent Development with OpenAI Agents SDK + FastAPI

## OpenAI Agents SDK

**Decision**: The OpenAI Agents SDK will be used to create the RAG agent.

**Rationale**: The specification explicitly requires using the OpenAI Agents SDK. This SDK provides the necessary tools for creating agents, defining custom tools (such as for retrieval), and managing the agent's lifecycle.

**Key Learnings**:
- Agents are created with a specific model (e.g., GPT-4) and a set of tools.
- A "tool" is a function that the agent can call to perform actions, such as retrieving information.
- The agent's instructions (prompt) must clearly define its purpose, constraints, and how to use its tools.

## FastAPI for Agent Exposure

**Decision**: FastAPI will be used as the backend framework to expose the RAG agent via a RESTful API.

**Rationale**: FastAPI is a modern, high-performance web framework for Python that is well-suited for building APIs. Its asynchronous capabilities are beneficial for handling I/O-bound operations like API calls to OpenAI and Qdrant.

**Key Learnings**:
- Create a FastAPI application instance.
- Define API endpoints (e.g., `/ask`) using decorators (`@app.post("/ask")`).
- Use Pydantic models to define request and response schemas for data validation and serialization.
- Run the application with an ASGI server like Uvicorn.

## Qdrant Client for Retrieval Tool

**Decision**: The `qdrant-client` Python SDK will be used to implement the agent's retrieval tool.

**Rationale**: The specification requires using Qdrant for retrieval. The `qdrant-client` SDK allows for efficient semantic search within the existing Qdrant collection, which is essential for the agent's retrieval capabilities.

**Key Learnings**:
- The retrieval tool will be a Python function that takes a query string as input.
- Inside the function, embed the query using the same Cohere model used for the documents.
- Use `qdrant_client.search()` with the query vector to find the most relevant document chunks.
- The function should return the retrieved text content to the agent.

## Grounding Agent Answers

**Decision**: The agent's instructions and the structure of the retrieval tool will be designed to ensure all answers are grounded in the retrieved book content.

**Rationale**: Preventing hallucinations is a critical requirement. This can be achieved by:
- Explicitly instructing the agent in its system prompt to only use the information provided by its retrieval tool.
- Structuring the retrieval tool to return a clear "no content found" message if no relevant documents are retrieved.
- Prompting the agent to state that it cannot answer if its retrieval tool returns no information.

## Best Practices for RAG Agent Development

- **Clear Instructions**: The agent's system prompt is crucial. It should be detailed, specific, and unambiguous about the agent's role, capabilities, and limitations.
- **Tool Design**: The retrieval tool should be well-defined with a clear function signature and docstring so the agent understands how to use it.
- **Error Handling**: Implement robust error handling for API calls (OpenAI, Qdrant) and other potential failure points.
- **Logging**: Implement comprehensive logging to track the agent's reasoning process, tool calls, and retrieved content. This is invaluable for debugging and evaluation.
- **Testing**: Develop a suite of test cases with questions that can, and cannot, be answered from the book content to validate the agent's grounding and accuracy.
