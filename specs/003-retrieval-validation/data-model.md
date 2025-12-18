# Data Model: Retrieval Pipeline Validation

This document defines the key entities and their attributes relevant to the retrieval pipeline validation.

## Entities

### 1. ValidationQuery

Represents a text string used to perform semantic searches against the Qdrant collection.

**Attributes**:
- `query_text` (string): The natural language query string.
- `expected_results` (List of ExpectedResult): A list of results expected to be returned by the semantic search for this query, used for validation.

### 2. ExpectedResult

Represents a known relevant document or chunk that should be retrieved by a specific `ValidationQuery`.

**Attributes**:
- `book_title` (string): The title of the book the relevant chunk belongs to.
- `source_url` (string): The URL of the book content the relevant chunk belongs to.
- `min_relevance_score` (float, optional): The minimum expected cosine similarity score for this result to be considered relevant.

### 3. RetrievedVector

Represents a vector (or "point" in Qdrant terminology) and its associated payload (metadata) returned from a Qdrant search or retrieval operation.

**Attributes**:
- `id` (string): The unique identifier of the vector in Qdrant.
- `vector` (list of floats): The numerical embedding vector.
- `payload` (Dictionary): A dictionary containing the metadata associated with the vector.
    - `source_url` (string): The original URL of the book content.
    - `chunk_id` (integer): The sequential ID of the text chunk.
    - `book_title` (string): The title of the book.
    - `timestamp` (datetime): The timestamp when the embedding was generated or upserted.
- `score` (float, optional): The similarity score (e.g., cosine similarity) of this vector to a query vector, if retrieved via search.

## Relationships

- One `ValidationQuery` can have many `ExpectedResult`s.
- `RetrievedVector`s are the actual data returned by Qdrant, which are compared against `ExpectedResult`s for validation.
