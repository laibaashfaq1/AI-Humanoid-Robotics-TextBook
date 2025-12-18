# Data Model: Embedding Pipeline

This document defines the key entities and their attributes for the embedding pipeline, based on the `spec.md` and clarified requirements for metadata.

## Entities

### 1. BookContent

Represents the information extracted from a single book URL.

**Attributes**:
- `url` (string): The canonical URL of the book content. This will serve as a unique identifier for deduplication.
- `html_content` (string): The raw HTML content fetched from the `url`.
- `cleaned_text` (string): The main textual content extracted from `html_content` after cleaning and boilerplate removal.

### 2. TextChunk

Represents a segment of `cleaned_text` suitable for embedding. A single `BookContent` can be broken down into multiple `TextChunk` entities.

**Attributes**:
- `id` (string): A unique identifier for the chunk, potentially a combination of `book_content_url` and `chunk_id`.
- `book_content_url` (string): Foreign key linking to the `BookContent` from which this chunk was derived.
- `chunk_id` (integer): A sequential identifier for the chunk within its `BookContent`.
- `text` (string): The actual text content of the chunk.

### 3. VectorEmbedding

Represents the numerical vector generated from a `TextChunk` using Cohere's embedding model. This entity will be stored in Qdrant.

**Attributes**:
- `id` (string): The same ID as the `TextChunk` it represents. This will be the `PointId` in Qdrant.
- `vector` (list of floats): The numerical embedding vector.
- `source_url` (string): The original URL of the `BookContent`. (From FR-008 metadata)
- `chunk_id` (integer): The sequential ID of the `TextChunk`. (From FR-008 metadata)
- `book_title` (string): The title of the book, extracted from the `BookContent`. (From FR-008 metadata)
- `timestamp` (datetime): The timestamp when the embedding was generated or upserted. (From FR-008 metadata)

## Relationships

- One `BookContent` can have many `TextChunk`s.
- One `TextChunk` generates one `VectorEmbedding`.
