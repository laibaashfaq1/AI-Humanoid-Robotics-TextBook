# Feature Specification: Embedding Pipeline Setup

**Feature Branch**: `002-embedding-pipeline`  
**Created**: 2025-12-11  
**Status**: Draft  
**Input**: User description: "Embedding Pipeline Setup Goal: Extract text from all published book URLs, generate embeddings using Cohere, and store them in Qdrant Cloud for retrieval. Target audience: Developers building the backend retrieval layer. Focus: - URL crawling and text extraction - Text cleaning and chunking - Cohere embedding generation - Qdrant vector storage with metadata Success criteria: - All book URLs crawled successfully - Clean article text extracted (no nav/footer) - Embeddings generated with Cohere - Vectors stored in Qdrant with correct dimensions - Test query returns relevant results Constraints: - Only use Cohere for embeddings - Qdrant Cloud Free Tier as vector DB - Avoid duplicates on reruns Not building: - Chatbot, agent logic, or frontend - UI/UX work - LLM answer generation - Do NOT initialize or create the repository; only perform the tasks described above"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Process a Single Book URL (Priority: P1)

As a developer, I want to provide a single book URL to the pipeline so that its content is extracted, embedded, and stored in the vector database, making it available for retrieval.

**Why this priority**: This is the core functionality of the pipeline and provides the fundamental building block for processing all book URLs. It allows for immediate testing and validation of the entire end-to-end process.

**Independent Test**: The pipeline can be run with a single URL. A subsequent test query against the vector database using a keyword from the book's text should return relevant chunks from that book, confirming successful processing.

**Acceptance Scenarios**:

1.  **Given** a valid book URL that has not been processed before, **When** the pipeline is executed, **Then** the book's text content is extracted, chunked, converted to embeddings, and stored as vectors in the Qdrant database.
2.  **Given** the pipeline has already processed a specific URL, **When** the pipeline is run again with the same URL, **Then** the system identifies the duplication and skips processing to avoid redundant data.

---

### User Story 2 - Process Multiple Book URLs (Priority: P2)

As a developer, I want to provide a list of book URLs to the pipeline so that all of them are processed in a batch, enabling efficient embedding of the entire library.

**Why this priority**: This builds on the core functionality to enable bulk processing, which is essential for the initial setup and for adding new collections of books.

**Independent Test**: The pipeline can be run with a list of several new URLs. After execution, test queries for content from each of the books should return relevant results, confirming all URLs were processed successfully.

**Acceptance Scenarios**:

1.  **Given** a list of valid book URLs that have not been processed, **When** the pipeline is executed, **Then** all URLs are crawled, and their content is successfully embedded and stored in the Qdrant database.
2.  **Given** a list containing a mix of new and previously processed URLs, **When** the pipeline is executed, **Then** only the new URLs are processed, and the existing ones are skipped.

---

### Edge Cases

-   What happens when a URL is invalid, returns a 404 error, or is otherwise inaccessible?
-   How does the system handle different content formats or unexpected HTML structures on the book pages?
-   What is the behavior if the Cohere API or Qdrant Cloud is temporarily unavailable?
-   How are extremely long documents handled during the chunking process?

## Requirements *(mandatory)*

### Functional Requirements

-   **FR-001**: The system MUST be able to crawl a list of given book URLs.
-   **FR-002**: The system MUST extract the main text content from each URL, making a best-effort attempt to exclude common boilerplate like navigation bars and footers.
-   **FR-003**: The system MUST clean the extracted text to remove irrelevant artifacts (e.g., excessive whitespace, script tags).
-   **FR-004**: The system MUST divide the cleaned text into smaller, semantically coherent chunks suitable for embedding.
-   **FR-005**: The system MUST generate vector embeddings for each text chunk using the Cohere API.
-   **FR-006**: The system MUST store the generated vectors in a Qdrant Cloud database.
-   **FR-007**: The system MUST be idempotent, preventing the duplicate processing and storage of content from the same URL on subsequent runs.
-   **FR-008**: The system MUST store metadata ('source_url', 'chunk_id', 'book_title', 'timestamp') alongside each vector.
-   **FR-009**: The text extraction mechanism MUST be configurable by specifying a list of CSS selectors to include and/or exclude.

### Key Entities

-   **Book Content**: Represents the raw and processed text from a single book URL. Attributes include the original URL, raw HTML, cleaned text, and text chunks.
-   **Vector Embedding**: Represents the numerical vector generated from a text chunk. Attributes include the vector itself and its dimensionality.
-   **Vector Metadata**: Represents the data stored alongside a vector to provide context. Attributes include `source_url`, `chunk_id`, `book_title`, and `timestamp`.

## Success Criteria *(mandatory)*

### Measurable Outcomes

-   **SC-001**: 100% of provided valid book URLs are successfully crawled and their content ingested by the pipeline.
-   **SC-002**: For a sample of processed documents, at least 95% of the stored text chunks consist of meaningful book content, not navigational or footer text.
-   **SC-003**: All vectors stored in the Qdrant database have the correct dimensions as specified by the Cohere embedding model used.
-   **SC-004**: A test query with a specific keyword from a source document returns the relevant text chunk(s) within the top 5 results at least 90% of the time.