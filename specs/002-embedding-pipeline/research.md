# Research Findings: Embedding Pipeline Setup

## UV Package Manager

**Decision**: Use `uv` for Python project management, dependency installation, and virtual environment creation.

**Rationale**: `uv` is a modern, fast, and all-in-one tool that streamlines Python development workflows. It provides a single command-line interface for tasks typically handled by `pip`, `pip-tools`, and `virtualenv`.

**Alternatives Considered**:
- `pip`/`virtualenv`: More traditional, but `uv` offers superior performance and integrated features.
- `Poetry`/`Rye`: More comprehensive project management tools, but `uv` is simpler and sufficient for a single-script project.

## Cohere Python Client for Embeddings

**Decision**: Use `cohere` Python SDK for generating vector embeddings.

**Rationale**: The `cohere` SDK provides a straightforward interface to access Cohere's powerful embedding models. It supports specifying input types and embedding types, which is crucial for optimizing embedding quality for retrieval tasks.

**Alternatives Considered**:
- Other embedding providers (e.g., OpenAI, Sentence Transformers): Cohere was explicitly requested in the specification.

## Qdrant Python Client for Vector Database

**Decision**: Use `qdrant-client` Python SDK to interact with Qdrant Cloud.

**Rationale**: The `qdrant-client` SDK provides all necessary functionalities to create collections, manage vector parameters, and upsert vectors with associated payloads (metadata). It also supports efficient batch operations.

**Alternatives Considered**:
- Other vector databases (e.g., Pinecone, Weaviate): Qdrant Cloud Free Tier was explicitly requested in the specification.

## Web Crawling and URL Extraction

**Decision**: Use `Invoke-WebRequest` (PowerShell) to retrieve `sitemap.xml` for initial URL discovery, and then `requests` and `BeautifulSoup4` (Python) for fetching individual pages and extracting text.

**Rationale**:
- `sitemap.xml` provides a structured list of all relevant URLs, which is more efficient than general crawling. `Invoke-WebRequest` proved effective for this.
- `requests` is a standard and robust Python library for making HTTP requests.
- `BeautifulSoup4` is a widely used and flexible library for parsing HTML and extracting specific elements, which is necessary for clean text extraction and configurable CSS selectors.

**Alternatives Considered**:
- Recursive crawling with `requests` and `BeautifulSoup4`: Less efficient for initial URL discovery if a sitemap is available.
- Specialized crawling frameworks (e.g., Scrapy): Overkill for this project's scope, which focuses on a single website and specific content extraction.

## Best Practices for Web Scraping/Text Extraction from HTML

**Decision**: Implement text extraction using `BeautifulSoup4` with configurable CSS selectors for inclusion/exclusion, focusing on identifying main content blocks and discarding boilerplate elements (headers, footers, navigation).

**Rationale**: This approach directly addresses the `FR-009` requirement for configurable text extraction and `SC-002` for clean article text. `BeautifulSoup4` provides the necessary tools to navigate and manipulate the HTML DOM effectively.

**Considerations**:
- Robust error handling for malformed HTML or missing elements.
- Iterative refinement of CSS selectors based on observed page structures.
- Prioritize content within `<article>`, `<main>`, or custom content divs.
- Remove script tags, style tags, and other non-content elements.
