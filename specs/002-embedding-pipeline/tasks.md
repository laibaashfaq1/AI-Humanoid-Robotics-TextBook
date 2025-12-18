# Tasks: Embedding Pipeline Setup

**Input**: Design documents from `/specs/002-embedding-pipeline/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: No specific request for tests was made in the feature specification, so test tasks will not be explicitly generated. However, it is assumed that appropriate testing will be performed as part of the implementation.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume single project - adjust based on plan.md structure

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create `backend` directory for the project.
- [X] T002 Initialize `uv` project within the `backend` directory.
- [X] T003 Create `main.py` in `backend/` for the core script.
- [X] T004 Create `.env` file in `backend/` for environment variables (e.g., COHERE_API_KEY, QDRANT_API_KEY, QDRANT_URL).
- [X] T005 Install primary dependencies (`cohere`, `qdrant-client`, `requests`, `beautifulsoup4`) using `uv` in `backend/requirements.txt`.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T006 [P] Implement `get_all_urls(base_url: str) -> List[str]` function in `backend/main.py` to fetch and parse `sitemap.xml` for content URLs.
- [X] T007 Initialize Qdrant client in `backend/main.py` using environment variables.
- [X] T008 Implement `create_collection(client: QdrantClient, collection_name: str, vector_size: int)` function in `backend/main.py` to create the `rag_embedding` collection if it doesn't exist.
- [X] T009 Define `vector_size` constant in `backend/main.py` based on Cohere embedding model (e.g., 1024 for `embed-v3.0` or `embed-v4.0`).

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Process a Single Book URL (Priority: P1) 🎯 MVP

**Goal**: Provide a single book URL to the pipeline so that its content is extracted, embedded, and stored in the vector database, making it available for retrieval.

**Independent Test**: Run the pipeline with a single known URL. Verify (e.g., via Qdrant UI/client) that the associated vectors and metadata are stored correctly and a simple search query returns relevant chunks.

### Implementation for User Story 1

- [X] T010 [US1] Implement `extract_text_from_url(url: str, selectors: Optional[Dict] = None) -> Tuple[str, str]` function in `backend/main.py`. This function should fetch HTML, use `BeautifulSoup4` with configurable CSS selectors to extract main content, and return clean text along with the extracted book title.
- [X] T011 [US1] Implement `chunk_text(text: str, chunk_size: int, chunk_overlap: int) -> List[str]` function in `backend/main.py` to divide cleaned text into smaller chunks.
- [X] T012 [US1] Implement `generate_embeddings(texts: List[str]) -> List[List[float]]` function in `backend/main.py` using the Cohere client to get vector embeddings.
- [X] T013 [US1] Implement `save_chunk_to_qdrant(client: QdrantClient, collection_name: str, url: str, book_title: str, chunk_id: int, text: str, embedding: List[float])` function in `backend/main.py` to upsert individual text chunks and their embeddings to Qdrant, including `source_url`, `chunk_id`, `book_title`, and `timestamp` metadata.
- [X] T014 [US1] Implement `process_single_url(url: str, client: QdrantClient, cohere_client: CohereClient, collection_name: str, selectors: Optional[Dict] = None)` orchestrator function in `backend/main.py` that ties together extraction, chunking, embedding, and saving.
- [X] T015 [US1] Add basic deduplication logic within `process_single_url` function to check if a URL has already been processed and skip if so (e.g., by querying Qdrant for existing `source_url`).
- [X] T016 [US1] Add error handling for network requests (e.g., 404s, connection issues) in `extract_text_from_url`.
- [X] T017 [US1] Add error handling for Cohere API failures and Qdrant client errors.

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Process Multiple Book URLs (Priority: P2)

**Goal**: Provide a list of book URLs to the pipeline so that all of them are processed in a batch, enabling efficient embedding of the entire library.

**Independent Test**: Run the pipeline with a list of several new URLs from the sitemap. Verify that all unique URLs are processed, and their content is embedded and stored correctly in Qdrant. Confirm that previously processed URLs are skipped.

### Implementation for User Story 2

- [X] T018 [US2] Implement `main()` orchestrator function in `backend/main.py` that retrieves all URLs (using `get_all_urls`), iterates through them, and calls `process_single_url` for each.
- [X] T019 [US2] Enhance deduplication logic to efficiently handle batch processing, ensuring that a URL, once processed, is not re-processed across runs.

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T020 Refactor `main.py` for better modularity and readability (e.g., separate utility functions into a `utils.py`).
- [X] T021 Add comprehensive logging to the pipeline (e.g., processing progress, errors, successful upserts).
- [X] T022 Implement a simple command-line interface (CLI) using `argparse` or similar for pipeline execution (e.g., `python main.py --url <single_url>` or `python main.py --all-sitemap`).
- [X] T023 Add basic performance metrics (e.g., time taken to process each URL, embedding generation rate).
- [X] T024 Review and enhance error handling for all potential failure points.
- [X] T025 Run local testing with dummy data and verify expected behavior.
- [X] T026 Prepare a `README.md` in the `backend/` directory with setup and usage instructions.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Depends on User Story 1 components (specifically `process_single_url`).

### Within Each User Story

- Core implementation functions before orchestrator functions.
- Specific function implementations before error handling or logging related to that function.

### Parallel Opportunities

- Within Phase 2, tasks T006, T007, T008, T009 can be considered for parallel work if different developers are assigned.
- Tasks T010, T011, T012, T013 within User Story 1 can be developed in parallel as they represent distinct, though sequentially used, components.
- Once Foundational phase completes, User Story 1 and parts of User Story 2 could technically be started in parallel, though sequential implementation of US1 is recommended due to its direct dependency for US2.
- Many tasks in the "Polish & Cross-Cutting Concerns" phase can be tackled in parallel.

---

## Parallel Example: Foundational Phase

```bash
# Different developers can work on these concurrently:
Task: "T006 [P] Implement get_all_urls(base_url: str) -> List[str] function in backend/main.py to fetch and parse sitemap.xml for content URLs."
Task: "T007 Initialize Qdrant client in backend/main.py using environment variables."
Task: "T008 Implement create_collection(client: QdrantClient, collection_name: str, vector_size: int) function in backend/main.py to create the rag_embedding collection if it doesn't exist."
```

## Parallel Example: User Story 1 - Core Implementation

```bash
# Different developers can work on these concurrently (after T006, T007, T008, T009 are done):
Task: "T010 [US1] Implement extract_text_from_url(url: str, selectors: Optional[Dict] = None) -> Tuple[str, str] function in backend/main.py."
Task: "T011 [US1] Implement chunk_text(text: str, chunk_size: int, chunk_overlap: int) -> List[str] function in backend/main.py."
Task: "T012 [US1] Implement generate_embeddings(texts: List[str]) -> List[List[float]] function in backend/main.py."
Task: "T013 [US1] Implement save_chunk_to_qdrant(client: QdrantClient, collection_name: str, url: str, book_title: str, chunk_id: int, text: str, embedding: List[float]) function in backend/main.py."
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
