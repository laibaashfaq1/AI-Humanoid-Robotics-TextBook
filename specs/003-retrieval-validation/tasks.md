# Tasks: Retrieval Pipeline Validation

**Input**: Design documents from `/specs/003-retrieval-validation/`
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

- [ ] T001 Create `validation_script` directory for the project.
- [ ] T002 Initialize `uv` project within the `validation_script` directory.
- [ ] T003 Create `main.py` in `validation_script/` for the core script.
- [ ] T004 Create `.env` file in `validation_script/` for environment variables (e.g., COHERE_API_KEY, QDRANT_API_KEY, QDRANT_URL).
- [ ] T005 Install primary dependencies (`cohere`, `qdrant-client`, `python-dotenv`) using `uv` in `validation_script/requirements.txt`.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T006 Implement function to load environment variables from `.env` in `validation_script/main.py`.
- [ ] T007 Initialize Qdrant client in `validation_script/main.py` using environment variables.
- [ ] T008 Initialize Cohere client in `validation_script/main.py` using environment variables.
- [ ] T009 Define `QDRANT_COLLECTION_NAME` and `COHERE_EMBEDDING_MODEL` constants in `validation_script/main.py`.
- [ ] T010 Implement function `connect_to_qdrant() -> QdrantClient` in `validation_script/main.py`.
- [ ] T011 Implement function `connect_to_cohere() -> cohere.Client` in `validation_script/main.py`.

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Validate Basic Vector Retrieval (Priority: P1) 🎯 MVP

**Goal**: Retrieve a specific number of stored vectors from Qdrant to confirm their existence and basic integrity, ensuring the Qdrant connection is functional.

**Independent Test**: The validation script can be run to retrieve 100 random vectors. The script should report success if 100 vectors are returned, and failure otherwise.

### Implementation for User Story 1

- [ ] T012 [US1] Implement `retrieve_random_vectors(client: QdrantClient, collection_name: str, limit: int) -> List[RetrievedVector]` function in `validation_script/main.py`.
- [ ] T013 [US1] Implement `validate_vector_retrieval(vectors: List[RetrievedVector], expected_count: int) -> bool` function in `validation_script/main.py` to check count and basic metadata presence.
- [ ] T014 [US1] Integrate `retrieve_random_vectors` and `validate_vector_retrieval` into a main validation flow in `validation_script/main.py`.

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Validate Semantic Search Accuracy (Priority: P1)

**Goal**: Perform semantic searches using a Cohere-embedded query against the Qdrant collection to verify the relevance of the top-k retrieved results, confirming the search mechanism works as expected.

**Independent Test**: The validation script can be run with a set of predefined test queries and expected relevant sections. For each query, the script should perform a semantic search, and then verify if the top-k results contain the expected `book_title` and `source_url`.

### Implementation for User Story 2

- [ ] T015 [US2] Implement `embed_query(cohere_client: cohere.Client, query_text: str) -> List[float]` function in `validation_script/main.py`.
- [ ] T016 [US2] Implement `perform_semantic_search(qdrant_client: QdrantClient, collection_name: str, query_vector: List[float], top_k: int) -> List[RetrievedVector]` function in `validation_script/main.py`, ensuring cosine similarity.
- [ ] T017 [US2] Implement `validate_semantic_search_results(query: ValidationQuery, results: List[RetrievedVector], threshold: float) -> bool` function in `validation_script/main.py` to check relevance based on expected results and cosine similarity.
- [ ] T018 [US2] Define a set of `ValidationQuery` objects with `ExpectedResult`s for testing semantic search in `validation_script/main.py`.
- [ ] T019 [US2] Integrate semantic search functions and validation into the main validation flow in `validation_script/main.py`.

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Validate End-to-End Pipeline Data Integrity (Priority: P2)

**Goal**: Validate the end-to-end data integrity of the retrieval pipeline to ensure all original book URLs have been correctly processed into vectors with accurate metadata.

**Independent Test**: The validation script can iterate through a sample of original book URLs, retrieve their expected vectors and metadata from Qdrant, and perform checks for completeness and consistency.

### Implementation for User Story 3

- [ ] T020 [US3] Implement `get_all_source_urls_from_qdrant(client: QdrantClient, collection_name: str) -> Set[str]` function in `validation_script/main.py` to get all unique source URLs from Qdrant metadata.
- [ ] T021 [US3] Implement `validate_data_integrity(client: QdrantClient, cohere_client: cohere.Client, collection_name: str, source_urls: Set[str]) -> bool` function in `validation_script/main.py`.
    - This function should iterate through source URLs.
    - For each URL, retrieve associated vectors.
    - Validate metadata presence (`source_url`, `chunk_id`, `book_title`, `timestamp`).
    - Validate vector dimensions match Cohere model.
    - Identify and report missing vectors or corrupted entries.
- [ ] T022 [US3] Integrate `validate_data_integrity` into the main validation flow in `validation_script/main.py`.

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T023 Implement CLI interface using `argparse` for running specific validation checks (`--basic-retrieval`, `--semantic-search`, `--data-integrity`, `--all`) in `validation_script/main.py`.
- [ ] T024 Add comprehensive logging (e.g., using Python's `logging` module) for all operations and validation results in `validation_script/main.py`.
- [ ] T025 Add basic performance metrics for each validation step (e.g., time taken) in `validation_script/main.py`.
- [ ] T026 Prepare a `README.md` in the `validation_script/` directory with setup and usage instructions.
- [ ] T027 Ensure the script handles an empty Qdrant collection gracefully in `validation_script/main.py`.
- [ ] T028 Add error handling for invalid or missing environment variables in `validation_script/main.py`.

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
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories

### Within Each User Story

- Helper functions (e.g., `retrieve_random_vectors`) before validation logic (e.g., `validate_vector_retrieval`).
- Core logic before integration into main flow.

### Parallel Opportunities

- Many tasks within each phase that are marked with `[P]` or operate on different logical components can be developed in parallel.
- Once the Foundational phase is complete, User Stories 1, 2, and 3 can be developed in parallel by different team members.

---

## Parallel Example: Foundational Phase

```bash
# Different developers can work on these concurrently:
Task: "T006 Implement function to load environment variables from .env in validation_script/main.py."
Task: "T007 Initialize Qdrant client in validation_script/main.py using environment variables."
Task: "T008 Initialize Cohere client in validation_script/main.py using environment variables."
Task: "T009 Define QDRANT_COLLECTION_NAME and COHERE_EMBEDDING_MODEL constants in validation_script/main.py."
Task: "T010 Implement function connect_to_qdrant() -> QdrantClient in validation_script/main.py."
Task: "T011 Implement function connect_to_cohere() -> cohere.Client in validation_script/main.py."
```

## Parallel Example: User Stories (after Foundational Phase)

```bash
# Developer A: User Story 1
Task: "T012 [US1] Implement retrieve_random_vectors(...) function in validation_script/main.py."
Task: "T013 [US1] Implement validate_vector_retrieval(...) function in validation_script/main.py."
Task: "T014 [US1] Integrate retrieve_random_vectors and validate_vector_retrieval into a main validation flow in validation_script/main.py."

# Developer B: User Story 2
Task: "T015 [US2] Implement embed_query(...) function in validation_script/main.py."
Task: "T016 [US2] Implement perform_semantic_search(...) function in validation_script/main.py."
Task: "T017 [US2] Implement validate_semantic_search_results(...) function in validation_script/main.py."
Task: "T018 [US2] Define a set of ValidationQuery objects with ExpectedResult"s for testing semantic search in validation_script/main.py."
Task: "T019 [US2] Integrate semantic search functions and validation into the main validation flow in validation_script/main.py."

# Developer C: User Story 3
Task: "T020 [US3] Implement get_all_source_urls_from_qdrant(...) function in validation_script/main.py."
Task: "T021 [US3] Implement validate_data_integrity(...) function in validation_script/main.py."
Task: "T022 [US3] Integrate validate_data_integrity into the main validation flow in validation_script/main.py."
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
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
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
