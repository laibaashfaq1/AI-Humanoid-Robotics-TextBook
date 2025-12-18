# Tasks: RAG Agent Development with OpenAI Agents SDK + FastAPI

**Input**: Design documents from `/specs/004-rag-agent-fastapi/`
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

- [X] T002 Initialize `uv` project within the `backend/` directory (implicitly, for `backend/requirements.txt`).
- [X] T003 Create `main.py` in `backend/` for the FastAPI entry point.
- [X] T004 Create `.env` file in `backend/` for environment variables (e.g., OPENAI_API_KEY, QDRANT_API_KEY, QDRANT_URL, COHERE_API_KEY).
- [X] T005 Install primary dependencies (`uv`, `fastapi`, `uvicorn`, `openai-agents`, `qdrant-client`, `python-dotenv`) in `backend/requirements.txt`.
- [X] T007 Create `book_agent.py`, `qdrant_retrieval.py`, and `schemas.py` files directly in `backend/`.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T008 [P] Implement function to load environment variables from `.env` in `backend/main.py`.
- [X] T009 [P] Initialize FastAPI app in `backend/main.py`.
- [X] T010 [P] Define request/response Pydantic models (`AskRequest`, `AskResponse`) in `backend/schemas.py`.
- [X] T011 [P] Implement Qdrant client initialization in `backend/qdrant_retrieval.py`.
- [X] T012 [P] Implement Cohere client initialization in `backend/qdrant_retrieval.py`.
- [X] T013 Implement the core retrieval function `retrieve_book_content(query: str) -> str` in `backend/qdrant_retrieval.py`.
    - This function should embed the query with Cohere, search Qdrant, and return formatted context.

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Ask a Question to the RAG Agent (Priority: P1) 🎯 MVP

**Goal**: Send a question to the FastAPI endpoint so that the RAG agent processes it using retrieved book content and returns a grounded answer.

**Independent Test**: Send a question to the `/ask` endpoint. Verify that the response contains a natural language answer related to the book content and that the retrieval function was used.

### Implementation for User Story 1

- [X] T014 [US1] Define the OpenAI Agent in `backend/book_agent.py`, including system prompt and tool definition for `retrieve_book_content`.
- [X] T015 [US1] Create the `/ask` endpoint in `backend/main.py` that takes a user question.
- [X] T016 [US1] Implement the logic within the `/ask` endpoint to invoke the OpenAI Agent with the user's question.
- [X] T017 [US1] Ensure the agent's response is returned from the `/ask` endpoint in the `AskResponse` format.
- [X] T018 [US1] Add basic error handling for the `/ask` endpoint (e.g., for invalid requests).

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Verify Agent Grounding (Priority: P1)

**Goal**: Confirm that the RAG agent exclusively uses retrieved book content for its answers to prevent hallucinations and ensure factual consistency.

**Independent Test**: Send questions where the correct answer exists only in the book content and verify correct responses. Send questions where the answer is general knowledge but not in the book and verify the agent declines to answer.

### Implementation for User Story 2

- [X] T019 [US2] Refine the agent's system prompt in `backend/book_agent.py` to be more explicit about only using retrieved content.
- [X] T020 [US2] Modify `retrieve_book_content` in `backend/qdrant_retrieval.py` to return a clear "no content found" message if no relevant documents are retrieved.
- [X] T021 [US2] Update the agent's instructions in `backend/book_agent.py` to handle the "no content found" case by stating its inability to answer.
- [X] T022 [US2] Add a local testing script (`backend/local_test.py`) to run predefined questions against the running FastAPI app and validate responses.

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T023 Implement comprehensive logging for agent operations, tool calls, and retrieved content.
- [X] T024 Add robust error handling for OpenAI and Qdrant API failures.
- [X] T025 Add performance monitoring for the `/ask` endpoint to track response times.
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

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories.
- **User Story 2 (P1)**: Depends on User Story 1 completion as it refines the agent's behavior and adds validation.

### Within Each User Story

- Helper functions (e.g., `retrieve_book_content`) before integration into agent tools.
- Agent and tool definition before endpoint implementation.

### Parallel Opportunities

- Many tasks within the Setup and Foundational phases can be done in parallel.
- Once the Foundational phase is complete, User Story 1 can be developed in parallel with some Polish tasks (e.g., `README.md`).

---

## Parallel Example: Foundational Phase

```bash
# Different developers can work on these concurrently:
Task: "T008 [P] Implement function to load environment variables from .env in backend/main.py."
Task: "T009 [P] Initialize FastAPI app in backend/main.py."
Task: "T010 [P] Define request/response Pydantic models (AskRequest, AskResponse) in backend/schemas.py."
Task: "T011 [P] Implement Qdrant client initialization in backend/qdrant_retrieval.py."
Task: "T012 [P] Implement Cohere client initialization in backend/qdrant_retrieval.py."
```

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

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
