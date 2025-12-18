# Implementation Plan: Embedding Pipeline Setup

**Branch**: `002-embedding-pipeline` | **Date**: 2025-12-11 | **Spec**: ./spec.md
**Input**: Feature specification from `/specs/002-embedding-pipeline/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

The primary requirement is to build an embedding pipeline that extracts text from published book URLs, generates Cohere embeddings, and stores them in Qdrant Cloud for retrieval. The technical approach involves using Python with the `uv` package manager, `cohere` for embeddings, and `qdrant-client` for vector storage. The pipeline will focus on efficient crawling, text cleaning, chunking, and idempotent upsertion of vectors with metadata.

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**: `uv` (package manager), `cohere` (for embeddings), `qdrant-client` (for vector database), `requests` (for web crawling), `BeautifulSoup4` (for HTML parsing)
**Storage**: Qdrant Cloud (vector database)
**Testing**: `pytest` (standard Python testing framework)
**Target Platform**: Linux/Windows (script will run in a Python environment)
**Project Type**: Single script (CLI tool)
**Performance Goals**: Efficient processing of book URLs, aiming for timely ingestion of new content.
**Constraints**: Only Cohere for embeddings, Qdrant Cloud Free Tier, avoid duplicates on reruns.
**Scale/Scope**: Process all available book URLs from the specified website, with potential for future expansion to other sources.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [X] **Accuracy**: All claims are source-backed with proper citations.
- [X] **Clarity**: Content targets a technical audience (Flesch score 60-75).
- [X] **Consistency**: Follows Docusaurus structure and consistent writing style.
- [X] **Reproducibility**: All code examples are tested and runnable.
- [X] **AI Alignment**: AI-generated outputs adhere to this constitution.

## Project Structure

### Documentation (this feature)

```text
specs/002-embedding-pipeline/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── main.py
├── requirements.txt
├── .env
├── .venv/
└── tests/
    └── unit/
```

**Structure Decision**: The project will be a single Python script (`main.py`) within a `backend` directory, managed by `uv`. This aligns with the user's request for a single file implementation and provides a clear separation for the backend logic. No frontend or mobile components are in scope.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|---|---|---|
| | | |