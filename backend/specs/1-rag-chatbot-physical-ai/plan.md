# Implementation Plan: RAG Chatbot for Physical AI Book

**Branch**: `1-rag-chatbot-physical-ai` | **Date**: 2025-12-20 | **Spec**: [link to spec]
**Input**: Feature specification from `/specs/1-rag-chatbot-physical-ai/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement a Retrieval-Augmented Generation (RAG) chatbot for the Physical AI & Humanoid Robotics book. The solution will use Cohere's embedding and generation models to provide accurate, book-specific answers to user queries. The system will support user-selected text as context and be embeddable in Docusaurus sites. Architecture will be stateless and privacy-focused, operating within free-tier service limits.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: Cohere SDK, FastAPI, Qdrant, Neon Postgres, Docusaurus
**Storage**: Qdrant Cloud Free Tier (vector storage), Neon Serverless Postgres (metadata)
**Testing**: pytest
**Target Platform**: Linux server (Render/Vercel/Fly.io), GitHub Pages (frontend)
**Project Type**: Web application (backend + frontend widget)
**Performance Goals**: <5 seconds response time for typical queries
**Constraints**: Free-tier limits (Neon: ~500MB, Qdrant: 1GB, Cohere: 1,000 calls/month)
**Scale/Scope**: Single book content, multi-user access, stateless conversations

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Accuracy via Content-Specific Retrieval**: Ensure all responses are grounded in book content with retrieval-augmented generation.
**User-Centric for AI/Robotics Students**: Design must prioritize learning journey of AI and robotics students at various skill levels.
**Reliable on Book Topics**: System must demonstrate exceptional reliability for ROS 2, Gazebo, Isaac Sim, and other robotics frameworks from the book.
**Privacy-Focused**: Verify no user queries are stored or logged, maintaining stateless conversations.
**Scalable with Free-Tier Services**: Architecture must leverage Neon Postgres, Qdrant Cloud Free within resource constraints (~500MB Neon, 1GB Qdrant, Cohere 1,000 calls/month free).
**Tech Stack Consistency**: Implementation must use Cohere SDK, FastAPI, Neon Postgres, Qdrant Cloud Free Tier with Docusaurus integration.

## Project Structure

### Documentation (this feature)

```text
specs/1-rag-chatbot-physical-ai/
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
├── src/
│   ├── models/
│   ├── services/
│   ├── api/
│   └── utils/
├── tests/
│   ├── unit/
│   ├── integration/
│   └── contract/
├── scripts/
│   └── prepare_embeddings.py
└── requirements.txt

frontend/
└── widget/
    ├── chat-widget.js
    ├── chat-widget.css
    └── index.html (demo)

docs/
└── book-content/
    └── physical-ai-book.md (source for embeddings)
```

**Structure Decision**: Web application with separate backend (FastAPI) and frontend (widget) components, with shared documentation directory for book content used for embeddings.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |