# Research: RAG Chatbot for Physical AI Book

**Feature**: RAG Chatbot for Physical AI Book
**Date**: 2025-12-20
**Branch**: 1-rag-chatbot-physical-ai

## Overview

This research document addresses the technical unknowns and decisions required for implementing the RAG chatbot for the Physical AI & Humanoid Robotics book. The implementation will use Cohere's embedding and generation models, FastAPI for the backend, and be designed for embedding in Docusaurus sites.

## Cohere SDK Implementation

### Decision: Use Cohere's embed-english-v3.0 for embeddings
**Rationale**: The embed-english-v3.0 model is specifically designed for English text embeddings and is recommended for RAG applications. It provides good performance within the free-tier limits.

**Alternatives considered**:
- Sentence Transformers models: Would require self-hosting, exceeding free-tier constraints
- OpenAI embeddings: Would require different tech stack than specified

### Decision: Use Cohere's command-r for generation
**Rationale**: The command-r model is optimized for RAG applications and provides good quality responses. It's specifically designed to work well with retrieved context.

**Alternatives considered**:
- command-r-plus: Higher quality but potentially higher cost, and command-r is sufficient for this use case
- Other LLM providers: Would violate tech stack consistency requirement

## Vector Database Architecture

### Decision: Use Qdrant Cloud Free Tier for vector storage
**Rationale**: Qdrant is efficient for similarity search in RAG applications and offers a free tier that meets our storage requirements. The 1GB limit should be sufficient for book content embeddings.

**Alternatives considered**:
- Pinecone: Would require different tech stack than specified
- Self-hosted vector DB: Would exceed free-tier constraints
- Chroma: Less efficient for large-scale similarity search

## Backend Architecture

### Decision: FastAPI with async endpoints
**Rationale**: FastAPI provides excellent performance for API endpoints with built-in async support, which is important for RAG applications that involve external API calls. It also has excellent documentation and validation features.

**Alternatives considered**:
- Flask: Less performant for async operations
- Django: Overkill for this use case
- Express.js: Would violate tech stack consistency requirement

## Database for Metadata

### Decision: Neon Serverless Postgres for metadata storage
**Rationale**: Neon provides a serverless Postgres option that fits within our free-tier requirements. It will store metadata about document chunks, user sessions (non-persistent), and system logs.

**Alternatives considered**:
- SQLite: Less scalable and doesn't fit the specified tech stack
- MongoDB: Would violate tech stack consistency requirement

## Frontend Widget Implementation

### Decision: Standalone JavaScript widget for Docusaurus embedding
**Rationale**: A standalone JavaScript widget can be easily embedded in Docusaurus sites using standard HTML/JS injection. This approach allows for seamless integration without modifying the Docusaurus build process.

**Alternatives considered**:
- React component: Would require Docusaurus to support React widgets
- Iframe: Would create additional complexity for communication between parent page and widget

## Privacy and Statelessness

### Decision: Stateless design with no user query storage
**Rationale**: To maintain privacy as required by the constitution, all conversations will be stateless. No user queries or conversation history will be stored. Context for user-selected text will be passed in each request.

**Implementation approach**:
- No database persistence for user queries
- Session state maintained only in client browser memory
- No logging of user queries or responses

## Book Content Processing

### Decision: Pre-process book content into chunks for embedding
**Rationale**: The Physical AI book content will be pre-processed into semantically meaningful chunks that can be efficiently embedded and retrieved. This approach improves response quality and reduces token usage.

**Chunking strategy**:
- Split by semantic boundaries (sections, paragraphs)
- Overlap chunks to preserve context
- Store metadata with each chunk for source attribution

## Performance Considerations

### Decision: Implement caching for common queries
**Rationale**: To meet the <5 second response time requirement, implement caching for frequently asked questions using a simple in-memory cache.

**Implementation approach**:
- Redis or in-memory cache for common responses
- Cache invalidation based on content updates
- TTL-based expiration to stay within memory limits

## Error Handling and Fallbacks

### Decision: Implement graceful degradation for service limits
**Rationale**: When free-tier limits are reached, the system should provide appropriate user-facing error messages and fallback options rather than failing silently.

**Implementation approach**:
- Monitor API usage against limits
- Fallback to simple responses when limits are reached
- Clear user messaging about service status