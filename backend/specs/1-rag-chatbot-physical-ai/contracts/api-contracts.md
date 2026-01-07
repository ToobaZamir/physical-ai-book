# API Contracts: RAG Chatbot for Physical AI Book

**Feature**: RAG Chatbot for Physical AI Book
**Date**: 2025-12-20
**Branch**: 1-rag-chatbot-physical-ai

## Overview

This document defines the API contracts for the RAG chatbot system. The APIs are designed to support retrieval-augmented generation while maintaining privacy and operating within free-tier constraints.

## API Endpoints

### 1. Query Processing API

#### POST /api/v1/query

Process a user query and return a response based on book content.

**Request**:
```json
{
  "query": "string (user's question)",
  "selected_text": "string (optional, text selected on the page)",
  "session_id": "string (temporary session identifier)"
}
```

**Response**:
```json
{
  "response_id": "string (identifier for the response)",
  "answer": "string (the generated answer)",
  "sources": [
    {
      "chunk_id": "string (ID of the book content chunk used)",
      "title": "string (title of the section)",
      "section": "string (book section identifier)",
      "page_number": "integer (page number in original book)"
    }
  ],
  "confidence": "float (confidence score between 0.0 and 1.0)",
  "response_time_ms": "integer (time taken to generate response)"
}
```

**Headers**:
- Content-Type: application/json
- Accept: application/json

**Status Codes**:
- 200: Successful response
- 400: Invalid request format
- 429: Rate limit exceeded
- 500: Internal server error

**Rate Limiting**:
- Per IP: 100 requests/hour
- Per session: 50 requests/hour

### 2. Health Check API

#### GET /api/v1/health

Check the health status of the service.

**Response**:
```json
{
  "status": "string (overall service status: 'healthy', 'degraded', 'unhealthy')",
  "timestamp": "string (ISO 8601 timestamp)",
  "services": {
    "cohere_api": "string (status of Cohere integration)",
    "qdrant": "string (status of vector database)",
    "neon": "string (status of metadata database)"
  }
}
```

**Status Codes**:
- 200: Service is healthy
- 503: Service is unhealthy

### 3. Book Content Metadata API

#### GET /api/v1/book/metadata

Get metadata about the book content available in the system.

**Response**:
```json
{
  "book_title": "string (title of the book)",
  "author": "string (author of the book)",
  "version": "string (version of the book content)",
  "total_chunks": "integer (number of content chunks)",
  "sections": [
    {
      "name": "string (name of the section)",
      "count": "integer (number of chunks in this section)"
    }
  ],
  "last_updated": "string (ISO 8601 timestamp of last content update)"
}
```

**Status Codes**:
- 200: Successful response
- 500: Internal server error

### 4. Similarity Search API

#### POST /api/v1/search

Perform a similarity search against book content (for testing and validation).

**Request**:
```json
{
  "query": "string (search query)",
  "top_k": "integer (number of results to return, default 5, max 10)"
}
```

**Response**:
```json
{
  "results": [
    {
      "chunk_id": "string (ID of the book content chunk)",
      "content": "string (content of the chunk - truncated for privacy)",
      "title": "string (title of the section)",
      "section": "string (book section identifier)",
      "page_number": "integer (page number in original book)",
      "similarity_score": "float (similarity score between 0.0 and 1.0)"
    }
  ]
}
```

**Status Codes**:
- 200: Successful response
- 400: Invalid request format
- 500: Internal server error

## Authentication & Authorization

The APIs are designed to be stateless and privacy-focused:
- No user authentication required
- Rate limiting by IP and session
- No persistent user data stored

## Error Handling

All API endpoints follow a consistent error response format:

```json
{
  "error": {
    "code": "string (error code)",
    "message": "string (human-readable error message)",
    "details": "object (optional, additional error details)"
  }
}
```

## Data Validation

### Query Validation
- query: Required, 1-1000 characters
- selected_text: Optional, 0-500 characters
- session_id: Required, 1-100 characters

### Content Validation
- All text content is validated for length and format
- No HTML or script tags allowed
- Content is sanitized before processing

## Rate Limiting & Throttling

To ensure fair usage within free-tier constraints:
- Global rate limit: 1000 requests/hour
- Per IP rate limit: 100 requests/hour
- Per session rate limit: 50 requests/hour
- Burst allowance: 10 requests per minute

## Privacy & Data Handling

- No user queries are stored or logged
- No user responses are stored or logged
- Session data is temporary and cleared after use
- IP addresses are only used for rate limiting as hashed values
- No personal information is collected or stored