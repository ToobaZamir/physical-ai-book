# Quickstart Guide: RAG Chatbot for Physical AI Book

**Feature**: RAG Chatbot for Physical AI Book
**Date**: 2025-12-20
**Branch**: 1-rag-chatbot-physical-ai

## Overview

This guide provides a quick start for setting up and running the RAG chatbot for the Physical AI & Humanoid Robotics book. The system uses Cohere's embedding and generation models, FastAPI for the backend, and is designed for embedding in Docusaurus sites.

## Prerequisites

- Python 3.11+
- Access to Cohere API (with embed-english-v3.0 and command-r models)
- Qdrant Cloud Free Tier account
- Neon Serverless Postgres account
- Node.js (for frontend widget development, optional)

## Environment Setup

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Set up Python Virtual Environment

```bash
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

### 4. Configure Environment Variables

Create a `.env` file in the backend directory:

```env
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_DATABASE_URL=your_neon_database_connection_string
SECRET_KEY=your_secret_key_for_session_management
DEBUG=false
```

## Backend Setup

### 1. Prepare Book Content

The book content needs to be processed into chunks for the RAG system:

```bash
cd backend
python scripts/prepare_embeddings.py
```

This script will:
- Read the Physical AI book content
- Split it into semantically meaningful chunks
- Generate embeddings using Cohere's embed-english-v3.0
- Store the chunks in Qdrant vector database
- Store metadata in Neon Postgres

### 2. Run Database Migrations (if applicable)

```bash
# If using an ORM with migrations
python -m alembic upgrade head
```

### 3. Start the Backend Server

```bash
cd backend
uvicorn src.main:app --reload --port 8000
```

The backend API will be available at `http://localhost:8000`.

## Frontend Widget Setup

### 1. Embed the Chat Widget in Your Docusaurus Site

Add this HTML snippet to your Docusaurus site:

```html
<!-- Chat Widget Container -->
<div id="physical-ai-chatbot"></div>

<!-- Load the Chat Widget -->
<script src="/path/to/chat-widget.js"></script>
<script>
  // Initialize the chat widget
  PhysicalAIChatbot.init({
    apiEndpoint: 'http://localhost:8000/api/v1',  // Update to your backend URL
    containerId: 'physical-ai-chatbot',
    title: 'Physical AI Assistant',
    placeholder: 'Ask about Physical AI & Robotics...'
  });
</script>
```

### 2. Widget Configuration Options

The widget supports the following configuration options:

```javascript
PhysicalAIChatbot.init({
  apiEndpoint: 'https://your-backend.com/api/v1',  // Backend API endpoint
  containerId: 'physical-ai-chatbot',              // ID of container element
  title: 'Physical AI Assistant',                  // Widget title
  placeholder: 'Ask about Physical AI & Robotics...', // Input placeholder
  theme: 'light',                                  // 'light' or 'dark'
  position: 'bottom-right',                        // 'bottom-left', 'bottom-right', 'top-left', 'top-right'
  initialMessage: 'Hello! I can help you understand Physical AI concepts.', // Initial message
  enableSelectedText: true                         // Enable selected text feature
});
```

## Testing the System

### 1. API Endpoints

Test the main query endpoint:

```bash
curl -X POST http://localhost:8000/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain ROS 2 nodes",
    "selected_text": "",
    "session_id": "test-session-123"
  }'
```

### 2. Health Check

```bash
curl http://localhost:8000/api/v1/health
```

### 3. Book Metadata

```bash
curl http://localhost:8000/api/v1/book/metadata
```

## Running Tests

### Backend Tests

```bash
cd backend
pytest tests/
```

### Specific Test Categories

```bash
# Unit tests
pytest tests/unit/

# Integration tests
pytest tests/integration/

# Contract tests
pytest tests/contract/
```

## Deployment

### Backend Deployment

The backend can be deployed to free-tier platforms like Render, Vercel, or Fly.io:

1. Ensure all environment variables are configured in the deployment platform
2. The application is configured to run with a WSGI/ASGI server
3. Database connections are properly configured for the deployment environment

### Frontend Integration

The frontend widget is designed to be embedded in Docusaurus sites on GitHub Pages. No special deployment is needed beyond including the JavaScript file in your site's static assets.

## Troubleshooting

### Common Issues

1. **Cohere API Errors**: Verify your API key is correct and you have access to the required models
2. **Qdrant Connection Errors**: Check your Qdrant URL and API key
3. **Neon Database Errors**: Verify your database connection string is correct
4. **Rate Limiting**: The system implements rate limiting; check logs for rate limit messages

### Debugging Tips

1. Enable debug mode by setting `DEBUG=true` in your environment
2. Check the application logs for error messages
3. Use the health check endpoint to verify service status
4. Test individual components separately if the full system isn't working

## Next Steps

1. Customize the widget styling to match your site's design
2. Add analytics to track usage and improve the system
3. Expand the book content if new editions are available
4. Monitor usage to ensure staying within free-tier limits