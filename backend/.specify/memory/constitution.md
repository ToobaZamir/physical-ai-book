<!--
Sync Impact Report:
- Version change: 1.1.0 → 1.2.0
- Major change: Switch from OpenAI to Cohere for embeddings and generation
- Updated principles: V (Scalable with Free-Tier Services), VI (Tech Stack Consistency)
- Updated sections: Key Standards, Constraints
- Templates requiring updates: plan-template.md, spec-template.md, tasks-template.md, checklist-template.md, agent-file-template.md
-->

# Integrated RAG Chatbot for Physical AI & Humanoid Robotics Book Constitution

## Core Principles

### I. Accuracy via Content-Specific Retrieval
Responses must be grounded in book content only, with retrieval-augmented generation ensuring factual accuracy. All answers must trace back to specific passages in the Physical AI & Humanoid Robotics book to maintain trustworthiness and educational value.

### II. User-Centric for AI/Robotics Students
Design and implementation must prioritize the learning journey of AI and robotics students. Features, interfaces, and responses should cater to different skill levels from beginner to advanced practitioners in robotics.

### III. Reliable on Book Topics (ROS 2, Gazebo, Isaac Sim, etc.)
System must demonstrate exceptional reliability when answering queries related to core book topics including ROS 2, Gazebo simulation, Isaac Sim, and other robotics frameworks covered in the Physical AI book. This includes handling technical depth appropriately.

### IV. Privacy-Focused (No Query Storage)
User privacy must be protected by not storing or logging individual queries. All conversations should be stateless with no persistent storage of user questions or interactions to maintain confidentiality.

### V. Scalable with Free-Tier Services
Architecture must leverage cost-effective free-tier services (Neon Postgres, Qdrant Cloud Free, Cohere free tier) to ensure sustainability. Implementation should handle scaling within free-tier resource constraints (~500MB Neon, 1GB Qdrant, Cohere 1,000 calls/month free).

### VI. Tech Stack Consistency
Implementation must adhere to the specified technology stack: Cohere SDK (for embeddings and generation), FastAPI, Neon Serverless Postgres, Qdrant Cloud Free Tier, and seamless integration with Docusaurus for GitHub Pages embedding.

## Key Standards

- Responses **only** from book content or selected text provided by users
- Embeddable widget in Docusaurus (GitHub Pages) for seamless integration
- Tech stack: Cohere SDK, FastAPI, Neon Serverless Postgres, Qdrant Cloud Free Tier
- Clear, technical responses written at Flesch-Kincaid grade level 8-12 for accessibility
- Graceful handling of out-of-scope queries with clear redirection
- Full testing coverage and comprehensive documentation (README.md)

## Constraints

- Must operate within free-tier limits (Neon: ~500MB, Qdrant: 1GB, Cohere free tier)
- Use Cohere models: embed-english-v3.0 for embeddings, command-r or command-r-plus for generation
- Deployment: GitHub Pages (frontend), free hosting service like Render/Vercel/Fly.io (backend)
- No external knowledge or web search capabilities
- Implementation using Spec-Kit Plus and Qwen methodology
- Timeline: 2-4 weeks part-time development

## Success Criteria

- Achieve ≥90% accuracy on test queries related to book content
- Properly handle user-provided selected text in context
- Zero critical bugs in production deployment
- Seamless embedding with <5s response time
- Passes 20+ test scenarios covering various book topics

## Governance

This constitution supersedes all other development practices for this project. All implementations must comply with these principles. Amendments require documentation of reasoning and impact assessment. All pull requests and reviews must verify constitutional compliance.

**Version**: 1.2.0 | **Ratified**: 2025-06-13 | **Last Amended**: 2025-12-20
