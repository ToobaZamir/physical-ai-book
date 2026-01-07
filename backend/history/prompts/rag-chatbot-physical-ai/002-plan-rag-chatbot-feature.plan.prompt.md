---
id: 003
title: Plan RAG Chatbot Feature
stage: plan
date: 2025-12-20
surface: agent
model: gpt-4
feature: rag-chatbot-physical-ai
branch: 1-rag-chatbot-physical-ai
user: Daniyal Shaikh
command: /sp.plan
labels: [planning, rag, chatbot, ai]
links:
  spec: specs/1-rag-chatbot-physical-ai/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-rag-chatbot-physical-ai/plan.md
  - specs/1-rag-chatbot-physical-ai/research.md
  - specs/1-rag-chatbot-physical-ai/data-model.md
  - specs/1-rag-chatbot-physical-ai/contracts/api-contracts.md
  - specs/1-rag-chatbot-physical-ai/quickstart.md
  - QWEN.md
tests:
  - none
---

## Prompt

**Project:** Integrated RAG Chatbot for Physical AI & Humanoid Robotics Book

**Target Audience:**  
AI/robotics students, educators, and enthusiasts learning about embodied intelligence, ROS 2, Gazebo, NVIDIA Isaac Sim, humanoid robotics hardware, and related topics.

**Focus:**  
- Retrieval-Augmented Generation using only the book's content  
- Support for user-selected/highlighted text as primary context  
- Seamless, privacy-focused embedding as a chat widget in the Docusaurus site (GitHub Pages)  
- Scalable architecture within free-tier services  

**Success Criteria:**  
- Chatbot accurately answers ≥90% of test queries based solely on book content (verified through manual evaluation)  
- Correctly incorporates and prioritizes user-selected text in responses  
- Zero critical bugs in production (no crashes, no data leaks)  
- Average response time <5 seconds for typical queries  
- Passes 20+ predefined test scenarios covering core book topics (e.g., "Explain ROS 2 nodes", "What GPU is required for Isaac Sim?", "Based on selected text about Unitree G1, summarize its capabilities")  
- Operates fully within free-tier limits without overages  
- Users can effectively clarify and deepen understanding of book concepts through interaction  

**Constraints:**  
- Mandatory tech stack: Cohere SDK (embed-english-v3.0 for embeddings, command-r or command-r-plus for generation), FastAPI backend, Neon Serverless Postgres (metadata), Qdrant Cloud Free Tier (vectors)  
- No external knowledge, web searches, or internet access in runtime  
- Deployment: Frontend static on GitHub Pages, backend on free-tier host (Render, Vercel, or Fly.io)  
- Responses must be concise, technical, and readable (Flesch-Kincaid grade 8-12)  
- Knowledge source limited exclusively to the Physical AI & Humanoid Robotics book content  
- Timeline: Complete implementation and deployment within 2-4 weeks (part-time effort)  
- No usage of paid tiers or services that incur costs beyond free limits  

**Not Building:**  
- Custom fine-tuning or training of any AI models  
- Comparison or benchmarking of different LLM/embedding providers  
- In-depth ethical or societal impact discussions of physical AI  
- Mobile applications or integrations outside Docusaurus/GitHub Pages  
- Multi-user real-time collaboration or chat history persistence  
- Advanced features like voice input/output or multimodal processing  

**Constitution Check** *GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*  
- **Accuracy via Content-Specific Retrieval**: All responses grounded only in book content with RAG  
- **User-Centric for AI/Robotics Students**: Interface and responses tailored to varying skill levels in robotics/AI  
- **Reliable on Book Topics**: High reliability for queries on ROS 2, Gazebo, Isaac Sim, hardware requirements, etc.  
- **Privacy-Focused**: Stateless design, no storage or logging of user queries  
- **Scalable with Free-Tier Services**: Architecture fits within Neon (~500MB), Qdrant (1GB), Cohere free-tier limits  
- **Tech Stack Consistency**: Exclusive use of Cohere SDK, FastAPI, Neon, Qdrant, Docusaurus embedding  

## Response snapshot

Completed comprehensive implementation planning for the RAG Chatbot feature, including:

1. Created detailed implementation plan (plan.md) outlining architecture and project structure
2. Conducted research (research.md) on technical decisions including Cohere SDK, vector databases, and privacy considerations
3. Defined data models (data-model.md) for book content, queries, and responses while maintaining privacy
4. Created API contracts (contracts/api-contracts.md) for all required endpoints
5. Developed quickstart guide (quickstart.md) for setting up and running the system
6. Updated agent context (QWEN.md) with current technology stack and project information

The plan ensures compliance with all constitution principles including accuracy via content-specific retrieval, user-centric design for AI/robotics students, reliability on book topics, privacy-focused approach with no query storage, scalability within free-tier services, and tech stack consistency.

## Outcome

- ✅ Impact: Comprehensive plan established for RAG chatbot implementation
- 🧪 Tests: N/A for planning artifacts
- 📁 Files: Created 6 planning artifacts across documentation, contracts, and agent context
- 🔁 Next prompts: Task breakdown with /sp.tasks
- 🧠 Reflection: Planning phase successfully completed with all required artifacts

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Break plan into implementable tasks