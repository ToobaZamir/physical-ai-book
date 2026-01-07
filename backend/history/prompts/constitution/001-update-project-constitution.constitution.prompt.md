---
id: 001
title: Update Project Constitution
stage: constitution
date: 2025-12-20
surface: agent
model: gpt-4
feature: none
branch: main
user: Daniyal Shaikh
command: /sp.constitution
labels: [constitution, setup, configuration]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - .specify/templates/plan-template.md
  - .specify/templates/spec-template.md
  - .specify/templates/tasks-template.md
  - .specify/templates/checklist-template.md
  - QWEN.md
  - .specify/templates/agent-file-template.md
tests:
  - none
---

## Prompt

Create or update the project constitution from interactive or provided principle inputs, ensuring all dependent templates stay in sync.

## Response snapshot

Updated the project constitution for the Integrated RAG Chatbot for Physical AI & Humanoid Robotics Book with the following core principles:
- Accuracy via content-specific retrieval
- User-centric for AI/robotics students
- Reliable on book topics (ROS 2, Gazebo, Isaac Sim, etc.)
- Privacy-focused (no query storage)
- Scalable with free-tier services
- Tech stack consistency

Also updated dependent templates to align with the new constitution:
- plan-template.md: Added constitution check section with all principles
- spec-template.md: Added constitution alignment requirements for user stories
- tasks-template.md: Added constitution-compliant foundational tasks
- checklist-template.md: Added constitution compliance verification items
- QWEN.md: Added project-specific guidelines section
- agent-file-template.md: Added constitution compliance section

## Outcome

- ✅ Impact: Established core project principles and governance structure
- 🧪 Tests: N/A for configuration files
- 📁 Files: 7 files updated to align with new constitution
- 🔁 Next prompts: Feature specification and planning
- 🧠 Reflection: Successfully established project foundation with clear principles

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin feature specification using new constitution