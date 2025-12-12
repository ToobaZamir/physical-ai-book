---
id: 1
title: Establish Initial Project Constitution
stage: constitution
date: 2025-12-12
surface: agent
model: Gemini
feature: none
branch: master
user: '[USER]'
command: /sp.constitution
labels: [constitution, project-setup]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - .specify/templates/plan-template.md
tests:
  - none
---

## Prompt

/sp.constitution

Project: Book on Physical AI & Humanoid Robotics: AI Systems in the Physical World. Embodied Intelligence. Bridging the gap between digital AI and physical robotics through modules on ROS 2, simulation tools like Gazebo and Unity, NVIDIA Isaac, and Vision-Language-Action integration, including hardware requirements and capstone projects.

Core principles:
- Accuracy in technical descriptions of robotics tools, hardware, and AI integration
- Practicality for educational use (targeted at students and practitioners with AI/engineering background)
- Comprehensiveness in covering simulated and real-world applications
- Innovation focus on embodied intelligence and humanoid interactions

Key standards:
- All technical claims must be verifiable against official documentation (e.g., ROS 2, NVIDIA Isaac, Gazebo)
- Citation format: APA or IEEE style for references to tools, papers, or hardware specs
- Source types: Minimum 60% from official vendor docs, peer-reviewed robotics/AI papers, and industry reports
- Plagiarism check: 0% tolerance; all content original or properly attributed
- Writing clarity: Technical yet accessible, Flesch-Kincaid grade 12-14 for engineering audience

Constraints:
- Structure: Divided into modules, weekly breakdowns, hardware sections, and assessments
- Length: 50-100 pages equivalent, including diagrams and code snippets
- Format: PDF or eBook with embedded links to resources, diagrams for hardware setups
- Minimum 20 references/sources for tools, hardware, and case studies

Success criteria:
- All hardware and software recommendations verified for compatibility and current availability
- Zero factual errors in technical specs or tool descriptions
- Passes review for educational value and practical applicability
- Comprehensive coverage of Physical AI principles from simulation to deployment

## Response snapshot

I have updated the project constitution to version 1.0.0, incorporating your specified principles and standards. The constitution and related plan templates have been updated. Suggested commit message: `docs: establish constitution v1.0.0`

## Outcome

- ✅ Impact: Established the foundational governance document for the project.
- 🧪 Tests: No tests were run.
- 📁 Files: Created/updated the constitution and updated the plan template.
- 🔁 Next prompts: User may proceed with other setup tasks or feature development.
- 🧠 Reflection: The process of updating the constitution and propagating changes to templates worked as expected.

## Evaluation notes (flywheel)

- Failure modes observed: None observed.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
