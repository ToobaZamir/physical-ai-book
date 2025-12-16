---
id: 2
title: Create Implementation Plan for Physical AI Book
stage: plan
date: 2025-12-12
surface: agent
model: Gemini
feature: 001-physical-ai-book
branch: 001-physical-ai-book
user: '[USER]'
command: /sp.plan
labels: [plan, architecture]
links:
  spec: specs/001-physical-ai-book/plan.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-book/plan.md
  - specs/001-physical-ai-book/research.md
  - specs/001-physical-ai-book/data-model.md
  - specs/001-physical-ai-book/contracts/validation_contracts.md
  - specs/001-physical-ai-book/quickstart.md
  - GEMINI.md
tests:
  - none
---

## Prompt

/sp.plan

Project: Physical AI & Humanoid Robotics – The Complete Practitioner’s Book (2026 edition)

Architecture Sketch
- Book is organized as a 13-week university capstone course in executable form
- Each chapter = 1–2 weeks of lectures + labs
- Three intertwined tracks running in parallel throughout the book:
  1. Simulation track (Isaac Sim → Gazebo → Unity)
  2. Edge/Real-robot track (Jetson Orin → Unitree G1/Go2)
  3. VLA cognitive track (Whisper → LLM → ROS 2 actions)
- Final convergence in Module 4 Capstone: one single repository that runs identically in high-fidelity sim and on real hardware

Final Section Structure (locked – no changes allowed after this plan)
Module 1 – ROS 2 Mastery (3 chapters)  
Module 2 – Digital Twins (3 chapters)  
Module 3 – The AI-Robot Brain (3 chapters)  
Module 4 – Vision-Language-Action & Capstone (3 chapters)  
Appendices (locked):
  A. Complete Economy Jetson Kit BOM + wiring diagrams (Dec 2025 prices)
  B. Full URDF + SDF for Unitree G1 (official + community-enhanced)
  C. Cloud vs On-Prem cost calculator spreadsheet
  D. Sim-to-Real transfer checklist & common failure modes
  E. QR-linked video playlist (100+ private 4K demo videos)

Research Approach
- Research-concurrent (write → discover gap → research → update chapter → repeat)
- Primary sources only for all technical claims:
  → Official ROS 2 documentation (2025-12-01 snapshot)
  → NVIDIA Isaac Sim 2025.2 release notes & GitHub examples
  → Unitree official SDK + community forks (tested commits)
  → ArXiv preprints accepted to ICRA/IROS/RSJ 2025 for VLA methods
- Weekly validation sprint every Sunday: run entire chapter code on clean VM + real Jetson

Quality Validation & Acceptance Criteria
- Every terminal command must succeed on fresh Ubuntu 22.04 + ROS Jazzy + Isaac Sim 2025.2
- Every launch file must start within 15 seconds on RTX 4080 workstation
- Every perception pipeline must achieve ≥95% mAP on custom apartment dataset (provided)
- Capstone repo must run end-to-end in <4 minutes in sim and <8 minutes on real Unitree G1
- Zero broken QR codes, zero 404 links (checked by automated CI)
- PDF passes accessibility check (PDF/UA) and prints correctly in grayscale

Decisions Requiring Permanent Documentation
1. ROS 2 distro: Jazzy Jalisco (2024-05 → 2029 LTS) vs Iron Irwini  
   → Chosen: Jazzy – only distro with official NVIDIA Isaac ROS 2 GEMs support in 2025
2. Simulation backbone: Isaac Sim vs MuJoCo vs PyBullet  
   → Chosen: Isaac Sim 2025.2 – only platform with GPU PhysX, USD, and direct RTX sensor simulation
3. Primary humanoid model: Unitree G1 ($16k) vs Boston Dynamics Atlas (unavailable) vs Figure 02 (no SDK)  
   → Chosen: Unitree G1 – only commercially available humanoid in 2025 with open ROS 2 controller injection
4. LLM for VLA: GPT-4o (API) vs LLaMA-3.1-70B (local)  
   → Chosen: Hybrid – LLaMA-3.1-70B-8bit on Jetson for inference, GPT-4o only for few-shot plan refinement during development
5. License: CC-BY-NC-SA 4.0 vs fully open-source  
   → Chosen: CC-BY-NC-SA 4.0 (allows university use, prevents commercial textbook resale)

Testing Strategy (CI/CD pipeline on GitHub Actions)
Phase 1 – Per-chapter validation (runs on every PR)
   - Spin up Ubuntu 22.04 + ROS Jazzy + Isaac Sim (cloud workstation)
   - colcon build --symlink-install
   - Run all ros2 launch commands in chapter
   - Fail if any node dies or launch takes >20s
Phase 2 – Full book integration test (nightly)
   - Deploy capstone repo to real Jetson Orin NX over SSH
   - Execute “Bring me a Coke” scenario in real apartment lab
   - Record video → upload → update QR code hash
Phase 3 – Release gate (manual trigger)
   - Generate PDF + ePub
   - Run accessibility checker, link checker, price scraper for BOM
   - Only merge to main if 100% pass

Timeline & Phases (16 weeks total, starting 15 Dec 2025)
Weeks 01-04 → Module 1 complete + CI pipeline operational  
Weeks 05-08 → Module 2 + first sim-to-real locomotion success on Unitree Go2  
Weeks 09-12 → Module 3 + perception stack running at 15 Hz on Jetson  
Weeks 13-15 → Module 4 + capstone end-to-end working in sim  
Week 16    → Full sim-to-real deployment on real Unitree G1 + final proofreading

Success = On 1 April 2026, any university robotics lab with ≥1 Unitree G1 and ≥5 RTX 40-series workstations can run the full 13-week course using only this book and the companion GitHub repo – zero missing steps, zero broken code.

## Response snapshot

I have created the implementation plan for the 'Physical AI & Humanoid Robotics' book.
The following artifacts have been created or updated in `specs/001-physical-ai-book/`:
- `plan.md`
- `research.md`
- `data-model.md`
- `contracts/validation_contracts.md`
- `quickstart.md`
The agent context file `GEMINI.md` has also been updated.
You can now proceed to the task creation phase with `/sp.tasks`.

## Outcome

- ✅ Impact: Created a comprehensive implementation plan and all associated design artifacts.
- 🧪 Tests: No tests were run.
- 📁 Files: Created 5 new design artifact files and updated the plan and agent context.
- 🔁 Next prompts: User can now proceed with `/sp.tasks`.
- 🧠 Reflection: The planning process was successful in translating the high-level project description into a concrete set of design documents.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
