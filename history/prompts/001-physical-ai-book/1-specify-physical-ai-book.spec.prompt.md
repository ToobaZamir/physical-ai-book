---
id: 1
title: Specify Physical AI Book
stage: spec
date: 2025-12-12
surface: agent
model: Gemini
feature: 001-physical-ai-book
branch: 001-physical-ai-book
user: '[USER]'
command: /sp.specify
labels: [spec, documentation]
links:
  spec: specs/001-physical-ai-book/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-book/spec.md
  - specs/001-physical-ai-book/checklists/requirements.md
tests:
  - none
---

## Prompt

/sp.specify Project: Physical AI & Humanoid Robotics – The Complete Practitioner’s Book Target length: ~280–320 pages (including code, diagrams, photos, QR codes to videos) Audience: Advanced undergraduate / graduate students, AI engineers entering robotics, robotics researchers wanting modern embodied AI stack Tone: Authoritative, precise, hands-on, zero fluff Visual style: Full-color, heavy use of annotated diagrams, real photos, ROS 2 rviz/Gazebo screenshots, Isaac Sim renders, and QR codes linking to 4K demo videos Code style: All code in Python 3.10+, ROS 2 Humble/Iron/Jazzy, formatted with black, tested on Ubuntu 22.04 Module & Chapter Structure (exactly 4 modules, 10 chapters total) Module 1: The Robotic Nervous System – ROS 2 Mastery (Weeks 1–5) Chapter 1.1 – Why ROS 2 Is the Only Serious Choice in 2025 DDS, real-time, safety certification, comparison with ROS 1 and alternatives Chapter 1.2 – Core Architecture Deep Dive Nodes, topics, services, actions, parameters, lifecycle nodes, managed nodes Chapter 1.3 – Building Production-Grade Humanoid Packages from Scratch Workspace layout, colcon, Python rclpy, C++ rclcpp, URDF/Xacro for 22+ DoF humanoids Module 2: Digital Twins – Simulation That Transfers to Reality (Weeks 6–8) Chapter 2.1 – Gazebo 2025: Ignition vs Classic Physics engines (DART vs Bullet), SDF vs URDF, plugins, sensor noise models Chapter 2.2 – NVIDIA Isaac Sim – The New Industry Standard Omniverse USD workflow, PhysX 5 GPU, domain randomization, synthetic data pipeline Chapter 2.3 – Unity for Human-Robot Interaction and High-Fidelity Rendering Unity Robotics Hub, ML-Agents, real-time digital humans, VR teleoperation bridge Module 3: The AI-Robot Brain – NVIDIA Isaac Platform & Perception (Weeks 8–11) Chapter 3.1 – Isaac ROS – GEMs, Hardware-Accelerated Perception Visual/Voxel/3D SLAM, AprilTag, DNN inference on Jetson, CUDA graphs Chapter 3.2 – End-to-End Perception Pipelines for Humanoids People detection (Detectron2 → OpenPIFPAF), hand keypoints), semantic segmentation, depth estimation Chapter 3.3 – Bipedal Locomotion & Whole-Body Control Nav2 for humanoids, model-predictive control, centroidal dynamics, reinforcement learning baselines (Isaac Gym → real Unitree G1) Module 4: Vision-Language-Action (VLA) – Making Humanoids Follow Natural Commands (Weeks 12–13 + Capstone) Chapter 4.1 – From Voice to Symbolic Task Plan Whisper → GPT-4o / LLaMA-3.1 → PDDL / Behavior Trees → ROS 2 action sequence Chapter 4.2 – Grounding Language in Perception and Manipulation CLIP + OWL-ViT + GraspNet, zero-shot object detection, affordance learning, RT-2/RT-X style policies Chapter 4.3 – Capstone Project – “Autonomous Apartment Humanoid” Full 60-page step-by-step guide: full simulated Unitree G1 in Isaac Sim receives voice command “Bring me a Coke from the kitchen”, navigates, opens fridge, grasps, returns. Includes full GitHub repo structure, sim-to-real deployment to real G1/Go2. Mandatory Rules (zero tolerance for violation) - Every command line must be copy-paste runnable and tested on clean Ubuntu 22.04 VM - Every hardware recommendation must include current (Dec 2025) price and purchase link - Every diagram must be original vector (draw.io or Figma source provided) - Every ROS 2 launch file and node shown must include QR code to GitHub repo with exact commit - No placeholder text, no “lorem ipsum” screenshots, no outdated ROS Noetic examples - All external claims (e.g., “Unitree G1 costs $16k”) must have footnote with date-checked source Deliverable format - Single PDF (A4) + ePub - GitHub repository with full source (LaTeX or Markdown → Pandoc), assets, code, and CI that rebuilds the book on every push - ISBN-ready, CC-BY-NC-SA 4.0 license Success = Book can be used as the primary textbook for a 13-week university Physical AI capstone course with zero modifications.

## Response snapshot

I have created the feature specification for the 'Physical AI & Humanoid Robotics' book.
The new feature branch is `001-physical-ai-book`.
The specification file is located at `specs/001-physical-ai-book/spec.md`.
A quality checklist for the specification has been created at `specs/001-physical-ai-book/checklists/requirements.md`.
The specification is complete and has passed all quality checks. You can now proceed to the planning phase with `/sp.plan`.

## Outcome

- ✅ Impact: Created a detailed feature specification for the book project.
- 🧪 Tests: No tests were run.
- 📁 Files: Created the spec file and the requirements checklist.
- 🔁 Next prompts: User can now proceed with `/sp.plan`.
- 🧠 Reflection: The process of creating the spec file was successful, even though the initial script execution had some issues. The recovery process of manually checking the created files and continuing the process worked well.

## Evaluation notes (flywheel)

- Failure modes observed: The `create-new-feature.ps1` script failed to execute correctly when called with a long JSON string, and also failed to create the prompts directory.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Investigate the PowerShell script execution to make it more robust.
