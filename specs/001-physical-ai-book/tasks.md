# Tasks: Physical AI & Humanoid Robotics Book

**Input**: Design documents from `specs/001-physical-ai-book/`
**Prerequisites**: plan.md, spec.md

## Phase 1: Module 1 – ROS 2 Mastery (Weeks 01-04 → Live by 12 Jan 2026)

**Goal**: The entire first module is live on the Docusaurus site, with all code examples working and tested.

- [x] T101 [US1] Write and publish Chapter 1.1: Why ROS 2 Is the Only Serious Choice in 2025 in `docs/module1/why-ros2.md`
- [x] T102 [US1] Write and publish Chapter 1.2: Core Architecture Deep Dive in `docs/module1/core-architecture.md`
- [x] T103 [US1] Write and publish Chapter 1.3: Building Production-Grade Humanoid Packages in `docs/module1/humanoid-packages.md`
- [ ] T104 [US1] Create 15+ fully working ROS 2 example packages (rclpy + rclcpp) in `code/module1/`
- [ ] T105 [P] [US1] Record and upload 12 demo videos (1440p) and link them in the chapters.
- [x] T106 [US1] Configure Docusaurus sidebars for Module 1.
- [ ] T107 [US1] Implement CI checks for Module 1 code (`colcon build` + `colcon test`).

## Phase 2: Module 2 – Digital Twins (Weeks 05-08 → Live by 09 Feb 2026)

**Goal**: The second module is live, with all simulation assets and examples working.

- [x] T201 [US2] Write and publish Chapter 2.1: Gazebo 2025 (Ignition Citadel + Classic Harmonic) in `docs/module2/gazebo.md`
- [x] T202 [US2] Write and publish Chapter 2.2: NVIDIA Isaac Sim 2025.2 – Full USD Workflow in `docs/module2/isaac-sim.md`
- [x] T203 [US2] Write and publish Chapter 2.3: Unity Robotics + Digital Humans in `docs/module2/unity.md`
- [x] T204 [US2] Publish complete Unitree G1 USD + SDF + URDF in `assets/robots/unitree_g1/`
- [ ] T205 [P] [US2] Create 20+ Isaac Sim standalone examples in `code/module2/isaac_sim_examples/`
- [ ] T206 [US2] Achieve and document first sim-to-real walking success on real Unitree Go2 in `docs/module2/sim-to-real.md`
- [ ] T207 [US2] Implement CI checks for Isaac Sim examples (headless via Docker).
- [x] T208 [P] [US2] Develop and deploy WebRTC live streaming demo page for Isaac Sim.

## Phase 3: Module 3 – The AI-Robot Brain (Weeks 09-12 → Live by 09 Mar 2026)

**Goal**: The third module is live, with a fully functional perception stack on real hardware.

- [x] T301 [US3] Write and publish Chapter 3.1: Isaac ROS GEMs + hardware-accelerated pipelines in `docs/module3/isaac-ros-gems.md`
- [x] T302 [US3] Write and publish Chapter 3.2: End-to-End Humanoid Perception (people, hands, objects) in `docs/module3/perception.md`
- [x] T303 [US3] Write and publish Chapter 3.3: Bipedal Locomotion & Whole-Body Control (MPC + RL baselines) in `docs/module3/control.md`
- [ ] T304 [US3] Deploy and validate full perception stack on 5 real Jetson Orin NX units.
- [x] T305 [P] [US3] Release pretrained checkpoints for Detectron2, OWL-ViT, GraspNet in `assets/models/`
- [x] T306 [P] [US3] Develop and deploy live dashboard showing real-time perception from lab humanoids.

## Phase 4: Module 4 + Capstone – Vision-Language-Action & Full Autonomy (Weeks 13-16 → Live by 31 Mar 2026)

**Goal**: The final module and capstone project are complete, live, and fully functional in both sim and real.

- [x] T401 [US4] Write and publish Chapter 4.1: Voice → LLM → Task Plan → ROS 2 Actions in `docs/module4/vla-planning.md`
- [x] T402 [US4] Write and publish Chapter 4.2: Language-Grounded Perception & Manipulation in `docs/module4/vla-grounding.md`
- [x] T403 [US4] Write and publish Chapter 4.3: 60-page Capstone – “Autonomous Apartment Humanoid” in `docs/capstone/`
- [ ] T404 [US4] Develop and test full end-to-end capstone repo in `code/capstone/`
- [ ] T405 [US4] Execute and film complete “Bring me a Coke” mission on real Unitree G1.
- [x] T406 [US4] Set up CI to generate final PDF + ePub deliverables.
- [x] T407 [US4] Create and populate Appendices A–E in `docs/appendices/`
- [x] T408 [P] [US4] Implement live-updating price scraper for hardware BOM in Appendix A.

## Post-Launch Continuous Phase (April 2026 onward)

- [x] T501 [P] Set up repository for community contributions (issue templates, PR workflow).
- [x] T502 [P] Establish process for weekly bug-fix PR reviews.
