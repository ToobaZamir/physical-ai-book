# Research & Key Decisions: Physical AI & Humanoid Robotics Book

This document summarizes the key architectural and technological decisions made for the project. These decisions are based on the project goals of creating a practical, state-of-the-art educational resource for 2025.

## 1. ROS 2 Distribution

-   **Decision**: ROS 2 Jazzy Jalisco (LTS)
-   **Rationale**: As of late 2025, Jazzy is the only ROS 2 Long-Term Support (LTS) release with official support for NVIDIA's Isaac ROS GEMs, which are critical for the hardware-accelerated perception modules of the book. Its LTS status also guarantees stability until 2029, making it a safe choice for a university course.
-   **Alternatives Considered**:
    -   **ROS 2 Iron Irwini**: While stable, it is not the target platform for NVIDIA's latest tools in 2025 and is not an LTS release.

## 2. Primary Simulation Backbone

-   **Decision**: NVIDIA Isaac Sim 2025.2
-   **Rationale**: Isaac Sim is the only simulator that meets all core requirements: high-fidelity, GPU-accelerated physics (PhysX 5), native support for the Omniverse USD format for photorealistic rendering, and built-in, hardware-accurate sensor simulation (RTX Lidar/Camera). This enables the highest-fidelity sim-to-real transfer.
-   **Alternatives Considered**:
    -   **MuJoCo**: Excellent for fast physics simulation but lacks photorealistic rendering and the comprehensive sensor simulation capabilities of Isaac Sim.
    -   **PyBullet**: A good open-source option, but not as feature-rich or performant for GPU-accelerated simulation as Isaac Sim.

## 3. Primary Humanoid Hardware Platform

-   **Decision**: Unitree G1
-   **Rationale**: The G1 is the only commercially available humanoid robot in the ~$16k price range (as of Dec 2025) that provides a sufficiently open ROS 2 interface to allow for custom controller injection and perception stack integration. This makes it feasible for university labs to acquire and for students to perform meaningful experiments on.
-   **Alternatives Considered**:
    -   **Boston Dynamics Atlas**: Not commercially available.
    -   **Figure 02**: No public SDK or open control interface available as of late 2025.

## 4. VLA Language Model

-   **Decision**: Hybrid approach: LLaMA-3.1-70B (local) for inference and GPT-4o (API) for development.
-   **Rationale**: This hybrid strategy provides the best of both worlds. The locally-run LLaMA-3.1 (quantized to 8-bit) allows the final capstone project to be self-contained and run on the edge (Jetson Orin), which is a key project goal. Using the more powerful GPT-4o during the development phase allows for faster iteration and few-shot refinement of task plans.
-   **Alternatives Considered**:
    -   **GPT-4o only**: Would require a constant internet connection and API key, making it unsuitable for a self-contained, reproducible educational project.
    -   **LLaMA-3.1 only**: Would be more challenging during the initial development and planning phase, where the reasoning capabilities of a frontier model like GPT-4o are beneficial.

## 5. Book & Code License

-   **Decision**: Creative Commons Attribution-NonCommercial-ShareAlike 4.0 (CC-BY-NC-SA 4.0)
-   **Rationale**: This license achieves the project's primary goals: allowing free use, modification, and sharing by students and universities while preventing commercial entities from simply repackaging and selling the content for profit. The "ShareAlike" clause ensures that any derivatives remain open under the same terms.
-   **Alternatives Considered**:
    -   **Fully Open Source (MIT/Apache)**: Would allow commercial exploitation that runs counter to the project's educational-first ethos.
