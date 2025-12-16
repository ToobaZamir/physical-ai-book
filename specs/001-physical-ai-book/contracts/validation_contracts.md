# Validation Contracts: Physical AI & Humanoid Robotics Book

This document defines the formal contracts for quality assurance, testing, and acceptance for the project. These contracts are enforced by the CI/CD pipeline.

## 1. Per-Chapter Validation Contract (CI on PR)

This contract is triggered on every pull request against the `main` branch.

-   **Pre-condition**: A pull request is opened or updated.
-   **Execution Environment**:
    -   Cloud workstation (GitHub Actions).
    -   OS: Ubuntu 22.04 (clean install).
    -   Software: ROS 2 Jazzy, NVIDIA Isaac Sim 2025.2.
-   **Validation Steps**:
    1.  `colcon build --symlink-install` must execute successfully in all `code/<chapter>/ros2_ws` directories modified in the PR.
    2.  All `ros2 launch` commands present in the chapter's markdown source must be extracted and executed.
-   **Post-condition (Success Criteria)**:
    -   The `colcon build` must exit with code 0.
    -   No `ros2 launch` command may take longer than 20 seconds to reach a stable state (i.e., all nodes running).
    -   No launched node may exit with an error code.
-   **Outcome**:
    -   **On Success**: The PR is marked as "checks passed".
    -   **On Failure**: The PR is blocked from merging.

## 2. Full Book Integration Contract (Nightly)

This contract is triggered nightly on the `main` branch.

-   **Pre-condition**: Scheduled time (e.g., midnight).
-   **Execution Environment**:
    -   Physical testbed with NVIDIA Jetson Orin NX.
    -   Robot: Unitree Go2 (for locomotion) and G1 (for final capstone).
    -   Network: SSH access to the testbed.
-   **Validation Steps**:
    1.  Deploy the latest `main` branch version of the `capstone` repository to the Jetson.
    2.  Execute the full "Bring me a Coke" scenario via an automated script.
    3.  Record a video of the robot's performance.
    4.  Upload the video to the designated storage.
    5.  Update a hash file or manifest with the new video's identifier.
-   **Post-condition (Success Criteria)**:
    -   The capstone scenario must complete successfully.
    -   The video must be successfully recorded and uploaded.
-   **Outcome**:
    -   **On Success**: A success notification is sent to the development team.
    -   **On Failure**: An alert is raised for manual inspection.

## 3. Release Gate Contract (Manual Trigger)

This contract is triggered manually before tagging a new release.

-   **Pre-condition**: A release candidate commit is identified.
-   **Execution Environment**:
    -   Cloud workstation (GitHub Actions).
-   **Validation Steps**:
    1.  **Generate Deliverables**: Run the Pandoc script to generate the final PDF and ePub files from the book source.
    2.  **Accessibility Check**: Run a PDF/UA accessibility checker on the generated PDF.
    3.  **Link Check**: Run a script to crawl all external URLs and internal QR code links from the source files.
    4.  **Price Scraper**: Run a script to check the prices in the Bill of Materials (BOM) appendix against the linked vendor websites.
-   **Post-condition (Success Criteria)**:
    -   The PDF and ePub must be generated without errors.
    -   The PDF must pass the accessibility check with zero errors.
    -   The link checker must report zero 404 errors.
    -   The price scraper must report that all prices are within a 10% tolerance of the values in the BOM.
-   **Outcome**:
    -   **On Success**: The release is cleared for tagging and publishing.
    -   **On Failure**: The release is blocked, and a report of the failures is generated.
