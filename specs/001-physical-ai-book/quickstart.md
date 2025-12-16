# Quickstart: Physical AI & Humanoid Robotics Book

This guide provides the essential steps to set up your development environment and get started with the code examples in the book.

## 1. System Requirements

-   **OS**: Ubuntu 22.04 LTS (clean installation recommended).
-   **GPU**: NVIDIA RTX 3070 or higher (RTX 4080 recommended for best performance).
-   **CPU**: 8-core processor or better.
-   **RAM**: 32 GB or more.
-   **Storage**: 100 GB of free space.

## 2. Environment Setup

### 2.1. NVIDIA Drivers

Install the latest NVIDIA drivers for your GPU.

```bash
sudo ubuntu-drivers autoinstall
sudo reboot
```

### 2.2. ROS 2 Jazzy

Install ROS 2 Jazzy Jalisco by following the official documentation.

```bash
# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt upgrade
sudo apt install ros-dev-tools ros-jazzy-desktop
```

### 2.3. NVIDIA Isaac Sim

1.  Install the Omniverse Launcher.
2.  From the Omniverse Launcher, install Isaac Sim version 2025.2.
3.  Follow the Isaac Sim documentation to set up the ROS 2 Humble bridge.

## 3. Repository Setup

Clone the book's GitHub repository.

```bash
git clone <repository-url>
cd <repository-directory>
```

## 4. Running Your First Example

1.  Navigate to the code directory for the first chapter.

    ```bash
    cd code/chapter-1/ros2_ws
    ```

2.  Build the ROS 2 workspace.

    ```bash
    colcon build --symlink-install
    ```

3.  Source the workspace and run the launch file.

    ```bash
    source install/setup.bash
    ros2 launch <package_name> <launch_file.py>
    ```

You should now see the example running in your terminal and in RViz2.
