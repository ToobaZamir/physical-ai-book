# Chapter 2.1: Gazebo 2025 (Ignition Citadel + Classic Harmonic)

## 2.1.1 Introduction to Robotic Simulation

Robotic simulation is an indispensable tool in modern robotics development, offering a safe, cost-effective, and efficient environment for testing algorithms, prototyping designs, and training AI models. Before deploying code to expensive and delicate physical hardware, simulation allows developers to iterate rapidly, debug complex interactions, and gather vast amounts of data under controlled conditions. Gazebo has long been a cornerstone of this practice, evolving to meet the growing demands of the robotics community.

## 2.1.2 The Evolution of Gazebo: Classic vs. Ignition

Gazebo, an acronym for "Gazebo: A Better Simulator for ROS," has undergone significant architectural shifts to accommodate the evolving needs of robotics. Its journey has led to two primary branches coexisting in 2025: Gazebo Classic (now maintained by Open Robotics) and Gazebo Ignition (part of the Gazebo Sim project, also by Open Robotics). While both aim to provide high-fidelity robot simulation, they differ substantially in their underlying architecture, performance characteristics, and target use cases.

### Gazebo Classic (Harmonic)

Gazebo Classic, often referred to by its melodic names like Harmonic, continues to be a robust and widely-used simulator, particularly for legacy ROS 1 and many ROS 2 projects. Its key strengths lie in its maturity, extensive community support, and a vast library of pre-existing robot models and environments.

#### Features and Strengths:
*   **Maturity and Stability:** Decades of development have refined its physics engine, rendering, and sensor models.
*   **Extensive Model Database:** A large online repository of robots, objects, and environments readily available.
*   **ROS 1 and ROS 2 Integration:** Deeply integrated with the ROS ecosystem through `gazebo_ros` packages.

#### Basic Simulation Concepts:
Gazebo Classic simulations are built around worlds, which contain models (robots, objects), and various sensors (cameras, LiDAR, IMUs) that interact within a physics engine.

**Example: Simple SDF Model for Gazebo Classic**
The following snippet illustrates a basic model definition in SDF (Simulation Description Format), a common way to describe objects in Gazebo Classic.

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="simple_box">
    <link name="box_link">
      <pose>0 0 0.5 0 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.166667</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.166667</iyy>
          <iyz>0</iyz>
          <izz>0.166667</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry><box><size>1 1 1</size></box></geometry>
        <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Red</name></script></material>
      </visual>
      <collision name="collision">
        <geometry><box><size>1 1 1</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
```
*Description:* This SDF defines a simple red cube with a size of 1x1x1 meter, including its visual and collision properties. Such models are fundamental building blocks for any Gazebo Classic world.

[QR Code: Link to Gazebo Classic SDF Tutorial]

### Gazebo Ignition (Citadel)

Gazebo Ignition, with releases like Citadel, represents a re-architected, modular, and distributed simulation platform designed for scalability and performance. It aims to overcome some of the limitations of Gazebo Classic by adopting a component-based design.

#### Key Improvements and Architecture:
Ignition's architecture is built on a set of loosely coupled libraries (e.g., Ignition Physics, Ignition Rendering, Ignition Transport). This modularity allows developers to swap out components (e.g., different physics engines) and use parts of the simulator independently.

**Diagram Placeholder: Gazebo Ignition Architecture Overview**
*(A block diagram illustrating the modular components of Gazebo Ignition: Ignition GUI, Rendering, Physics, Transport, Math, etc., and how they interact.)*

#### Differentiations from Classic:
*   **Modular Design:** Easier to extend and integrate with external tools.
*   **Distributed Simulation:** Supports running different components on separate machines.
*   **Modern Rendering:** Utilizes Ogre 3D and offers improved visual fidelity.
*   **First-Class ROS 2 Support:** Designed with ROS 2 in mind, offering a more native integration.

**Example: Basic Ignition World (Python)**
While Ignition supports SDF, it also allows for Python-based world creation through its API. This conceptual snippet shows how one might define a simple world.

```python
# Conceptual Python code for Ignition world setup (actual API calls might differ slightly)
from ignition.gazebo import World, Model, BoxShape

# Initialize a new world
world = World("simple_ignition_world")

# Create a ground plane
world.add_model(Model("ground_plane").add_geometry(BoxShape(size=[100, 100, 0.01])))

# Add a simple box
box_model = Model("my_box")
box_model.set_pose([0, 0, 0.5])
box_model.add_visual(BoxShape(size=[1, 1, 1], color=[1, 0, 0, 1])) # Red box
box_model.add_collision(BoxShape(size=[1, 1, 1]))
world.add_model(box_model)

# Simulate for a few steps
# world.run(iterations=1000)
print("Ignition world configured conceptually.")
```
*Description:* This conceptual Python code demonstrates how one could programmatically build a world in Gazebo Ignition, adding a ground plane and a box model.

[QR Code: Link to Gazebo Ignition API Documentation]

## 2.1.3 Choosing the Right Gazebo for Your Project

The decision between Gazebo Classic and Gazebo Ignition often depends on project requirements, existing infrastructure, and desired features.

| Feature / Aspect       | Gazebo Classic (Harmonic)                                  | Gazebo Ignition (Citadel)                                        |
| :--------------------- | :--------------------------------------------------------- | :--------------------------------------------------------------- |
| **Maturity**           | High (Stable, battle-tested)                               | Moderate (Actively developing, but robust)                       |
| **Architecture**       | Monolithic (Single process)                                | Modular, Distributed (Independent libraries)                     |
| **Rendering**          | Ogre 1.x (Functional)                                      | Ogre 2.x, PBR (Physically Based Rendering) support               |
| **ROS Integration**    | `gazebo_ros` (ROS 1 & ROS 2 compatible via bridges)        | Native `ros_gz_sim` (First-class ROS 2 support)                  |
| **Extensibility**      | Plugins (C++)                                              | Plugins (C++, Python API for components)                         |
| **Community / Models** | Very large, extensive existing model database              | Growing, still building its asset base                             |
| **Use Cases**          | Established projects, educational, quick prototyping       | High-performance, distributed, custom component integration, new projects |

For new projects targeting modern ROS 2 features, especially those requiring high visual fidelity, custom physics, or distributed simulation, Gazebo Ignition is generally the recommended choice. For projects with legacy ROS 1 codebases or those prioritizing stability and a vast existing model library, Gazebo Classic remains a viable option.

## 2.1.4 Advanced Simulation Techniques (Briefly)

Beyond basic world and model creation, both Gazebo Classic and Ignition offer advanced features:
*   **Plugins:** Custom C++ code to extend simulator functionality (e.g., custom sensors, actuators, control algorithms).
*   **Scripting:** Python scripting for manipulating simulations, data logging, and automated testing.
*   **Custom Sensors:** Designing and integrating novel sensor models for specific research needs.

## 2.1.5 Conclusion

Gazebo, in its Classic and Ignition forms, provides powerful platforms for robotic simulation. Understanding their respective strengths and architectural nuances is key to selecting and utilizing the appropriate tool for your development needs. As robotics continues its rapid advancement, these simulators will remain at the forefront, enabling the safe and efficient realization of complex robotic systems.
