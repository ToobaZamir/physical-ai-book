# Chapter 1.2: Core Architecture Deep Dive

Understanding the core architecture of ROS 2 is essential for building reliable, scalable, and production-grade robotic systems. Unlike ROS 1, which relied on a centralized master, ROS 2 adopts a fully distributed design that is better suited for modern robotics applications such as humanoids, autonomous vehicles, and multi-robot fleets.

This chapter provides a deep dive into the fundamental architectural components of ROS 2 and explains how they work together to form a robust robotics middleware.

---

## Distributed System Design

ROS 2 is built as a **distributed system**, meaning there is no single point of control or failure. Each component operates independently and communicates over standardized interfaces.

Key benefits of this approach include:

- Improved fault tolerance  
- Easier scaling across multiple machines  
- Native support for real-time systems  
- Seamless deployment on embedded, desktop, and cloud platforms  

This design makes ROS 2 suitable not only for research but also for industrial and commercial robotics.

---

## Nodes: The Building Blocks

A **node** is the fundamental executable unit in ROS 2. Each node performs a specific task within the system.

Examples of node responsibilities include:

- Reading sensor data  
- Controlling actuators  
- Performing perception or planning  
- Managing system state  

Nodes are intentionally kept small and focused, which improves modularity and maintainability.

ROS 2 nodes can be implemented in:
- Python (rclpy)
- C++ (rclcpp)

This flexibility allows developers to balance ease of development with performance requirements.

---

## Topics: Asynchronous Communication

**Topics** enable asynchronous, many-to-many communication between nodes using a publish–subscribe model.

Typical use cases for topics include:

- Camera images  
- LiDAR scans  
- Joint states  
- Robot pose and odometry  

Publishers send messages to a named topic, and subscribers receive messages without needing direct knowledge of each other. This loose coupling is critical for scalable systems.

ROS 2 enhances topic communication through **Quality of Service (QoS)** settings, allowing developers to control reliability, latency, and data delivery behavior—an essential feature for real-time humanoid control.

---

## Services: Request–Response Interaction

**Services** provide synchronous communication between nodes using a request–response pattern.

They are best suited for:

- Configuration changes  
- One-time commands  
- System status queries  

For example, a node may request another node to reset a sensor or reload parameters. Unlike topics, services block until a response is received, making them unsuitable for continuous data flow.

---

## Actions: Long-Running Tasks

**Actions** are designed for tasks that take time to complete and require feedback.

They are commonly used for:

- Navigation goals  
- Manipulation tasks  
- Walking or motion execution in humanoids  

Actions support:
- Goal submission  
- Continuous feedback  
- Cancellation  
- Final result reporting  

This makes them ideal for high-level task execution and behavior coordination.

---

## DDS: The Middleware Backbone

At the core of ROS 2 lies **DDS (Data Distribution Service)**, an industry-standard middleware used in mission-critical systems.

DDS is responsible for:

- Automatic discovery of nodes  
- Message serialization and transport  
- Enforcing Quality of Service policies  
- Reliable real-time communication  

Because ROS 2 is built on DDS, it inherits proven capabilities used in aerospace, defense, and autonomous systems.

---

## Parameters: Runtime Configuration

**Parameters** allow nodes to expose configurable values that can be changed at runtime.

Common examples include:

- Controller gains  
- Sensor thresholds  
- File paths  
- Operating modes  

This enables dynamic tuning and system-wide introspection without recompiling code.

---

## Lifecycle Nodes: Controlled Execution

ROS 2 introduces **lifecycle nodes**, which follow a managed state machine.

Typical states include:

- Unconfigured  
- Inactive  
- Active  
- Finalized  

Lifecycle nodes allow deterministic startup and shutdown behavior, which is crucial for complex humanoid robots where components must be initialized in a strict order.

---

## Architectural Summary

The ROS 2 architecture is designed to support modern robotic systems by offering:

- Modular node-based design  
- Flexible communication patterns  
- Real-time capable middleware  
- Strong scalability and reliability  

Mastering these architectural concepts provides the foundation needed to build advanced humanoid and Physical AI systems.


