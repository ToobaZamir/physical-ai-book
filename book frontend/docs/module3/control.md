# Chapter 3.3: Bipedal Locomotion & Whole-Body Control (MPC + RL baselines)

## 3.3.1 Introduction: The Grand Challenge of Humanoid Control

Controlling a bipedal robot to walk, run, jump, and interact with its environment gracefully and robustly is arguably one of the most complex challenges in robotics. Humanoids are inherently unstable systems, constantly battling gravity while requiring a vast range of motion and dynamic stability. This chapter delves into the advanced control methodologies that enable such feats, focusing on Model Predictive Control (MPC) and Reinforcement Learning (RL) as primary tools for achieving both stable locomotion and versatile whole-body control.

## 3.3.2 Foundations of Bipedal Locomotion

At the heart of bipedal locomotion control lie fundamental concepts that quantify and manage the robot's dynamic stability.

### Zero Moment Point (ZMP) and Capture Point (CP)
*   **Zero Moment Point (ZMP):** A critical concept that defines the point on the ground where the net moment of all forces (gravity, inertia) acting on the robot is zero. For stable walking, the ZMP must remain within the robot's support polygon (the convex hull of its feet on the ground).
*   **Capture Point (CP):** An evolution of ZMP, the CP indicates where the robot's center of pressure needs to be to prevent falling, given its current state. It helps predict and manage stability during dynamic motions.

**Diagram Placeholder: ZMP and Capture Point Concepts in Bipedal Walking**
*(A diagram illustrating a bipedal robot in motion, showing the Center of Mass (CoM) trajectory, the Support Polygon, and the projection of the ZMP and CP onto the ground plane.)*

### Walking Pattern Generation
The process of generating a stable and desired walking motion for a humanoid involves planning footstep locations, body trajectories, and ensuring dynamic balance. This can be achieved through various methods, from pre-defined gait patterns to real-time trajectory optimization.

## 3.3.3 Model Predictive Control (MPC) for Humanoids

Model Predictive Control (MPC) is an optimization-based control strategy that uses a model of the system to predict its future behavior over a finite horizon. It then calculates a sequence of control inputs that minimizes a cost function (e.g., energy consumption, deviation from desired trajectory) while satisfying constraints (e.g., joint limits, ZMP within support polygon).

### MPC Principles:
*   **Predictive Model:** A mathematical representation of the robot's dynamics.
*   **Cost Function:** Defines the control objectives (what to optimize).
*   **Constraints:** Physical limits of the robot and environmental boundaries.
*   **Optimization:** A solver finds the optimal control sequence. Only the first control action is applied, and the process repeats at the next time step (receding horizon).

### Application to Whole-Body Control (WBC)
MPC is particularly powerful for Whole-Body Control (WBC) of humanoids, where it can simultaneously optimize for multiple tasks such as:
*   Maintaining balance and stability.
*   Tracking desired CoM trajectories.
*   Achieving end-effector poses for manipulation.
*   Avoiding joint limits and self-collisions.

**Conceptual Example: MPC Formulation for Humanoid Balance**
```python
# Conceptual pseudo-code for an MPC problem formulation for humanoid balance
class HumanoidMPC:
    def __init__(self, robot_model, dt, prediction_horizon):
        self.model = robot_model # Simplified model of humanoid dynamics
        self.dt = dt
        self.horizon = prediction_horizon
        # Define state (CoM position, velocity, etc.) and control inputs (joint torques/forces)

    def solve_mpc(self, current_state, desired_com_traj, foot_contact_forces):
        """
        Formulates and solves an MPC problem for the humanoid.
        """
        # Define optimization variables (future states and control inputs)
        # Define cost function:
        #   - Penalize deviation from desired CoM trajectory
        #   - Penalize large joint torques/forces
        #   - Penalize ZMP violating support polygon
        # Define constraints:
        #   - Robot dynamics equations
        #   - Joint limits, velocity limits
        #   - Force limits (e.g., maximum ground reaction force)
        #   - Contact constraints (foot on ground)

        # Use an optimization solver (e.g., quadratic programming)
        # optimal_controls = solver.solve(cost_function, constraints)
        
        # Return the first optimal control action
        return optimal_control_action[0]

# In a control loop:
# current_state = get_robot_state()
# desired_trajectory = planner.get_desired_trajectory()
# mpc_solver = HumanoidMPC(...)
# control_action = mpc_solver.solve_mpc(current_state, desired_trajectory, contact_info)
# apply_control_action(control_action)
```
*Description:* This conceptual class illustrates how an MPC problem for a humanoid robot would be formulated, defining a cost function and constraints to achieve stable and goal-oriented movements.

## 3.3.4 Reinforcement Learning (RL) Baselines for Locomotion

Reinforcement Learning (RL) has emerged as a powerful paradigm for learning complex behaviors, including bipedal locomotion, often surpassing traditional control methods in adaptability and robustness.

### RL Basics:
*   **Agent:** The humanoid robot learning to walk.
*   **Environment:** The simulation (or real world) where the robot acts.
*   **State:** The robot's current configuration (joint angles, velocities, IMU readings) and often environmental observations.
*   **Action:** The control commands (e.g., joint torques, desired positions) sent to the robot.
*   **Reward:** A scalar signal from the environment indicating how well the agent is performing a desired task (e.g., positive for forward motion, negative for falling).

### Learning Gaits and Policies:
RL algorithms train a neural network (the policy) to map states to optimal actions by maximizing cumulative reward. This can involve:
*   **Learning from Scratch:** Discovering walking gaits purely through trial and error.
*   **Refining Existing Gaits:** Improving the robustness or efficiency of pre-designed gaits.

### Common RL Algorithms (Conceptual):
*   **Proximal Policy Optimization (PPO):** A popular policy gradient method known for its stability and good performance in robotics tasks.
*   **Soft Actor-Critic (SAC):** An off-policy algorithm that optimizes a stochastic policy, often achieving state-of-the-art results in continuous control.

### Sim-to-Real Transfer for RL Policies:
A key challenge for RL is transferring policies learned in simulation to the real world. Techniques like **domain randomization** (varying simulation parameters during training) help make policies more robust to real-world discrepancies.

[QR Code: Link to a humanoid locomotion RL framework, e.g., Isaac Gym or DeepMimic research]

## 3.3.5 Whole-Body Control (WBC) Integration

Modern humanoid control often combines the strengths of various techniques within a Whole-Body Control (WBC) framework. WBC seeks to coordinate all of the robot's degrees of freedom to achieve multiple tasks simultaneously, often prioritizing them.

### Hierarchical Control Structures:
Control systems for humanoids are frequently hierarchical:
*   **High-Level:** Task planning, gait generation (e.g., footstep placement).
*   **Mid-Level:** MPC or RL policies generating CoM trajectories and desired contact forces.
*   **Low-Level:** Joint-level controllers translating desired torques/positions into motor commands.

**Diagram Placeholder: Hierarchical Humanoid Control Architecture**
*(A layered diagram showing high-level planners, mid-level whole-body controllers (MPC/RL), and low-level joint controllers.)*

### Task Prioritization:
WBC allows for dynamic prioritization of tasks. For example, maintaining balance might have higher priority than achieving a precise end-effector position.

## 3.3.6 Conclusion

Bipedal locomotion and whole-body control are at the forefront of humanoid robotics research. The combination of model-based approaches like MPC with data-driven methods like RL offers powerful solutions for generating robust, agile, and adaptable movements. As these techniques mature, we move closer to a future where humanoids can navigate and interact with the world with unprecedented dexterity and intelligence.
