---
id: humanoid-control
title: Control
sidebar_label: Control
sidebar_position: 2
---

## Understanding Humanoid Robot Control {#ch6-sec2-understanding-control}

**Humanoid robot control** involves the algorithms and systems that enable bipedal robots to maintain balance, execute movements, and interact with their environment. Control systems must handle the complex dynamics of walking, running, and manipulation while ensuring stability and safety.

### Key Control Concepts: {#ch6-sec2-control-concepts}

1.  **Balance Control**: Maintaining upright posture using feedback from IMU and force sensors
2.  **Walking Pattern Generation**: Creating stable gait patterns for locomotion
3.  **Whole-Body Control**: Coordinating multiple joints for complex tasks
4.  **Compliance Control**: Adjusting stiffness for safe human interaction
5.  **State Estimation**: Determining robot pose and velocity from sensor data
6.  **Trajectory Tracking**: Following planned paths with minimal error

### Balance Control System: {#ch6-sec2-balance-control}

```python
import rclpy
from rclpy.node import Node
import numpy as np
from dataclasses import dataclass

@dataclass
class RobotState:
    com_pos: np.ndarray
    com_vel: np.ndarray
    feet_pos: dict
    feet_force: dict

class BalanceController(Node):
    def __init__(self):
        super().__init__("balance_controller")

        # Control parameters
        self.dt = 0.01
        self.kp, self.kd, self.ki = 0.8, 0.2, 0.05
        self.zmp_int = np.zeros(2)
        self.last_err = np.zeros(2)

        # Timer loop
        self.create_timer(self.dt, self.loop)
        self.get_logger().info("Balance Controller Ready")

    # ------------------ STATE ------------------
    def get_state(self) -> RobotState:
        return RobotState(
            com_pos=np.array([0., 0., 0.85]),
            com_vel=np.zeros(3),
            feet_pos={
                "l": np.array([0.1, 0.15, 0]),
                "r": np.array([0.1, -0.15, 0])
            },
            feet_force={
                "l": np.array([0, 0, 250]),
                "r": np.array([0, 0, 250])
            }
        )

    # ------------------ ZMP ------------------
    def desired_zmp(self, s) -> np.ndarray:
        return (s.feet_pos['l'][:2] + s.feet_pos['r'][:2]) / 2

    def current_zmp(self, s) -> np.ndarray:
        fl, fr = s.feet_force['l'][2], s.feet_force['r'][2]
        if fl + fr == 0:
            return np.zeros(2)
        return (s.feet_pos['l'][:2] * fl + s.feet_pos['r'][:2] * fr) / (fl + fr)

    # ------------------ CONTROL ------------------
    def zmp_control(self, err: np.ndarray) -> np.ndarray:
        p = self.kp * err
        d = self.kd * (err - self.last_err) / self.dt
        self.zmp_int += err * self.dt
        i = self.ki * self.zmp_int
        self.last_err = err.copy()
        return p + d + i

    def torques(self, adj: np.ndarray) -> dict:
        M, g = 50, 9.81
        moment = M * g * adj
        return {
            f"{lr}_{j}_{ax}": moment[i] * s
            for i, ax in enumerate(["pitch", "roll"])
            for lr in ["l", "r"]
            for j, s in [("ankle", 0.5), ("hip", 0.3)]
        }

    # ------------------ FALL PREVENTION ------------------
    def capture_point(self, s) -> np.ndarray:
        h = s.com_pos[2]
        omega = np.sqrt(9.81 / h)
        cp = s.com_pos[:2] + s.com_vel[:2] / omega
        return cp - self.desired_zmp(s)

    # ------------------ MAIN LOOP ------------------
    def loop(self):
        s = self.get_state()
        err = self.desired_zmp(s) - self.current_zmp(s)

        adj = self.zmp_control(err)
        tau = self.torques(adj)

        # Send (simulated)
        for j, t in tau.items():
            _ = np.clip(t, -100, 100) # torque limits

def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
```

### Walking Pattern Generation: {#ch6-sec2-walking-pattern}

```python
class WalkingPatternGenerator:
    """Generate walking patterns for humanoid robots."""
    
    def __init__(self, step_length=0.3, step_width=0.2, step_height=0.05):
        self.step_length = step_length
        self.step_width = step_width
        self.step_height = step_height
        
        # Walking parameters
        self.step_time = 0.8  # seconds per step
        self.double_support_ratio = 0.2  # 20% double support
        
    def generate_trajectory(self, num_steps=4, direction='forward'):
        """Generate foot trajectory for walking."""
        trajectories = {'left': [], 'right': []}
        
        # Initial positions
        left_pos = np.array([0.0, self.step_width/2, 0.0])
        right_pos = np.array([0.0, -self.step_width/2, 0.0])
        
        for step in range(num_steps):
            # Determine which foot moves
            if step % 2 == 0:
                moving_foot = 'right' if direction == 'forward' else 'left'
                supporting_foot = 'left' if direction == 'forward' else 'right'
            else:
                moving_foot = 'left' if direction == 'forward' else 'right'
                supporting_foot = 'right' if direction == 'forward' else 'left'
            
            # Generate swing trajectory
            swing_traj = self.generate_swing_trajectory(
                start_pos=eval(f'{moving_foot}_pos'),
                step_num=step,
                direction=direction
            )
            
            trajectories[moving_foot].extend(swing_traj)
            
            # Update positions
            if moving_foot == 'left':
                left_pos[0] += self.step_length if direction == 'forward' else -self.step_length
            else:
                right_pos[0] += self.step_length if direction == 'forward' else -self.step_length
        
        return trajectories
    
    def generate_swing_trajectory(self, start_pos, step_num, direction):
        """Generate swing foot trajectory."""
        num_points = int(self.step_time / 0.01)  # 10ms resolution
        trajectory = []
        
        for i in range(num_points):
            t = i / num_points
            
            # Swing phase: 0-0.8 (80% of step time)
            # Lift, move forward, lower
            if t < 0.4:  # Lifting
                z = start_pos[2] + self.step_height * (t / 0.4)
            elif t < 0.8:  # Lowering
                z = start_pos[2] + self.step_height * (1 - (t - 0.4) / 0.4)
            else:  # Double support
                z = start_pos[2]
            
            # Forward motion
            x = start_pos[0] + self.step_length * t if direction == 'forward' else start_pos[0] - self.step_length * t
            
            # Lateral motion (for stepping sideways)
            y = start_pos[1]
            
            trajectory.append(np.array([x, y, z]))
        
        return trajectory
```

### Whole-Body Control: {#ch6-sec2-whole-body}

```python
class WholeBodyController:
    """Whole-body control with task prioritization."""
    
    def __init__(self, num_joints=32):
        self.num_joints = num_joints
        
        # Task priorities
        self.tasks = {
            'balance': 0,      # Highest priority
            'foot_tracking': 1,
            'com_tracking': 2,
            'posture': 3       # Lowest priority
        }
    
    def solve(self, tasks):
        """Solve whole-body control with task prioritization."""
        # Use hierarchical quadratic programming
        # τ = J₁⁺F₁ + (I - J₁⁺J₁)J₂⁺F₂ + ...
        
        solution = np.zeros(self.num_joints)
        
        for task_name in sorted(self.tasks, key=self.tasks.get):
            if task_name in tasks:
                task = tasks[task_name]
                solution = self.add_task(solution, task)
        
        return solution
```

### Key Control Algorithms: {#ch6-sec2-key-algorithms}

*   **Zero Moment Point (ZMP)**: Traditional balance control method
*   **Capture Point**: Fall prevention and recovery
*   **Model Predictive Control (MPC)**: Optimal control with constraints
*   **Inverse Dynamics**: Computing required joint torques
*   **Virtual Model Control**: Simulating virtual components
*   **Impedance Control**: Regulating stiffness and damping

### Control System Architecture: {#ch6-sec2-architecture}

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   High-Level    │    │   Mid-Level     │    │   Low-Level     │
│     Control     │───▶     Control      ───▶     Control      │
│  (Task Planning)│    │ (Motion Gen)    │    │  (Joint Ctrl)   │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   State Est.    │    │   Dynamics      │    │   Motor Drivers │
│   (Filtering)   │    │   (Model)       │    │   (Hardware)    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### Best Practices: {#ch6-sec2-best-practices}

1.  **Start Simple**: Begin with static balance before dynamic walking
2.  **Use Simulation**: Test control algorithms in Gazebo/Isaac Sim first
3.  **Safety First**: Implement torque limits and emergency stops
4.  **Parameter Tuning**: Systematically tune control gains
5.  **Real-time Performance**: Ensure control loop meets timing requirements
6.  **Robust Design**: Handle sensor noise and model uncertainties

### Common Challenges: {#ch6-sec2-challenges}

*   **Model Uncertainty**: Inaccurate dynamics modeling
*   **Sensor Noise**: Filtering IMU and force sensor data
*   **Computational Load**: Real-time optimization requirements
*   **Actuator Limitations**: Saturation and bandwidth constraints
*   **Ground Variations**: Uneven terrain and slipping
*   **External Disturbances**: Pushes and environmental interactions

### References and Further Reading: {#ch6-sec2-references}

*   **Book**: *[Introduction to Humanoid Robotics](https://link.springer.com/book/10.1007/978-3-642-54536-8)* by Shuuji Kajita
*   **Paper**: *[Biped Walking Pattern Generation](https://ieeexplore.ieee.org/document/1302386)* by Kajita et al.
*   **Library**: [Open Dynamics Engine (ODE)](http://www.ode.org/) for dynamics simulation
*   **Framework**: [ROS Control](http://wiki.ros.org/ros_control) for hardware interface
*   **Tool**: [Gazebo](http://gazebosim.org/) for control testing
*   **Course**: [Underactuated Robotics](https://underactuated.csail.mit.edu/) by Russ Tedrake
