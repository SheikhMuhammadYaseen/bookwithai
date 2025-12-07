---
id: humanoid-locomotion
title: Locomotion
sidebar_label: Locomotion
sidebar_position: 3
---

## Understanding Humanoid Locomotion {#ch6-sec3-understanding-locomotion}

**Humanoid locomotion** refers to the ability of bipedal robots to walk, run, and navigate through complex environments. Unlike wheeled robots, humanoids must maintain dynamic balance while moving, requiring sophisticated control of multiple degrees of freedom and real-time adaptation to terrain variations.

### Key Locomotion Concepts: {#ch6-sec3-locomotion-concepts}

1.  **Gait Cycle**: The sequence of leg movements in walking/running
2.  **Double Support Phase**: Both feet on ground (stable)
3.  **Single Support Phase**: One foot on ground (dynamic)
4.  **Swing Phase**: Leg moving forward through air
5.  **Stance Phase**: Leg supporting body weight
6.  **Step Length/Width**: Spatial parameters of gait
7.  **Cadence**: Steps per minute

### Gait Generation System: {#ch6-sec3-gait-generation}

```python
import rclpy
from rclpy.node import Node
import numpy as np
from enum import Enum

class Phase(Enum):
    LEFT_SWING = 1
    DOUBLE = 2
    RIGHT_SWING = 3

class MiniLocomotion(Node):
    def __init__(self):
        super().__init__('mini_locomotion')
        # gait params
        self.step_len = 0.3; self.step_w = 0.18; self.step_h = 0.05
        self.step_time = 0.8; self.dt = 0.01
        # state
        self.phase = Phase.DOUBLE; self.progress = 0.0; self.step_count = 0
        self.left_pos = np.array([0., self.step_w/2, 0.])
        self.right_pos = np.array([0., -self.step_w/2, 0.])
        # timer
        self.create_timer(self.dt, self.loop)
        self.get_logger().info("MiniLocomotion ready")

    def loop(self):
        self.progress += self.dt / self.step_time
        if self.progress >= 1.0:
            self.progress = 0.0
            self.step_count += 1
            self.phase = self.next_phase()

        # generate swing for active foot
        if self.phase == Phase.LEFT_SWING:
            swing = self.swing_traj(self.left_pos, +1)
            self.left_pos[0] += self.step_len * self.progress if self.progress==0 else 0
        elif self.phase == Phase.RIGHT_SWING:
            swing = self.swing_traj(self.right_pos, +1)
            self.right_pos[0] += self.step_len * self.progress if self.progress==0 else 0
        else:
            swing = None

        com = self.compute_com_target()
        joints = self.simple_ik(com)
        self.send_commands(joints)

    def next_phase(self):
        seq = [Phase.RIGHT_SWING, Phase.DOUBLE, Phase.LEFT_SWING, Phase.DOUBLE]
        return seq[self.step_count % 4]

    def swing_traj(self, start, forward=1):
        t = self.progress
        # simple bell lift
        z = self.step_h * np.sin(np.pi * min(max(t,0),1))
        x = start[0] + forward * self.step_len * t
        y = start[1]
        return np.array([x, y, z])

    def compute_com_target(self):
        # CoM sits over support (midpoint during double support)
        if self.phase == Phase.LEFT_SWING:
            support = self.right_pos
        elif self.phase == Phase.RIGHT_SWING:
            support = self.left_pos
        else:
            support = (self.left_pos + self.right_pos) / 2
        return support + np.array([0., 0., 0.85])

    def simple_ik(self, com_target):
        # Very simplified: return hip/knee angles that "point" toward COM target
        joints = {}
        for side, foot in (('left', self.left_pos), ('right', self.right_pos)):
            dx = com_target[0] - foot[0]
            dz = com_target[2] - foot[2]
            hip_pitch = np.arctan2(dx, dz)
            knee = max(0.0, np.pi/2 - abs(hip_pitch))
            joints[f'{side}_hip_pitch'] = hip_pitch
            joints[f'{side}_knee'] = knee
        return joints

    def send_commands(self, joints):
        # In real system send to actuators; here we log
        for j, val in joints.items():
            # convert to degrees for readability
            deg = np.degrees(val)
            self.get_logger().debug(f"{j}: {deg:.1f} deg")

def main():
    rclpy.init()
    node = MiniLocomotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

### Dynamic Walking Control: {#ch6-sec3-dynamic-walking}

```python
class DynamicWalker:
    """Dynamic walking with balance control."""
    
    def __init__(self):
        self.com_height = 0.85
        self.pendulum_period = 2 * np.pi * np.sqrt(self.com_height / 9.81)
    
    def calculate_capture_point(self, com_pos, com_vel):
        """Calculate capture point for dynamic walking."""
        omega = np.sqrt(9.81 / self.com_height)
        capture_point = com_pos[:2] + com_vel[:2] / omega
        return capture_point
    
    def adjust_step_timing(self, capture_point, foot_position):
        """Adjust step timing based on capture point."""
        distance = np.linalg.norm(capture_point - foot_position[:2])
        
        # If capture point is far from foot, speed up next step
        if distance > 0.2:  # 20cm threshold
            return 0.6  # Faster step time
        else:
            return 0.8  # Normal step time
```

### Terrain Adaptation: {#ch6-sec3-terrain-adaptation}

```python
class TerrainAdapter:
    """Adapt walking to different terrains."""
    
    def adapt_to_terrain(self, terrain_type, foot_sensors):
        """Adapt gait parameters based on terrain."""
        adaptations = {
            'flat': {'step_height': 0.03, 'stiffness': 1.0},
            'stairs': {'step_height': 0.15, 'stiffness': 1.5},
            'uneven': {'step_height': 0.08, 'stiffness': 0.8},
            'slippery': {'step_height': 0.02, 'stiffness': 0.5}
        }
        
        return adaptations.get(terrain_type, adaptations['flat'])
```

### Locomotion Modes: {#ch6-sec3-locomotion-modes}

| Mode | Speed | Stability | Energy Use |
|------|-------|-----------|------------|
| **Static Walking** | Slow | High | Low |
| **Dynamic Walking** | Medium | Medium | Medium |
| **Running** | Fast | Low | High |
| **Crawling** | Very Slow | Very High | Low |
| **Side-stepping** | Slow | High | Medium |

### Key Algorithms: {#ch6-sec3-key-algorithms}

*   **Inverted Pendulum Model**: Simplified dynamics for walking
*   **Linear Quadratic Regulator (LQR)**: Optimal balance control
*   **Model Predictive Control (MPC)**: Future state prediction
*   **Central Pattern Generators (CPG)**: Biological inspiration
*   **Reinforcement Learning**: Learned locomotion policies

### Common Issues: {#ch6-sec3-common-issues}

*   **Foot Slip**: Insufficient friction or improper foot placement
*   **Over-oscillation**: Poorly tuned control gains
*   **Energy Inefficiency**: Suboptimal gait patterns
*   **Limited Speed**: Actuator bandwidth constraints
*   **Turning Difficulty**: Coordinating multiple joints during turns
*   **Stair Navigation**: Step height and depth adaptation

### References and Further Reading: {#ch6-sec3-references}

*   **Book**: *[Bipedal Robots: Modeling, Design and Walking Synthesis](https://www.wiley.com/en-us/Bipedal+Robots%3A+Modeling%2C+Design+and+Walking+Synthesis-p-9781848214752)* by Christine Chevallereau
*   **Paper**: *[Dynamic Bipedal Walking](https://ieeexplore.ieee.org/document/1302386)* by Kajita et al.
*   **Library**: [Open Dynamic Engine](http://www.ode.org/) for simulation
*   **Framework**: [Drake](https://drake.mit.edu/) for dynamics and control
*   **Dataset**: [Human Gait Database](https://www.physionet.org/content/gaitdb/1.0.0/) for reference
*   **Tool**: [MATLAB Robotics Toolbox](https://www.mathworks.com/products/robotics.html) for algorithm development
