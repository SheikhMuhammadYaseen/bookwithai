---
id: motors
title: Motors and Motion Control
sidebar_label: Motors
sidebar_position: 4
---

## Understanding Motors for Physical AI {#ch3-sec4-understanding-motors}

**Motors** are the muscles of Physical AI systems. Different types suit different needs:

**Quick Comparison**:
- **DC Brushed**: Simple, cheap. Used in Roomba vacuums
- **BLDC**: High efficiency, complex control. Used in drones
- **Servos**: Precise positioning. Used in robotic arms
- **Steppers**: Open-loop control. Used in 3D printers

### Key Parameters {#ch3-sec4-key-parameters}
- **Torque**: Rotational force (Nm)
- **Speed**: RPM
- **Efficiency**: Power out/power in
- **Backlash**: Play in gears

### Motor Control in ROS2 {#ch3-sec4-motor-control-ros2}

**Common ROS2 Packages**:
- `ros2_control`: Standard framework
- `dynamixel_sdk`: For smart servos
- `odrive_ros2`: For high-performance BLDC

**Example Control Stack**:
```
ROS2 Node → PID Controller → Motor Driver → Motor
                 ↓
           Encoder Feedback
```

### Code Example: Basic Motor Control {#ch3-sec4-code-example-motors}

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SimpleMotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Subscribers
        self.create_subscription(Float32, 'target_speed', self.speed_callback, 10)
        
        # Publishers
        self.pwm_pub = self.create_publisher(Float32, 'pwm_output', 10)
        
        # Simple PID gains
        self.Kp = 0.5
        self.target_speed = 0.0
        self.current_speed = 0.0
        
    def speed_callback(self, msg):
        self.target_speed = msg.data
        
    def control_loop(self):
        # Simple P control
        error = self.target_speed - self.current_speed
        pwm = self.Kp * error
        
        # Limit output
        pwm = max(min(pwm, 1.0), -1.0)
        
        # Publish
        msg = Float32()
        msg.data = pwm
        self.pwm_pub.publish(msg)

def main():
    rclpy.init()
    node = SimpleMotorController()
    rclpy.spin(node)
```

### Practical Tips {#ch3-sec4-practical-tips}

1. **Start Simple**: Use servos for joints, DC motors for wheels
2. **Add Encoders**: Essential for closed-loop control
3. **Monitor Temperature**: Motors overheat easily
4. **Use Proper Drivers**: H-bridge for DC, ESC for BLDC
5. **Implement Safety**: E-stop and current limiting

**Real Examples**:
- Boston Dynamics Spot: BLDC motors with harmonic drives
- Industrial arms: AC servos with high precision
- DIY robots: Micro servos or NEMA steppers
