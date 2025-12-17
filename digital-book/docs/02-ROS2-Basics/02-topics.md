---
id: ros2-topics
title: ROS2 Topics
sidebar_label: Topics
sidebar_position: 2
---

# ROS2 Topics: Core Concepts & Essentials

## Understanding ROS2 Topics {#ch2-sec2-understanding-ros2-topics}

1. **Many-to-Many Communication**
   - Multiple publishers → One topic
   - Multiple subscribers ← One topic

2. **Asynchronous**
   - No timing coordination needed
   - Publishers and subscribers operate independently

3. **Typed Messages**
   - Each topic has specific message type
   - Ensures data structure consistency

4. **Anonymous**
   - Publishers don't know subscribers
   - Subscribers don't know publishers

## Topic Structure {#ch2-sec2-topic-structure}

### Standard Naming Convention
```
/namespace/topic_name
```
Examples:
- `/cmd_vel` - Robot velocity commands
- `/camera/image_raw` - Camera feed
- `/sensors/lidar` - LIDAR data
- `/robot/status` - Robot status

## Basic Publisher Example {#ch2-sec2-basic-publisher}

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        # Create publisher: topic_name, msg_type, queue_size
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.publish_msg)
    
    def publish_msg(self):
        msg = String()
        msg.data = "Hello ROS2"
        self.pub.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
```

## Basic Subscriber Example {#ch2-sec2-basic-subscriber}

```python
class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        # Create subscriber
        self.sub = self.create_subscription(
            String,                    # Message type
            'chatter',                 # Topic name
            self.message_callback,     # Callback function
            10                         # Queue size
        )
    
    def message_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
```

## Essential ROS2 Topic Commands {#ch2-sec2-topic-commands}

| Command | Purpose | Example |
|---------|---------|---------|
| `ros2 topic list` | List active topics | `ros2 topic list -t` (with types) |
| `ros2 topic echo` | Display messages | `ros2 topic echo /chatter` |
| `ros2 topic info` | Show topic info | `ros2 topic info /chatter` |
| `ros2 topic pub` | Publish once | `ros2 topic pub /chatter std_msgs/String "data: 'Hi'"` |
| `ros2 topic hz` | Check rate | `ros2 topic hz /chatter` |

## Critical Parameters {#ch2-sec2-critical-parameters}

### Queue Size
- **Small (1-5)**: Commands, critical data
- **Medium (10)**: Sensor data
- **Large (50+)**: Logging, non-critical

### QoS Settings
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

# For reliable delivery (commands)
reliable_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE
)

# For best effort (sensors)
best_effort_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT
)
```

## Important Considerations {#ch2-sec2-important-considerations}

1. **Topic Names Must Match Exactly**
   - Case-sensitive
   - `/cmd_vel` ≠ `/Cmd_vel` ≠ `/cmd_Vel`

2. **Message Types Must Match**
   - Publisher: `std_msgs/String`
   - Subscriber: `std_msgs/String`

3. **Queue Overflow**
   - Messages dropped if queue full
   - Adjust queue size based on data rate

4. **Network Considerations**
   - Topics work across machines
   - Use ROS_DOMAIN_ID for isolation

## Common Message Types {#ch2-sec2-common-messages}

| Message Type | Common Use | Import Path |
|-------------|------------|-------------|
| `std_msgs/String` | Text data | `from std_msgs.msg import String` |
| `std_msgs/Int32` | Integer data | `from std_msgs.msg import Int32` |
| `geometry_msgs/Twist` | Velocity commands | `from geometry_msgs.msg import Twist` |
| `sensor_msgs/Image` | Camera images | `from sensor_msgs.msg import Image` |

## Quick Reference: Publisher vs Subscriber {#ch2-sec2-quick-reference}

| Aspect | Publisher | Subscriber |
|--------|-----------|------------|
| Purpose | Send data | Receive data |
| Creation | `create_publisher()` | `create_subscription()` |
| Callback | Timer-based (optional) | Message-based (required) |
| Active | Publishes periodically | Waits for messages |

## Minimal Working Example {#ch2-sec2-minimal-example}

```bash
# Terminal 1: Publisher
ros2 run my_pkg simple_publisher

# Terminal 2: Subscriber  
ros2 run my_pkg simple_subscriber

# Terminal 3: Monitor
ros2 topic list
ros2 topic echo /chatter
ros2 topic info /chatter
```

## Key Takeaways {#ch2-sec2-key-takeaways}

1. **Topics enable decoupled communication**
2. **Publishers and subscribers connect via topic names**
3. **Message types must be consistent**
4. **Use CLI tools for debugging**
5. **Adjust QoS based on application needs**

**Remember**: Topics are ROS2's primary communication mechanism - master them to build effective robotic systems.
