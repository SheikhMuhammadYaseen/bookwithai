---
id: ros2-nodes
title: ROS2 Nodes
sidebar_label: Nodes
sidebar_position: 1
---



## Understanding ROS2 Nodes {#ch2-sec1-understanding-ros2-nodes}

In ROS2, a **node** is the fundamental building block for creating a robotic application. Think of a node as a small, independent program that performs a specific task. For example, you might have one node for controlling a robot's wheels, another for reading data from a laser scanner, and a third for planning a path.

These nodes communicate with each other using ROS2's communication patterns like topics, services, and actions. By combining many single-purpose nodes, you can create complex and robust robotic systems.

### Key Characteristics of Nodes: {#ch2-sec1-key-characteristics-nodes}

*   **Modularity**: They allow you to break down a complex system into smaller, manageable parts.
*   **Reusability**: A well-designed node can be reused across different robotics projects.
*   **Fault Isolation**: If one node crashes, it doesn't necessarily bring down the entire system.
*   **Language Independence**: Nodes written in different programming languages (like Python and C++) can communicate with each other seamlessly.

---

### Code Example: A Simple Python Publisher Node {#ch2-sec1-code-example-publisher}

Here is a basic example of a ROS2 node written in Python. This node, which we'll call `minimal_publisher`, continuously publishes a "Hello World" message with a counter to a topic named `topic`.

```python
# Import the necessary ROS2 client library for Python
import rclpy
from rclpy.node import Node

# Import the String message type
from std_msgs.msg import String


class MinimalPublisher(Node):
    """
    A minimal publisher node that publishes a string message.
    """
    def __init__(self):
        # Initialize the Node with the name 'minimal_publisher'
        super().__init__('minimal_publisher')
        
        # Create a publisher that sends String messages to the 'topic' topic
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        
        # Set a timer to call the timer_callback function every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize a counter
        self.i = 0

    def timer_callback(self):
        # Create a new String message
        msg = String()
        
        # Set the message data
        msg.data = 'Hello World: %d' % self.i
        
        # Publish the message
        self.publisher_.publish(msg)
        
        # Log the published message to the console
        self.get_logger().info('Publishing: "%s"' % msg.data)
        
        # Increment the counter
        self.i += 1


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create an instance of the MinimalPublisher node
    minimal_publisher = MinimalPublisher()

    # Keep the node running so it can continue to publish messages
    rclpy.spin(minimal_publisher)

    # Clean up and destroy the node when done
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Breakdown of the Code: {#ch2-sec1-breakdown-code}

1.  **Imports**: We import `rclpy` (the ROS2 Python client library), `Node` for creating our node class, and `String` which is the message type we'll be publishing.
2.  **`MinimalPublisher` Class**: Our node is defined in this class, which inherits from `rclpy.node.Node`.
3.  **`__init__` (Constructor)**:
    *   `super().__init__('minimal_publisher')`: We call the parent `Node` class's constructor and give our node the name `minimal_publisher`. All nodes in a ROS2 system must have a unique name.
    *   `self.create_publisher(...)`: This is where the publisher is created. It's configured to publish messages of type `String` on a topic named `topic`. The `10` is the queue size, which limits the number of outgoing messages if they are being sent faster than they can be processed.
    *   `self.create_timer(...)`: This sets up a timer that triggers the `timer_callback` function every 0.5 seconds. This is a common way to control the rate of a node's operations.
4.  **`timer_callback` Method**:
    *   This function is executed every time the timer fires.
    *   It creates a `String` message object, populates its `data` field, and then uses `self.publisher_.publish(msg)` to send it out on the `topic`.
    *   `self.get_logger().info(...)` is the standard ROS2 way to print messages to the console for logging and debugging.
5.  **`main` Function**:
    *   `rclpy.init()`: Initializes the ROS2 client library.
    *   `minimal_publisher = MinimalPublisher()`: Creates an instance of our node.
    *   `rclpy.spin(minimal_publisher)`: This is the main event loop. It keeps the script from exiting and allows the node to continue operating and responding to events (like the timer firing).
    *   `destroy_node()` and `rclpy.shutdown()`: These are cleanup functions that are called when the `spin` function is interrupted (e.g., by pressing Ctrl+C).


