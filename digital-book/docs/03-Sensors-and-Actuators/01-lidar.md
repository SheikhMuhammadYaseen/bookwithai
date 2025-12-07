---
id: lidar
title: LIDAR
sidebar_label: LIDAR
sidebar_position: 1
---

## Understanding LIDAR for Physical AI {#ch3-sec1-understanding-lidar}

**LIDAR** (Light Detection and Ranging) is a remote sensing technology that measures distance by illuminating a target with laser light and analyzing the reflected light. In Physical AI and robotics, LIDAR sensors are crucial for **perception**, **mapping**, and **navigation**. They generate precise 3D point clouds of the environment, allowing robots to detect obstacles, build maps (SLAM), and localize themselves within those maps.

### How LIDAR Works {#ch3-sec1-how-lidar-works}

A typical rotating 2D LIDAR (like the popular RPLIDAR or SICK TIM series) emits laser beams in a plane (usually horizontal). By measuring the **Time of Flight (ToF)** of each laser pulse as it reflects off surfaces, the sensor calculates distances. As the head rotates (e.g., 5-15 Hz), it builds a 360° scan of distances around the robot. 3D LIDARs (like Velodyne or Ouster) use multiple laser beams stacked vertically to capture full 3D data.

### Key Characteristics of LIDAR Data: {#ch3-sec1-key-characteristics-lidar}

*   **High Accuracy and Precision**: Typically centimeter-level accuracy, superior to ultrasonic or simple infrared sensors.
*   **Wide Field of View**: 2D LIDARs often provide a full 360° horizontal field of view.
*   **Operates in Various Lighting Conditions**: Unlike cameras, LIDAR is not affected by ambient light (sunlight, darkness), though it can be challenged by highly reflective or absorptive surfaces.
*   **Output Format**: Data is usually published as a **PointCloud2** message in ROS2, which contains arrays of (x, y, z) points, or as a **LaserScan** message for 2D data.

### Integrating LIDAR with ROS2 {#ch3-sec1-integrating-lidar-ros2}

Most LIDAR sensors come with a ROS2 driver package that publishes sensor data to topics. A common workflow is:

1.  **Install the Driver**: For example, for Slamtec RPLIDAR:
    ```bash
    sudo apt install ros-${ROS_DISTRO}-rplidar-ros
    ```
2.  **Launch the Driver Node**: This node communicates with the sensor hardware and publishes data.
    ```bash
    ros2 launch rplidar_ros rplidar.launch.py
    ```
3.  **Subscribe to the Topic**: The driver typically publishes a `sensor_msgs/msg/LaserScan` topic named `/scan`. You can visualize it using RViz2.

### Code Example: Reading LIDAR Data in a ROS2 Python Node {#ch3-sec1-code-example-lidar}

Here's a simple subscriber node that processes incoming `LaserScan` messages to detect the closest obstacle in front of the robot.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarSafety(Node):
    def __init__(self):
        super().__init__('lidar_safety')
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, 10)
        self.get_logger().info('LIDAR Safety Node Started.')

    def cb(self, msg):
        # Front sector: -30° to +30°
        start = int((-math.pi/6 - msg.angle_min)/msg.angle_increment)
        end = int((math.pi/6 - msg.angle_min)/msg.angle_increment)
        front = [r for i,r in enumerate(msg.ranges[start:end]) if r > msg.range_min and not math.isinf(r)]
        
        if front:
            d = min(front)
            self.get_logger().info(f'Front obstacle: {d:.2f} m')
            if d < 0.5: self.get_logger().warn('OBSTACLE TOO CLOSE! STOP OR TURN!')
        else:
            self.get_logger().info('No front obstacles.')

def main(args=None):
    rclpy.init(args=args); node=LidarSafety()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
```