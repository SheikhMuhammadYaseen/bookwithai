---
id: imu
title: IMU
sidebar_label: IMU
sidebar_position: 3
---

## Understanding IMU for Physical AI {#ch3-sec3-understanding-imu}

An **Inertial Measurement Unit (IMU)** is a sensor that measures a system's specific force, angular rate, and sometimes magnetic field. It's essentially the "inner ear" of a robot, providing crucial information about its orientation, acceleration, and rotation. In Physical AI, IMUs are vital for tasks requiring balance (humanoids), dead reckoning (when GPS is unavailable), and sensor fusion with cameras or LIDAR.

### Components of an IMU: {#ch3-sec3-components-imu}

1.  **Accelerometer**: Measures linear acceleration (including gravity) along three axes (x, y, z). Units: m/s² or g (9.81 m/s²).
2.  **Gyroscope**: Measures angular velocity (rotation rate) around three axes. Units: rad/s or degrees/s.
3.  **Magnetometer (optional)**: Measures the Earth's magnetic field to determine heading (compass). Units: μT (microtesla).

### Key IMU Parameters: {#ch3-sec3-key-parameters}

*   **Sampling Rate**: How frequently measurements are taken (typically 100-1000 Hz).
*   **Noise Density**: Level of random noise in measurements.
*   **Bias Stability**: How much the sensor's zero-point drifts over time.
*   **Full Scale Range**: Maximum measurable acceleration/rotation rate.
*   **Communication Interface**: I2C, SPI, or UART for data transfer.

### Sensor Fusion for Orientation Estimation {#ch3-sec3-sensor-fusion}

Raw IMU data is noisy and drifts over time. To obtain a stable orientation estimate, sensor fusion algorithms combine accelerometer, gyroscope, and magnetometer data:

*   **Complementary Filter**: A simple, efficient filter that combines high-frequency gyro data with low-frequency accelerometer data.
*   **Madgwick Filter**: Popular in robotics for its balance of accuracy and computational efficiency.
*   **Kalman Filter**: Optimal estimator that models sensor noise and system dynamics; used in advanced applications.

### Integrating IMU with ROS2 {#ch3-sec3-integrating-imu-ros2}

IMU data in ROS2 is typically published as a `sensor_msgs/msg/Imu` message containing:
*   `orientation` (quaternion)
*   `angular_velocity` (vector3)
*   `linear_acceleration` (vector3)
*   `orientation_covariance`, `angular_velocity_covariance`, `linear_acceleration_covariance`

Many IMUs (like the BNO055, MPU9250, or BMI088) have ROS2 drivers that handle sensor fusion internally.

### Code Example: Complete IMU Data Processing Node {#ch3-sec3-code-example-imu}

Here's a complete ROS2 node that reads IMU data, performs sensor fusion using a complementary filter, and publishes orientation with visualization.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
import numpy as np, math, time

class IMUProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')
        self.sub = self.create_subscription(Imu, '/imu/data_raw', self.cb, 10)
        self.pub_fused = self.create_publisher(Imu, '/imu/data_fused', 10)
        self.pub_euler = self.create_publisher(Vector3, '/imu/euler_angles', 10)
        self.roll = self.pitch = self.yaw = 0
        self.alpha, self.dt = 0.98, 0.01
        self.gyro_bias = np.zeros(3)
        self.cal_count, self.cal_samples, self.is_calibrated = 0, 100, False
        self.last_time, self.samples = time.time(), 0

    def calibrate(self, gx, gy, gz):
        if self.cal_count<self.cal_samples:
            self.gyro_bias += [gx, gy, gz]; self.cal_count+=1
            if self.cal_count==self.cal_samples: self.gyro_bias/=self.cal_samples; self.is_calibrated=True
            return False
        return True

    @staticmethod
    def euler_from_acc(ax, ay, az):
        return math.atan2(ay, az), math.atan2(-ax, math.sqrt(ay**2+az**2))

    def comp_filter(self, ax, ay, az, gx, gy, gz, dt):
        gx, gy, gz = gx-self.gyro_bias[0], gy-self.gyro_bias[1], gz-self.gyro_bias[2]
        r,p = self.euler_from_acc(ax, ay, az)
        self.roll = math.radians(self.alpha*math.degrees(self.roll+gx*dt)+(1-self.alpha)*math.degrees(r))
        self.pitch = math.radians(self.alpha*math.degrees(self.pitch+gy*dt)+(1-self.alpha)*math.degrees(p))
        self.yaw += gz*dt

    @staticmethod
    def quat_from_euler(r,p,y):
        cr, sr = math.cos(r/2), math.sin(r/2)
        cp, sp = math.cos(p/2), math.sin(p/2)
        cy, sy = math.cos(y/2), math.sin(y/2)
        q = Quaternion()
        q.w = cy*cp*cr + sy*sp*sr; q.x = cy*cp*sr - sy*sp*cr
        q.y = sy*cp*sr + cy*sp*cr; q.z = sy*cp*cr - cy*sp*sr
        return q

    def cb(self, msg: Imu):
        dt = time.time()-self.last_time; self.last_time=time.time()
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        if not self.is_calibrated: self.calibrate(gx, gy, gz); return
        self.comp_filter(ax, ay, az, gx, gy, gz, dt)
        fused = Imu(); fused.header=msg.header; fused.orientation=self.quat_from_euler(self.roll,self.pitch,self.yaw)
        fused.angular_velocity.x, fused.angular_velocity.y, fused.angular_velocity.z = gx, gy, gz
        fused.linear_acceleration = msg.linear_acceleration
        self.pub_fused.publish(fused)
        euler = Vector3(); euler.x,euler.y,euler.z=math.degrees(self.roll),math.degrees(self.pitch),math.degrees(self.yaw)
        self.pub_euler.publish(euler)
        self.samples += 1

class Madgwick:
    def __init__(self, beta=0.1, fs=100):
        self.q = np.array([1,0,0,0]); self.beta=beta; self.dt=1/fs
    def update(self,gx,gy,gz,ax,ay,az):
        q=self.q; norm=math.sqrt(ax**2+ay**2+az**2)
        if norm==0: return q
        ax,ay,az=ax/norm,ay/norm,az/norm
        f=np.array([2*(q[1]*q[3]-q[0]*q[2])-ax,2*(q[0]*q[1]+q[2]*q[3])-ay,2*(0.5-q[1]**2-q[2]**2)-az])
        J=np.array([[-2*q[2],2*q[3],-2*q[0],2*q[1]],[2*q[1],2*q[0],2*q[3],2*q[2]],[0,-4*q[1],-4*q[2],0]])
        step = J.T@f; step/=np.linalg.norm(step) if np.linalg.norm(step)>0 else 1
        qdot = 0.5*np.array([-q[1]*gx-q[2]*gy-q[3]*gz,q[0]*gx+q[2]*gz-q[3]*gy,q[0]*gy-q[1]*gz+q[3]*gx,q[0]*gz+q[1]*gy-q[2]*gx])-self.beta*step
        q+=qdot*self.dt; self.q=q/np.linalg.norm(q); return self.q

def main(args=None):
    rclpy.init(args=args); node=IMUProcessor()
    try: rclpy.spin(node)
    except KeyboardInterrupt: node.get_logger().info("Stopped")
    finally: node.destroy_node(); rclpy.shutdown()

if __name__=="__main__": main()
```
