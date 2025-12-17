---
id: cameras
title: Cameras
sidebar_label: Cameras
sidebar_position: 2
---

## Understanding Cameras for Physical AI {#ch3-sec2-understanding-cameras}

**Cameras** are the primary visual sensors in Physical AI systems, providing rich, high-dimensional data about the environment. They enable robots to perceive color, texture, shape, depth, and motion—capabilities essential for object recognition, scene understanding, navigation, and human-robot interaction. Unlike LIDAR, cameras capture photometric information that closely resembles human vision, making them indispensable for tasks requiring semantic understanding.

### Types of Cameras in Robotics: {#ch3-sec2-types-cameras}

1.  **RGB Cameras**: Standard color cameras that capture red, green, and blue channels. Used for object detection, facial recognition, and visual serving.
2.  **Depth Cameras**: Provide per-pixel depth information alongside RGB data. Common technologies include:
    *   **Stereo Vision**: Uses two cameras to compute depth via triangulation (e.g., Intel RealSense D400 series).
    *   **Structured Light**: Projects a pattern and analyzes its deformation (e.g., early Kinect).
    *   **Time-of-Flight (ToF)**: Measures the time for light to return (e.g., Intel RealSense LiDAR camera L515).
3.  **Event Cameras**: Bio-inspired sensors that detect per-pixel brightness changes asynchronously, offering very high dynamic range and minimal latency.

### Key Camera Parameters: {#ch3-sec2-key-parameters}

*   **Resolution**: Number of pixels (e.g., 1920×1080, 4K). Higher resolution provides more detail but requires more processing power.
*   **Frame Rate**: Frames per second (FPS). Critical for tracking fast-moving objects.
*   **Field of View (FOV)**: Angular extent of the scene captured. Wide FOV is good for navigation; narrow FOV is better for detail.
*   **Dynamic Range**: Ability to capture both bright and dark areas in the same scene.
*   **Calibration**: Process of determining intrinsic (focal length, optical center) and extrinsic (position, orientation) parameters. Essential for accurate measurements.

### Integrating Cameras with ROS2 {#ch3-sec2-integrating-cameras-ros2}

ROS2 provides the `image_pipeline` suite of packages to handle camera data. The typical workflow involves:

1.  **Install Camera Drivers**:
    ```bash
    # For Intel RealSense cameras
    sudo apt install ros-${ROS_DISTRO}-realsense2-camera
    
    # For USB cameras
    sudo apt install ros-${ROS_DISTRO}-usb-cam
    
    # For OpenCV integration
    sudo apt install ros-${ROS_DISTRO}-cv-bridge
    ```

2.  **Launch the Camera Node**:
    ```bash
    # For RealSense
    ros2 launch realsense2_camera rs_launch.py
    
    # For USB camera
    ros2 run usb_cam usb_cam_node_exe
    ```

3.  **View Camera Output**:
    ```bash
    # View RGB stream
    ros2 run rqt_image_view rqt_image_view
    
    # Or use RViz2
    rviz2
    ```

### Code Example: Processing Camera Images with OpenCV in ROS2 {#ch3-sec2-code-example-cameras}

Here's a complete ROS2 node that subscribes to an RGB image topic, converts it to OpenCV format, applies computer vision processing, and publishes the result.

```python
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, numpy as np, glob

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')
        self.sub = self.create_subscription(Image, '/camera/color/image_raw', self.cb, 10)
        self.pub = self.create_publisher(Image, '/image_processed', 10)
        self.bridge = CvBridge()
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades+'haarcascade_frontalface_default.xml')
        self.frames, self.start = 0, self.get_clock().now()

    def cb(self, msg):
        self.frames += 1
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e: self.get_logger().error(f'CV Bridge failed: {e}'); return

        processed = self.process_image(img)
        try:
            out = self.bridge.cv2_to_imgmsg(processed, 'bgr8'); out.header = msg.header
            self.pub.publish(out)
        except Exception as e: self.get_logger().error(f'Publish failed: {e}')

        if self.frames % 30 == 0:
            elapsed = (self.get_clock().now() - self.start).nanoseconds / 1e9
            self.get_logger().info(f'FPS: {self.frames/elapsed:.1f}')

    def process_image(self, img):
        proc = img.copy(); gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        for (x,y,w,h) in self.face_cascade.detectMultiScale(gray,1.1,5):
            cv2.rectangle(proc,(x,y),(x+w,y+h),(0,255,0),2)
            cv2.putText(proc,'Face',(x,y-5),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1)
        edges = cv2.Canny(gray,100,200); proc = cv2.addWeighted(proc,0.7,cv2.cvtColor(edges,cv2.COLOR_GRAY2BGR),0.3,0)
        h,w = proc.shape[:2]; cv2.putText(proc,f'Res:{w}x{h}',(10,30),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,255,255),2)
        cv2.putText(proc,f'Faces:{len(self.face_cascade.detectMultiScale(gray,1.1,5))}',(10,60),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,255,255),2)
        cv2.drawMarker(proc,(w//2,h//2),(0,0,255),cv2.MARKER_CROSS,20,2)
        for x in range(100,w,100): cv2.line(proc,(x,0),(x,h),(50,50,50),1)
        for y in range(100,h,100): cv2.line(proc,(0,y),(w,y),(50,50,50),1)
        return proc

def calibrate_camera(chess=(9,6), sq=0.025):
    crit = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,0.001)
    objp = np.zeros((chess[0]*chess[1],3),np.float32); objp[:,:2]=np.mgrid[0:chess[0],0:chess[1]].T.reshape(-1,2); objp*=sq
    objpoints,imgpoints=[],[]
    for f in glob.glob('calibration_images/*.jpg'):
        img=cv2.imread(f); gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret,corners=cv2.findChessboardCorners(gray,chess,None)
        if ret:
            objpoints.append(objp); corners=cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),crit); imgpoints.append(corners)
            cv2.drawChessboardCorners(img,chess,corners,ret); cv2.imshow('Calib',img); cv2.waitKey(500)
    cv2.destroyAllWindows()
    return cv2.calibrateCamera(objpoints,imgpoints,gray.shape[::-1],None,None)

def main(args=None):
    rclpy.init(args=args); node=CameraProcessor()
    try: rclpy.spin(node)
    except KeyboardInterrupt: node.get_logger().info('Stopped')
    finally: node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
```

**Key points preserved**:

* ROS2 subscription/publishing using `CvBridge`
* Face detection & drawing
* Edge detection & overlays (FPS, resolution, center marker, grid)
* Camera calibration utility
