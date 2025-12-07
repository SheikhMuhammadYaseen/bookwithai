---
id: gazebo
title: Gazebo Simulation
sidebar_label: Gazebo
sidebar_position: 2
---

## Introduction to Gazebo {#ch4-sec2-introduction}

**Gazebo** is the most widely used robot simulator in ROS/ROS2. It provides realistic physics simulation (via ODE, Bullet, or Simbody), sensor simulation, and 3D visualization. Gazebo allows you to test robotics software without physical hardware.

### Key Features

- **Physics Simulation**: Gravity, friction, collisions, joints
- **Sensor Simulation**: Cameras, LIDAR, IMU, force sensors
- **Plugins**: Interface with ROS2 via Gazebo-ROS plugins
- **World Building**: Create custom environments with obstacles, lighting, and textures

### Basic Concepts

- **World**: The environment/arena containing models
- **Model**: A robot or object with links, joints, sensors
- **Link**: Rigid body part with visual/collision properties
- **Joint**: Connection between links (revolute, prismatic, fixed, etc.)
- **Plugin**: Code that connects Gazebo to ROS2

### ROS2 Integration

Gazebo integrates with ROS2 through:
- `gazebo_ros_pkgs`: ROS2 packages for Gazebo interface
- `spawn_entity.py`: ROS2 node to spawn models into simulation
- **Launch files**: Start Gazebo world and spawn robots

### Basic Usage

1. **Install Gazebo ROS2 packages**:
```bash
sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs
```

2. **Launch empty world**:
```bash
ros2 launch gazebo_ros gazebo.launch.py
```

3. **Spawn a robot**:
```bash
ros2 run gazebo_ros spawn_entity.py -topic /robot_description -entity my_robot
```

### Example: Simple Robot in Gazebo

```xml
<!-- SDF model for a simple wheeled robot -->
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="simple_robot">
    <pose>0 0 0.1 0 0 0</pose>
    
    <link name="base_link">
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.2</radius>
            <length>0.1</length>
          </cylinder>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.2</radius>
            <length>0.1</length>
          </cylinder>
        </geometry>
      </collision>
    </link>
    
    <!-- Add wheels, sensors, plugins... -->
  </model>
</sdf>
```

### Best Practices

1. **Start Simple**: Test basic movement before complex behaviors
2. **Use Realistic Physics**: Adjust mass, inertia, friction for realism
3. **Test Sensor Noise**: Add Gaussian noise to simulate real sensors
4. **Record Data**: Use Gazebo's logging for debugging
5. **Performance**: Reduce polygon count for complex scenes