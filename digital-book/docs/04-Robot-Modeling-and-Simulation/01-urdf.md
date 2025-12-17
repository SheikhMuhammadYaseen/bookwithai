---
id: urdf
title: URDF Robot Modeling
sidebar_label: URDF
sidebar_position: 1
---

## Understanding URDF for Physical AI {#ch4-sec1-understanding-urdf}

**URDF (Unified Robot Description Format)** is an XML specification used in ROS to describe robots' physical and kinematic properties. URDF files define everything about a robot: its links (rigid bodies), joints (connections between links), visual appearance, collision geometry, and inertial properties. For Physical AI systems, URDF is essential for simulation, visualization, and motion planning.

### Key Components of URDF: {#ch4-sec1-key-components}

1.  **Links**: Represent rigid bodies with mass, inertia, and visual/collision geometry.
2.  **Joints**: Define the connection between links, including type (revolute, prismatic, fixed, etc.), limits, and dynamics.
3.  **Visual**: Describes how the robot looks in visualization tools (RViz).
4.  **Collision**: Defines simplified geometry for collision checking in simulators.
5.  **Inertial**: Specifies mass and inertia properties for physics simulation.

### Basic URDF Structure: {#ch4-sec1-basic-structure}

A minimal URDF file contains:
- A robot tag as the root element
- Multiple link elements defining robot parts
- Joint elements connecting the links

### Code Example: Complete Simple Robot URDF {#ch4-sec1-code-example-urdf}

Here's a complete URDF file for a simple differential drive robot with two wheels:

```xml
<?xml version="1.0"?>
<robot name="simple_robot" xmlns:xacro="http://ros.org/wiki/xacro">

  <!-- Materials -->
  <material name="blue"><color rgba="0.1 0.1 0.8 1"/></material>
  <material name="black"><color rgba="0.1 0.1 0.1 1"/></material>
  <material name="gray"><color rgba="0.7 0.7 0.7 1"/></material>

  <!-- Base link -->
  <link name="base_link">
    <visual><geometry><box size="0.4 0.2 0.1"/></geometry><material name="blue"/></visual>
    <collision><geometry><box size="0.4 0.2 0.1"/></geometry></collision>
    <inertial><mass value="2.0"/><inertia ixx="0.02" iyy="0.01" izz="0.02"/></inertial>
  </link>

  <!-- Wheel Macro -->
  <xacro:property name="wheel_r" value="0.05"/>
  <xacro:property name="wheel_w" value="0.03"/>
  <xacro:macro name="wheel" params="name x y">
    <link name="${name}_wheel">
      <visual><geometry><cylinder radius="${wheel_r}" length="${wheel_w}"/></geometry><material name="black"/></visual>
      <collision><geometry><cylinder radius="${wheel_r}" length="${wheel_w}"/></geometry></collision>
      <inertial><mass value="0.2"/><inertia ixx="0.00025" iyy="0.0005" izz="0.00025"/></inertial>
    </link>
    <joint name="${name}_joint" type="continuous">
      <parent link="base_link"/><child link="${name}_wheel"/>
      <origin xyz="${x} ${y} 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>

  <!-- Wheels -->
  <xacro:wheel name="left" x="-0.15" y="0.13"/>
  <xacro:wheel name="right" x="-0.15" y="-0.13"/>

  <!-- Caster -->
  <link name="caster_wheel">
    <visual><geometry><sphere radius="0.02"/></geometry><material name="gray"/></visual>
    <collision><geometry><sphere radius="0.02"/></geometry></collision>
    <inertial><mass value="0.05"/><inertia ixx="0.00001" iyy="0.00001" izz="0.00001"/></inertial>
  </link>
  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/><child link="caster_wheel"/>
    <origin xyz="0.15 0 0" rpy="0 0 0"/>
  </joint>

</robot>