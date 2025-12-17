---
sidebar_position: 5
---



# Chapter 5: URDF Robot Modeling Basics

## 5.1 What is URDF? {#ch5-sec1-what-is-urdf}

URDF (Unified Robot Description Format) is an XML-based file format that is used to describe the physical structure of a robot. It is a a tree-like structure of links and joints that define the robot's kinematics and dynamics. URDF is a crucial tool for robot modeling and simulation.

## 5.2 The Structure of a URDF File {#ch5-sec2-urdf-structure}

A URDF file is composed of two main elements:

*   **Links:** Links represent the rigid parts of the robot, such as the body, arms, and legs.
*   **Joints:** Joints connect the links together and define how they can move relative to each other.

This section will provide a detailed explanation of the URdf syntax and how to use it to create a model of a robot.

## 5.3 Creating Your First URDF File {#ch5-sec3-first-urdf}

In this section, we will walk through the process of creating a simple URDF file for a two-wheeled robot. This will give you a hands-on introduction to the basics of URDF modeling.

## 5.4 Visualizing a URDF File {#ch5-sec4-visualizing-urdf}

Once you have created a URDF file, you can use a tool like RViz to visualize it. RViz is a 3D visualization tool for ROS that can be used to display robot models, sensor data, and more. This section will show you how to use RViz to visualize your URDF file.

## Exercises

### Exercise 1: URDF Fundamentals

1.  What is the purpose of a URDF file?
2.  What are the two main elements of a URDF file?
3.  What is the relationship between links and joints in a URDF file?

### Exercise 2: Modeling Challenge

Create a URDF file for a simple robotic arm with two links and two joints. The first joint should be a revolute joint, and the second joint should be a prismatic joint. Visualize your URDF file in RViz and take a screenshot of the result.

