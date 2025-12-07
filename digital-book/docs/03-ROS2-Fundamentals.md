---
sidebar_position: 3
---



# Chapter 3: ROS2 Fundamentals

## 3.1 Introduction to ROS2 {#ch3-sec1-intro-to-ros2}

ROS2 (Robot Operating System 2) is a set of software libraries and tools that help you build robot applications. It is a complete rewrite of ROS1 that is designed to be more robust, secure, and scalable. ROS2 is the backbone of many modern robotics systems, providing a standardized way for different components to communicate with each other.

## 3.2 Core Concepts of ROS2 {#ch3-sec2-core-concepts}

There are a few core concepts in ROS2 that are essential to understand:

*   **Nodes:** A node is a process that performs some computation. A ROS2 system is typically composed of many nodes, each responsible for a specific task.
*   **Topics:** Topics are named buses over which nodes exchange messages. Nodes can publish messages to a topic or subscribe to a topic to receive messages.
*   **Services:** Services are another way for nodes to communicate. They are based on a request-response model, where one node sends a request to another node and waits for a response.
*   **Actions:** Actions are similar to services, but they are designed for long-running tasks. They provide feedback on the progress of the task and can be preempted.

## 3.3 Setting up a ROS2 Workspace {#ch3-sec3-workspace-setup}

Before you can start using ROS2, you need to set up a workspace. A workspace is a directory that contains your ROS2 packages. This section will guide you through the process of creating and configuring a new ROS2 workspace.

## 3.4 Creating Your First ROS2 Node {#ch3-sec4-first-node}

The best way to learn ROS2 is by doing. In this section, we will walk through the process of creating a simple "hello world" node in Python. This will give you a hands-on introduction to the basics of writing ROS2 code.

## Exercises {#ch3-sec5-exercises}

### Exercise 1: ROS2 Concepts Quiz {#ch3-sec5-ex1}

1.  What is the main difference between a topic and a service in ROS2?
2.  What is the purpose of a ROS2 workspace?
3.  What is a ROS2 node?

### Exercise 2: Practical Task {#ch3-sec5-ex2}

Follow the instructions in the "Creating Your First ROS2 Node" section to create and run your own "hello world" node. Take a screenshot of the output and submit it.

