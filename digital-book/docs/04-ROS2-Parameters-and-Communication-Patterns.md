---
sidebar_position: 4
---



# Chapter 4: ROS2 Parameters and Communication Patterns

## 4.1 Understanding ROS2 Parameters {#ch4-sec1-understanding-parameters}

ROS2 parameters are a way to configure nodes without having to recompile them. They are a set of values that a node can read at runtime to modify its behavior. This is a powerful feature that allows for a great deal of flexibility in a ROS2 system.

## 4.2 Using Parameters in a ROS2 Node {#ch4-sec2-using-parameters}

This section will demonstrate how to use parameters in a ROS2 node. We will cover how to declare parameters, how to get and set their values, and how to use them to control the behavior of a node.

## 4.3 Communication Patterns in ROS2 {#ch4-sec3-communication-patterns}

In the previous chapter, we introduced the basic communication methods in ROS2: topics, services, and actions. In this chapter, we will delve deeper into these methods and explore some of the common communication patterns that are used in robotics.

## 4.4 Publisher/Subscriber Pattern {#ch4-sec4-pub-sub-pattern}

The publisher/subscriber pattern is a one-to-many communication pattern where a publisher node sends messages to a topic, and any number of subscriber nodes can receive those messages. This is a very common pattern in ROS2 and is used for things like broadcasting sensor data.

## 4.5 Request/Response Pattern {#ch4-sec5-req-res-pattern}

The request/response pattern is a one-to-one communication pattern where a client node sends a request to a server node and waits for a response. This is used for tasks that can be completed quickly, such as querying the state of a node.

## Exercises

### Exercise 1: Parameter Power

1.  Explain the purpose of ROS2 parameters.
2.  How can you change the value of a parameter at runtime?
3.  What is the difference between the publisher/subscriber pattern and the request/response pattern?

### Exercise 2: Code Challenge

Write a ROS2 node that has a parameter called "speed". The node should print the value of the "speed" parameter to the console every second.

