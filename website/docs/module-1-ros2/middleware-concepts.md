---
sidebar_position: 2
title: 'ROS 2 Middleware Concepts'
---

# ROS 2 Middleware Concepts

Middleware is the software layer that enables communication between different components of a robot system. In ROS 2, this is accomplished through a publish-subscribe model and service-based communication.

## Key Concepts

### Nodes
Nodes are the fundamental units of computation in ROS 2. Each node runs a specific task and can communicate with other nodes through topics, services, or actions.

### Topics and Messages
Topics allow nodes to publish and subscribe to data streams. Messages are the data structures that are passed between nodes via topics.

### Services
Services provide request-response communication between nodes, allowing one node to request data or action from another.

### Actions
Actions are similar to services but are designed for long-running tasks with feedback.

## Architecture

ROS 2 uses a DDS (Data Distribution Service) implementation for communication, providing a more robust and scalable architecture than ROS 1.