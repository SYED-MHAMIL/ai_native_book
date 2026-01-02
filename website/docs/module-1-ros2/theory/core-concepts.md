---
sidebar_position: 2
title: "ROS 2 Core Concepts and Architecture"
description: "Fundamental building blocks of ROS 2 systems for humanoid robotics"
tags: [ros2, architecture, nodes, topics, services]
---

# ROS 2 Core Concepts and Architecture

## Overview

This section covers the fundamental building blocks of ROS 2 systems that form the nervous system of humanoid robots. Understanding these core concepts is essential for developing distributed robotic applications.

## Nodes

Nodes are the fundamental building blocks of ROS 2 applications. Each node typically performs a specific task, such as controlling a sensor, processing data, or executing a behavior. Nodes communicate with each other through topics, services, and actions.

### Creating a Node

```python
import rclpy
from rclpy.node import Node

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.get_logger().info('Robot Controller Node Started')
```

## Topics and Messages

Topics enable asynchronous communication between nodes through a publish-subscribe pattern. Publishers send messages to topics, and subscribers receive messages from topics. This decouples nodes and allows for flexible system architectures.

### Publisher Example

```python
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'robot_status', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Robot is operational'
        self.publisher.publish(msg)
```

### Subscriber Example

```python
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'robot_status',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
```

## Services

Services provide synchronous request-response communication between nodes. A service client sends a request to a service server, which processes the request and returns a response.

### Service Definition

```python
# example_interfaces/srv/AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

### Service Server

```python
from example_interfaces.srv import AddTwoInts

class ServiceServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response
```

## Actions

Actions provide asynchronous request-response communication with feedback and status updates. They're ideal for long-running tasks where the client needs to know the progress and can cancel the operation.

### Action Definition

```python
# example_interfaces/action/Fibonacci.action
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

## Parameters

Parameters allow nodes to be configured at runtime. They can be set at startup, changed during execution, and shared between nodes.

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        self.declare_parameter('robot_name', 'my_robot')
        self.declare_parameter('max_velocity', 1.0)

        robot_name = self.get_parameter('robot_name').value
        max_velocity = self.get_parameter('max_velocity').value
```

## Lifecycle Nodes

Lifecycle nodes provide a standardized way to manage the state of nodes, including initialization, activation, and deactivation. This is particularly important for safety-critical robotic applications.

## Quality of Service (QoS)

QoS settings allow fine-tuning of communication behavior, including reliability, durability, and history policies. This is crucial for real-time robotic applications.

## Module Summary

This section covered the core concepts of ROS 2:

- **Nodes**: The basic execution units
- **Topics**: Asynchronous publish-subscribe communication
- **Services**: Synchronous request-response communication
- **Actions**: Asynchronous communication with feedback
- **Parameters**: Runtime configuration
- **Lifecycle**: State management
- **QoS**: Communication behavior tuning

These concepts form the foundation of distributed robotic systems and are essential for building complex humanoid robots.

## Next Steps

Continue to the [Practical Implementation](../practical/setup.md) section to learn how to implement these concepts in real robotic systems.