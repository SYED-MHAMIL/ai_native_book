---
sidebar_position: 3
title: 'Nodes, Topics, and Services'
---

# Nodes, Topics, and Services

In this section, we'll explore the core communication mechanisms in ROS 2: nodes, topics, and services.

## Nodes

Nodes are processes that perform computation. In ROS 2, nodes are designed to be modular and communicate with each other through topics, services, and actions.

### Creating a Node
```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

## Topics

Topics are named buses over which nodes exchange messages. They implement a publish/subscribe communication pattern.

### Publisher Example
```python
publisher = node.create_publisher(String, 'topic_name', 10)
```

### Subscriber Example
```python
subscription = node.create_subscription(
    String,
    'topic_name',
    callback_function,
    10)
```

## Services

Services provide a request/response communication pattern between nodes.

### Service Server
```python
service = node.create_service(AddTwoInts, 'add_two_ints', callback_function)
```

### Service Client
```python
client = node.create_client(AddTwoInts, 'add_two_ints')
```