---
sidebar_position: 4
title: 'rclpy Python Agent Bridge'
---

# rclpy Python Agent Bridge

rclpy is the Python client library for ROS 2. It provides Python bindings for ROS 2, allowing Python developers to create ROS 2 nodes and interact with the ROS 2 ecosystem.

## Installation

```bash
pip install rclpy
```

## Basic Usage

### Creating a Node
```python
import rclpy
from rclpy.node import Node

class MyPythonNode(Node):
    def __init__(self):
        super().__init__('my_python_node')
        self.get_logger().info('My Python Node has started')
```

### Publisher Example
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.i += 1
```

### Subscriber Example
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

## Advanced Features

rclpy provides many advanced features including:
- Parameter handling
- Timers and callbacks
- Service and action clients/servers
- Lifecycle nodes
- Quality of Service (QoS) settings