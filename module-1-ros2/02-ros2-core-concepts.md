---
sidebar_position: 2
title: "Core ROS 2 Communication Primitives"
description: "Understanding nodes, topics, and services with humanoid examples"
tags: [ros2, robotics, ai, middleware, communication]
---

# Core ROS 2 Communication Primitives

## Introduction to Core Concepts

ROS 2 provides three fundamental communication primitives that enable different software components (called "nodes") to interact: nodes, topics, and services. Understanding these primitives is essential for building any ROS 2 application, especially for humanoid robots where multiple sensors, controllers, and decision-making systems must coordinate seamlessly.

These primitives form the foundation of the publish-subscribe and request-response communication patterns that make ROS 2 powerful and flexible for robotics applications.

## Nodes: What They Represent in Humanoids

In ROS 2, a **node** is a process that performs computation. It's the basic unit of execution in a ROS 2 system. In the context of humanoid robots, nodes represent specific functional units:

- **Sensor nodes**: Camera drivers, IMU readers, joint encoders
- **Controller nodes**: Joint position controllers, balance controllers, walking pattern generators
- **Processing nodes**: Vision processing, path planning, behavior decision-making
- **Interface nodes**: Human-robot interaction, external communication bridges

Each node in a humanoid robot system typically handles one specific aspect of the robot's functionality. For example, a camera driver node continuously captures images and publishes them to a topic, while a vision processing node subscribes to those images and performs object recognition.

Nodes are implemented as objects that inherit from the `rclpy.node.Node` class in Python or the corresponding base class in other languages. They can create publishers, subscribers, services, and other ROS 2 entities to communicate with other nodes.

## Topics: Continuous Data Streams (Sensors, Joints)

**Topics** enable the publish-subscribe communication pattern, which is ideal for continuous data streams like sensor readings or joint positions. In a humanoid robot, topics are used for:

- Sensor data streams (camera images, LIDAR scans, IMU readings)
- Joint state updates (positions, velocities, efforts)
- Robot state information (battery level, system status)
- Control commands (joint trajectories, velocity commands)

When a node publishes to a topic, all nodes subscribed to that topic receive the message. This one-to-many communication pattern is perfect for broadcasting sensor data to multiple processing nodes simultaneously.

Topics in ROS 2 use message types that define the structure of the data being transmitted. Common message types for humanoid robots include:
- `sensor_msgs/JointState`: Joint positions, velocities, and efforts
- `sensor_msgs/Image`: Camera image data
- `sensor_msgs/Imu`: Inertial measurement unit data
- `geometry_msgs/Twist`: Velocity commands
- `std_msgs/Float64`: Single floating-point values

## Services: Request/Response Actions

**Services** enable request-response communication, which is appropriate for actions that have a clear request and response, with the server providing a response to each request. Services are ideal for operations that:

- Have a defined start and end
- Require a specific response
- Are not continuously needed
- Need to ensure request processing

In humanoid robots, services might be used for:
- Changing robot operational modes
- Requesting specific calibration procedures
- Saving current robot configuration
- Executing specific behaviors or actions

A service has a client that sends a request and a server that processes the request and sends back a response. The service definition includes both the request and response message types.

## Pub/Sub Model with Humanoid Examples

The publish-subscribe model in ROS 2 allows for loose coupling between nodes. Publishers send messages to topics without knowing who (if anyone) is subscribed, and subscribers receive messages from topics without knowing who published them.

### Camera System Example:
```
┌─────────────────┐    ┌──────────────────────┐    ┌──────────────────┐
│   Camera Node   │───▶│  /camera/image_raw   │───▶│ Vision Processing│
│ (Publisher)     │    │     (Topic)          │    │    Node (Subscriber) │
└─────────────────┘    └──────────────────────┘    └──────────────────┘
```

### Joint Control Example:
```
┌─────────────────┐    ┌──────────────────────┐    ┌──────────────────┐
│ Trajectory Node │───▶│ /joint_trajectory    │───▶│ Joint Controller │
│ (Publisher)     │    │     (Topic)          │    │    Node (Subscriber) │
└─────────────────┘    └──────────────────────┘    └──────────────────┘
```

### Balance Control Example:
```
┌─────────────────┐    ┌──────────────────────┐    ┌──────────────────┐
│   IMU Node      │───▶│   /imu/data          │───▶│ Balance Control  │
│ (Publisher)     │    │     (Topic)          │    │    Node (Subscriber) │
└─────────────────┘    └──────────────────────┘    └──────────────────┘
```

## ROS 2 Graph Visualization

The ROS 2 graph refers to the network of nodes and their communication connections. You can visualize this graph using ROS 2 tools, which helps understand how your humanoid robot's software components are interconnected.

The graph includes:
- **Nodes**: Represented as circles or rectangles
- **Topics**: Represented as ellipses or ovals
- **Connections**: Lines showing publishers and subscribers

For a humanoid robot, the graph might look like this conceptually:

```
┌─────────────────┐              ┌──────────────────────┐              ┌──────────────────┐
│   Camera Node   │─────────────▶│  /camera/image_raw   │─────────────▶│ Vision Processing│
└─────────────────┘              └──────────────────────┘              │    Node          │
       │                                │                              └──────────────────┘
       │                                ▼                                    │
       │                      ┌─────────────────────────┐                    │
       │                      │    Perception Fusion    │◀───────────────────┘
       │                      │      Node               │
       │                      └─────────────────────────┘
       │                                │
       │                                ▼
┌─────────────────┐              ┌──────────────────────┐              ┌──────────────────┐
│   IMU Node      │─────────────▶│    /imu/data         │─────────────▶│ Balance Control  │
└─────────────────┘              └──────────────────────┘              │    Node          │
       │                                │                              └──────────────────┘
       │                                ▼                                    │
       │                      ┌─────────────────────────┐                    │
       │                      │    Walking Pattern      │◀───────────────────┘
       │                      │      Generator          │
       │                      └─────────────────────────┘
       │                                │
       │                                ▼
┌─────────────────┐              ┌──────────────────────┐              ┌──────────────────┐
│ Joint Encoder   │─────────────▶│  /joint_states       │─────────────▶│ Motion Planning  │
│    Node         │              └──────────────────────┘              │    Node          │
└─────────────────┘                                                   └──────────────────┘
```

## Minimal rclpy Publisher Example

Here's a minimal example of a publisher node that could simulate a humanoid sensor:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import time

class HumanoidSensorPublisher(Node):
    def __init__(self):
        super().__init__('humanoid_sensor_publisher')

        # Create a publisher for joint states
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Create a timer to publish data at regular intervals (50 Hz)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize counter for oscillating joint positions
        self.i = 0

        self.get_logger().info('Humanoid Sensor Publisher started')

    def timer_callback(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Define joint names for a simple humanoid leg
        msg.name = ['hip_joint', 'knee_joint', 'ankle_joint']

        # Generate oscillating joint positions (simulating walking motion)
        msg.position = [
            0.1 * math.sin(self.i * 0.1),      # hip joint
            0.2 * math.sin(self.i * 0.1 + 0.5), # knee joint
            0.15 * math.sin(self.i * 0.1 + 1.0) # ankle joint
        ]

        # Set velocities and efforts to zero
        msg.velocity = [0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0]

        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing joint states: {msg.position}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    humanoid_sensor_publisher = HumanoidSensorPublisher()

    try:
        rclpy.spin(humanoid_sensor_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        humanoid_sensor_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Minimal rclpy Subscriber Example

Here's a minimal example of a subscriber node that could process the joint states from the publisher:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')

        # Create a subscription to the joint states topic
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)  # QoS depth

        self.subscription  # prevent unused variable warning
        self.get_logger().info('Joint State Subscriber started')

    def listener_callback(self, msg):
        # Process the received joint state message
        self.get_logger().info(f'Received joint positions: {msg.position}')

        # Example processing: Check if any joint is approaching limits
        for i, pos in enumerate(msg.position):
            if abs(pos) > 1.5:  # Example limit check
                self.get_logger().warn(f'Joint {msg.name[i]} position {pos} is approaching limit!')

        # Additional processing could include:
        # - Sending control commands based on joint positions
        # - Logging data for analysis
        # - Updating robot state estimates

def main(args=None):
    rclpy.init(args=args)
    joint_state_subscriber = JointStateSubscriber()

    try:
        rclpy.spin(joint_state_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        joint_state_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Message Flow Explanations

The message flow in ROS 2 follows specific patterns that ensure reliable communication:

### Topic Message Flow:
1. **Publisher Creation**: A node creates a publisher for a specific topic with a message type
2. **Message Preparation**: The node prepares a message of the appropriate type
3. **Message Publishing**: The node calls `publish()` to send the message to the topic
4. **Message Distribution**: DDS middleware distributes the message to all subscribers
5. **Message Reception**: Subscribers receive the message in their callback functions
6. **Message Processing**: Subscribers process the received message

### Service Message Flow:
1. **Service Server**: A node creates a service server that waits for requests
2. **Service Client**: Another node creates a service client to make requests
3. **Request Sending**: The client sends a request to the service
4. **Request Processing**: The server receives and processes the request
5. **Response Sending**: The server sends back a response
6. **Response Reception**: The client receives and processes the response

### Quality of Service (QoS) Considerations:
Different types of messages require different QoS settings:
- **Reliable**: For critical messages where no data loss is acceptable (e.g., joint commands)
- **Best Effort**: For less critical messages where some loss is acceptable (e.g., camera images)
- **Keep Last**: For state messages where only the most recent value matters
- **Keep All**: For messages where history is important

## Learning Goals

By the end of this chapter, you will be able to:
- Define and distinguish between nodes, topics, and services in ROS 2
- Explain what nodes represent in the context of humanoid robots
- Understand how topics enable continuous data streams for sensors and joints
- Describe when to use services for request/response actions
- Visualize the ROS 2 graph with nodes and topics
- Implement basic publisher and subscriber nodes in Python
- Understand message flow patterns in ROS 2 communication

### Prerequisites
- Understanding of basic ROS 2 concepts from Chapter 1
- Basic Python programming knowledge

### Estimated Time
- 45-60 minutes

## Recap

In this chapter, we've explored the three fundamental communication primitives of ROS 2: nodes, topics, and services. We've seen how nodes represent functional units in humanoid robots, from sensor drivers to controllers. We've learned how topics enable the publish-subscribe pattern for continuous data streams like sensor readings and joint states. We've examined how services provide request-response communication for discrete actions.

We've visualized the ROS 2 graph to understand how nodes interconnect through topics, and we've seen practical examples of publisher and subscriber nodes that could be used in humanoid robot applications. Finally, we've understood the message flow patterns and QoS considerations that are important for reliable robot communication.

These concepts form the core of ROS 2 communication and are essential for building any robotic application. In the next chapter, we'll dive into practical Python development with rclpy, implementing these concepts in code.