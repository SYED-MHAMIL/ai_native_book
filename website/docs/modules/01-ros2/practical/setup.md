---
sidebar_position: 1
title: "ROS 2 Practical Setup and Implementation"
description: "Hands-on guide to implementing ROS 2 systems for humanoid robotics"
tags: [ros2, practical, setup, implementation]
---

# ROS 2 Practical Setup and Implementation

## Overview

This practical guide will walk you through implementing ROS 2 systems for humanoid robotics. You'll create actual nodes, set up communication patterns, and build a simple robotic system.

## Prerequisites

Before starting this practical session, ensure you have:

- ROS 2 Humble Hawksbill installed and verified
- Basic Python programming knowledge
- Understanding of ROS 2 concepts (covered in the theory section)
- A working development environment

## Setting Up Your Workspace

### Creating a ROS 2 Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_humanoid_ws/src
cd ~/ros2_humanoid_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build the workspace (even though it's empty)
colcon build
```

### Sourcing the Workspace

Add the workspace to your environment:

```bash
# Add to your .bashrc to source automatically
echo "source ~/ros2_humanoid_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Creating Your First ROS 2 Package

### Using the Package Creation Tool

```bash
cd ~/ros2_humanoid_ws/src
ros2 pkg create --build-type ament_python humanoid_controller --dependencies rclpy std_msgs geometry_msgs sensor_msgs
```

### Package Structure

The created package will have this structure:

```
humanoid_controller/
├── humanoid_controller/
│   ├── __init__.py
│   └── humanoid_controller.py
├── test/
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

## Implementing a Basic Robot Controller Node

### Creating the Controller Node

Edit `humanoid_controller/humanoid_controller/humanoid_controller.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Create publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_publisher = self.create_publisher(String, 'robot_status', 10)

        # Create subscribers
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Create a timer for periodic updates
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz

        self.get_logger().info('Humanoid Controller Node Started')
        self.joint_positions = {}

    def joint_state_callback(self, msg):
        """Callback for joint state updates"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]

    def control_loop(self):
        """Main control loop"""
        # Publish robot status
        status_msg = String()
        status_msg.data = f'Operating with {len(self.joint_positions)} joints'
        self.status_publisher.publish(status_msg)

        # Example: Send a simple velocity command
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.1  # Move forward slowly
        cmd_msg.angular.z = 0.0  # No rotation
        self.cmd_vel_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Updating the Package Entry Point

Edit `humanoid_controller/setup.py` to add the entry point:

```python
import os
from glob import glob
from setuptools import setup

package_name = 'humanoid_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Humanoid robot controller package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'humanoid_controller = humanoid_controller.humanoid_controller:main',
        ],
    },
)
```

## Building and Running Your Package

### Building the Package

```bash
cd ~/ros2_humanoid_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select humanoid_controller
source install/setup.bash
```

### Running the Controller Node

```bash
ros2 run humanoid_controller humanoid_controller
```

## Creating a Simple Publisher Node

Create `humanoid_controller/humanoid_controller/simple_publisher.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()

    try:
        rclpy.spin(simple_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        simple_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Add this to the entry points in `setup.py`:

```python
entry_points={
    'console_scripts': [
        'humanoid_controller = humanoid_controller.humanoid_controller:main',
        'simple_publisher = humanoid_controller.simple_publisher:main',
    ],
},
```

## Creating a Simple Subscriber Node

Create `humanoid_controller/humanoid_controller/simple_subscriber.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()

    try:
        rclpy.spin(simple_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        simple_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Testing the Publisher-Subscriber System

1. Build the updated package:
```bash
cd ~/ros2_humanoid_ws
colcon build --packages-select humanoid_controller
source install/setup.bash
```

2. Run the publisher in one terminal:
```bash
ros2 run humanoid_controller simple_publisher
```

3. Run the subscriber in another terminal:
```bash
ros2 run humanoid_controller simple_subscriber
```

You should see the subscriber receiving messages from the publisher.

## Creating a Service Server

Create `humanoid_controller/humanoid_controller/service_server.py`:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    service_server = ServiceServer()

    try:
        rclpy.spin(service_server)
    except KeyboardInterrupt:
        pass
    finally:
        service_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating a Service Client

Create `humanoid_controller/humanoid_controller/service_client.py`:

```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    client = ServiceClient()
    response = client.send_request(int(sys.argv[1]), int(sys.argv[2]))

    client.get_logger().info(f'Result of add_two_ints: {response.sum}')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Testing the Service System

1. Build the package again:
```bash
cd ~/ros2_humanoid_ws
colcon build --packages-select humanoid_controller
source install/setup.bash
```

2. Run the service server in one terminal:
```bash
ros2 run humanoid_controller service_server
```

3. Call the service from another terminal:
```bash
ros2 run humanoid_controller service_client 2 3
```

## Creating an Action Server

Create `humanoid_controller/humanoid_controller/action_server.py`:

```python
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Result: {result.sequence}')

        return result

def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()

    try:
        rclpy.spin(fibonacci_action_server)
    except KeyboardInterrupt:
        pass
    finally:
        fibonacci_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Testing Your Implementation

### Running All Components Together

1. Build everything:
```bash
cd ~/ros2_humanoid_ws
colcon build
source install/setup.bash
```

2. Test with multiple terminals:

Terminal 1 (Publisher):
```bash
ros2 run humanoid_controller simple_publisher
```

Terminal 2 (Subscriber):
```bash
ros2 run humanoid_controller simple_subscriber
```

Terminal 3 (Service Server):
```bash
ros2 run humanoid_controller service_server
```

Terminal 4 (Service Client):
```bash
ros2 run humanoid_controller service_client 5 7
```

### Using ROS 2 Tools for Verification

Check that all nodes are communicating properly:

```bash
# List all topics
ros2 topic list

# Echo a topic to see messages
ros2 topic echo /chatter std_msgs/msg/String

# List all services
ros2 service list

# List all nodes
ros2 node list

# Show node graph
ros2 run rqt_graph rqt_graph
```

## Advanced Practical Exercise: Humanoid Joint Controller

Create a more complex node that simulates controlling humanoid joints:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import time

class HumanoidJointController(Node):
    def __init__(self):
        super().__init__('humanoid_joint_controller')

        # Joint names for a simple humanoid model
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ]

        # Publishers and subscribers
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.trajectory_sub = self.create_subscription(
            JointTrajectory, 'joint_trajectory', self.trajectory_callback, 10)

        # Timer for publishing joint states
        self.timer = self.create_timer(0.05, self.publish_joint_states)  # 20Hz

        # Initialize joint positions
        self.joint_positions = {name: 0.0 for name in self.joint_names}
        self.get_logger().info('Humanoid Joint Controller Started')

    def trajectory_callback(self, msg):
        """Handle incoming trajectory commands"""
        if msg.joint_names and len(msg.points) > 0:
            # Get the first point of the trajectory
            point = msg.points[0]

            # Update joint positions
            for i, joint_name in enumerate(msg.joint_names):
                if joint_name in self.joint_positions and i < len(point.positions):
                    self.joint_positions[joint_name] = point.positions[i]

            self.get_logger().info(f'Updated trajectory for {len(msg.joint_names)} joints')

    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.name = list(self.joint_positions.keys())
        msg.position = list(self.joint_positions.values())

        # Add some simple oscillating motion for demonstration
        current_time = time.time()
        for i, name in enumerate(self.joint_names):
            if 'hip' in name:
                self.joint_positions[name] = 0.2 * math.sin(current_time * 0.5)
            elif 'knee' in name:
                self.joint_positions[name] = 0.1 * math.sin(current_time * 0.5 + math.pi/2)

        self.joint_state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidJointController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Module Summary

In this practical session, you:

- Created a ROS 2 workspace and package
- Implemented publisher, subscriber, service, and action nodes
- Tested communication between different node types
- Built a humanoid joint controller simulation
- Used ROS 2 tools for verification and debugging

These practical implementations provide the foundation for building complex humanoid robotic systems.

## Next Steps

Continue to the [Advanced Topics](../advanced/index.md) section to learn about more sophisticated ROS 2 patterns and best practices for humanoid robotics.