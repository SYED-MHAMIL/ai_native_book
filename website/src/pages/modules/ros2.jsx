import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '../../components/ui/card';
import { Button } from '../../components/ui/button';

function Ros2Slides() {
  const [currentSlide, setCurrentSlide] = useState(0);

  const slides = [
    {
      title: "ROS 2: Robotic Nervous System",
      content: (
        <div>
          <h2 className="text-2xl font-bold mb-4">Welcome to ROS 2</h2>
          <p className="mb-4">
            Welcome to the first module of robot.ai, where we explore the foundation of robotic communication - ROS 2 (Robot Operating System 2).
          </p>
          <p className="mb-4">
            ROS 2 serves as the nervous system of your robot, enabling different components to communicate effectively.
          </p>
          <h3 className="text-xl font-semibold mb-2">What You'll Learn</h3>
          <ul className="list-disc list-inside space-y-1 mb-4">
            <li>Middleware concepts: Understanding how ROS 2 enables distributed robotic systems</li>
            <li>ROS 2 nodes, topics, services: Core communication patterns in robotics</li>
            <li>rclpy Python agent bridge: Connecting Python agents to ROS controllers</li>
            <li>URDF for humanoid robots: Describing robot structure and kinematics</li>
          </ul>
        </div>
      )
    },
    {
      title: "ROS 2 Nodes",
      content: (
        <div>
          <h2 className="text-2xl font-bold mb-4">ROS 2 Nodes</h2>
          <p className="mb-4">
            Nodes are the fundamental building blocks of ROS 2 applications. Each node typically performs a specific task, such as controlling a sensor, processing data, or executing a behavior.
          </p>
          <pre className="bg-gray-100 p-4 rounded-md mb-4">
            <code>{`import rclpy
from rclpy.node import Node

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.get_logger().info('Robot Controller Node Started')`}</code>
          </pre>
          <p className="mb-4">
            Nodes communicate with each other through topics, services, and actions.
          </p>
        </div>
      )
    },
    {
      title: "Topics and Services",
      content: (
        <div>
          <h2 className="text-2xl font-bold mb-4">Topics and Services</h2>
          <h3 className="text-xl font-semibold mb-2">Topics</h3>
          <p className="mb-4">
            Topics enable asynchronous communication between nodes through a publish-subscribe pattern. Publishers send messages to topics, and subscribers receive messages from topics.
          </p>
          <h3 className="text-xl font-semibold mb-2">Services</h3>
          <p className="mb-4">
            Services provide synchronous request-response communication for operations that require acknowledgment.
          </p>
          <h3 className="text-xl font-semibold mb-2">Actions</h3>
          <p className="mb-4">
            Actions offer asynchronous request-response with feedback and status updates for extended operations.
          </p>
        </div>
      )
    },
    {
      title: "rclpy Python Agent Bridge",
      content: (
        <div>
          <h2 className="text-2xl font-bold mb-4">rclpy Python Agent Bridge</h2>
          <p className="mb-4">
            rclpy is the Python client library for ROS 2 that allows Python agents to communicate with ROS 2 systems.
          </p>
          <pre className="bg-gray-100 p-4 rounded-md mb-4">
            <code>{`import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PythonAgent(Node):
    def __init__(self):
        super().__init__('python_agent')
        self.publisher = self.create_publisher(String, 'agent_commands', 10)

    def send_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)`}</code>
          </pre>
          <p className="mb-4">
            This enables Python-based AI agents to interact with ROS 2 robotic systems.
          </p>
        </div>
      )
    },
    {
      title: "URDF for Humanoid Robots",
      content: (
        <div>
          <h2 className="text-2xl font-bold mb-4">URDF for Humanoid Robots</h2>
          <p className="mb-4">
            URDF (Unified Robot Description Format) is used to describe robot structure and kinematics in ROS 2.
          </p>
          <pre className="bg-gray-100 p-4 rounded-md mb-4">
            <code>{`<robot name="humanoid_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </visual>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>`}</code>
          </pre>
          <p className="mb-4">
            URDF files define the physical structure, joints, and kinematic relationships of humanoid robots.
          </p>
        </div>
      )
    }
  ];

  const totalSlides = slides.length;

  const handleNext = () => {
    if (currentSlide < totalSlides - 1) {
      setCurrentSlide(currentSlide + 1);
    }
  };

  const handlePrev = () => {
    if (currentSlide > 0) {
      setCurrentSlide(currentSlide - 1);
    }
  };

  const currentSlideData = slides[currentSlide];

  return (
    <Layout title={currentSlideData.title} description="ROS 2 module slides">
      <div className="min-h-screen bg-gray-50">
        {/* Header */}
        <header className="bg-white shadow-sm">
          <div className="container mx-auto px-4 py-4 flex justify-between items-center">
            <a href="/" className="flex items-center space-x-2">
              <h1 className="text-xl font-bold text-gray-900">robot.ai</h1>
            </a>
            <nav>
              <a href="/docs/intro" className="text-gray-700 hover:text-gray-900 font-medium">
                Documentation
              </a>
            </nav>
          </div>
        </header>

        <div className="container mx-auto px-4 py-8">
          <div className="max-w-4xl mx-auto">
            {/* Slide Content */}
            <Card className="mb-8">
              <CardHeader>
                <CardTitle>{currentSlideData.title}</CardTitle>
                <CardDescription>Module 1: The Robotic Nervous System (ROS 2)</CardDescription>
              </CardHeader>
              <CardContent>
                <div className="prose prose-gray max-w-none">
                  {currentSlideData.content}
                </div>
              </CardContent>
            </Card>

            {/* Slide Navigation */}
            <div className="flex justify-between items-center mt-8">
              <div>
                {currentSlide > 0 && (
                  <Button variant="outline" onClick={handlePrev}>
                    ← Previous
                  </Button>
                )}
              </div>
              <div className="text-gray-600">
                Slide {currentSlide + 1} of {totalSlides}
              </div>
              <div>
                {currentSlide < totalSlides - 1 && (
                  <Button variant="default" onClick={handleNext}>
                    Next →
                  </Button>
                )}
                {currentSlide === totalSlides - 1 && (
                  <a href="/docs/modules/02-gazebo-unity/intro">
                    <Button variant="default">
                      Next Module →
                    </Button>
                  </a>
                )}
              </div>
            </div>
          </div>
        </div>

        {/* Footer */}
        <footer className="bg-gray-900 text-white py-8 mt-12">
          <div className="container mx-auto px-4 text-center">
            <p>© {new Date().getFullYear()} robot.ai. All rights reserved.</p>
          </div>
        </footer>
      </div>
    </Layout>
  );
}

export default Ros2Slides;