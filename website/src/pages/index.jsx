import React from 'react';
import Layout from '@theme/Layout';

import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '../components/ui/card.jsx';
import { Button } from '../components/ui/button';

function LandingPage() {
  const modules = [
    {
      id: 1,
      title: "Module 1: The Robotic Nervous System (ROS 2)",
      description: "Middleware for robot control.",
      details: [
        "ROS 2 Nodes, Topics, and Services",
        "Python agents bridged to ROS controllers via rclpy",
        "URDF for humanoid robots"
      ]
    },
    {
      id: 2,
      title: "Module 2: The Digital Twin (Gazebo & Unity)",
      description: "Physics simulation and environments.",
      details: [
        "Gravity and collision simulation in Gazebo",
        "High-fidelity Unity interaction",
        "Sensor simulation (LiDAR, Depth, IMU)"
      ]
    },
    {
      id: 3,
      title: "Module 3: The AI-Robot Brain (NVIDIA Isaac)",
      description: "Perception and motion intelligence.",
      details: [
        "Isaac Sim synthetic data and photoreal simulation",
        "Isaac ROS accelerated VSLAM and navigation",
        "Nav2 for humanoid path planning"
      ]
    },
    {
      id: 4,
      title: "Module 4: Vision-Language-Action (VLA)",
      description: "Multimodal reasoning and behavior.",
      details: [
        "VLA concept overview",
        "Role in humanoid interaction",
        "Real-world application scenarios"
      ]
    }
  ];

  return (
    <Layout title="robot.ai" description="Welcome to robot.ai - A comprehensive platform for humanoid robotics education">
      {/* Header */}
        <header className="bg-white shadow-sm">
          <div className="container mx-auto px-4 py-4 flex justify-between items-center">
            <div className="flex items-center space-x-2">
              <h1 className="text-2xl font-bold text-gray-900">robot.ai</h1>
            </div>
           
          </div>
        </header>

      {/* Hero Section */}
      <section className="py-20 bg-gradient-to-b from-gray-50 to-white">
        <div className="container mx-auto px-4 text-center">
          <h1 className="text-5xl font-bold text-gray-900 mb-6">
            Welcome to robot.ai
            
          </h1>
          <p className="text-xl text-gray-600 mb-8 max-w-3xl mx-auto">
            A comprehensive learning platform for humanoid robotics, combining ROS 2, simulation, AI, and multimodal systems.
          </p>
          <Button
            variant="default"
            size="lg"
            onClick={() => document.getElementById('modules').scrollIntoView({ behavior: 'smooth' })}
          >
            Explore Modules
          </Button>
        </div>
      </section>

      {/* Module Cards Section */}
      <section id="modules" className="py-20 bg-white">
        <div className="container mx-auto px-4">
          <h2 className="text-3xl font-bold text-center text-gray-900 mb-16">
            Learning Modules
          </h2>
          <div className="grid grid-cols-1 md:grid-cols-2 gap-8">
            {modules.map((module) => (
              <Card key={module.id} className="flex flex-col h-full">
                <CardHeader>
                  <CardTitle>{module.title}</CardTitle>
                  <CardDescription>{module.description}</CardDescription>
                </CardHeader>
                <CardContent className="flex-grow">
                  <ul className="space-y-2 mb-6">
                    {module.details.map((detail, index) => (
                      <li key={index} className="flex items-start">
                        <span className="text-green-500 mr-2">•</span>
                        <span className="text-gray-700">{detail}</span>
                      </li>
                    ))}
                  </ul>
                  <Button
                    variant="outline"
                    onClick={() => window.location.href = `/docs/${module.id === 1 ? 'module-1-ros2' : module.id === 2 ? 'module-2-digital-twin' : module.id === 3 ? 'module-3-isaac' : 'module-4-vla'}`}
                  >
                    View Details / Slides
                  </Button>
                </CardContent>
              </Card>
            ))}
          </div>
        </div>
      </section>

      {/* Footer */}
      <footer className="bg-gray-900 text-white py-12">
        <div className="container mx-auto px-4">
          <div className="grid grid-cols-1 md:grid-cols-3 gap-8">
            <div>
              <h3 className="text-lg font-semibold mb-4">Documentation</h3>
              <ul className="space-y-2">
                <li><a href="/docs/intro" className="text-gray-300 hover:text-white">Introduction</a></li>
                <li><a href="/docs/module-1-ros2" className="text-gray-300 hover:text-white">ROS 2 Module</a></li>
                <li><a href="/docs/module-2-digital-twin" className="text-gray-300 hover:text-white">Simulation Module</a></li>
                <li><a href="/docs/module-3-isaac" className="text-gray-300 hover:text-white">AI Module</a></li>
                <li><a href="/docs/module-4-vla" className="text-gray-300 hover:text-white">VLA Module</a></li>
              </ul>
            </div>
            <div>
              <h3 className="text-lg font-semibold mb-4">Tech Stack</h3>
              <div className="flex flex-wrap gap-2">
                <span className="bg-gray-800 px-3 py-1 rounded-full text-sm">ROS 2</span>
                <span className="bg-gray-800 px-3 py-1 rounded-full text-sm">Gazebo</span>
                <span className="bg-gray-800 px-3 py-1 rounded-full text-sm">Unity</span>
                <span className="bg-gray-800 px-3 py-1 rounded-full text-sm">Isaac</span>
                <span className="bg-gray-800 px-3 py-1 rounded-full text-sm">VLA</span>
              </div>
            </div>
            <div>
              <h3 className="text-lg font-semibold mb-4">About</h3>
              <p className="text-gray-300">
                A comprehensive learning platform for humanoid robotics education,
                bridging digital intelligence and physical intelligence.
              </p>
            </div>
          </div>
          <div className="border-t border-gray-800 mt-8 pt-8 text-center text-gray-400">
            <p>© 2025 robot.ai. All rights reserved.</p>
          </div>
        </div>
      </footer>
     </Layout>
  );
}

export default LandingPage;