import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '../../components/ui/card';
import { Button } from '../../components/ui/button';

function GazeboUnitySlides() {
  const [currentSlide, setCurrentSlide] = useState(0);

  const slides = [
    {
      title: "The Digital Twin (Gazebo & Unity)",
      content: (
        <div>
          <h2 className="text-2xl font-bold mb-4">Welcome to Digital Twins</h2>
          <p className="mb-4">
            Welcome to the second module of robot.ai, where we explore digital twins - virtual replicas of physical robots and environments.
          </p>
          <p className="mb-4">
            Digital twins enable safe, cost-effective testing and training before deploying to physical hardware.
          </p>
          <h3 className="text-xl font-semibold mb-2">What You'll Learn</h3>
          <ul className="list-disc list-inside space-y-1 mb-4">
            <li>Physics & collision simulation: Understanding gravity and environment physics in Gazebo</li>
            <li>Environment design: Creating realistic test scenarios for humanoid robots</li>
            <li>Unity high-fidelity rendering: Advanced visualization capabilities</li>
            <li>Sensor simulation: LiDAR, depth cameras, and IMU simulation</li>
          </ul>
        </div>
      )
    },
    {
      title: "Physics & Collision Simulation",
      content: (
        <div>
          <h2 className="text-2xl font-bold mb-4">Physics & Collision Simulation</h2>
          <p className="mb-4">
            Gazebo provides sophisticated physics simulation that accurately models the dynamics of humanoid robots, their sensors, and their environments.
          </p>
          <h3 className="text-xl font-semibold mb-2">Physics Engine Components</h3>
          <ul className="list-disc list-inside space-y-1 mb-4">
            <li>Collision detection and response</li>
            <li>Rigid body dynamics</li>
            <li>Joint constraints and actuators</li>
            <li>Contact forces and friction</li>
          </ul>
          <p className="mb-4">
            These components work together to simulate realistic interactions between objects in the virtual environment.
          </p>
        </div>
      )
    },
    {
      title: "Environment Design",
      content: (
        <div>
          <h2 className="text-2xl font-bold mb-4">Environment Design</h2>
          <p className="mb-4">
            Creating realistic test scenarios is crucial for validating robot behavior before deployment to the physical world.
          </p>
          <h3 className="text-xl font-semibold mb-2">Environment Elements</h3>
          <ul className="list-disc list-inside space-y-1 mb-4">
            <li>Terrain: Various surface types and elevations</li>
            <li>Objects: Static and dynamic objects with different materials</li>
            <li>Lighting: Dynamic lighting conditions</li>
            <li>Weather: Environmental effects</li>
          </ul>
          <p className="mb-4">
            Well-designed environments enable comprehensive testing of robot capabilities.
          </p>
        </div>
      )
    },
    {
      title: "Unity High-Fidelity Rendering",
      content: (
        <div>
          <h2 className="text-2xl font-bold mb-4">Unity High-Fidelity Rendering</h2>
          <p className="mb-4">
            Unity provides advanced visualization capabilities with photorealistic rendering for creating immersive simulation experiences.
          </p>
          <h3 className="text-xl font-semibold mb-2">Unity Advantages</h3>
          <ul className="list-disc list-inside space-y-1 mb-4">
            <li>Advanced graphics and rendering</li>
            <li>Cross-platform deployment</li>
            <li>Powerful development tools</li>
            <li>Asset store with diverse models</li>
          </ul>
          <p className="mb-4">
            Unity's capabilities complement Gazebo's physics simulation for comprehensive digital twin development.
          </p>
        </div>
      )
    },
    {
      title: "Sensor Simulation",
      content: (
        <div>
          <h2 className="text-2xl font-bold mb-4">Sensor Simulation</h2>
          <p className="mb-4">
            Realistic sensor simulation is crucial for bridging the gap between simulation and reality.
          </p>
          <h3 className="text-xl font-semibold mb-2">Sensor Types</h3>
          <ul className="list-disc list-inside space-y-1 mb-4">
            <li><strong>LiDAR:</strong> 2D and 3D laser range finders</li>
            <li><strong>Depth Cameras:</strong> RGB and stereo cameras</li>
            <li><strong>IMU:</strong> Inertial measurement units</li>
            <li><strong>Force/Torque:</strong> Joint and contact force measurements</li>
          </ul>
          <p className="mb-4">
            Accurate sensor models ensure that algorithms trained in simulation perform well on real robots.
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
    <Layout title={currentSlideData.title} description="Gazebo and Unity module slides">
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
                <CardDescription>Module 2: The Digital Twin (Gazebo & Unity)</CardDescription>
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
                {currentSlide === 0 && (
                  <a href="/docs/modules/01-ros2/intro">
                    <Button variant="outline">
                      ← ROS 2 Module
                    </Button>
                  </a>
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
                  <a href="/docs/modules/03-nvidia-isaac/intro">
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

export default GazeboUnitySlides;