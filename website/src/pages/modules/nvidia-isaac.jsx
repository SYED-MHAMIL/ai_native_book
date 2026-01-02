import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '../../components/ui/card';
import { Button } from '../../components/ui/button';

function NvidiaIsaacSlides() {
  const [currentSlide, setCurrentSlide] = useState(0);

  const slides = [
    {
      title: "The AI-Robot Brain (NVIDIA Isaac)",
      content: (
        <div>
          <h2 className="text-2xl font-bold mb-4">Welcome to NVIDIA Isaac</h2>
          <p className="mb-4">
            Welcome to the third module of robot.ai, where we explore the AI brain of humanoid robots using NVIDIA Isaac.
          </p>
          <p className="mb-4">
            NVIDIA Isaac provides the perception and motion intelligence that enables robots to understand and interact with the world.
          </p>
          <h3 className="text-xl font-semibold mb-2">What You'll Learn</h3>
          <ul className="list-disc list-inside space-y-1 mb-4">
            <li>Isaac Sim synthetic data: Creating photoreal simulation and training data</li>
            <li>Isaac ROS accelerated VSLAM and navigation: Visual SLAM and path planning</li>
            <li>Nav2 for humanoid locomotion: Advanced navigation for walking robots</li>
          </ul>
        </div>
      )
    },
    {
      title: "Isaac Sim Synthetic Data",
      content: (
        <div>
          <h2 className="text-2xl font-bold mb-4">Isaac Sim Synthetic Data</h2>
          <p className="mb-4">
            Isaac Sim is a high-fidelity simulation environment for developing and testing AI robots with photorealistic rendering and physically accurate simulation.
          </p>
          <h3 className="text-xl font-semibold mb-2">Key Features</h3>
          <ul className="list-disc list-inside space-y-1 mb-4">
            <li>Photorealistic rendering: NVIDIA RTX technology for realistic sensor simulation</li>
            <li>Physically accurate simulation: Advanced physics for reliable transfer to reality</li>
            <li>Synthetic data generation: Create large datasets for training AI models</li>
            <li>Domain randomization: Improve model robustness through varied environments</li>
          </ul>
          <p className="mb-4">
            Synthetic data enables training of AI models without requiring physical data collection.
          </p>
        </div>
      )
    },
    {
      title: "Isaac ROS Accelerated VSLAM",
      content: (
        <div>
          <h2 className="text-2xl font-bold mb-4">Isaac ROS Accelerated VSLAM</h2>
          <p className="mb-4">
            Isaac ROS provides GPU-accelerated perception and navigation capabilities that run directly on ROS 2.
          </p>
          <h3 className="text-xl font-semibold mb-2">GPU Acceleration Benefits</h3>
          <ul className="list-disc list-inside space-y-1 mb-4">
            <li>Hardware-accelerated compute: Leverage GPU parallelism for real-time processing</li>
            <li>TensorRT optimization: Optimize deep learning models for inference</li>
            <li>CUDA acceleration: Parallel processing on GPU</li>
            <li>Production-ready: Optimized for deployment on edge devices</li>
          </ul>
          <p className="mb-4">
            VSLAM (Visual Simultaneous Localization and Mapping) enables robots to build maps while navigating.
          </p>
        </div>
      )
    },
    {
      title: "Nav2 for Humanoid Locomotion",
      content: (
        <div>
          <h2 className="text-2xl font-bold mb-4">Nav2 for Humanoid Locomotion</h2>
          <p className="mb-4">
            Navigation2 (Nav2) provides advanced navigation capabilities for mobile robots, including humanoid systems.
          </p>
          <h3 className="text-xl font-semibold mb-2">Navigation Components</h3>
          <ul className="list-disc list-inside space-y-1 mb-4">
            <li>Global planner: Path planning from start to goal</li>
            <li>Local planner: Dynamic obstacle avoidance</li>
            <li>Controller: Smooth trajectory execution</li>
            <li>Behavior trees: Complex navigation behaviors</li>
          </ul>
          <p className="mb-4">
            For humanoid robots, Nav2 can be adapted for bipedal locomotion and complex terrain navigation.
          </p>
        </div>
      )
    },
    {
      title: "AI Perception Systems",
      content: (
        <div>
          <h2 className="text-2xl font-bold mb-4">AI Perception Systems</h2>
          <p className="mb-4">
            NVIDIA Isaac provides state-of-the-art perception capabilities for robotic systems.
          </p>
          <h3 className="text-xl font-semibold mb-2">Perception Capabilities</h3>
          <ul className="list-disc list-inside space-y-1 mb-4">
            <li>Object detection and recognition: Real-time performance on edge devices</li>
            <li>Spatial understanding: SLAM, 3D reconstruction, and semantic segmentation</li>
            <li>Human-robot interaction: Pose estimation, gesture recognition, and speech processing</li>
            <li>Multimodal fusion: Combine data from multiple sensors</li>
          </ul>
          <p className="mb-4">
            These capabilities form the foundation of the AI robot brain for humanoid systems.
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
    <Layout title={currentSlideData.title} description="NVIDIA Isaac module slides">
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
                <CardDescription>Module 3: The AI-Robot Brain (NVIDIA Isaac)</CardDescription>
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
                  <a href="/docs/modules/02-gazebo-unity/intro">
                    <Button variant="outline">
                      ← Gazebo/Unity Module
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
                  <a href="/docs/modules/04-vla/intro">
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

export default NvidiaIsaacSlides;