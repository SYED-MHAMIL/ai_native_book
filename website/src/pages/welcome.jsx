import React from 'react';
import Layout from '@theme/Layout';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '../components/ui/card';
import { Button } from '../components/ui/button';

function WelcomeSlide() {
  return (
    <Layout title="Welcome to robot.ai" description="Welcome slide for the robot.ai learning platform">
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

        <div className="container mx-auto px-4 py-16">
          <div className="max-w-4xl mx-auto text-center">
            {/* Welcome Slide */}
            <Card className="mb-8">
              <CardHeader>
                <CardTitle className="text-3xl">Welcome to robot.ai</CardTitle>
                <CardDescription className="text-lg">
                  A comprehensive learning platform for humanoid robotics
                </CardDescription>
              </CardHeader>
              <CardContent>
                <div className="prose prose-gray max-w-none text-left">
                  <p className="text-lg mb-6">
                    This platform provides structured learning experiences for humanoid robotics,
                    combining ROS 2, simulation, AI, and multimodal systems.
                  </p>

                  <h3 className="text-xl font-semibold mb-4">Learning Journey</h3>
                  <ol className="list-decimal list-inside space-y-2 mb-6">
                    <li><strong>Module 1:</strong> The Robotic Nervous System (ROS 2)</li>
                    <li><strong>Module 2:</strong> The Digital Twin (Gazebo & Unity)</li>
                    <li><strong>Module 3:</strong> The AI-Robot Brain (NVIDIA Isaac)</li>
                    <li><strong>Module 4:</strong> Vision-Language-Action (VLA)</li>
                  </ol>

                  <p className="mb-6">
                    Each module contains structured slides with examples, exercises, and practical applications.
                  </p>

                  <div className="flex justify-center mt-8">
                    <a href="/docs/modules/01-ros2/intro">
                      <Button variant="default" size="lg">
                        Begin Learning →
                      </Button>
                    </a>
                  </div>
                </div>
              </CardContent>
            </Card>
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

export default WelcomeSlide;