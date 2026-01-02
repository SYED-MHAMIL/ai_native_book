import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '../../components/ui/card';
import { Button } from '../../components/ui/button';

function VlaSlides() {
  const [currentSlide, setCurrentSlide] = useState(0);

  const slides = [
    {
      title: "Vision-Language-Action (VLA)",
      content: (
        <div>
          <h2 className="text-2xl font-bold mb-4">Welcome to VLA Systems</h2>
          <p className="mb-4">
            Welcome to the fourth module of robot.ai, where we explore Vision-Language-Action (VLA) systems.
          </p>
          <p className="mb-4">
            VLA systems represent the cutting edge of integrated AI for humanoid robotics, combining perception, understanding, and action.
          </p>
          <h3 className="text-xl font-semibold mb-2">What You'll Learn</h3>
          <ul className="list-disc list-inside space-y-1 mb-4">
            <li>VLA concept overview: Understanding multimodal AI systems</li>
            <li>Role in humanoid interaction: How VLA enables human-like robot behavior</li>
            <li>Real-world application scenarios: Practical implementations of VLA systems</li>
          </ul>
        </div>
      )
    },
    {
      title: "VLA Concept Overview",
      content: (
        <div>
          <h2 className="text-2xl font-bold mb-4">VLA Concept Overview</h2>
          <p className="mb-4">
            Vision-Language-Action (VLA) systems combine visual perception, language understanding, and action execution in unified models.
          </p>
          <h3 className="text-xl font-semibold mb-2">Core Components</h3>
          <ul className="list-disc list-inside space-y-1 mb-4">
            <li><strong>Vision:</strong> Understanding the visual world through cameras and sensors</li>
            <li><strong>Language:</strong> Processing and understanding human language commands</li>
            <li><strong>Action:</strong> Executing physical behaviors based on perception and language</li>
          </ul>
          <p className="mb-4">
            These components work together to enable robots to understand and interact with the world in human-like ways.
          </p>
        </div>
      )
    },
    {
      title: "Multimodal Integration",
      content: (
        <div>
          <h2 className="text-2xl font-bold mb-4">Multimodal Integration</h2>
          <p className="mb-4">
            The key challenge in VLA systems is integrating information across different modalities effectively.
          </p>
          <h3 className="text-xl font-semibold mb-2">Integration Techniques</h3>
          <ul className="list-disc list-inside space-y-1 mb-4">
            <li>Shared embedding spaces: Representing different modalities in common representations</li>
            <li>Cross-attention mechanisms: Allowing modalities to attend to each other</li>
            <li>Fusion architectures: Combining information at different processing levels</li>
            <li>End-to-end learning: Training systems to perform all modalities jointly</li>
          </ul>
          <p className="mb-4">
            These techniques enable the creation of unified models that can process multiple types of information simultaneously.
          </p>
        </div>
      )
    },
    {
      title: "Humanoid Interaction",
      content: (
        <div>
          <h2 className="text-2xl font-bold mb-4">Humanoid Interaction</h2>
          <p className="mb-4">
            VLA systems enable humanoid robots to interact with humans in natural, intuitive ways.
          </p>
          <h3 className="text-xl font-semibold mb-2">Interaction Capabilities</h3>
          <ul className="list-disc list-inside space-y-1 mb-4">
            <li>Natural language commands: Following spoken instructions</li>
            <li>Visual scene understanding: Identifying objects and their relationships</li>
            <li>Context-aware behavior: Acting appropriately based on situation</li>
            <li>Learning from demonstration: Adapting to new tasks through observation</li>
          </ul>
          <p className="mb-4">
            These capabilities make humanoid robots more accessible and useful in human environments.
          </p>
        </div>
      )
    },
    {
      title: "Real-World Applications",
      content: (
        <div>
          <h2 className="text-2xl font-bold mb-4">Real-World Applications</h2>
          <p className="mb-4">
            VLA systems have numerous practical applications in humanoid robotics.
          </p>
          <h3 className="text-xl font-semibold mb-2">Application Scenarios</h3>
          <ul className="list-disc list-inside space-y-1 mb-4">
            <li>Assistive robotics: Helping elderly or disabled individuals</li>
            <li>Education: Interactive teaching assistants</li>
            <li>Healthcare: Patient care and rehabilitation support</li>
            <li>Customer service: Interactive receptionists and guides</li>
          </ul>
          <p className="mb-4">
            These applications demonstrate the potential of VLA systems to make robots more useful and accessible.
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
    <Layout title={currentSlideData.title} description="Vision-Language-Action module slides">
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
                <CardDescription>Module 4: Vision-Language-Action (VLA)</CardDescription>
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
                  <a href="/docs/modules/03-nvidia-isaac/intro">
                    <Button variant="outline">
                      ← NVIDIA Isaac Module
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
                  <a href="/">
                    <Button variant="default">
                      Return to Home →
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

export default VlaSlides;