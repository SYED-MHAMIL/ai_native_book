import React from 'react';
import { LandingPage, DocumentationReader } from '../components';
import { ModuleCardGrid } from '../components/modules';
import { SlideLayout } from '../components/slides';
import { Tabs, TabsList, TabsTrigger, TabsContent, TabContent } from '../components/navigation';

const ExperienceDemo = () => {
  // Sample data for modules
  const modules = [
    {
      title: "ROS 2 Fundamentals",
      description: "Learn the basics of Robot Operating System 2, including nodes, topics, services, and actions.",
      lessons: 12,
      duration: "4 weeks",
      level: "Beginner",
      progress: 45,
      tags: ["ROS 2", "Basics", "Nodes"],
      onClick: () => console.log("ROS 2 Fundamentals clicked")
    },
    {
      title: "Gazebo Simulation",
      description: "Master robot simulation with Gazebo, including world creation, robot models, and physics engines.",
      lessons: 10,
      duration: "3 weeks",
      level: "Intermediate",
      progress: 0,
      tags: ["Gazebo", "Simulation", "Physics"],
      onClick: () => console.log("Gazebo Simulation clicked")
    },
    {
      title: "NVIDIA Isaac",
      description: "Explore NVIDIA Isaac for robotics development with GPU acceleration and AI capabilities.",
      lessons: 15,
      duration: "5 weeks",
      level: "Advanced",
      progress: 75,
      tags: ["NVIDIA", "AI", "GPU"],
      onClick: () => console.log("NVIDIA Isaac clicked")
    }
  ];

  // Sample documentation content
  const docContent = (
    <div>
      <h2>Experience Design Philosophy</h2>
      <p>Our educational platform follows enterprise-grade UI/UX principles to provide the best learning experience for robotics engineers.</p>

      <h3>Key Features</h3>
      <ul>
        <li>Progressive learning modules</li>
        <li>Interactive content</li>
        <li>Hands-on practice</li>
        <li>Real-world applications</li>
      </ul>

      <h3>Design Principles</h3>
      <p>Our interface follows modern design principles focused on clarity, accessibility, and user engagement.</p>
    </div>
  );

  // Sample slide content
  const slideContent = (
    <div>
      <h2>Slide Content</h2>
      <p>This is an example of a slide in our slide-based learning system.</p>
      <p>Each slide provides focused content with clear learning objectives.</p>
      <div className="mt-4 p-4 bg-blue-50 rounded-lg">
        <h3>Key Takeaway</h3>
        <p>Effective learning happens when content is well-structured and visually appealing.</p>
      </div>
    </div>
  );

  return (
    <div className="min-h-screen bg-gray-50">
      {/* Demo of Tabs Component */}
      <Tabs defaultValue="landing" className="p-4">
        <TabsList>
          <TabsTrigger value="landing">Landing Page</TabsTrigger>
          <TabsTrigger value="modules">Module Cards</TabsTrigger>
          <TabsTrigger value="documentation">Documentation</TabsTrigger>
          <TabsTrigger value="slides">Slides</TabsTrigger>
        </TabsList>

        <TabsContent value="landing">
          <LandingPage
            title="Experience Design Demo - Landing Page"
            description="Demonstrating the professional landing page UI component"
            modules={modules}
            ctaText="Explore Platform"
            heroTitle="Experience Design Demo"
            heroSubtitle="Showcasing enterprise-grade UI components for educational platforms"
          />
        </TabsContent>

        <TabContent value="modules">
          <div className="container mx-auto px-4 py-8">
            <h1 className="text-3xl font-bold text-gray-900 mb-6">Module Cards Demo</h1>
            <ModuleCardGrid modules={modules} />
          </div>
        </TabContent>

        <TabContent value="documentation">
          <DocumentationReader
            title="Experience Design Documentation"
            description="Detailed documentation about our experience design principles"
            content={docContent}
            breadcrumbs={[
              { label: 'Home', href: '/' },
              { label: 'Documentation', href: '/docs' },
              { label: 'Experience Design' }
            ]}
            toc={[
              { title: 'Overview', href: '#overview' },
              { title: 'Design Principles', href: '#principles' },
              { title: 'Components', href: '#components' },
              { title: 'Implementation', href: '#implementation' }
            ]}
          />
        </TabContent>

        <TabContent value="slides">
          <SlideLayout
            title="Slide Experience Demo"
            description="Demonstrating the slide-based learning experience"
            currentSlide={1}
            totalSlides={3}
            onNext={() => console.log('Next slide')}
            onPrev={() => console.log('Previous slide')}
          >
            {slideContent}
          </SlideLayout>
        </TabContent>
      </Tabs>
    </div>
  );
};

export default ExperienceDemo;