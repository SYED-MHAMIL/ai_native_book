import React from 'react';
import Layout from '@theme/Layout';
import { Button } from '../ui/button';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '../ui/card';

const LandingPage = ({
  title = 'robot.ai - Educational Robotics Platform',
  description = 'Learn robotics with structured modules covering ROS 2, Gazebo, NVIDIA Isaac, and Vision Language Actions',
  modules = [],
  ctaText = 'Get Started',
  onCtaClick,
  heroTitle,
  heroSubtitle
}) => {
  return (
    <Layout title={title} description={description}>
      <div className="min-h-screen bg-gradient-to-b from-gray-50 to-white">
        {/* Hero Section */}
        <section className="bg-gradient-to-r from-gray-900 to-gray-800 text-white">
          <div className="container mx-auto px-4 py-16 md:py-24">
            <div className="max-w-4xl mx-auto text-center">
              <h1 className="text-4xl md:text-6xl font-bold mb-6">
                {heroTitle || 'Master Robotics with Structured Learning'}
              </h1>
              <p className="text-xl md:text-2xl text-gray-200 mb-8">
                {heroSubtitle || 'Comprehensive educational modules covering ROS 2, Gazebo, NVIDIA Isaac, and Vision Language Actions'}
              </p>
              <div className="flex flex-col sm:flex-row gap-4 justify-center">
                <Button
                  size="lg"
                  className="bg-blue-600 hover:bg-blue-700 text-white px-8 py-3 text-lg"
                  onClick={onCtaClick}
                >
                  {ctaText}
                </Button>
                <Button
                  size="lg"
                  variant="outline"
                  className="border-white text-white hover:bg-white hover:text-gray-900 px-8 py-3 text-lg"
                  onClick={() => window.scrollTo({ top: document.querySelector('.modules-section')?.offsetTop - 100, behavior: 'smooth' })}
                >
                  Explore Modules
                </Button>
              </div>
            </div>
          </div>
        </section>

        {/* Modules Section */}
        <section className="modules-section py-16 bg-white">
          <div className="container mx-auto px-4">
            <div className="text-center mb-12">
              <h2 className="text-3xl md:text-4xl font-bold text-gray-900 mb-4">
                Learning Modules
              </h2>
              <p className="text-lg text-gray-600 max-w-2xl mx-auto">
                Structured educational content designed to take you from beginner to advanced robotics concepts
              </p>
            </div>

            <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-8 max-w-6xl mx-auto">
              {modules.map((module, index) => (
                <Card key={index} className="h-full hover:shadow-lg transition-shadow duration-200">
                  <CardHeader>
                    <CardTitle className="flex items-center gap-3">
                      <div className="w-10 h-10 bg-blue-100 rounded-lg flex items-center justify-center">
                        <span className="text-blue-600 font-bold">{index + 1}</span>
                      </div>
                      <span>{module.title}</span>
                    </CardTitle>
                    <CardDescription>{module.description}</CardDescription>
                  </CardHeader>
                  <CardContent>
                    <div className="space-y-4">
                      <div className="flex items-center justify-between text-sm text-gray-500">
                        <span>Lessons: {module.lessons}</span>
                        <span>Duration: {module.duration}</span>
                      </div>
                      <div className="pt-4">
                        <Button
                          variant="outline"
                          className="w-full"
                          onClick={() => module.onClick && module.onClick()}
                        >
                          Start Learning
                        </Button>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              ))}
            </div>
          </div>
        </section>

        {/* Features Section */}
        <section className="py-16 bg-gray-50">
          <div className="container mx-auto px-4">
            <div className="text-center mb-12">
              <h2 className="text-3xl md:text-4xl font-bold text-gray-900 mb-4">
                Why Learn with robot.ai?
              </h2>
              <p className="text-lg text-gray-600 max-w-2xl mx-auto">
                Professional-grade educational content designed for comprehensive robotics learning
              </p>
            </div>

            <div className="grid grid-cols-1 md:grid-cols-3 gap-8 max-w-6xl mx-auto">
              <div className="text-center p-6">
                <div className="w-16 h-16 bg-blue-100 rounded-full flex items-center justify-center mx-auto mb-4">
                  <svg className="w-8 h-8 text-blue-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 6.253v13m0-13C10.832 5.477 9.246 5 7.5 5S4.168 5.477 3 6.253v13C4.168 18.477 5.754 18 7.5 18s3.332.477 4.5 1.253m0-13C13.168 5.477 14.754 5 16.5 5c1.746 0 3.332.477 4.5 1.253v13C19.832 18.477 18.246 18 16.5 18c-1.746 0-3.332.477-4.5 1.253" />
                  </svg>
                </div>
                <h3 className="text-xl font-semibold mb-2">Structured Learning</h3>
                <p className="text-gray-600">Progressive modules that build upon each other for comprehensive understanding</p>
              </div>

              <div className="text-center p-6">
                <div className="w-16 h-16 bg-green-100 rounded-full flex items-center justify-center mx-auto mb-4">
                  <svg className="w-8 h-8 text-green-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9.663 17h4.673M12 3v1m6.364 1.636l-.707.707M21 12h-1M4 12H3m3.343-5.657l-.707-.707m2.828 9.9a5 5 0 117.072 0l-.548.547A3.374 3.374 0 0014 18.469V19a2 2 0 11-4 0v-.531c0-.895-.356-1.754-.988-2.386l-.548-.547z" />
                  </svg>
                </div>
                <h3 className="text-xl font-semibold mb-2">Hands-on Practice</h3>
                <p className="text-gray-600">Practical exercises and real-world scenarios to reinforce learning</p>
              </div>

              <div className="text-center p-6">
                <div className="w-16 h-16 bg-purple-100 rounded-full flex items-center justify-center mx-auto mb-4">
                  <svg className="w-8 h-8 text-purple-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
                  </svg>
                </div>
                <h3 className="text-xl font-semibold mb-2">Modern Technologies</h3>
                <p className="text-gray-600">Learn with cutting-edge tools like ROS 2, Gazebo, NVIDIA Isaac, and more</p>
              </div>
            </div>
          </div>
        </section>

        {/* Footer */}
        <footer className="bg-gray-900 text-white py-12">
          <div className="container mx-auto px-4">
            <div className="text-center">
              <h3 className="text-2xl font-bold mb-4">robot.ai</h3>
              <p className="text-gray-400 mb-6">
                Empowering the next generation of robotics engineers
              </p>
              <div className="flex justify-center space-x-6">
                <a href="#" className="text-gray-400 hover:text-white transition-colors">Documentation</a>
                <a href="#" className="text-gray-400 hover:text-white transition-colors">Modules</a>
                <a href="#" className="text-gray-400 hover:text-white transition-colors">About</a>
              </div>
              <p className="text-gray-500 mt-8">
                © {new Date().getFullYear()} robot.ai. All rights reserved.
              </p>
            </div>
          </div>
        </footer>
      </div>
    </Layout>
  );
};

export default LandingPage;