import React from 'react';
import Layout from '@theme/Layout';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '../components/ui/card';
import { Button } from '../components/ui/button';

const SlideLayout = ({ title, description, children, currentSlide, totalSlides, onNext, onPrev, showNavigation = true }) => {
  return (
    <Layout title={title} description={description}>
      <div className="min-h-screen bg-gray-50">
        {/* Slide Header */}
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
                <CardTitle>{title}</CardTitle>
                {description && <CardDescription>{description}</CardDescription>}
              </CardHeader>
              <CardContent>
                <div className="prose prose-gray max-w-none">
                  {children}
                </div>
              </CardContent>
            </Card>

            {/* Slide Navigation */}
            {showNavigation && (
              <div className="flex justify-between items-center mt-8">
                <div>
                  {onPrev && (
                    <Button variant="outline" onClick={onPrev}>
                      ← Previous
                    </Button>
                  )}
                </div>
                <div className="text-gray-600">
                  Slide {currentSlide} of {totalSlides}
                </div>
                <div>
                  {onNext && (
                    <Button variant="default" onClick={onNext}>
                      Next →
                    </Button>
                  )}
                </div>
              </div>
            )}
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
};

export default SlideLayout;