import React from 'react';
import Layout from '@theme/Layout';
import { Button } from '../ui/button';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '../ui/card';

const SlideLayout = ({
  title,
  description,
  children,
  currentSlide = 1,
  totalSlides = 1,
  onNext,
  onPrev,
  showNavigation = true,
  showProgress = true,
  breadcrumbs = [],
  moduleTitle = '',
  onModuleSelect
}) => {
  const progressPercentage = totalSlides > 0 ? Math.round((currentSlide / totalSlides) * 100) : 0;

  return (
    <Layout title={title} description={description}>
      <div className="min-h-screen bg-gray-50">
        {/* Header */}
        <header className="bg-white shadow-sm border-b">
          <div className="container mx-auto px-4 py-4">
            <div className="flex flex-col lg:flex-row lg:items-center lg:justify-between gap-4">
              {/* Breadcrumbs */}
              <div className="flex items-center space-x-2 text-sm text-gray-600">
                <a href="/" className="hover:text-gray-900">Home</a>
                <span>/</span>
                {moduleTitle && (
                  <>
                    <select
                      value={moduleTitle}
                      onChange={(e) => onModuleSelect && onModuleSelect(e.target.value)}
                      className="bg-transparent border-none text-gray-900 font-medium hover:text-blue-600 cursor-pointer"
                    >
                      <option>{moduleTitle}</option>
                    </select>
                    <span>/</span>
                  </>
                )}
                <span className="text-gray-900 font-medium">{title}</span>
              </div>

              {/* Progress indicator */}
              {showProgress && (
                <div className="flex items-center space-x-4">
                  <div className="text-sm text-gray-600">
                    Slide {currentSlide} of {totalSlides}
                  </div>
                  <div className="w-32 bg-gray-200 rounded-full h-2">
                    <div
                      className="bg-blue-600 h-2 rounded-full transition-all duration-300"
                      style={{ width: `${progressPercentage}%` }}
                    ></div>
                  </div>
                </div>
              )}
            </div>
          </div>
        </header>

        <div className="container mx-auto px-4 py-8">
          <div className="max-w-4xl mx-auto">
            {/* Slide Content */}
            <Card className="mb-8 shadow-sm">
              <CardHeader className="border-b bg-gray-50">
                <CardTitle className="text-2xl">{title}</CardTitle>
                {description && <CardDescription className="mt-2">{description}</CardDescription>}
              </CardHeader>
              <CardContent className="p-8">
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
                <div className="text-gray-600 text-sm">
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