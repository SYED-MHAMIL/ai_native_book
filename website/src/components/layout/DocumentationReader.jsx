import React, { useState } from 'react';
import DocumentationLayout from './DocumentationLayout';

const DocumentationReader = ({
  title,
  description,
  content,
  breadcrumbs = [],
  toc = [],
  onPrev,
  onNext,
  showNavigation = true
}) => {
  const [fontSize, setFontSize] = useState('base');
  const [lineHeight, setLineHeight] = useState('normal');
  const [textWidth, setTextWidth] = useState('normal');

  const fontSizeClasses = {
    small: 'text-sm',
    base: 'text-base',
    large: 'text-lg',
    xlarge: 'text-xl'
  };

  const lineHeightClasses = {
    tight: 'leading-tight',
    normal: 'leading-normal',
    loose: 'leading-loose'
  };

  const textWidthClasses = {
    narrow: 'max-w-2xl',
    normal: 'max-w-4xl',
    wide: 'max-w-6xl'
  };

  const handleFontSizeChange = (size) => {
    setFontSize(size);
  };

  const handleLineHeightChange = (height) => {
    setLineHeight(height);
  };

  const handleTextWidthChange = (width) => {
    setTextWidth(width);
  };

  return (
    <DocumentationLayout
      title={title}
      description={description}
      breadcrumbs={breadcrumbs}
      toc={toc}
      showToc={toc.length > 0}
    >
      <div className={`${fontSizeClasses[fontSize]} ${lineHeightClasses[lineHeight]} ${textWidthClasses[textWidth]} mx-auto`}>
        <div className="prose prose-gray max-w-none">
          {content}
        </div>

        {/* Reading preferences */}
        <div className="mt-8 p-4 bg-gray-50 rounded-lg">
          <h3 className="font-medium text-gray-900 mb-3">Reading Preferences</h3>
          <div className="flex flex-wrap gap-4 items-center">
            <div className="flex items-center space-x-2">
              <label className="text-sm text-gray-600">Font Size:</label>
              <select
                value={fontSize}
                onChange={(e) => handleFontSizeChange(e.target.value)}
                className="border border-gray-300 rounded px-2 py-1 text-sm"
              >
                <option value="small">Small</option>
                <option value="base">Normal</option>
                <option value="large">Large</option>
                <option value="xlarge">Extra Large</option>
              </select>
            </div>

            <div className="flex items-center space-x-2">
              <label className="text-sm text-gray-600">Line Height:</label>
              <select
                value={lineHeight}
                onChange={(e) => handleLineHeightChange(e.target.value)}
                className="border border-gray-300 rounded px-2 py-1 text-sm"
              >
                <option value="tight">Tight</option>
                <option value="normal">Normal</option>
                <option value="loose">Loose</option>
              </select>
            </div>

            <div className="flex items-center space-x-2">
              <label className="text-sm text-gray-600">Text Width:</label>
              <select
                value={textWidth}
                onChange={(e) => handleTextWidthChange(e.target.value)}
                className="border border-gray-300 rounded px-2 py-1 text-sm"
              >
                <option value="narrow">Narrow</option>
                <option value="normal">Normal</option>
                <option value="wide">Wide</option>
              </select>
            </div>
          </div>
        </div>

        {/* Navigation */}
        {showNavigation && (onPrev || onNext) && (
          <div className="flex justify-between items-center mt-8 pt-8 border-t">
            <div>
              {onPrev && (
                <button
                  onClick={onPrev}
                  className="inline-flex items-center px-4 py-2 border border-gray-300 text-sm font-medium rounded-md text-gray-700 bg-white hover:bg-gray-50"
                >
                  ← Previous
                </button>
              )}
            </div>
            <div>
              {onNext && (
                <button
                  onClick={onNext}
                  className="inline-flex items-center px-4 py-2 border border-transparent text-sm font-medium rounded-md text-white bg-blue-600 hover:bg-blue-700"
                >
                  Next →
                </button>
              )}
            </div>
          </div>
        )}
      </div>
    </DocumentationLayout>
  );
};

export default DocumentationReader;