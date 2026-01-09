import React, { useState } from 'react';
import SlideLayout from './SlideLayout';

const SlideDeck = ({
  title,
  description,
  slides,
  moduleTitle = '',
  onModuleSelect
}) => {
  const [currentSlideIndex, setCurrentSlideIndex] = useState(0);

  const currentSlide = slides[currentSlideIndex];
  const totalSlides = slides.length;

  const handleNext = () => {
    if (currentSlideIndex < totalSlides - 1) {
      setCurrentSlideIndex(currentSlideIndex + 1);
    }
  };

  const handlePrev = () => {
    if (currentSlideIndex > 0) {
      setCurrentSlideIndex(currentSlideIndex - 1);
    }
  };

  const handleSlideSelect = (index) => {
    if (index >= 0 && index < slides.length) {
      setCurrentSlideIndex(index);
    }
  };

  return (
    <SlideLayout
      title={currentSlide.title || title}
      description={currentSlide.description || description}
      currentSlide={currentSlideIndex + 1}
      totalSlides={totalSlides}
      onNext={handleNext}
      onPrev={handlePrev}
      showNavigation={true}
      showProgress={true}
      moduleTitle={moduleTitle}
      onModuleSelect={onModuleSelect}
    >
      {currentSlide.content}
    </SlideLayout>
  );
};

export default SlideDeck;