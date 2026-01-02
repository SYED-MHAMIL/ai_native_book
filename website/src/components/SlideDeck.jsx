import React, { useState } from 'react';
import SlideLayout from './SlideLayout';

const SlideDeck = ({ title, description, slides }) => {
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

  return (
    <SlideLayout
      title={currentSlide.title || title}
      description={currentSlide.description || description}
      currentSlide={currentSlideIndex + 1}
      totalSlides={totalSlides}
      onNext={handleNext}
      onPrev={handlePrev}
      showNavigation={true}
    >
      {currentSlide.content}
    </SlideLayout>
  );
};

export default SlideDeck;