# Data Model: Docusaurus Frontend & Landing Experience

## Entities

### LandingPage
- **id**: string (unique identifier)
- **title**: string ("Welcome to robot.ai")
- **description**: string (platform description)
- **ctaText**: string ("Explore Modules")
- **modules**: Module[] (Card of modules to display)

### Module
- **id**: string (unique identifier, e.g., "module-1-ros2")
- **title**: string (e.g., "The Robotic Nervous System (ROS 2)")
- **description**: string (one-line focus description)
- **buttonText**: string ("View Details / Slides")
- **contentPath**: string (path to module documentation)
- **icon**: string (optional icon identifier)
- **order**: number (display order)

### ModuleSlide
- **id**: string (unique identifier)
- **moduleId**: string (reference to parent module)
- **title**: string (slide title)
- **content**: string (markdown content)
- **order**: number (slide order within module)
- **type**: SlideType (e.g., "intro", "concept", "example", "summary")

### SlideType
- **values**: "intro" | "concept" | "example" | "summary" | "exercise"

### NavigationItem
- **id**: string (unique identifier)
- **label**: string (e.g., "Documentation")
- **path**: string (navigation path)
- **position**: "left" | "right" (position in navbar)

### FooterLink
- **id**: string (unique identifier)
- **label**: string (link text)
- **url**: string (destination URL)
- **type**: "doc" | "external" (link type)

## Relationships

```
LandingPage 1 -- * Module
Module 1 -- * ModuleSlide
LandingPage 1 -- * NavigationItem
LandingPage 1 -- * FooterLink
```

## Validation Rules

### LandingPage Validation
- title must be 5-50 characters
- description must be 10-200 characters
- must have exactly 4 modules
- ctaText must exist

### Module Validation
- title must be 5-100 characters
- description must be 10-150 characters
- buttonText must be 5-20 characters
- contentPath must be valid Docusaurus path format
- order must be 1-4

### ModuleSlide Validation
- title must be 5-100 characters
- content must not be empty
- order must be sequential within module
- type must be valid SlideType value

## State Transitions

### ModuleCard State
- **Initial**: Display title and description
- **Hover**: Highlight with visual feedback
- **Active**: Navigate to module content

### Slide State
- **Initial**: Current slide displayed
- **Next**: Transition to next slide
- **Previous**: Transition to previous slide
- **Complete**: Mark module as completed

## UI Components Mapping

### Landing Page Components
- **Header**: NavigationItem[] → Navbar
- **Hero**: LandingPage.title, LandingPage.description, LandingPage.ctaText
- **ModuleCards**: Module[] → Card components
- **Footer**: FooterLink[] → Footer section

### Documentation Components
- **SlideContainer**: ModuleSlide[] → Slide presentation
- **SlideNavigation**: ModuleSlide order → Next/Previous controls
- **ProgressIndicator**: ModuleSlide state → Progress tracking

## API Contracts (Future Integration)

### Module Service
- GET /api/modules - Retrieve all modules
- GET /api/modules/{id} - Retrieve specific module
- GET /api/modules/{id}/slides - Retrieve module slides

### User Progress Service
- POST /api/progress - Save user progress
- GET /api/progress/{userId} - Retrieve user progress
- PUT /api/progress/{progressId} - Update progress