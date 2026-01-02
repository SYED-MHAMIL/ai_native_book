# Quickstart: Docusaurus Frontend & Landing Experience

## Prerequisites

- Node.js 18+ installed
- npm or yarn package manager
- Basic knowledge of React and TypeScript

## Setup Instructions

### 1. Initialize Docusaurus Project

```bash
npx create-docusaurus@latest website classic
cd website
```

### 2. Install TypeScript Support

```bash
npm install --save-dev typescript @types/react @types/node
```

### 3. Install Tailwind CSS

```bash
npm install -D tailwindcss postcss autoprefixer
npx tailwindcss init -p
```

Add to `tailwind.config.js`:
```js
module.exports = {
  content: [
    "./src/**/*.{js,jsx,ts,tsx}",
    "./docs/**/*.{md,mdx}",
  ],
  theme: {
    extend: {},
  },
  plugins: [],
}
```

Add to `src/css/custom.css`:
```css
@tailwind base;
@tailwind components;
@tailwind utilities;
```

### 4. Install shadcn UI Components

```bash
npx shadcn@latest init
npx shadcn@latest add card button tabs navigation-menu badge
```

### 5. Update Docusaurus Configuration

Modify `docusaurus.config.js`:
```js
module.exports = {
  // ... other config
  themeConfig: {
    navbar: {
      title: 'robot.ai',
      items: [
        {
          to: '/docs/overview/welcome',
          label: 'Documentation',
          position: 'right',
        },
      ],
    },
  },
};
```

### 6. Create Landing Page

Create `src/pages/index.tsx`:
```tsx
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <main>
        {/* Landing page content with header, hero, module cards, and footer */}
      </main>
    </Layout>
  );
}
```

### 7. Start Development Server

```bash
npm run start
```

## Key Features Implementation

### Module Cards Component
- Create a responsive grid of module cards
- Each card displays title, description, and "View Details" button
- Cards link to respective module documentation

### Slide Navigation
- Implement slide-like navigation for module documentation
- Add previous/next buttons for sequential learning
- Track progress through modules

### Responsive Design
- Ensure layout works on mobile, tablet, and desktop
- Use Tailwind's responsive utility classes
- Test across different screen sizes

## Running the Application

### Development
```bash
npm run start
```

### Build
```bash
npm run build
```

### Serve Build Locally
```bash
npm run serve
```

## Deployment

The site can be deployed to GitHub Pages following Docusaurus deployment guidelines:

1. Configure `docusaurus.config.js` with your site's URL
2. Run `npm run build`
3. Deploy the `build` folder to GitHub Pages

## Troubleshooting

### Common Issues
- If Tailwind styles don't appear, ensure the content paths in `tailwind.config.js` include Docusaurus files
- If shadcn components don't work, check that they're properly installed and imported
- For module resolution issues, verify TypeScript configuration

### Testing
- Test the landing page on different devices
- Verify all navigation links work correctly
- Ensure documentation sections load properly