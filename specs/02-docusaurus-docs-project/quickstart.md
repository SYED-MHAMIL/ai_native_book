# Quickstart Guide: Docusaurus Documentation Project

## Prerequisites

Before starting with the Docusaurus documentation project, ensure you have:

- Node.js (LTS version recommended, currently 18.x or higher)
- npm or yarn package manager
- Git for version control
- A code editor (VS Code recommended)

## Installation Steps

### 1. Clone or Create the Repository

```bash
# If starting fresh
mkdir physical-ai-book
cd physical-ai-book

# Initialize git repository
git init
```

### 2. Install Docusaurus

```bash
# Create a new Docusaurus project
npx create-docusaurus@latest website classic

# Navigate to the project directory
cd website
```

### 3. Project Structure Setup

After installation, your project should have this structure:

```
website/
├── blog/                 # Blog posts (optional)
├── docs/                 # Documentation files
├── src/
│   ├── components/       # Custom React components
│   ├── css/              # Custom styles
│   └── pages/            # Custom pages
├── static/               # Static files (images, etc.)
├── docusaurus.config.js  # Site configuration
├── sidebars.js          # Sidebar navigation
└── package.json         # Dependencies and scripts
```

### 4. Configure the Site

Edit `docusaurus.config.js` to match your book project:

```javascript
// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const lightCodeTheme = require('prism-react-renderer/themes/github');
const darkCodeTheme = require('prism-react-renderer/themes/dracula');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Bridging Digital Intelligence and Physical Intelligence',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-book-website.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub Pages: https://username.github.io/repo-name/
  baseUrl: '/physical-ai-book/',

  // GitHub pages deployment config
  organizationName: 'your-org', // Usually your GitHub org/user name
  projectName: 'physical-ai-book', // Usually your repo name
  deploymentBranch: 'gh-pages', // Branch that GitHub Pages will deploy from

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/your-org/physical-ai-book/tree/main/website/',
        },
        blog: {
          showReadingTime: true,
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/your-org/physical-ai-book/tree/main/website/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI Book',
        logo: {
          alt: 'Physical AI & Humanoid Robotics Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Documentation',
          },
          {to: '/blog', label: 'Blog', position: 'left'},
          {
            href: 'https://github.com/your-org/physical-ai-book',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Introduction',
                to: '/docs/intro',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/docusaurus',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/docusaurus',
              },
              {
                label: 'Twitter',
                href: 'https://twitter.com/docusaurus',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'Blog',
                to: '/blog',
              },
              {
                label: 'GitHub',
                href: 'https://github.com/your-org/physical-ai-book',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book. Built with Docusaurus.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
      },
    }),
};

module.exports = config;
```

### 5. Configure Sidebars

Update `sidebars.js` to reflect the book structure:

```javascript
// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'autogenerated',
      dirName: '.', // Generate sidebar from docs folder structure
    },
  ],
};

module.exports = sidebars;
```

### 6. Add Initial Content

Create the introduction page in `docs/intro.md`:

```markdown
---
sidebar_position: 1
---

# Introduction

Welcome to the Physical AI & Humanoid Robotics book! This comprehensive guide bridges digital intelligence and physical intelligence, providing you with the knowledge and tools needed to understand and build intelligent robotic systems.

## What You'll Learn

- The fundamentals of ROS 2 as the robotic nervous system
- Digital twin technologies using Gazebo and Unity
- NVIDIA Isaac for AI-powered robot brains
- Vision-Language-Action systems for advanced robotics

## Target Audience

This book is designed for:
- Robotics engineers and researchers
- AI and machine learning practitioners
- Students in robotics and AI fields
- Developers interested in embodied intelligence

## How to Use This Book

The content is organized in progressive modules:
1. Start with the fundamentals in Module 1
2. Progress through each module sequentially
3. Practice with the hands-on exercises
4. Apply concepts to real-world scenarios

Let's begin this journey into the fascinating world of Physical AI!
```

### 7. Run the Development Server

```bash
# Start the development server
npm run start

# This command starts a local development server and opens up a browser window.
# Most changes are reflected live without having to restart the server.
```

### 8. Build for Production

```bash
# Build the static site
npm run build

# This command generates static content into the build directory
# and can be served using any static hosting service.
```

### 9. Deploy to GitHub Pages

```bash
# Deploy to GitHub Pages
GIT_USER=<Your GitHub username> npm run deploy

# This command builds your site and pushes it to the gh-pages branch
```

## Next Steps

1. Add more content to the `docs/` directory following the structure defined in the data model
2. Customize the styling in `src/css/custom.css`
3. Add custom components in `src/components/`
4. Set up the module-specific content following the book constitution
5. Integrate with Claude Code for automated content generation
6. Prepare for AI integration with the RAG system