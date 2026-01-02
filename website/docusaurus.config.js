// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'robot.ai',
  tagline: 'A comprehensive learning platform for humanoid robotics',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-username.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/ai_book_hack_I/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'mohamil', // Usually your GitHub org/user name.
  projectName: 'ai-book-hack', // Usually your repo name.

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
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
        editUrl:
          'https://github.com/SYED-MHAMIL/ai_native_book/tree/main/website/',
      },
      blog: false,
      theme: {
        customCss: [
          require.resolve('./src/css/tailwind.css'),
          require.resolve('./src/css/custom.css'),
        ],
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
        title: 'robot.ai',
        logo: {
          alt: 'robot.ai Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            to: '/docs/overview/welcome',
            label: 'Documentation',
            position: 'right'
          },
          {
            href: 'https://github.com/mohamil/ai-book-hack',
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
              {
                label: 'Module 1: ROS 2',
                to: '/docs/module-1-ros2',
              },
              {
                label: 'Module 2: Simulation',
                to: '/docs/module-2-digital-twin',
              },
              {
                label: 'Module 3: AI Brain',
                to: '/docs/module-3-isaac',
              },
              {
                label: 'Module 4: VLA',
                to: '/docs/module-4-vla',
              },
            ],
          },
          {
            title: 'Modules',
            items: [
              {
                label: 'The Robotic Nervous System',
                to: '/docs/module-1-ros2',
              },
              {
                label: 'The Digital Twin',
                to: '/docs/module-2-digital-twin',
              },
              {
                label: 'The AI-Robot Brain',
                to: '/docs/module-3-isaac',
              },
              {
                label: 'Vision-Language-Action',
                to: '/docs/module-4-vla',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/SYED-MHAMIL/ai_native_book',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} robot.ai. All rights reserved.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;