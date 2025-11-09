import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'RoboCon Docs',
  tagline: 'Internal website for RoboCon Contractors and Employees only',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://sdk.roboconinc.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'roboconinc', // Usually your GitHub org/user name.
  projectName: 'robocon-sdk', // Usually your repo name.

  onBrokenLinks: 'throw',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  markdown: {
    mermaid: true,
  },

  themes: ['@docusaurus/theme-mermaid'],

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Remove this to remove the "edit this page" links.
          // editUrl: 'https://github.com/roboconinc/robocon-sdk/tree/main/',
        },
        blog: false, // Disable blog for SDK documentation
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],
  plugins: [
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'implementation',
        path: 'implementation',
        routeBasePath: 'implementation',
        sidebarPath: './implementation-sidebars.ts',
      },
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'RoboCon Docs',
      logo: {
        alt: 'RoboCon Docs Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'sdkSidebar',
          position: 'left',
          label: 'Documentation',
        },
        {
          type: 'docSidebar',
          sidebarId: 'hardwareManualsSidebar',
          position: 'left',
          label: 'Hardware Manuals',
        },
        {
          type: 'docSidebar',
          sidebarId: 'implementationSidebar',
          position: 'left',
          label: 'Implementation',
        },
        {
          type: 'docSidebar',
          sidebarId: 'hardwareSidebar',
          position: 'left',
          label: 'Hardware',
          docsPluginId: 'implementation',
        },
        {
          type: 'docSidebar',
          sidebarId: 'newsSidebar',
          position: 'left',
          label: 'News',
        },
        {
          type: 'dropdown',
          label: 'Tasks',
          position: 'left',
          items: [
            {
              type: 'doc',
              docId: 'task/person/index',
              label: 'Person',
            },
          ],
        },
        {
          href: 'https://www.roboconinc.com',
          label: 'Main Website',
          position: 'right',
        },
        {
          href: 'https://roboconinc.com/marketplace',
          label: 'Marketplace',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Documentation',
          items: [
            {
              label: 'Getting Started',
              to: '/docs/getting-started/introduction',
            },
            {
              label: 'API Reference',
              to: '/docs/api-reference/motor-control',
            },
            {
              label: 'ROS 2 & Nav 2',
              to: '/docs/architecture/ros2-overview',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'Marketplace',
              href: 'https://roboconinc.com/marketplace',
            },
            {
              label: 'Main Website',
              href: 'https://www.roboconinc.com',
            },
            {
              label: 'Support',
              href: 'https://support.roboconinc.com',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/roboconinc/robocon-sdk',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} ROBOCON INC. All rights reserved.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
    mermaid: {
      theme: { light: 'neutral', dark: 'dark' },
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
