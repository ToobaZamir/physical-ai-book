// @ts-check
const prismThemes = require('prism-react-renderer').themes;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline:
    'A complete practitioner’s guide to ROS 2, Digital Twins, and Vision-Language-Action systems',

  // ✅ favicon file tumhare folder mein hai
  favicon: 'img/favicon.ico',
   url: 'https://toobazamir.github.io',
   baseUrl: '/physical-ai-book/',

  organizationName: 'ToobaZamir',
  projectName: 'physical-ai-book',


  onBrokenLinks: 'ignore',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          routeBasePath: '/docs',
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],

  themeConfig: {
    image: 'img/social-card.png',

    colorMode: {
      defaultMode: 'light',
      respectPrefersColorScheme: true,
    },

    // ✅ NAVBAR SAHI JAGAH PAR
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.png',
        height: 40,
      },
      items: [
        {
          to: '/docs/intro',
          label: 'Docs',
          position: 'left',
        },
      ],
    },

    // ✅ FOOTER
    footer: {
      style: 'dark',
      links: [
      
        {
          title: 'Community',
          items: [
            { label: 'GitHub', href: 'https://github.com/ToobaZamir/physical-ai-book' },
          ],
        },
      ],
      copyright: `© ${new Date().getFullYear()} Tooba Zamir — Physical AI & Humanoid Robotics.`,
    },

    // ✅ PRISM
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  },
};

module.exports = config;
