// @ts-check
/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',

    {
      type: 'category',
      label: 'Module 1: ROS 2 Mastery',
      items: [
        'module1/why-ros2',
        'module1/core-architecture',
        'module1/humanoid-packages',
      ],
    },

    {
      type: 'category',
      label: 'Module 2: Digital Twins',
      items: [
        'module2/gazebo',
        'module2/isaac-sim',
        'module2/unity',
      ],
    },

    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain',
      items: [
        'module3/isaac-ros-gems',
        'module3/perception',
        'module3/control',
      ],
    },

    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      items: [
        'module4/vla-planning',
        'module4/vla-grounding',
      ],
    },

    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        'capstone/index',
      ],
    },

    {
      type: 'category',
      label: 'Appendices',
      items: [
        'appendices/appendix-a',
        'appendices/appendix-b',
        'appendices/appendix-c',
        'appendices/appendix-d',
        'appendices/appendix-e',
      ],
    },
  ],
};

export default sidebars;
