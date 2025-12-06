import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // But you can create a sidebar manually
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: ROS 2',
      items: ['module1/chapter1', 'module1/chapter2'],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation',
      items: ['module2/chapter3', 'module2/chapter4', 'module2/chapter5'],
    },
    {
      type: 'category',
      label: 'Module 3: AI for Robotics',
      items: ['module3/chapter6', 'module3/chapter7'],
    },
    {
      type: 'category',
      label: 'Module 4: VLA & Capstone',
      items: ['module4/chapter8', 'module4/chapter9', 'module4/chapter10'],
    },
    'conclusion',
  ],
};

export default sidebars;
