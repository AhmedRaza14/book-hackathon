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
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Week 1: Introduction to Physical AI',
      items: [
        'week-01/theory',
        'week-01/code-lab',
        'week-01/simulation',
        'week-01/urdu-summary'
      ],
    },
    {
      type: 'category',
      label: 'Week 2: Mathematical Foundations',
      items: [
        'week-02/theory',
        'week-02/code-lab',
        'week-02/simulation',
        'week-02/urdu-summary'
      ],
    },
    {
      type: 'category',
      label: 'Week 3: Physics Simulation',
      items: [
        'week-03/theory',
        'week-03/code-lab',
        'week-03/simulation',
        'week-03/urdu-summary'
      ],
    },
    {
      type: 'category',
      label: 'Week 4: Digital Twins',
      items: [
        'week-04/theory',
        'week-04/code-lab',
        'week-04/simulation',
        'week-04/urdu-summary'
      ],
    },
    {
      type: 'category',
      label: 'Week 5: ROS 2 Fundamentals',
      items: [
        'week-05/theory',
        'week-05/code-lab',
        'week-05/simulation',
        'week-05/urdu-summary'
      ],
    },
    {
      type: 'category',
      label: 'Week 6: Advanced ROS 2 Concepts',
      items: [
        'week-06/theory',
        'week-06/code-lab',
        'week-06/simulation',
        'week-06/urdu-summary'
      ],
    },
    {
      type: 'category',
      label: 'Week 7: Sensor Integration',
      items: [
        'week-07/theory',
        'week-07/code-lab',
        'week-07/simulation',
        'week-07/urdu-summary'
      ],
    },
    {
      type: 'category',
      label: 'Week 8: Control Systems',
      items: [
        'week-08/theory',
        'week-08/code-lab',
        'week-08/simulation',
        'week-08/urdu-summary'
      ],
    },
    {
      type: 'category',
      label: 'Week 9: NVIDIA Isaac Platform',
      items: [
        'week-09/theory',
        'week-09/code-lab',
        'week-09/simulation',
        'week-09/urdu-summary'
      ],
    },
    {
      type: 'category',
      label: 'Week 10: Vision-Language Integration',
      items: [
        'week-10/theory',
        'week-10/code-lab',
        'week-10/simulation',
        'week-10/urdu-summary'
      ],
    },
    {
      type: 'category',
      label: 'Week 11: Cognitive Planning',
      items: [
        'week-11/theory',
        'week-11/code-lab',
        'week-11/simulation',
        'week-11/urdu-summary'
      ],
    },
    {
      type: 'category',
      label: 'Week 12: Advanced AI Integration',
      items: [
        'week-12/theory',
        'week-12/code-lab',
        'week-12/simulation',
        'week-12/urdu-summary'
      ],
    },
    {
      type: 'category',
      label: 'Week 13: Human-Robot Interaction',
      items: [
        'week-13/theory',
        'week-13/code-lab',
        'week-13/simulation',
        'week-13/urdu-summary'
      ],
    },
    {
      type: 'category',
      label: 'Week 14: Navigation and Manipulation',
      items: [
        'week-14/theory',
        'week-14/code-lab',
        'week-14/simulation',
        'week-14/urdu-summary'
      ],
    },
    {
      type: 'category',
      label: 'Week 15: Capstone Project',
      items: [
        'week-15/theory',
        'week-15/code-lab',
        'week-15/simulation',
        'week-15/urdu-summary'
      ],
    },
  ],
};

export default sidebars;
