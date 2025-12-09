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
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Foundations',
      items: [
        'foundations/physics-fundamentals',
        'foundations/ai-fundamentals',
        'foundations/maths-prerequisites',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Core Concepts',
      items: [
        'core/physical-ai-integration',
        'core/modeling-physical-systems',
        'core/neural-odes',
        'core/lagrangian-nets',
        'core/hamiltonian-nets',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Applications',
      items: [
        'applications/robotics',
        'applications/control-systems',
        'applications/optimization',
        'applications/simulation',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Advanced Topics',
      items: [
        'advanced/pde-learning',
        'advanced/physics-informed-nns',
        'advanced/uncertainty-quantification',
        'advanced/reinforcement-learning',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Case Studies',
      items: [
        'case-studies/pendulum-modeling',
        'case-studies/fluid-dynamics',
        'case-studies/molecular-dynamics',
        'case-studies/robot-control',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Future Directions',
      items: [
        'future/research-trends',
        'future/challenges',
      ],
      collapsed: false,
    },
  ],
};

export default sidebars;
