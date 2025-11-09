import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Sidebar configuration for the Implementation docs plugin
 * This file is used by the implementation plugin which serves content from the implementation/ directory
 */

const sidebars: SidebarsConfig = {
  // Hardware Sidebar (Implementation plugin)
  hardwareSidebar: [
    {
      type: 'doc',
      id: 'hardware/index',
      label: 'Hardware Overview',
    },
    {
      type: 'category',
      label: 'Components',
      items: [
        {
          type: 'doc',
          id: 'hardware/robotic-arms',
        },
      ],
    },
    {
      type: 'category',
      label: 'Arm',
      items: [
        {
          type: 'doc',
          id: 'hardware/arm/borunte-brtirxz1515a',
        },
        {
          type: 'doc',
          id: 'hardware/arm/borunte-brtirus2550a',
        },
        {
          type: 'doc',
          id: 'hardware/arm/borunte-brtirxz0805a',
        },
      ],
    },
    {
      type: 'category',
      label: 'Robots',
      items: [
        {
          type: 'doc',
          id: 'hardware/robots/index',
        },
        {
          type: 'doc',
          id: 'hardware/robots/transporter-tracked-1000kg',
        },
        {
          type: 'doc',
          id: 'hardware/robots/transporter-wheeled-1000kg',
        },
      ],
    },
  ],
};

export default sidebars;

