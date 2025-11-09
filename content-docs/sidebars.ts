import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Creating a sidebar enables you to:
 * - create an ordered group of docs
 * - render a sidebar for each doc of that group
 * - provide next/previous navigation
 *
 * The sidebars can be generated from the filesystem, or explicitly defined here.
 */

const sidebars: SidebarsConfig = {
  // News Sidebar
  newsSidebar: [
    {
      type: 'doc',
      id: 'news/index',
      label: 'News Overview',
    },
    {
      type: 'category',
      label: 'Published',
      items: [
        'news/published',
      ],
    },
    {
      type: 'category',
      label: 'Drafts',
      items: [
        'news/drafts',
        'news/drafts/robocon-servicer-tracked-15kg-introduction',
      ],
    },
  ],
  // Tasks Sidebar
  tasksSidebar: [
    {
      type: 'doc',
      id: 'task/person/index',
      label: 'Contractors',
    },
    {
      type: 'category',
      label: 'Ammar Iqbal',
      items: [
        {
          type: 'category',
          label: 'Tasks',
          items: [
            'task/person/ammar-iqbal/news-events-page',
          ],
        },
      ],
    },
  ],
  // Hardware Manuals Sidebar
  hardwareManualsSidebar: [
    {
      type: 'doc',
      id: 'hardware-manuals/index',
      label: 'Robot Models',
    },
    {
      type: 'category',
      label: 'Mini Crane Tracked 3000kg',
      items: [
        'hardware-manuals/mini-crane-tracked-3000kg-hydraulic',
        'hardware-manuals/mini-crane-tracked-3000kg-hydraulic-questions',
      ],
    },
  ],
  // Implementation Sidebar
  implementationSidebar: [
    {
      type: 'category',
      label: 'Zero-Trust Consensus Protocol',
      items: [
        'zero-trust-consensus-protocol-implementation/overview',
        'zero-trust-consensus-protocol-implementation/network-client',
        'zero-trust-consensus-protocol-implementation/consensus-manager',
        'zero-trust-consensus-protocol-implementation/ledger-manager',
        'zero-trust-consensus-protocol-implementation/security-manager',
        'zero-trust-consensus-protocol-implementation/discovery-manager',
        'zero-trust-consensus-protocol-implementation/ros2-integration',
      ],
    },
    {
      type: 'category',
      label: 'TSBT-VLA System',
      items: [
        'tsbt-vla-system-implementation/overview',
        'tsbt-vla-system-implementation/track1-yolov11',
        'tsbt-vla-system-implementation/object-extruder',
        'tsbt-vla-system-implementation/3d-world-scene',
        'tsbt-vla-system-implementation/track2-ros2-sensors',
        'tsbt-vla-system-implementation/track3-ros2-user-input',
        'tsbt-vla-system-implementation/track4-deepseek-llm',
        'tsbt-vla-system-implementation/behavior-tree-execution',
      ],
    },
    {
      type: 'category',
      label: 'CAD-to-Behavior Engine',
      items: [
        'cad-to-behavior-engine-implementation/overview',
        'cad-to-behavior-engine-implementation/patent',
        'cad-to-behavior-engine-implementation/implementation-details',
      ],
    },
    {
      type: 'category',
      label: 'Robots',
      items: [
        'implementation/robots/servicer-tracked-arms',
      ],
    },
  ],
  // SDK Documentation Sidebar
  sdkSidebar: [
    {
      type: 'doc',
      id: 'getting-started/introduction',
      label: 'Introduction',
    },
    {
      type: 'category',
      label: 'Getting Started',
      items: [
        'getting-started/installation',
        'getting-started/quick-start',
        'getting-started/your-first-application',
      ],
    },
    {
      type: 'category',
      label: 'Architecture',
      items: [
        'architecture/ros2-overview',
        'architecture/nav2-integration',
        'architecture/robocon-os',
        'architecture/multi-robot-communication',
      ],
    },
    {
      type: 'category',
      label: 'Zero-Trust Consensus Protocol',
      items: [
        'zero-trust-consensus-protocol/overview',
        'zero-trust-consensus-protocol/api-reference',
        'zero-trust-consensus-protocol/discovery-flow',
        'zero-trust-consensus-protocol/voting-flow',
        'zero-trust-consensus-protocol/examples',
      ],
    },
    {
      type: 'category',
      label: 'TSBT-VLA System',
      items: [
        'architecture/tsbt-vla-system/overview',
        'architecture/tsbt-vla-system/patent',
        'architecture/tsbt-vla-system/3d-world-to-text',
        'architecture/tsbt-vla-system/sensor-to-text',
        'architecture/tsbt-vla-system/user-input-to-action',
        'architecture/tsbt-vla-system/llm-processing',
        'architecture/tsbt-vla-system/object-segmentation-module',
      ],
    },
    {
      type: 'category',
      label: 'API Reference',
      items: [
        'api-reference/motor-control',
        'api-reference/driver-control',
        'api-reference/sensor-interfaces',
        'api-reference/behavior-trees',
        'api-reference/ros2-topics-and-dds',
        'api-reference/multi-robot-communication',
      ],
    },
    {
      type: 'category',
      label: 'Behavior Tree Nodes',
      items: [
        'behavior-tree/node-reference',
        'behavior-tree/low-level-nodes',
        'behavior-tree/world_find',
        'behavior-tree/object_pick',
        'api-reference/behavior-tree-high-level-nodes',
      ],
    },
    {
      type: 'category',
      label: 'Motor Control',
      items: [
        'motor-control/basic-motion-routines',
        'motor-control/low-level-control',
        'motor-control/unitree-compatibility',
        'motor-control/examples',
      ],
    },
    {
      type: 'category',
      label: 'ROS 2 Integration',
      items: [
        'ros2/nodes-and-topics',
        'ros2/publishers-and-subscribers',
        'ros2/services-and-actions',
        'ros2/custom-messages',
      ],
    },
    {
      type: 'category',
      label: 'Nav 2 Integration',
      items: [
        'nav2/path-planning',
        'nav2/navigation-stack',
        'nav2/costmaps',
        'nav2/planners',
      ],
    },
    {
      type: 'category',
      label: 'AI Programs',
      items: [
        'ai-programs/overview',
        'ai-programs/packaging',
        'ai-programs/deployment',
        'ai-programs/marketplace-integration',
      ],
    },
    {
      type: 'category',
      label: 'Robots',
      items: [
        'robots/robot-models',
        {
          type: 'category',
          label: 'Crane Family',
          items: [
            'robots/crane-family',
            {
              type: 'category',
              label: 'Mini Crane Tracked 3000kg',
              items: [
                'robots/mini-crane-tracked-3000kg-hydraulic',
                'robots/mini-crane-tracked-3000kg-electric',
              ],
            },
            'robots/crane-family/crane-foldable-patent',
          ],
        },
        {
          type: 'category',
          label: 'Front Loader Tracked 300kg',
          items: [
            'robots/front-loader-tracked-300kg',
            'robots/front-loader-tracked-300kg-comparison',
            'robots/front-loader-tracked-300kg-patents',
          ],
        },
        {
          type: 'category',
          label: 'Excavator Tracked',
          items: [
            'robots/excavator-tracked',
            'robots/excavator-tracked-comparison',
            'robots/excavator-tracked-patents',
          ],
        },
        'robots/demolisher-tracked',
        'robots/transporter-tracked',
        'robots/transporter-wheeled',
        'robots/servicer-tracked',
        'robots/servicer-wheeled',
        'robots/sheather-tracked',
        'robots/sheather-wheeled',
      ],
    },
    {
      type: 'category',
      label: 'Deployment',
      items: [
        'deployment/raspberry-pi5-setup',
        'deployment/ai-computer-setup',
        'deployment/runtime-configuration',
        'deployment/troubleshooting',
      ],
    },
    {
      type: 'doc',
      id: 'can-bus',
      label: 'CAN Bus Hardware IDs',
    },
  ],
};

export default sidebars;
