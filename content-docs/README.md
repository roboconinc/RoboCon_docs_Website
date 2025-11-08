# ROBOCON SDK Documentation

Documentation website for the ROBOCON SDK, built with Docusaurus.

## Overview

The ROBOCON SDK is a comprehensive development toolkit for building intelligent applications that run on ROBOCON OS. ROBOCON OS is based on ROS 2 and Nav 2 architectures, with an additional DeepSeek LLM that intelligently interfaces sensors and behavior trees.

## Features

- **Unitree-Compatible API**: Low-level motor and driver control with an interface similar to Unitree robots
- **ROS 2 Integration**: Full ROS 2 support for standard robotics workflows
- **Nav 2 Navigation**: Autonomous navigation and path planning
- **DeepSeek LLM**: Intelligent sensor fusion and behavior tree management

## Local Development

```bash
# Install dependencies
npm install

# Start development server
npm start

# Build for production
npm run build

# Serve built site
npm run serve
```

## Documentation Structure

- **Getting Started**: Installation, quick start, first application
- **Architecture**: ROS 2, Nav 2, DeepSeek LLM, ROBOCON OS
- **API Reference**: Motor control, driver control, sensors, behavior trees
- **Motor Control**: Motion routines, low-level control, Unitree compatibility
- **ROS 2 Integration**: Nodes, topics, services, actions
- **Nav 2 Integration**: Path planning, navigation stack, costmaps
- **AI Programs**: Packaging, deployment, marketplace integration
- **Deployment**: Hardware setup, configuration, troubleshooting

## Deployment

The documentation site is configured for deployment at:

- **URL**: https://sdk.roboconinc.com
- **Base URL**: /

## Configuration

Main configuration file: `docusaurus.config.ts`

- Update `url` and `baseUrl` as needed
- Configure navigation in `sidebars.ts`
- Customize theme in `src/css/custom.css`

## Resources

- **Main Website**: https://www.roboconinc.com
- **Marketplace**: https://roboconinc.com/marketplace
- **Support**: https://support.roboconinc.com

## License

Copyright © ROBOCON INC. All rights reserved.
