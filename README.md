# ROBOCON SDK Documentation Website

This repository contains the Docusaurus-based documentation website for the ROBOCON SDK.

## Project Structure

```
RoboCon_SDK_Website/
├── contracts/              # Project contracts and agreements
├── content-docs/              # Docusaurus documentation site
│   ├── docs/             # Documentation markdown files
│   ├── src/              # React components and CSS
│   ├── static/           # Static assets
│   └── docusaurus.config.ts
└── README.md
```

## Quick Start

### Development

```bash
cd content-docs
npm install
npm start
```

The site will be available at `http://localhost:3000`

### Production Build

```bash
cd content-docs
npm run build
npm run serve
```

## Documentation Structure

The documentation is organized into the following sections:

1. **Getting Started** - Installation, quick start, first application
2. **Architecture** - ROS 2, Nav 2, DeepSeek LLM, ROBOCON OS
3. **API Reference** - Motor control, driver control, sensors, behavior trees
4. **Motor Control** - Motion routines, low-level control, Unitree compatibility
5. **ROS 2 Integration** - Nodes, topics, services, actions
6. **Nav 2 Integration** - Path planning, navigation stack, costmaps
7. **AI Programs** - Packaging, deployment, marketplace integration
8. **Deployment** - Hardware setup, configuration, troubleshooting

## Deployment

The documentation site is configured for deployment at:

- **URL**: https://sdk.roboconinc.com
- **Base URL**: /

For deployment instructions, see the contract in `contracts/contract_sdk_website_2025-10-30.md`

## Theme

The website theme is inspired by Unitree's documentation design, featuring:

- Clean, professional layout
- Blue and orange color scheme
- Responsive design
- Dark mode support

## Resources

- **Main Website**: https://www.roboconinc.com
- **Marketplace**: https://roboconinc.com/marketplace
- **Support**: https://support.roboconinc.com

## License

Copyright © ROBOCON INC. All rights reserved.

