# Marketplace Integration

How to package and submit AI programs to the ROBOCON Marketplace.

## Overview

The ROBOCON Marketplace (https://roboconinc.com/marketplace) allows developers to:

- Share AI programs with the community
- Monetize your applications
- Discover ready-to-use solutions
- Collaborate on projects

## Marketplace Structure

The marketplace hosts:

- **AI Programs**: Complete applications ready to deploy
- **Motion Routines**: Pre-built motion patterns
- **Behavior Trees**: Reusable behavior definitions
- **ROS 2 Packages**: ROS 2 node collections

## Packaging Your Application

### Create Package Manifest

Create `manifest.yaml`:

```yaml
name: my_robocon_app
version: 1.0.0
author: Your Name
description: Description of your application
category: navigation  # navigation, manipulation, inspection, etc.
tags:
  - autonomous
  - navigation
  - construction
license: MIT
requirements:
  - robocon_sdk >= 1.0.0
  - ros2_humble
dependencies:
  - rclpy
  - nav2_msgs
```

### Package Structure

```
my_robocon_app/
├── manifest.yaml
├── README.md
├── package.xml
├── setup.py
├── src/
│   └── my_app/
│       ├── __init__.py
│       └── nodes/
└── config/
    └── app_config.yaml
```

### Create README

Include comprehensive documentation:

```markdown
# My ROBOCON Application

## Description
Brief description of what the application does.

## Features
- Feature 1
- Feature 2

## Installation
```bash
# Installation instructions
```

## Usage
```python
# Usage examples
```

## Configuration
Configuration options and parameters.

## License
MIT
```

## Submitting to Marketplace

### Step 1: Build .deb Package

```bash
# Build ROS 2 package with colcon
colcon build --packages-select my_robocon_app

# Create .deb package (see Packaging guide for details)
bloom-generate rosdebian --os-name ubuntu --os-version jammy --ros-distro jazzy
fakeroot debian/rules binary

# Result: my_robocon_app_1.0.0_all.deb
```

### Step 2: Prepare Marketplace Submission

Ensure you have:
- ✅ `.deb` package file (e.g., `my_robocon_app_1.0.0_all.deb`)
- ✅ `manifest.yaml` with package metadata
- ✅ README.md with documentation
- ✅ Screenshots or demo videos (optional but recommended)

### Step 3: Upload to Marketplace

1. Visit https://roboconinc.com/marketplace/submit
2. Create a developer account
3. Upload your `.deb` package
4. Upload `manifest.yaml` (or fill in metadata via web form)
5. Upload README.md and media assets
6. Submit for review

### Step 4: Review Process

- Package validation
- Code review
- Security checks
- Documentation review

## Marketplace API

### Programmatic Submission

```python
from robocon_marketplace import MarketplaceClient

client = MarketplaceClient(api_key='your_api_key')

# Upload .deb package
result = client.upload_package(
    package_path='my_app_1.0.0_all.deb',  # .deb file
    manifest_path='manifest.yaml',
    readme_path='README.md',
    package_format='deb'  # Specify distribution format
)

print(f"Submission ID: {result.submission_id}")
```

### Update Existing Package

```python
# Update package version with new .deb
client.update_package(
    package_id='my_app',
    version='1.1.0',
    package_path='my_app_1.1.0_all.deb',  # New .deb file
    package_format='deb'
)
```

## Best Practices

### Documentation

- Clear installation instructions
- Usage examples
- Configuration guide
- Troubleshooting section

### Code Quality

- Follow ROS 2 conventions
- Include unit tests
- Handle errors gracefully
- Document code clearly

### Versioning

- Use semantic versioning
- Document breaking changes
- Maintain changelog

## Marketplace Guidelines

### Acceptable Content

- Original applications
- Open source packages
- Educational resources
- Community contributions

### Prohibited Content

- Malicious code
- Copyright violations
- Inappropriate content
- Non-functional packages

## Monetization

### Free Packages

Free packages are available to all users.

### Paid Packages

Set pricing for your application:

```yaml
# In manifest.yaml
pricing:
  type: one_time  # or subscription
  amount: 29.99   # USD
```

### Revenue Share

- Developer receives 70% of revenue
- ROBOCON takes 30% platform fee

## Discovery

Help users find your package:

- Use descriptive names
- Add relevant tags
- Write clear descriptions
- Include screenshots/videos

## Support

Provide support for your packages:

- GitHub repository
- Issue tracker
- Documentation site
- Community forum

## Next Steps

- [Packaging Guide](packaging.md) - Detailed packaging
- [Deployment Guide](deployment.md) - Deployment instructions
- [AI Programs Overview](overview.md) - AI program concepts

