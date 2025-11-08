# Packaging AI Programs

How to package your AI program for deployment and marketplace distribution.

## Distribution Format: .deb Packages

ROBOCON OS runs on **Ubuntu 24.04.2 LTS**, making **.deb (Debian) packages** the recommended distribution format for the marketplace. This provides:

- ✅ **Native Ubuntu integration** - Direct `apt` package manager support
- ✅ **Dependency management** - Automatic handling of ROS 2 and system dependencies
- ✅ **Easy installation** - Simple `sudo apt install ./package.deb` workflow
- ✅ **Version control** - Built-in versioning and upgrade mechanisms
- ✅ **Security** - Digital signatures for package verification
- ✅ **Marketplace-ready** - Perfect for app store-style distribution

> **Note:** `.deb` is the distribution format (like `.apk` for Android), while `colcon` is the build tool to compile your ROS 2 package. You use `colcon` to build, then package the result as a `.deb`.

## Package Structure

```
my_ai_program/
├── manifest.yaml          # ROBOCON marketplace metadata
├── package.xml            # ROS 2 package definition
├── setup.py               # Python package setup
├── debian/                # Debian packaging files
│   ├── control            # Package metadata & dependencies
│   ├── changelog          # Version history
│   ├── rules              # Build rules
│   └── postinst           # Post-installation script
└── src/
    └── my_app/
        ├── __init__.py
        └── nodes/
```

## Complete .deb Package Build Guide for Ubuntu 24.04.2 LTS

This guide walks through creating a `.deb` package from a ROS 2 package for ROBOCON OS.

### Prerequisites

Ensure you're working on Ubuntu 24.04.2 LTS (or compatible system):

```bash
# Check Ubuntu version
lsb_release -a

# Install essential packaging tools
sudo apt update
sudo apt install -y build-essential devscripts debhelper dh-python
sudo apt install -y python3-bloom python3-rosdep fakeroot
sudo apt install -y python3-colcon-common-extensions

# Verify ROS 2 Jazzy is installed
source /opt/ros/jazzy/setup.bash
ros2 --version
```

## Build and Package Workflow

### Step 1: Build with Colcon

First, build your ROS 2 package using colcon:

```bash
# Navigate to your workspace
cd ~/robocon_ws

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Build your package
colcon build --packages-select my_ai_program

# Source the workspace
source install/setup.bash
```

### Step 2: Create .deb Package

#### Option A: Using bloom (Recommended for ROS 2 Packages)

Bloom is the standard ROS 2 tool for creating Debian packages. It automatically handles ROS 2 dependencies and conventions.

```bash
# Navigate to your package directory
cd ~/robocon_ws/src/my_ai_program

# Generate Debian packaging files for Ubuntu 24.04 (Noble)
# Note: ROS 2 Jazzy uses Ubuntu 24.04 (noble), not jammy
bloom-generate rosdebian --os-name ubuntu --os-version noble --ros-distro jazzy

# This creates:
# - debian/control (package metadata and dependencies)
# - debian/changelog (version history)
# - debian/rules (build rules)
# - debian/compat (compatibility level)

# Review and edit debian/control if needed
# Add ROBOCON-specific dependencies:
# Depends: ros-jazzy-rclpy, ros-jazzy-nav2-msgs, behaviortree-cpp-v3

# Build the .deb package
fakeroot debian/rules binary

# The .deb file will be created in parent directory
# Example: ../robocon-my-ai-program_1.0.0-1_amd64.deb
```

**What bloom does:**
- Automatically extracts dependencies from `package.xml`
- Creates proper Debian control files
- Sets up ROS 2 installation paths
- Handles Python dependencies
- Generates version numbers

#### Option B: Manual Debian Packaging (Full Control)

For complete control over the packaging process, create Debian files manually:

```bash
# Navigate to package directory
cd ~/robocon_ws/src/my_ai_program

# Create debian directory
mkdir -p debian

# Create debian/control file
cat > debian/control << 'EOF'
Source: robocon-my-ai-program
Section: devel
Priority: optional
Maintainer: Your Name <your.email@example.com>
Build-Depends: debhelper (>= 11),
               dh-python,
               python3-all,
               cmake,
               libbehaviortree-cpp-dev,
               ros-jazzy-rclcpp,
               ros-jazzy-rclpy,
               python3-colcon-common-extensions
Standards-Version: 4.6.0

Package: robocon-my-ai-program
Architecture: any
Depends: ${python3:Depends},
         ${misc:Depends},
         ros-jazzy-rclpy (>= 3.3.0),
         ros-jazzy-rclcpp,
         ros-jazzy-nav2-msgs,
         ros-jazzy-geometry-msgs,
         ros-jazzy-sensor-msgs,
         libbehaviortree-cpp-v3,
         python3-numpy,
         python3-opencv
Description: ROBOCON AI Program - My AI Program
 A complete AI program for ROBOCON robots using BehaviorTree.CPP.
 This package provides autonomous navigation and task execution
 capabilities integrated with ROBOCON OS.
EOF

# Create debian/changelog
cat > debian/changelog << 'EOF'
robocon-my-ai-program (1.0.0-1) noble; urgency=low

  * Initial release
  * Red flag detection and autonomous digging application
  * BehaviorTree.CPP integration
  * ROS 2 Jazzy support

 -- Your Name <your.email@example.com>  Thu, 01 Jan 2025 12:00:00 +0000
EOF

# Create debian/rules (build instructions)
cat > debian/rules << 'EOF'
#!/usr/bin/make -f
%:
	dh $@ --with python3

override_dh_auto_configure:
	# Source ROS 2 environment and configure
	. /opt/ros/jazzy/setup.sh && \
	cmake -DCMAKE_INSTALL_PREFIX=/opt/ros/jazzy -B build -S .

override_dh_auto_build:
	# Build with ROS 2
	. /opt/ros/jazzy/setup.sh && \
	cmake --build build

override_dh_auto_install:
	# Install to staging directory
	. /opt/ros/jazzy/setup.sh && \
	DESTDIR=$(pwd)/debian/robocon-my-ai-program \
	cmake --install build
EOF
chmod +x debian/rules

# Create debian/compat
echo "11" > debian/compat

# Create source tarball
cd ..
tar czf robocon-my-ai-program_1.0.0.orig.tar.gz my_ai_program/

# Build package
cd my_ai_program
dpkg-buildpackage -us -uc -b

# Output: ../robocon-my-ai-program_1.0.0-1_amd64.deb
```

#### Option C: Using dpkg-buildpackage with ROS 2 Integration

```bash
# After colcon build, create debian directory
mkdir -p debian

# Create debian/control
cat > debian/control << EOF
Source: robocon-my-ai-program
Section: devel
Priority: optional
Maintainer: Your Name <email@example.com>
Build-Depends: debhelper (>= 11), dh-python, python3-all
Standards-Version: 4.1.3

Package: robocon-my-ai-program
Architecture: all
Depends: \${python3:Depends}, \${misc:Depends},
         ros-jazzy-rclpy,
         ros-jazzy-nav2-msgs,
         python3-numpy
Description: My ROBOCON AI Program
 A complete AI program for ROBOCON robots.
EOF

# Create debian/changelog
dch --create --package robocon-my-ai-program --newversion 1.0.0 "Initial release"

# Build
dpkg-buildpackage -us -uc
```

### Step 3: Verify Package

After building, verify the package integrity and contents:

```bash
# Navigate to where .deb was created
cd ~/robocon_ws

# Inspect package contents
dpkg -c robocon-my-ai-program_1.0.0-1_amd64.deb

# Check package metadata
dpkg -I robocon-my-ai-program_1.0.0-1_amd64.deb

# Check dependencies
dpkg-deb -f robocon-my-ai-program_1.0.0-1_amd64.deb Depends

# Verify package structure
dpkg -c robocon-my-ai-program_1.0.0-1_amd64.deb | grep -E "(opt/ros|usr/lib)"
```

### Step 4: Test Installation on Ubuntu 24.04.2 LTS

Before distributing, test installation:

```bash
# Install the package
sudo apt install ./robocon-my-ai-program_1.0.0-1_amd64.deb

# Verify installation
dpkg -l | grep robocon-my-ai-program

# Check installed files
dpkg -L robocon-my-ai-program

# Test the program (source ROS 2 first)
source /opt/ros/jazzy/setup.bash
ros2 run my_ai_program my_node

# Uninstall to test cleanup
sudo apt remove robocon-my-ai-program
```

## Manifest File (Marketplace Metadata)

Create `manifest.yaml` alongside your package for marketplace submission:

```yaml
name: my_ai_program
version: 1.0.0
author: Your Name
description: Description of your application
category: navigation
tags:
  - autonomous
  - navigation
  - construction
license: MIT
package_format: deb  # Distribution format
package_file: robocon-my-ai-program_1.0.0_all.deb
requirements:
  - robocon_sdk >= 1.0.0
  - ros2_humble
dependencies:
  - ros-jazzy-rclpy
  - ros-jazzy-nav2-msgs
```

## Alternative Distribution Formats

While `.deb` is recommended, other options exist:

### Docker Containers
- **Use case**: Complex applications with many dependencies
- **Pros**: Complete isolation, portable across Linux systems
- **Cons**: Larger size, requires Docker runtime

### Snap Packages
- **Use case**: Self-contained applications with strict confinement
- **Pros**: Cross-distribution compatibility, automatic updates
- **Cons**: Larger size, less integration with system packages

### AppImage
- **Use case**: Portable single-file applications
- **Pros**: No installation needed, portable
- **Cons**: No dependency management, larger files

## Best Practices

1. **Dependency Declaration**: Always declare ROS 2 and system dependencies in `debian/control`
2. **Versioning**: Use semantic versioning (e.g., 1.0.0, 1.0.1, 1.1.0)
3. **Installation Paths**: Follow ROS 2 conventions:
   - Executables: `/opt/ros/jazzy/lib/my_ai_program/`
   - Python modules: `/opt/ros/jazzy/lib/python3.10/site-packages/`
4. **Post-install Scripts**: Use `debian/postinst` to:
   - Register with ROBOCON OS
   - Set up launch files
   - Configure permissions

## Next Steps

- [Deployment](deployment.md) - Deploy packaged programs
- [Marketplace Integration](marketplace-integration.md) - Submit .deb packages to marketplace

