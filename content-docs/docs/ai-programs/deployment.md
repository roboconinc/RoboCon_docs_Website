# AI Program Deployment

Deploy your AI programs to ROBOCON robots.

## Installation from .deb Package (Recommended)

The primary deployment method is installing `.deb` packages, either from the marketplace or locally.

### From Marketplace

```bash
# Install via marketplace (when available)
# Marketplace will handle download and installation automatically
```

### From Local .deb File

```bash
# Download or copy .deb package to robot
scp robocon-my-ai-program_1.0.0_all.deb robot@robot-ip:~/

# Install package
ssh robot@robot-ip
sudo apt install ./robocon-my-ai-program_1.0.0_all.deb

# Verify installation
dpkg -l | grep robocon-my-ai-program
```

### From APT Repository (Future)

When packages are hosted in an APT repository:

```bash
# Add repository
echo "deb https://packages.roboconinc.com/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/robocon.list

# Install package
sudo apt update
sudo apt install robocon-my-ai-program
```

## Alternative Deployment Methods

### Direct File Deployment (Development)

For development and testing, you can deploy directly:

```bash
# Copy package source to robot
scp -r my_ai_program robot@robot-ip:/home/robot/workspace/

# Build on robot
ssh robot@robot-ip
cd ~/workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-select my_ai_program
```

### Container Deployment

For complex applications with many dependencies:

```bash
# Build container
docker build -t my_ai_program .

# Deploy to robot
docker push registry.roboconinc.com/my_ai_program

# On robot, pull and run
docker pull registry.roboconinc.com/my_ai_program
docker run --rm my_ai_program
```

## Runtime Execution

### Launch Installed Program

After installation via `.deb`, programs are available in ROS 2 workspace:

```bash
# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Launch program
ros2 launch my_ai_program app.launch.py
```

### ROBOCON OS Integration

Programs installed via `.deb` automatically integrate with ROBOCON OS:
- Registered with ROBOCON runtime manager
- Available through ROBOCON OS launcher
- Resource limits and security sandboxing applied
- Automatic dependency resolution

## Package Management

### Upgrade Package

```bash
# Upgrade to newer version
sudo apt update
sudo apt upgrade robocon-my-ai-program
```

### Remove Package

```bash
# Uninstall package
sudo apt remove robocon-my-ai-program

# Remove configuration files
sudo apt purge robocon-my-ai-program
```

### List Installed Programs

```bash
# List all ROBOCON packages
dpkg -l | grep robocon

# Check package info
dpkg -s robocon-my-ai-program
```

## Next Steps

- [Packaging](packaging.md) - Create .deb packages
- [Marketplace Integration](marketplace-integration.md) - Share on marketplace

