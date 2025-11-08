# Installation

This guide will help you install the ROBOCON SDK on your development machine.

## Prerequisites

Before installing the ROBOCON SDK, ensure you have:

- **Operating System**: Ubuntu 24.04.2 LTS
- **ROS 2**: Jazzy Jalisco
- **DDS**: Cyclone DDS
- **Python**: 3.10 or higher
- **CMake**: 3.8 or higher
- **Git**: For cloning repositories

### Platform-Specific Requirements

**For Raspberry Pi 5 (Boot Computer):**
- Raspberry Pi 5 with Waveshare Isolated CAN Bus Hat
- CAN utilities: `can-utils`, `python3-can`
- RS485 ports configured: `/dev/ttySC0`, `/dev/ttySC1`
- QT5 for GUI: `python3-pyqt5`

**For AI Computer:**
- ASRock B450M-HDV R4.0 AM4 (or compatible)
- GIGABYTE Radeon RX 7600 XT (or compatible AMD GPU)
- ROCm SDK for GPU acceleration
- Python ML libraries: `torch`, `transformers`

## Installation Methods

## ROS 2 Jazzy Installation

### Step 1: Set Locale

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

### Step 2: Add ROS 2 Repository

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 3: Install ROS 2 Jazzy

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop
sudo apt install ros-jazzy-ros-base
```

### Step 4: Configure DDS

```bash
# Set Cyclone DDS as the default DDS implementation
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
```

## ROBOCON OS Installation

### Source Installation

```bash
# Create workspace
mkdir -p ~/robocon_ws/src
cd ~/robocon_ws/src

# Clone ROBOCON OS repository
git clone https://github.com/roboconinc/Robocon-OS.git

# For Raspberry Pi 5: Clone minimal components
# For AI Computer: Clone all components
git clone https://github.com/roboconinc/RoboCon_Edge-LLM.git
git clone https://github.com/roboconinc/RoboCon_World-Server.git
git clone https://github.com/roboconinc/RoboCon_Network-Client.git
git clone https://github.com/roboconinc/RoboCon_CV.git

# Install dependencies
cd ~/robocon_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash
echo "source ~/robocon_ws/install/setup.bash" >> ~/.bashrc
```

### Platform-Specific Installation

See the detailed installation guides:
- [Raspberry Pi 5 Setup](../deployment/raspberry-pi5-setup.md)
- [AI Computer Setup](../deployment/ai-computer-setup.md)

## Verify Installation

After installation, verify that everything is set up correctly:

```bash
# Check ROS 2 environment
source /opt/ros/jazzy/setup.bash
printenv | grep ROS

# Check DDS implementation
echo $RMW_IMPLEMENTATION  # Should output: rmw_cyclonedds_cpp

# Source ROBOCON workspace
source ~/robocon_ws/install/setup.bash

# Check for installed packages
ros2 pkg list | grep robocon

# List available topics
ros2 topic list
```

Expected packages include:
- `robot_mini_crane_bringup`
- `motor_driver_ids830abs`
- `golden_motor_eza48400`
- `sensor_bw_mins50`
- `lidar_hinson_se_1035`
- `depth_camera_oak_d_s2`
- `robocon_hw_monitor_gui`

### Hardware-Specific Verification

**For Raspberry Pi 5:**
```bash
# Check CAN bus
ip link show can0

# Check RS485 ports
ls -la /dev/ttySC*

# Test CAN bus
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
candump can0
```

**For AI Computer:**
```bash
# Verify GPU
lspci | grep -i amd

# Test Edge-LLM
python3 -m robocon_edge_llm.server --test

# Test CV components
ros2 run robocon_cv perception_node --test
```

## Development Dependencies

For full development capabilities, install additional tools:

**For Raspberry Pi 5:**
```bash
# Install CAN utilities
sudo apt install can-utils python3-can

# Install GUI dependencies
sudo apt install python3-pyqt5

# Install serial communication
sudo apt install python3-pip
python3 -m pip install pyserial pymodbus
```

**For AI Computer:**
```bash
# Install development tools
sudo apt install \
  python3-pip \
  python3-colcon-common-extensions \
  ros-jazzy-desktop \
  build-essential \
  cmake \
  git

# Install Python ML dependencies (careful with numpy version)
python3 -m pip install "numpy<2.0.0"
python3 -m pip install torch transformers opencv-python pyyaml setuptools

# Install ROCm for AMD GPU support
sudo apt install rocm-hip-sdk rocm-opencl-sdk
```

## Next Steps

Once installation is complete:

- [Quick Start Guide](quick-start.md) - Run your first example
- [ROS 2 Overview](../architecture/ros2-overview.md) - Understand the architecture
- [Motor Control](../motor-control/basic-motion-routines.md) - Start controlling motors

## Troubleshooting

### Common Issues

**Issue**: `robocon-sdk: command not found`

**Solution**: Ensure the workspace is sourced:
```bash
source ~/robocon_ws/install/setup.bash
```

**Issue**: ROS 2 packages not found

**Solution**: Verify ROS 2 installation:
```bash
printenv | grep ROS_DISTRO
```

**Issue**: Permission errors during installation

**Solution**: Use `sudo` for system-wide installation or ensure proper permissions for user installation.

## Support

If you encounter issues not covered here:

- Check the [Troubleshooting Guide](../deployment/troubleshooting.md)
- Visit our [Support Portal](https://support.roboconinc.com)
- Open an issue on [GitHub](https://github.com/roboconinc/robocon-sdk/issues)

