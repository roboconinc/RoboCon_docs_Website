# Raspberry Pi 5 Setup

Complete setup guide for Raspberry Pi 5 as the boot computer for ROBOCON OS.

## Hardware Requirements

- **Raspberry Pi 5**
- **Waveshare Isolated CAN Bus Hat**
- **MicroSD Card** (32GB or larger, recommended 64GB)
- **Power Supply**: 5V 5A USB-C power adapter

## Operating System Installation

### Step 1: Install Ubuntu 24.04.2 LTS

1. Download Ubuntu 24.04.2 LTS Server image for Raspberry Pi 5
2. Flash to microSD card using Raspberry Pi Imager or `dd`
3. Insert microSD card and boot Raspberry Pi 5

### Step 2: Initial Configuration

```bash
# Update system
sudo apt update && sudo apt upgrade

# Install essential packages
sudo apt install -y git python3-pip python3-venv build-essential cmake
sudo apt install -y python3-colcon-common-extensions
sudo apt install -y python3-pyqt5 python3-opencv
sudo apt install -y can-utils python3-can
```

## ROS 2 Jazzy Installation

Follow the standard ROS 2 Jazzy installation (see [Installation Guide](../getting-started/installation.md)).

```bash
# Set locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository and install
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-jazzy-ros-base ros-jazzy-rclpy

# Configure DDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
```

## CAN Bus Configuration

### Setup Waveshare CAN Bus Hat

```bash
# Enable CAN interface (usually auto-detected)
# Check if can0 interface exists
ip link show can0

# If not, enable SPI and configure device tree
# Edit /boot/firmware/config.txt to enable SPI and CAN overlay
```

### Configure CAN Bus

```bash
# Setup CAN bus (500kbps standard)
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# Test CAN bus
candump can0

# Make persistent (add to /etc/network/interfaces or systemd)
```

## RS485 Configuration

The Waveshare CAN Bus Hat also provides RS485 ports:

```bash
# Check RS485 ports
ls -la /dev/ttySC*

# Set permissions
sudo chmod 666 /dev/ttySC0
sudo chmod 666 /dev/ttySC1

# Add user to dialout group for serial access
sudo usermod -a -G dialout $USER
```

## ROBOCON OS Installation

### Clone Repository

```bash
# Create workspace
mkdir -p ~/robocon_ws/src
cd ~/robocon_ws/src

# Clone ROBOCON OS
git clone https://github.com/roboconinc/Robocon-OS.git

# Clone minimal components for Raspberry Pi 5
git clone https://github.com/roboconinc/RoboCon_Network-Client.git
```

### Build Hardware Packages

```bash
cd ~/robocon_ws

# Build hardware-specific packages
colcon build --packages-select \
  robot_mini_crane_bringup \
  motor_driver_ids830abs \
  golden_motor_eza48400 \
  sensor_bw_mins50 \
  energy_meter_acrel_amc16_dett \
  energy_meter_acrel_amc16z_fdk24 \
  controller_jpf4816_fan_speed \
  relay_xinlihui_n4roc04_24v \
  pressure_sensor_ps_1l_nv \
  r4ivb02_analog_voltage_reader \
  serial_sc0_master \
  battery_charger_epc602_4840_ep_01 \
  plc_10ioa12_12di_12do_24v \
  robocon_hw_monitor_gui \
  robot_custom_interfaces \
  robocon_network_client

# Source workspace
source install/setup.bash
echo "source ~/robocon_ws/install/setup.bash" >> ~/.bashrc
```

## GUI Configuration

For hardware monitor GUI on LCD display:

```bash
# Install QT5
sudo apt-get install python3-pyqt5

# Set display environment variables
export DISPLAY=:0
export QT_QPA_PLATFORM=wayland
echo "export DISPLAY=:0" >> ~/.bashrc
echo "export QT_QPA_PLATFORM=wayland" >> ~/.bashrc
```

## Hardware Testing

### Test CAN Bus

```bash
# Enable CAN
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# Monitor CAN messages
candump can0

# Send test message (if devices connected)
cansend can0 123#DEADBEEF
```

### Test RS485 Ports

```bash
# Check port availability
ls -la /dev/ttySC*

# Test serial communication
# (Use appropriate test script for connected device)
```

### Test ROS 2

```bash
# Source ROS 2 and workspace
source /opt/ros/jazzy/setup.bash
source ~/robocon_ws/install/setup.bash

# Check available packages
ros2 pkg list | grep robocon

# Launch hardware monitor GUI
ros2 run robocon_hw_monitor_gui robocon_hw_monitor_gui_node

# Launch robot bringup
ros2 launch robot_mini_crane_bringup real_robot.launch.py
```

## Network Configuration

### Configure Static IP (if needed)

```bash
# Edit network configuration
sudo nano /etc/netplan/01-netcfg.yaml

# Example configuration:
# network:
#   version: 2
#   ethernets:
#     eth0:
#       addresses:
#         - 192.168.1.100/24
#       gateway4: 192.168.1.1
#       nameservers:
#         addresses: [8.8.8.8, 8.8.4.4]

sudo netplan apply
```

### Configure ROS 2 DDS Discovery

```bash
# Set DDS discovery for network communication with AI Computer
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>'
echo "export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>'" >> ~/.bashrc
```

## Startup Configuration

### Create Startup Service

Create `/etc/systemd/system/robocon-startup.service`:

```ini
[Unit]
Description=RoboCon OS Startup Service
After=network.target
Wants=network.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/local/bin/robocon_startup.sh
User=robocon

[Install]
WantedBy=multi-user.target
```

### Create Startup Script

Create `/usr/local/bin/robocon_startup.sh`:

```bash
#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/robocon/robocon_ws/install/setup.bash

# Setup CAN bus
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# Setup RS485 ports
sudo chmod 666 /dev/ttySC0
sudo chmod 666 /dev/ttySC1

# Launch robot
ros2 launch robot_mini_crane_bringup real_robot.launch.py &
```

Make executable:
```bash
sudo chmod +x /usr/local/bin/robocon_startup.sh
sudo systemctl enable robocon-startup.service
```

## Verification

```bash
# Check CAN bus
ip link show can0

# Check RS485 ports
ls -la /dev/ttySC*

# Check ROS 2 topics
ros2 topic list

# Check hardware status
ros2 run robocon_hw_monitor_gui robocon_hw_monitor_gui_node
```

## Next Steps

- [AI Computer Setup](ai-computer-setup.md) - Configure AI computer
- [Runtime Configuration](runtime-configuration.md) - Runtime settings
- [Troubleshooting](troubleshooting.md) - Common issues

