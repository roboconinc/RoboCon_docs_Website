# AI Computer Setup

Complete setup guide for AI Computer (high-performance computing) for ROBOCON OS.

## Hardware Requirements

- **Motherboard**: ASRock B450M-HDV R4.0 AM4 (or compatible)
- **CPU**: AMD AM4 socket CPU (Ryzen recommended)
- **GPU**: GIGABYTE Radeon RX 7600 XT (or compatible AMD GPU)
- **RAM**: 16GB or more recommended
- **Storage**: SSD recommended for faster performance

## Operating System Installation

### Step 1: Install Ubuntu 24.04.2 LTS

1. Download Ubuntu 24.04.2 LTS Desktop or Server
2. Create bootable USB
3. Install Ubuntu on the system

### Step 2: Initial Configuration

```bash
# Update system
sudo apt update && sudo apt upgrade

# Install essential packages
sudo apt install -y git python3-pip python3-venv build-essential cmake
sudo apt install -y python3-colcon-common-extensions
sudo apt install -y python3-opencv python3-torch python3-torchvision
sudo apt install -y python3-flask python3-sqlalchemy python3-redis
sudo apt install -y python3-requests python3-websockets python3-asyncio
```

## ROS 2 Jazzy Installation

Follow the standard ROS 2 Jazzy installation (see [Installation Guide](../getting-started/installation.md)).

```bash
# Install ROS 2 Jazzy Desktop
sudo apt install ros-jazzy-desktop

# Configure DDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
```

## GPU Configuration (AMD)

### Install ROCm SDK

```bash
# Install ROCm for AMD GPU support
sudo apt install rocm-hip-sdk rocm-opencl-sdk

# Verify GPU detection
lspci | grep -i amd

# Configure environment variables
export HIP_VISIBLE_DEVICES=0
export HSA_OVERRIDE_GFX_VERSION=11.0.0
echo "export HIP_VISIBLE_DEVICES=0" >> ~/.bashrc
echo "export HSA_OVERRIDE_GFX_VERSION=11.0.0" >> ~/.bashrc
```

### Test GPU

```bash
# Test PyTorch GPU support
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}'); print(f'Device: {torch.cuda.get_device_name(0) if torch.cuda.is_available() else \"CPU\"}')"
```

## ROBOCON OS Installation

### Clone All Repositories

```bash
# Create workspace
mkdir -p ~/robocon_ws/src
cd ~/robocon_ws/src

# Clone ROBOCON OS
git clone https://github.com/roboconinc/Robocon-OS.git

# Clone all AI components
git clone https://github.com/roboconinc/RoboCon_Edge-LLM.git
git clone https://github.com/roboconinc/RoboCon_World-Server.git
git clone https://github.com/roboconinc/RoboCon_Network-Client.git
git clone https://github.com/roboconinc/RoboCon_CV.git
```

### Install Python Dependencies

```bash
# Install Python dependencies (careful with numpy)
python3 -m pip install "numpy<2.0.0"
python3 -m pip install torch transformers opencv-python pyyaml setuptools

# Install Edge-LLM dependencies
cd RoboCon_Edge-LLM
pip install -r requirements.txt
python3 setup.py install

# Install World Server dependencies
cd ../RoboCon_World-Server
pip install -r requirements.txt
python3 setup_database.py

# Install CV dependencies
cd ../RoboCon_CV
pip install -r requirements.txt
```

### Build ROS 2 Packages

```bash
cd ~/robocon_ws

# Build all packages with parallel compilation
colcon build --parallel-workers 8 --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source workspace
source install/setup.bash
echo "source ~/robocon_ws/install/setup.bash" >> ~/.bashrc
```

## Edge-LLM Setup

### Configure Edge-LLM

```bash
# Set environment variable
export ROBOCON_EDGE_LLM_PATH=~/robocon_ws/src/RoboCon_Edge-LLM
echo "export ROBOCON_EDGE_LLM_PATH=~/robocon_ws/src/RoboCon_Edge-LLM" >> ~/.bashrc

# Test Edge-LLM
python3 -m robocon_edge_llm.server --test

# Start Edge-LLM service
python3 -m robocon_edge_llm.server
```

### Test Edge-LLM

```bash
# Test with curl
curl -X POST http://localhost:8080/process \
  -H "Content-Type: application/json" \
  -d '{"command": "move the crane to position A"}'
```

## World Server Setup

### Configure World Server

```bash
# Set environment variable
export ROBOCON_WORLD_SERVER_URL=http://localhost:5000
echo "export ROBOCON_WORLD_SERVER_URL=http://localhost:5000" >> ~/.bashrc

# Start World Server
python3 -m robocon_world_server.main

# Test World Server
curl -X POST http://localhost:5000/register_robot \
  -H "Content-Type: application/json" \
  -d '{"robot_id": "mini_crane_001", "type": "mini_crane"}'
```

## Computer Vision Setup

### Test CV Components

```bash
# Source workspace
source ~/robocon_ws/install/setup.bash

# Start perception node
ros2 run robocon_cv perception_node

# Start object detection
ros2 run robocon_cv object_detection_node

# Start pose estimation
ros2 run robocon_cv pose_estimation_node
```

## Network Client Setup

### Configure Network Client

```bash
# Source workspace
source ~/robocon_ws/install/setup.bash

# Start network client
ros2 run robocon_network_client network_client_node

# Configure connection
ros2 param set /network_client_node server_url "http://world-server:5000"
ros2 param set /network_client_node robot_id "mini_crane_001"
```

## Network Configuration

### Configure DDS for Multi-Computer Communication

```bash
# Set DDS discovery for network communication
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>'
echo "export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>'" >> ~/.bashrc

# Configure hosts file
sudo nano /etc/hosts
# Add:
# 192.168.1.100 ai-computer
# 192.168.1.101 rpi5-crane-001
```

## Startup Configuration

### Create Startup Service

Create `/etc/systemd/system/robocon-ai-startup.service`:

```ini
[Unit]
Description=RoboCon AI Computer Startup Service
After=network.target
Wants=network.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/local/bin/robocon_ai_startup.sh
User=robocon

[Install]
WantedBy=multi-user.target
```

### Create Startup Script

Create `/usr/local/bin/robocon_ai_startup.sh`:

```bash
#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/robocon/robocon_ws/install/setup.bash

# Start Edge-LLM
python3 -m robocon_edge_llm.server &

# Start World Server
python3 -m robocon_world_server.main &

# Start Network Client
ros2 run robocon_network_client network_client_node &

# Start CV components
ros2 run robocon_cv perception_node &
ros2 run robocon_cv object_detection_node &
```

Make executable:
```bash
sudo chmod +x /usr/local/bin/robocon_ai_startup.sh
sudo systemctl enable robocon-ai-startup.service
```

## Verification

```bash
# Verify GPU
lspci | grep -i amd

# Test ROS 2
ros2 topic list

# Test Edge-LLM
python3 -m robocon_edge_llm.server --test

# Test World Server
python3 -m robocon_world_server.main --test

# Test CV
ros2 run robocon_cv perception_node --test
```

## Performance Optimization

### GPU Performance

```bash
# Monitor GPU usage
rocm-smi

# Set GPU performance mode
sudo rocm-smi --setperflevel high
```

### Build Optimization

```bash
# Build with optimizations
colcon build --parallel-workers 8 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O3 -march=native"
```

## Next Steps

- [Raspberry Pi 5 Setup](raspberry-pi5-setup.md) - Configure boot computer
- [Runtime Configuration](runtime-configuration.md) - Runtime settings
- [Troubleshooting](troubleshooting.md) - Common issues

