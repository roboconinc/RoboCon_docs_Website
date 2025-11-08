# ROBOCON SDK Introduction

Welcome to the ROBOCON SDK documentation! The ROBOCON SDK is a comprehensive development toolkit for building intelligent applications that run on ROBOCON OS.

## What is ROBOCON SDK?

The ROBOCON SDK enables developers to create sophisticated applications for ROBOCON construction machinery and compatible robots. It provides:

- **Low-level motor and driver control** with CAN bus and RS485 communication
- **ROS 2 Jazzy integration** for standard robotics development workflows
- **Hardware abstraction** for diverse sensor and actuator integration
- **Edge-LLM integration** for intelligent command interpretation and behavior tree management

## ROBOCON OS Architecture

ROBOCON OS is built on multiple core technologies:

1. **ROS 2 Jazzy** - Robot Operating System 2 for distributed robotics software development
2. **DDS: Cyclone** - Data Distribution Service implementation for ROS 2 communication
3. **Edge-LLM** - Edge-based Large Language Model for natural language processing and command interpretation
4. **Multi-Robot Communication Protocol** - Consensus-based task allocation system with zero-trust identity verification
5. **World-Server** - Centralized world state management and coordination
6. **Network-Client** - Network communication and client management
7. **RoboCon CV** - Computer vision and perception modules

## Hardware Platform

ROBOCON OS runs on a dual-computer architecture:

### Raspberry Pi 5 (Boot Computer)
- **Purpose**: CAN Bus and RS485 communication host
- **Hardware**: Waveshare Isolated CAN Bus Hat
- **Responsibilities**:
  - Motor driver communication (CAN bus)
  - Sensor data acquisition (RS485, CAN)
  - Hardware monitoring and control
  - Real-time hardware interfaces

### AI Computer (High-Performance Computing)
- **Hardware**: ASRock B450M-HDV R4.0 AM4 with GIGABYTE Radeon RX 7600 XT
- **Purpose**: AI/ML processing, computer vision, and centralized coordination
- **Responsibilities**:
  - Edge-LLM processing
  - Computer vision (YOLOv11, depth estimation)
  - World Server coordination
  - Network client management
  - Behavior tree execution

## Supported Robot Types

The ROBOCON SDK supports multiple robot platforms:

- **Mini Crane** - Compact construction crane robot
- **Mini Loader** - Small-scale loader robot
- **Mini Excavator** - Miniature excavator robot
- **Transporter** - Material transport robot
- **RoboCon Servicer Tracked** - Tracked service robot variant
- **RoboCon Servicer Wheeled** - Wheeled intelligent service robot variant
- **RoboCon Sheather Tracked** - Tracked sheathing robot variant
- **RoboCon Sheather Wheeled** - Wheeled intelligent sheathing robot variant

## Key Features

### Motor Control

The SDK provides fine-grained control over multiple motor types:

- **IDS830ABS Motor Drivers** - CAN bus-based linear actuators for precise positioning
- **Golden Motor EZA48400** - CAN bus wheel motors for left/right drive control
- **Qiangdeli Actuators** - Multiple actuator control (1/2/3/4/5)
- **CAN Bus Communication** - Standard 500kbps CAN interface for motor control
- **RS485 Serial Communication** - Multi-device serial bus for sensors and controllers

### Sensor Integration

Comprehensive sensor support:

- **LiDAR Sensors**: Hinson SE-1035, DE-4511 (Ethernet), Benewake TF03-180 (CAN bus)
- **IMU**: BW MINS50 via RS485
- **Depth Cameras**: OAK-D S2 with YOLOv11 on-camera inference, TM815 IX E1
- **Energy Meters**: Accrel AMC16, AMC16Z-FDK24
- **Pressure Sensors**: PS-1L-NV with R4IVB02 analog reader
- **Control Systems**: JPF4816 Fan Controller, N4ROC04 Relay, PLC 10IOA12

### ROS 2 Integration

Full ROS 2 Jazzy support:

- Standard ROS 2 nodes, topics, services, and actions
- Custom message types for hardware interfaces
- Hardware abstraction layer for consistent API
- Behavior tree integration for complex behaviors

### AI and Vision

Advanced AI capabilities:

- **Edge-LLM**: On-device natural language processing
- **YOLOv11 Instance Segmentation**: On-camera inference for real-time object detection
- **Computer Vision Pipeline**: Depth estimation, point cloud generation
- **World Server**: Multi-robot coordination and state management

### Multi-Robot Coordination

Consensus-based multi-robot task allocation system:

- **Zero-Trust Identity Verification**: Motion ledger authentication without assuming credentials are genuine
- **Consensus-Based Voting**: Distributed decision-making for task allocation
- **Discovery Protocol**: Bluetooth LE and Wi-Fi 802.11 transport-agnostic discovery
- **Behavior Tree Synchronization**: Automatic insertion of agreed-upon action subtrees
- **Motion Ledger**: Blockchain-structured tamper-evident records of robot movements

### Hardware Abstraction

The SDK provides:

- Unified interface for diverse hardware
- Automatic hardware detection
- Robot-specific package loading
- GUI-based hardware monitoring

## Use Cases

The ROBOCON SDK is ideal for:

- **Construction Automation** - Automating construction machinery tasks
- **Autonomous Navigation** - Building navigation solutions for mobile robots
- **Research & Development** - Prototyping new robotics applications
- **Educational Projects** - Learning ROS 2, Nav 2, and robotics development

## Next Steps

- [Installation Guide](installation.md) - Set up the SDK on your system
- [Quick Start](quick-start.md) - Get up and running in minutes
- [Your First Application](your-first-application.md) - Build your first ROBOCON application

## Compatibility

The ROBOCON SDK supports:

- **ROBOCON-manufactured robots** running ROBOCON OS
- **Third-party robots** that can run ROBOCON OS
- Standard ROS 2 distributions (Humble, Iron, Jazzy)

