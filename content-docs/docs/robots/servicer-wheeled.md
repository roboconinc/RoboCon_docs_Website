# RoboCon Servicer Wheeled 15kg

The RoboCon Servicer Wheeled 15kg is an intelligent service robot variant with enhanced AI capabilities for advanced service and maintenance tasks. It uses wheeled base mobility instead of tracks for improved efficiency on smooth surfaces.

## Model Specifications

- **Year**: 2025
- **Make**: RoboCon
- **Model**: Servicer Wheeled 15kg
- **Payload Capacity**: 15kg per arm
- **Base Configuration**: Wheeled mobility system
- **Arms**: Dual 6-DOF robotic arms (Borunte BRTIRUS2030A)
- **Runtime**: 8+ hours per battery pack

## Overview

The RoboCon Servicer Wheeled features:
- **Intelligent Operation**: Enhanced AI capabilities for autonomous service tasks
- **Robotic Arm**: Manipulation capabilities for complex service operations
- **Mobile Base**: Navigation for service tasks
- **Advanced Sensors**: Comprehensive sensor suite for environment perception
- **AI Integration**: Integration with DeepSeek LLM and vision systems

## Hardware Drivers

The RoboCon Servicer Wheeled uses similar hardware to RoboCon Servicer Tracked with additional intelligent features:

### Robotic Arm

#### Arm Borunte BRTIRUS2030A
- **Package**: `arm_borunte_brtirus2030a_driver`
- **Type**: 6-DOF robotic arm controller
- **Application**: Manipulation and service tasks
- **ROS 2 Integration**: 
  - Uses `arm_borunte_brtirus2030a_controller` for control
  - MoveIt integration available via `arm_borunte_brtirus2030a_moveit`
- **ROS 2 Topics**:
  - Arm joint states and commands
  - Controller status and feedback

### Base Movement

#### Base Ackermann Controller
- **Package**: `base_ackermann_controller`
- **Type**: Differential drive base controller
- **Application**: Base movement and navigation
- **ROS 2 Topics**:
  - `/cmd_vel` (geometry_msgs/Twist) - Velocity command

#### Twist Mux
- **Package**: `twist_mux`
- **Type**: Velocity command multiplexer
- **Application**: Prioritizes velocity commands from multiple sources

### Motors

#### Golden Motor EZA48400
- **Package**: `golden_motor_eza48400`
- **Type**: Wheel motors for base movement
- **CAN Bus**: 500kbps, Node IDs: 0xEF (left), 0xF0 (right)
- **Application**: Base drive motors
- **ROS 2 Topics**:
  - `/base/robot_base_cmd_pub_` (Float32MultiArray)
  - `/base/wheel_motors_feedback` (Float32MultiArray)
  - `/left_wheel_golden_motor_status` (GoldenMotorEZA48400Status)
  - `/right_wheel_golden_motor_status` (GoldenMotorEZA48400Status)

#### Motor Driver IDS830ABS
- **Package**: `motor_driver_ids830abs`
- **Type**: CAN-based linear actuator controller
- **Application**: Auxiliary actuators
- **ROS 2 Topics**:
  - `/motor_driver_ids830abs/command` (Float32MultiArray)
  - `/ids830abs_status` (IDS830ABSStatus)

### Sensors

#### Sensor_BW-MINS50 (IMU)
- **Package**: `sensor_bw_mins50`
- **Type**: 9-axis IMU
- **Application**: Orientation and motion sensing
- **ROS 2 Topics**: `/imu/data` (sensor_msgs/Imu)

#### Serial Master SC0
- **Package**: `serial_sc0_master`
- **Type**: Serial communication master
- **Application**: Communication with sensors and peripherals

#### Vision Systems (if equipped)
- **Cameras**: Depth cameras and RGB cameras for visual perception
- **LiDAR**: For mapping and navigation

### Power Management

#### Battery Charger EPC602 4840 EP 01
- **Package**: `battery_charger_epc602_4840_ep_01`
- **Type**: Intelligent battery charger
- **ROS 2 Topics**: `/battery_charger_epc602_status`

### Monitoring

#### RoboCon Hardware Monitor GUI
- **Package**: `robocon_hw_monitor_gui`
- **Type**: Hardware monitoring GUI
- **Application**: Real-time hardware status monitoring

## ROS 2 Development Details

### Bringup Package

**Package Name**: `Robot_iServicer_bringup`

The bringup package initializes and coordinates all ROS 2 nodes and hardware drivers required for the RoboCon Servicer Wheeled robot. The robot's boot sequence automatically detects the hardware configuration and launches this bringup package.

### Launching the Robot

```bash
ros2 launch Robot_iServicer_bringup real_robot.launch.py
```

### What the Bringup Activates

The `Robot_iServicer_bringup` package's `real_robot.launch.py` launch file activates the same ROS 2 system components as the tracked variant, with the following differences:

#### Key Differences from Tracked Variant:
- **Base Configuration**: Wheeled base instead of tracked, optimized for smooth surface operation
- **AI Integration**: Enhanced AI capabilities and vision systems for intelligent operations
- **Navigation**: Improved navigation stack for wheeled mobility

See the RoboCon Servicer Tracked documentation for complete bringup activation details. The wheeled variant uses the same hardware drivers with additional intelligent features enabled.

## Sample Code

See the RoboCon Servicer Tracked documentation for similar code examples. The wheeled variant uses the same hardware interfaces with additional AI capabilities.

## Next Steps

- [API Reference](../api-reference/motor-control.md) - Detailed motor control APIs
- [ROS 2 Integration](../ros2/nodes-and-topics.md) - ROS 2 topics and nodes
- [AI Programs](../ai-programs/overview.md) - AI program development
- [Deployment](../deployment/runtime-configuration.md) - Runtime configuration

