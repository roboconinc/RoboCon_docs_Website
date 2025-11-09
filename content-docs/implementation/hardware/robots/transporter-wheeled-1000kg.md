# RoboCon Transporter Wheeled 1000kg

The RoboCon Transporter Wheeled 1000kg is a wheeled material transport robot designed for moving materials and equipment efficiently on smooth surfaces.

## Overview

The RoboCon Transporter - Wheeled features:
- **Material Transport**: Designed for carrying loads up to 1000kg
- **Maneuverability**: Efficient movement for transport tasks on paved surfaces
- **Load Handling**: Capable of carrying various materials
- **Standard ROBOCON Components**: Uses common hardware drivers and sensors
- **Wheeled Base**: Wheeled base for speed and efficiency on smooth terrain

## Hardware Drivers

The RoboCon Transporter - Wheeled uses ROBOCON hardware components optimized for transport tasks:

### Motors and Actuators

#### Golden Motor EZA48400
- **Package**: `golden_motor_eza48400`
- **Type**: Wheel motors for base movement
- **CAN Bus**: 500kbps, Node IDs: 0xEF (left), 0xF0 (right)
- **Application**: Base movement and mobility
- **ROS 2 Topics**:
  - `/base/robot_base_cmd_pub_` (Float32MultiArray) - Command
  - `/base/wheel_motors_feedback` (Float32MultiArray) - Feedback
  - `/left_wheel_golden_motor_status` (GoldenMotorEZA48400Status)
  - `/right_wheel_golden_motor_status` (GoldenMotorEZA48400Status)

#### Motor Driver IDS830ABS
- **Package**: `motor_driver_ids830abs`
- **Type**: CAN-based linear actuator controller
- **Application**: Load handling mechanisms, steering
- **ROS 2 Topics**:
  - `/motor_driver_ids830abs/command` (Float32MultiArray)
  - `/ids830abs_status` (IDS830ABSStatus)

### Base Controller

#### Base Ackermann Controller
- **Package**: `base_ackermann_controller`
- **Type**: Differential drive base controller
- **Application**: Base movement control
- **ROS 2 Topics**:
  - `/cmd_vel` (geometry_msgs/Twist) - Velocity command

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

### Power Management

#### Battery Charger EPC602 4840 EP 01
- **Package**: `battery_charger_epc602_4840_ep_01`
- **Type**: Intelligent battery charger
- **ROS 2 Topics**: `/battery_charger_epc602_status`

## ROS 2 Development Details

### Bringup Package

**Package Name**: `Robot_Transporter_Wheeled_bringup`

The bringup package initializes and coordinates all ROS 2 nodes and hardware drivers required for the RoboCon Transporter Wheeled robot. The robot's boot sequence automatically detects the hardware configuration and launches this bringup package.

### Launching the Robot

```bash
ros2 launch Robot_Transporter_Wheeled_bringup real_robot.launch.py
```

### What the Bringup Activates

The `Robot_Transporter_Wheeled_bringup` package's `real_robot.launch.py` launch file activates the same ROS 2 system components as the tracked variant, with wheeled base configuration:

#### Key Differences from Tracked Variant:
- **Base Configuration**: Wheeled base optimized for smooth surface operation
- **Navigation**: Enhanced navigation capabilities for indoor/smooth terrain
- **Efficiency**: Improved energy efficiency on smooth surfaces

See the [RoboCon Transporter Tracked 1000kg](transporter-tracked-1000kg) documentation for complete bringup activation details. The wheeled variant uses the same hardware drivers with wheeled mobility optimization.

## Related Documentation

- [Public Documentation: RoboCon Transporter Wheeled 1000kg](/docs/robots/transporter-wheeled)
- [API Reference](/docs/api-reference/motor-control) - Detailed motor control APIs
- [ROS 2 Integration](/docs/ros2/nodes-and-topics) - ROS 2 topics and nodes
- [Deployment](/docs/deployment/runtime-configuration) - Runtime configuration

