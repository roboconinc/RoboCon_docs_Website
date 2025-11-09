# RoboCon Transporter Tracked 1000kg

The RoboCon Transporter Tracked 1000kg is a tracked material transport robot designed for moving materials and equipment efficiently.

## Overview

The RoboCon Transporter - Tracked features:
- **Material Transport**: Designed for carrying loads up to 1000kg
- **Maneuverability**: Efficient movement for transport tasks
- **Load Handling**: Capable of carrying various materials
- **Standard ROBOCON Components**: Uses common hardware drivers and sensors
- **Tracked Base**: Tracked undercarriage for stability and terrain capability

## Hardware Drivers

The RoboCon Transporter - Tracked uses ROBOCON hardware components optimized for transport tasks:

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

**Package Name**: `Robot_Transporter_bringup`

The bringup package initializes and coordinates all ROS 2 nodes and hardware drivers required for the RoboCon Transporter Tracked robot. The robot's boot sequence automatically detects the hardware configuration and launches this bringup package.

### Launching the Robot

```bash
ros2 launch Robot_Transporter_bringup real_robot.launch.py
```

### What the Bringup Activates

The `Robot_Transporter_bringup` package's `real_robot.launch.py` launch file activates the following ROS 2 system components:

#### 1. **Base Mobility System**
- **Package**: `base_ackermann_controller`
- **Node**: Base motion controller
- **Activates**: Tracked base control, velocity command processing
- **ROS 2 Topics**: `/cmd_vel` (geometry_msgs/Twist)

#### 2. **Motor Control System**
- **Package**: `golden_motor_eza48400`
- **Nodes**: Left and right track motor drivers
- **Activates**: CAN bus motor control for tracked mobility, motor feedback
- **Package**: `motor_driver_ids830abs`
- **Node**: Load handling mechanism controller
- **Activates**: Actuator control for material handling via CAN bus

#### 3. **Sensor System**
- **Package**: `sensor_bw_mins50`
- **Node**: IMU sensor node
- **Activates**: Orientation and motion sensing
- **Package**: `serial_sc0_master`
- **Node**: Serial communication master
- **Activates**: Communication with sensors and peripherals

#### 4. **Power Management**
- **Package**: `battery_charger_epc602_4840_ep_01`
- **Node**: Battery charger monitor
- **Activates**: Battery status monitoring and charging management

### ROS 2 Topic Network

**Command Topics:**
- `/cmd_vel` (geometry_msgs/Twist) - Base velocity commands
- `/motor_driver_ids830abs/command` (std_msgs/Float32MultiArray) - Load handling control

**Feedback Topics:**
- `/imu/data` (sensor_msgs/Imu) - IMU orientation data
- `/base/wheel_motors_feedback` (std_msgs/Float32MultiArray) - Track motor feedback
- `/battery_charger_epc602_status` - Battery status

## Related Documentation

- [Public Documentation: RoboCon Transporter Tracked 1000kg](/docs/robots/transporter-tracked)
- [API Reference](/docs/api-reference/motor-control) - Detailed motor control APIs
- [ROS 2 Integration](/docs/ros2/nodes-and-topics) - ROS 2 topics and nodes
- [Deployment](/docs/deployment/runtime-configuration) - Runtime configuration

