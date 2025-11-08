# RoboCon Sheather Wheeled

The RoboCon Sheather Wheeled is an intelligent sheathing robot variant with enhanced AI capabilities for autonomous construction sheathing operations. It uses wheeled base mobility instead of tracks. Available in 50kg and 60kg payload variants.

## Model Specifications

- **Year**: 2025
- **Make**: RoboCon
- **Model**: Sheather Wheeled (50kg or 60kg)
- **Payload Capacity**: 50kg or 60kg (depending on variant)
- **Base Configuration**: Wheeled mobility system
- **Lift Platform**: Vertical lift system with articulated arm
- **Articulated Arm**: BORUNTE BRTIRUS2550A 6-DOF robotic arm
- **Runtime**: 8+ hours per battery pack

## Overview

The RoboCon Sheather Wheeled features:
- **Intelligent Sheathing**: AI-enhanced sheathing operations with autonomous decision-making
- **Precise Control**: Fine-grained control with sensor feedback
- **Mobile Base**: Wheeled base for intelligent navigation and optimal positioning
- **Lift Platform**: Vertical lift system for extended reach capability
- **Articulated Arm**: BORUNTE BRTIRUS2550A 6-DOF robotic arm mounted on lift platform
- **Advanced Sensors**: Comprehensive sensor suite for environment perception
- **AI Integration**: Integration with DeepSeek LLM for intelligent planning and execution

**Reference**: [BORUNTE BRTIRUS2550A Product Page](https://www.borunte.top/products/1-19bb73a057ca41a69d648843772f60ca)

## Hardware Drivers

The RoboCon Sheather Wheeled uses similar hardware to RoboCon Sheather Tracked with additional intelligent features:

### Lift Platform

#### Lift Extension System
- **Type**: Vertical lift mechanism
- **Application**: Raises and lowers the arm platform for extended reach
- **ROS 2 Topics**:
  - `/lift/extend_cmd` (std_msgs/Float64) - Extension command in meters
  - `/lift/position_status` (std_msgs/Float64) - Current lift position

### Articulated Arm

#### BORUNTE BRTIRUS2550A Robotic Arm
- **Package**: `arm_borunte_brtirus2550a_driver`
- **Type**: 6-DOF articulated robotic arm
- **Mounting**: Mounted on top of the lift platform
- **Application**: Precise manipulation for intelligent sheathing operations
- **ROS 2 Integration**: 
  - Uses `arm_borunte_brtirus2550a_controller` for control
  - Joint trajectory control via `/arm_brtirus2550a/joint_trajectory`
  - Joint states published on `/joint_states`
- **Communication**: Serial communication (libserial)
- **Reference**: [BORUNTE BRTIRUS2550A Product Page](https://www.borunte.top/products/1-19bb73a057ca41a69d648843772f60ca)

**Joint Configuration:**
- **Joint 1**: Base rotation (shoulder rotation) - ±180°
- **Joint 2**: Shoulder pitch - ±90°
- **Joint 3**: Elbow pitch - 0° to 180°
- **Joint 4**: Wrist roll - ±180°
- **Joint 5**: Wrist pitch - ±90°
- **Joint 6**: End effector rotation - ±180° or continuous

### Motors and Actuators

#### Golden Motor EZA48400
- **Package**: `golden_motor_eza48400`
- **Type**: Wheel motors for base movement
- **CAN Bus**: 500kbps, Node IDs: 0xEF (left), 0xF0 (right)
- **Application**: Base movement and intelligent positioning
- **ROS 2 Topics**:
  - `/base/robot_base_cmd_pub_` (Float32MultiArray)
  - `/base/wheel_motors_feedback` (Float32MultiArray)
  - `/left_wheel_golden_motor_status` (GoldenMotorEZA48400Status)
  - `/right_wheel_golden_motor_status` (GoldenMotorEZA48400Status)

#### Motor Driver IDS830ABS
- **Package**: `motor_driver_ids830abs`
- **Type**: CAN-based linear actuator controller
- **Application**: Sheathing tool control with intelligent feedback
- **ROS 2 Topics**:
  - `/motor_driver_ids830abs/command` (Float32MultiArray)
  - `/ids830abs_status` (IDS830ABSStatus)

### Base Controller

#### Base Ackermann Controller
- **Package**: `base_ackermann_controller`
- **Type**: Differential drive base controller
- **Application**: Intelligent base movement control
- **ROS 2 Topics**:
  - `/cmd_vel` (geometry_msgs/Twist) - Velocity command

### Sensors

#### Sensor_BW-MINS50 (IMU)
- **Package**: `sensor_bw_mins50`
- **Type**: 9-axis IMU
- **Application**: Precise orientation sensing for intelligent positioning
- **ROS 2 Topics**: `/imu/data` (sensor_msgs/Imu)

#### Vision Systems (if equipped)
- **Cameras**: Depth cameras and RGB cameras for visual perception
- **LiDAR**: For mapping and intelligent navigation

#### Serial Master SC0
- **Package**: `serial_sc0_master`
- **Type**: Serial communication master
- **Application**: Communication with sensors and peripherals

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

**Package Name**: `Robot_iSheather_bringup`

The bringup package initializes and coordinates all ROS 2 nodes and hardware drivers required for the RoboCon Sheather Wheeled robot. The robot's boot sequence automatically detects the hardware configuration and launches this bringup package.

### Launching the Robot

```bash
ros2 launch Robot_iSheather_bringup real_robot.launch.py
```

### What the Bringup Activates

The `Robot_iSheather_bringup` package's `real_robot.launch.py` launch file activates the same ROS 2 system components as the tracked variant, with the following differences:

#### Key Differences from Tracked Variant:
- **Base Configuration**: Wheeled base instead of tracked, optimized for smooth surface operation
- **AI Integration**: Enhanced AI capabilities and vision systems for intelligent operations
- **Navigation**: Improved navigation stack for wheeled mobility

See the RoboCon Sheather Tracked documentation for complete bringup activation details. The wheeled variant uses the same hardware drivers with additional intelligent features enabled.

## Comparison with Similar Robots

### RoboCon Sheather Wheeled 50kg vs. Boston Dynamics Stretch

The RoboCon Sheather Wheeled 50kg is designed for similar applications as [Boston Dynamics Stretch](https://www.bostondynamics.com/products/stretch), a mobile robot designed for warehouse operations and material handling.

**Similarities:**
- **Mobile Base with Lift**: Both robots feature a mobile base (wheeled) with a vertical lift platform
- **Articulated Arm**: Both include an articulated robotic arm mounted on the lift platform
- **Material Handling**: Both designed for picking, placing, and moving materials (packages, boxes, construction materials)
- **Payload Capacity**: Similar payload handling capabilities (RoboCon Sheather: 50kg, Stretch: up to 50 pounds/23kg standard, with higher capacity options)
- **Autonomous Operation**: Both designed for autonomous operation in structured environments
- **Industrial Applications**: Both optimized for repetitive material handling tasks

**Key Differences:**
- **Application Focus**: RoboCon Sheather optimized for construction sheathing operations; Stretch designed primarily for warehouse logistics
- **Arm Configuration**: RoboCon Sheather uses BORUNTE BRTIRUS2550A 6-DOF arm; Stretch features Boston Dynamics' custom arm design
- **Lift Height**: RoboCon Sheather lift optimized for construction applications; Stretch lift designed for warehouse shelving
- **End Effector**: RoboCon Sheather uses construction-specific tools; Stretch uses vacuum-based gripping for packages
- **Operating System**: RoboCon Sheather built on ROBOCON OS (ROS 2); Stretch uses Boston Dynamics' proprietary control system
- **Base Mobility**: RoboCon Sheather uses standard wheeled base; Stretch features omni-directional base with advanced mobility
- **Payload Specifications**: RoboCon Sheather offers 50kg and 60kg variants; Stretch handles up to 50 pounds standard with customization options

**Best Use Cases:**
- **RoboCon Sheather Wheeled 50kg**: Construction sheathing, material placement, sheathing operations, ROS 2 development, outdoor/construction sites
- **Boston Dynamics Stretch**: Warehouse logistics, parcel handling, truck loading/unloading, package sorting, indoor warehouse operations

**Technical Comparison:**

| Feature | RoboCon Sheather Wheeled 50kg | Boston Dynamics Stretch |
|---------|------------------------------|------------------------|
| **Payload** | 50kg (also available in 60kg) | Up to 50 pounds (23kg standard) |
| **Base** | Wheeled | Omni-directional wheeled |
| **Lift System** | Vertical lift platform | Custom lift mechanism |
| **Arm** | BORUNTE BRTIRUS2550A (6-DOF) | Custom Boston Dynamics arm |
| **End Effector** | Construction tools | Vacuum gripper (package handling) |
| **OS** | ROBOCON OS (ROS 2) | Boston Dynamics OS |
| **Applications** | Construction sheathing | Warehouse logistics |
| **Mobility** | Standard wheeled | Advanced omni-directional |

## Sample Code

See the RoboCon Sheather Tracked documentation for similar code examples. The wheeled variant uses the same hardware interfaces with additional AI capabilities.

## Next Steps

- [API Reference](../api-reference/motor-control.md) - Detailed motor control APIs
- [ROS 2 Integration](../ros2/nodes-and-topics.md) - ROS 2 topics and nodes
- [AI Programs](../ai-programs/overview.md) - AI program development
- [Deployment](../deployment/runtime-configuration.md) - Runtime configuration

