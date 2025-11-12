# RoboCon Mini Crane Tracked 3000kg - Hydraulic

**Brand/Make:** RoboCon  
**Model Family:** Mini Crane  
**Base Option:** Tracked  
**Max Payload:** 3000kg  
**Actuation Type:** Hydraulic

The RoboCon Mini Crane Tracked 3000kg - Hydraulic is a revolutionary foldable crane assembly designed for compact storage, rapid deployment, and versatile material handling operations. This variant utilizes hydraulic cylinder mechanisms for boom extension and retraction, providing high-force actuation suitable for heavy lifting operations.

## Overview

The Mini Crane Hydraulic variant offers:

- **Hydraulic Actuation**: High-power hydraulic cylinders for boom lift extension/retraction
- **High Load Capacity**: Optimized for maximum lifting force (3000kg capacity)
- **Proven Reliability**: Traditional hydraulic systems with enhanced control
- **High-Power Operations**: Ideal for heavy-duty lifting tasks

## Key Innovations

The Mini Crane incorporates several patented innovations that set it apart from traditional mobile crane systems:

### 1. Foldable Compact Design

- **Compact Footprint**: Folds to a footprint equal to or less than **1.2 m × 1.2 m** for efficient storage and transport
- **Rapid Deployment**: Quick transition between storage and operational configurations
- **Telescoping Boom**: Multi-section telescopic boom with sequential extension/retraction
- **Retractable Legs**: Plurality of extensible legs that deploy for stability and retract for mobility

### 2. Configurable End Module System

The end module is **automatically configurable** to operate as multiple attachment types:

#### Hook Assembly
- Utilizes rope or cable connections to hold panels and loads
- Provides rotational freedom during lifting operations
- Suitable for loads requiring flexible attachment points

#### Vacuum-Suction Assembly
- Removably attaches to panels through pneumatic suction
- Ideal for flat, smooth surfaces like construction panels
- Enables rapid attachment and release without mechanical fasteners

#### Fork Assembly
- Lifts panels through mechanical engagement
- Provides stable support for palletized or flat loads
- Precision positioning for material placement

#### End-Effector Platform
The end module can accommodate multiple **removably attached end-effectors**:
- **Robotic Arms**: Multi-degree-of-freedom manipulation
- **Rotary Drills**: Drilling operations in various materials
- **Screwdrivers**: Automated fastening with torque control
- **Nail Drivers**: Impact mechanisms for construction fastening
- **Grippers**: Articulated fingers for grasping objects of various shapes

### 3. Intelligent Control System

#### System Controller Architecture

The Mini Crane features an advanced system controller with:

- **Computational Processor**: Primary processing unit for command inputs, sensor data processing, and operational control
- **Computational Planner**: Advanced planning capabilities for coordinating complex crane operations
- **Machine Learning Module**: Adaptive operational optimization based on historical performance data
- **Behavior Tree Module**: Structured control frameworks for predictable operational sequences

#### Visual-Language-Action Model

- Converts **natural-language instructions** into behavior trees (XML format)
- Enables intuitive control through spoken or written commands
- Automatically generates operational sequences from high-level task descriptions

#### Multi-Sensor Perception

- **2D-LiDAR Sensors**: Environment mapping and obstacle detection
- **3D-LiDAR Sensors**: Detailed 3D environmental data for navigation and task execution
- **Vision Systems**: Target assembly data and visual feedback
- **IMU Sensors**: Orientation and motion sensing

### 4. Coordinated Deployment Sequence

The Mini Crane follows an intelligent deployment and retraction sequence:

1. **Receiving Commands**: System receives operational commands
2. **Leg Deployment**: Plurality of extensible legs extend to create stable support platform
3. **Boom Extension**: Extensible boom lift extends to operational position
4. **Task Execution**: Crane performs lifting, positioning, or manipulation operations
5. **Retraction**: Prior to mobility, boom retracts, then legs retract

This sequence ensures safety and prevents component damage during transitions.

### 5. Multi-Crane Coordination

The Mini Crane can coordinate with other foldable crane assemblies:

- **Cooperative Lifting**: Multiple cranes work together on large loads
- **Task Synchronization**: Coordinated movements between crane units
- **Dynamic Task Allocation**: Real-time coordination based on capabilities and positioning
- **Inter-Crane Communication**: Exchange of operational status and planning data

## Technical Specifications

### Physical Dimensions

| Property | Value |
|----------|-------|
| Folded Footprint | ≤ 1.2 m × 1.2 m |
| Boom Sections | 6-section telescopic design |
| Support Legs | 4 extensible legs (configurable) |
| Base Type | Tracked undercarriage |
| Max Payload | 3000 kg |

### Power Systems

#### Mini Crane Hydraulic
- Hydraulic pump and reservoir system
- High-pressure hydraulic cylinders
- Integrated valve assemblies for flow control
- Hydraulic fluid system with filters and coolers

### Control Interface

- **Network Connectivity**: Cloud-based coordination and remote monitoring
- **Client Device Support**: Mobile devices and control interfaces
- **Display Terminals**: Centralized monitoring and visualization
- **ROS 2 Integration**: Full ROS 2 topic and service support

## Hardware Drivers

### Motors and Actuators

#### Motor_Golden 144VDC 20kW
- **Package**: `motor_golden_144vdc_20kw`
- **Type**: High-power DC motor
- **Voltage**: 144V DC
- **Power**: 20kW
- **Application**: Primary drive motor for crane operations
- **ROS 2 Topics**:
  - `/golden_motor_left/status`
  - `/golden_motor_right/status`

#### Motor_M112-72V12K028B34P0 (Hydraulic Pump Motor)
- **Package**: `motor_m112_72v12k028b34p0`
- **Type**: DC motor driving hydraulic pump
- **Voltage**: 72V DC
- **Power**: 12kW
- **Application**: Hydraulic pump drive for boom and leg actuators

#### Motor Driver IDS830ABS
- **Package**: `motor_driver_ids830abs`
- **Type**: CAN-based motor driver
- **Application**: Linear actuator control for steering and outriggers
- **ROS 2 Topics**:
  - `/base/ids830abs_feedback` (std_msgs/Float32MultiArray)
  - `/ids830abs_status` (robot_custom_interfaces/IDS830ABSStatus)

### Pan-Tilt and Rotary Systems

#### Pantilt_JEC J-PT-760
- **Package**: `pantilt_jec_jpt_760`
- **Type**: Pan-tilt camera mount
- **Application**: Camera positioning and stabilization
- **ROS 2 Topics**: `/pantilt_jec_jpt_760/status`

#### Rotary Platform_3F DK120
- **Package**: `rotary_platform_3f_dk120`
- **Type**: Rotary platform
- **Application**: Base rotation for crane positioning

#### Rotary Platform_Newgear PTN085-18
- **Package**: `rotary_platform_newgear_ptn085_18`
- **Type**: High-precision rotary platform
- **Application**: Fine positioning and orientation control

### Sensors

#### Lidar_Hinson SE-1035
- **Package**: `lidar_hinson_se_1035`
- **Type**: 2D LiDAR sensor
- **Range**: Up to 10m
- **Application**: Obstacle detection and environment mapping
- **ROS 2 Topics**: `/lidar_hinson_se_1035/scan` (sensor_msgs/LaserScan)

#### Lidar_TF03-180
- **Package**: `lidar_benewake_tf03_180`
- **Type**: Single-point LiDAR
- **Range**: Up to 180m
- **Application**: Long-range distance measurement
- **ROS 2 Topics**: `/tf03_180/range`

#### Sensor_BW-MINS50 (IMU)
- **Package**: `sensor_bw_mins50`
- **Type**: 9-axis IMU (accelerometer, gyroscope, magnetometer)
- **Application**: Orientation and motion sensing
- **ROS 2 Topics**: `/imu/data` (sensor_msgs/Imu)

### Power Management

#### Battery_LFP 72V 300Ah
- **Package**: `battery_lfp_72v_300ah`
- **Type**: Lithium Iron Phosphate battery
- **Voltage**: 72V
- **Capacity**: 300Ah

#### Battery Charger EPC602 4840 EP 01
- **Package**: `battery_charger_epc602_4840_ep_01`
- **Type**: Intelligent battery charger
- **ROS 2 Topics**: `/battery_charger_epc602_status` (robot_custom_interfaces/BatteryChargerEPC602Status)

### Control Systems

#### PLC_2AO-8AI-8DI-8DO 24V
- **Package**: `plc_2ao_8ai_8di_8do_24v`
- **Type**: Programmable Logic Controller
- **I/O**: 2 Analog Outputs, 8 Analog Inputs, 8 Digital Inputs, 8 Digital Outputs
- **ROS 2 Topics**: `/plc_2ao_8ai_8di_8do_24v/status`

#### PLC 10IOA12 (12DI-12DO)
- **Package**: `plc_10ioa12_12di_12do_24v`
- **Type**: Programmable Logic Controller
- **I/O**: 12 Digital Inputs, 12 Digital Outputs
- **ROS 2 Topics**: `/plc10ioa12/status` (robot_custom_interfaces/PLC10IOA12Status)

### Vision Systems

#### Camera_JEC ZN2133
- **Package**: `camera_jec_zn2133`
- **Type**: Industrial camera
- **Application**: Visual inspection and monitoring
- **ROS 2 Topics**: `/camera_jec_zn2133/image_raw` (sensor_msgs/Image)

#### Depth Camera TM815 IX E1
- **Package**: `depth_camera_tm815_ix_e1`
- **Type**: 3D depth camera
- **Application**: 3D environment perception and object detection
- **ROS 2 Topics**:
  - `/camera/color/image_raw` (sensor_msgs/Image)
  - `/camera/depth/image_raw` (sensor_msgs/Image)
  - `/camera/pointcloud` (sensor_msgs/PointCloud2)

### Additional Components

- **Energy Meters**: `energy_meter_acrel_amc16_dett`, `energy_meter_acrel_amc16z_fdk24`
- **Fan Controller**: `controller_jpf4816_fan_speed`
- **Relay Controller**: `relay_xinlihui_n4roc04_24v`
- **Pressure Sensor**: `pressure_sensor_ps_1l_nv` (Hydraulic pressure monitoring)
- **Analog Voltage Reader**: `r4ivb02_analog_voltage_reader`

## Parts List

The following table lists all major components, parts, model numbers, manufacturers, and brands used in the RoboCon Mini Crane Tracked 3000kg - Hydraulic:

| Component Category | Component Name | Model Number / Part Number | Manufacturer / Brand | Application |
|-------------------|----------------|---------------------------|---------------------|-------------|
| **Motor** | Drive Motor | HPM-20KW / Golden 144VDC 20kW | Golden Motor | Primary drive motor for crane operations |
| **Motor** | Hydraulic Pump Motor | M112-72V12K028B34P0 | - | Hydraulic pump drive for boom and leg actuators |
| **Motor Driver** | CAN Motor Driver | IDS830ABS | - | Linear actuator control for steering and outriggers |
| **Pan-Tilt** | Pan-Tilt Camera Mount | J-PT-760 | JEC | Camera positioning and stabilization |
| **Rotary Platform** | Rotary Platform | DK120 | 3F | Base rotation for crane positioning |
| **Rotary Platform** | High-Precision Rotary Platform | PTN085-18 | Newgear | Fine positioning and orientation control |
| **LiDAR** | 2D LiDAR Sensor | SE-1035 | Hinson | Obstacle detection and environment mapping |
| **LiDAR** | Single-Point LiDAR | TF03-180 | Benewake | Long-range distance measurement (up to 180m) |
| **IMU Sensor** | 9-Axis IMU | BW-MINS50 | BWSensing | Orientation and motion sensing |
| **Battery** | Lithium Iron Phosphate Battery | LFP 72V 300Ah | - | Main power supply |
| **Battery Charger** | Intelligent Battery Charger | EPC602-4840-EP-01 | - | Battery charging system |
| **PLC** | Programmable Logic Controller | 2AO-8AI-8DI-8DO 24V | - | Control system I/O (2 Analog Outputs, 8 Analog Inputs, 8 Digital Inputs, 8 Digital Outputs) |
| **PLC** | Programmable Logic Controller | 10IOA12 (12DI-12DO) | - | Control system I/O (12 Digital Inputs, 12 Digital Outputs) |
| **Camera** | Industrial Camera | ZN2133 | JEC | Visual inspection and monitoring |
| **Depth Camera** | 3D Depth Camera | TM815 IX E1 | Botu | 3D environment perception and object detection |
| **Energy Meter** | Energy Meter | AMC16-DETT | Accrel | Power consumption monitoring |
| **Energy Meter** | Energy Meter | AMC16Z-FDK24 | Accrel | Power consumption monitoring |
| **Fan Controller** | Fan Speed Controller | JPF4816 | - | Cooling system control |
| **Relay Controller** | Relay Module | N4ROC04-24V | Xinlihui | Relay control system |
| **Pressure Sensor** | Negative Pressure Sensor | PS-1L-NV | - | Hydraulic pressure monitoring |
| **Analog Voltage Reader** | Analog Voltage Reader | R4IVB02 | - | Analog voltage reading |

## ROS 2 Development Details

### Bringup Package

**Package Name**: `robot_mini_crane_bringup`

The bringup package initializes and coordinates all ROS 2 nodes and hardware drivers required for the RoboCon Mini Crane Tracked 3000kg - Hydraulic robot. The robot's boot sequence automatically detects the hardware configuration and launches the appropriate bringup variant.

### Launching the Robot

#### Hydraulic Variant

```bash
ros2 launch robot_mini_crane_bringup robot_mini_crane_hydraulic.launch.py
```

#### Coordinated Multi-Crane

```bash
ros2 launch robot_mini_crane_bringup robot_mini_crane_coordinated.launch.py
```

### What the Bringup Activates

The `robot_mini_crane_bringup` package activates a comprehensive set of ROS 2 nodes for crane operation:

#### 1. **Crane Control System**
- **Boom Control**: Hydraulic cylinder mechanisms for extension/retraction
- **Rotation Platform**: Rotary motion control for crane base
- **Outrigger System**: Hydraulic stability leg deployment and positioning
- **End Effector Control**: Sucker or fork attachment control
- **Hydraulic System**: Pump control, valve management, pressure monitoring

#### 2. **Motor Control System**
- **Package**: `motor_driver_ids830abs`
- **Nodes**: Multiple CAN-based motor controllers
- **Activates**: Outriggers, rotation platforms, end effectors
- **Package**: `motor_m112_72v12k028b34p0`
- **Activates**: Hydraulic pump motor drive

#### 3. **Sensor System**
- **Package**: `sensor_bw_mins50` (IMU)
- **Package**: `lidar_hinson_SE_1035` (LiDAR)
- **Package**: `lidar_tf03_180` (Ranging LiDAR)
- **Package**: `pressure_sensor_ps_1l_nv` (Hydraulic pressure monitoring)
- **Activates**: Spatial awareness, obstacle detection, load monitoring

#### 4. **Vision System**
- **Package**: `camera_jec_zn2133` (RGB camera)
- **Package**: `camera_tm815_ix_e1` (Depth camera)
- **Activates**: Visual feedback, depth perception, material identification

#### 5. **Power Management**
- **Package**: `battery_charger_epc602_4840_ep_01`
- **Package**: Energy meter monitoring
- **Activates**: Battery status, power consumption tracking

#### 6. **Monitoring and Diagnostics**
- **Package**: `robocon_hw_monitor_gui`
- **Activates**: Real-time hardware status, diagnostics dashboard

### ROS 2 Topic Network

**Command Topics:**
- `/motor_driver_ids830abs/command` (std_msgs/Float32MultiArray) - Actuator control
- `/hydraulic_pump/command` (std_msgs/Float32) - Hydraulic pump speed control
- `/end_module/configure` (service) - End module configuration

**Feedback Topics:**
- `/imu/data` (sensor_msgs/Imu) - IMU orientation data
- `/lidar_hinson_se_1035/scan` (sensor_msgs/LaserScan) - LiDAR scan data
- `/ids830abs_status` (robot_custom_interfaces/IDS830ABSStatus) - Actuator status
- `/pressure_sensor_ps_1l_nv/pressure` (std_msgs/Float32) - Hydraulic pressure
- `/battery_charger_epc602_status` (robot_custom_interfaces/BatteryChargerEPC602Status) - Battery status

## Sample Code

### Controlling Hydraulic Boom Extension

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class HydraulicBoomControl(Node):
    def __init__(self):
        super().__init__('hydraulic_boom_control')
        
        # Hydraulic pump speed control
        self.pump_cmd_pub = self.create_publisher(
            Float32,
            '/hydraulic_pump/command',
            10
        )
        
        # Pressure monitoring
        self.pressure_sub = self.create_subscription(
            Float32,
            '/pressure_sensor_ps_1l_nv/pressure',
            self.pressure_callback,
            10
        )
    
    def extend_boom(self, target_height_m: float):
        """Extend boom to target height using hydraulic system"""
        # Set hydraulic pump speed for extension
        pump_speed = Float32()
        pump_speed.data = 0.8  # 80% pump speed for extension
        self.pump_cmd_pub.publish(pump_speed)
        self.get_logger().info(f"Extending boom to {target_height_m}m")
    
    def pressure_callback(self, msg):
        pressure = msg.data
        if pressure > 18.0:  # 18 MPa max pressure
            self.get_logger().warn(f"High pressure warning: {pressure:.2f} MPa")

def main(args=None):
    rclpy.init(args=args)
    node = HydraulicBoomControl()
    node.extend_boom(3.0)  # Extend to 3 meters
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Comparison: Hydraulic vs Electric

For patent details and variant considerations (including actuation options), see [Foldable Crane Patent Overview](./crane-foldable-patent.md).

**Key Advantages of Hydraulic Variant:**
- Maximum load capacity for heavy-duty operations
- High-force actuation through hydraulic cylinders
- Proven reliability in industrial applications
- Suitable for continuous heavy lifting operations

## Applications

The Mini Crane Hydraulic is designed for diverse applications including:

- **Construction**: Panel lifting, material positioning, tool deployment
- **Manufacturing**: Assembly operations, component placement
- **Warehousing**: Load handling, pallet operations
- **Infrastructure**: Maintenance tasks, equipment positioning
- **Multi-Crane Operations**: Cooperative lifting of large loads

## Safety Features

- **Automatic Retraction**: Boom and legs retract before mobility operations
- **Load Monitoring**: Real-time load sensing and overload protection
- **Stability Control**: Automatic leg adjustment for uneven terrain
- **Hydraulic Safety**: Pressure relief valves and system protection
- **Emergency Stop**: Comprehensive emergency stop systems
- **Sensor Fusion**: Multiple sensor systems for safe operation

## Next Steps

- [Foldable Crane Patent Overview](./crane-foldable-patent.md) - Patent scope and diagrams
- [API Reference](../api-reference/motor-control.md) - Detailed motor control APIs
- [Behavior Trees](../api-reference/behavior-trees.md) - Behavior tree control structures
- [ROS 2 Integration](../ros2/nodes-and-topics.md) - ROS 2 topics and nodes
- [Multi-Robot Communication](../api-reference/multi-robot-communication.md) - Multi-crane coordination
- [Deployment](../deployment/runtime-configuration.md) - Runtime configuration

