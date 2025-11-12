# RoboCon Mini Crane Tracked 3000kg Electric

This folder contains asset deliverables for the RoboCon Mini Crane Tracked 3000kg Electric robot.

## Asset List

The following table lists all major components, parts, model numbers, manufacturers, and brands used in the RoboCon Mini Crane Tracked 3000kg - Electric:

| Component Category | Component Name | Model Number / Part Number | Manufacturer / Brand | Specification | Application |
|-------------------|----------------|---------------------------|---------------------|---------------|-------------|
| **Motor** | Drive Motor (Left) | HPM-20KW / Golden 144VDC 20kW | Golden Motor | 144V DC, 20kW | Primary drive motor for crane operations (left track) |
| **Motor** | Drive Motor (Right) | HPM-20KW / Golden 144VDC 20kW | Golden Motor | 144V DC, 20kW | Primary drive motor for crane operations (right track) |
| **Motor** | AC Servo Motor | EMDA440H2JD0 380V 4400W | - | 380V AC, 4400W | Boom slewing control, rotation platforms |
| **Electric Actuator** | Boom Actuator | Electric linear actuator / ball-screw | - | 72V DC or 380V AC | Boom extension and retraction |
| **Electric Actuator** | Outrigger Joint 1 Actuator (Base Rotation) | Electric servo motor | - | Electric servo with position feedback | Outrigger base rotation (360° range) |
| **Electric Actuator** | Outrigger Joint 2 Actuator (Primary Extension) | Electric linear actuator / ball-screw | - | Electric linear actuator | Primary outrigger section extension/retraction |
| **Electric Actuator** | Outrigger Joint 3 Actuator (Secondary Extension) | Electric linear actuator / ball-screw | - | Electric linear actuator | Secondary outrigger section extension |
| **Electric Actuator** | Outrigger Joint 4 Actuator (Horizontal Pivot) | Electric pivot actuator | - | Electric pivot actuator | Horizontal angle adjustment (±45° range) |
| **Electric Actuator** | Outrigger Joint 5 Actuator (Ground Contact/Flex) | Electric actuator with force feedback | - | Electric actuator with force feedback | Ground contact point with adaptive flex |
| **Electric Actuator** | End Effector Actuators | Various electric actuators | - | Varies by end-effector type | Hook winch, vacuum pump, fork positioning, tool actuators |
| **Motor Driver** | CAN Motor Driver | IDS830ABS | - | CAN-based motor driver | Linear actuator control for steering and outriggers |
| **Pan-Tilt** | Pan-Tilt Camera Mount | J-PT-760 | JEC (Tianjin Jiajie Shengchuang Technology Co., Ltd.) | Pan-tilt camera mount | Camera positioning and stabilization |
| **Rotary Platform** | Rotary Platform | DK120 | 3F | Rotary platform (electric) | Base rotation for crane positioning |
| **Rotary Platform** | High-Precision Rotary Platform | PTN085-18 | Newgear | High-precision rotary platform (electric) | Fine positioning and orientation control |
| **LiDAR** | 2D LiDAR Sensor | SE-1035 | Hinson (Shenzhen Hinson Intelligent Systems Co., Ltd.) | 2D LiDAR, up to 10m range | Obstacle detection and environment mapping |
| **LiDAR** | Single-Point LiDAR | TF03-180 | Benewake | Single-point LiDAR, up to 180m range | Long-range distance measurement |
| **IMU Sensor** | 9-Axis IMU | BW-MINS50 | BWSensing | 9-axis (accelerometer, gyroscope, magnetometer) | Orientation and motion sensing |
| **Force/Torque Sensor** | Outrigger Ground Contact Sensor | Force/torque sensor | - | Force/torque sensor | Ground contact force feedback for outrigger Joint 5 |
| **Battery** | Lithium Iron Phosphate Battery | LFP 72V 300Ah | - | 72V, 300Ah capacity | Primary power source for all electric actuators |
| **Battery Charger** | Intelligent Battery Charger | EPC602-4840-EP-01 | - | Intelligent battery charger | Battery charging system |
| **PLC** | Programmable Logic Controller | 2AO-8AI-8DI-8DO 24V | - | 2 Analog Outputs, 8 Analog Inputs, 8 Digital Inputs, 8 Digital Outputs | Control system I/O |
| **PLC** | Programmable Logic Controller | 10IOA12 (12DI-12DO) | - | 12 Digital Inputs, 12 Digital Outputs | Control system I/O |
| **Camera** | Industrial Camera | ZN2133 | JEC (Tianjin Jiajie Shengchuang Technology Co., Ltd.) | Industrial camera | Visual inspection and monitoring |
| **Depth Camera** | 3D Depth Camera | TM815 IX E1 | Botu | 3D depth camera | 3D environment perception and object detection |
| **Energy Meter** | Energy Meter | AMC16-DETT | Accrel | Energy meter | Power consumption monitoring |
| **Energy Meter** | Energy Meter | AMC16Z-FDK24 | Accrel | Energy meter | Power consumption monitoring |
| **Fan Controller** | Fan Speed Controller | JPF4816 | - | Fan speed controller | Cooling system control |
| **Relay Controller** | Relay Module | N4ROC04-24V | Xinlihui | 24V relay module | Relay control system |
| **Pressure Sensor** | Negative Pressure Sensor | PS-1L-NV | - | Negative pressure sensor | System pressure monitoring |
| **Analog Voltage Reader** | Analog Voltage Reader | R4IVB02 | - | Analog voltage reader | Analog voltage reading |
| **Electric Actuator Controller** | CAN-based Actuator Controller | Various | - | CAN-based controllers | Electric actuator control via CAN bus |
| **Electric Actuator Controller** | Serial-based Actuator Controller | Various | - | Serial-based controllers | Electric actuator control via serial communication |
| **Chassis** | Boom Assembly | - | - | 6-section telescopic design with electric actuators | Telescoping boom for lifting operations |
| **Chassis** | Outrigger System | - | - | 4 extensible outriggers (5 joints each) | Stability and support platform with automatic terrain adaptation |
| **Chassis** | Tracked Undercarriage | - | - | Tracked base | Base mobility system |
| **End Effector** | Hook Assembly | - | - | Rope/cable connection with electric winch | Flexible load attachment with electric winch control |
| **End Effector** | Vacuum-Suction Assembly | - | - | Pneumatic suction with electric vacuum pump | Panel attachment via suction with electric pump control |
| **End Effector** | Fork Assembly | - | - | Mechanical engagement with electric positioning | Palletized load handling with electric fork positioning |
| **End Effector** | End-Effector Platform | - | - | Removable attachment platform | Accommodates various electric end-effectors |
| **Control System** | Computational Processor | - | - | Primary processing unit | Command inputs, sensor data processing, operational control |
| **Control System** | Computational Planner | - | - | Advanced planning capabilities | Coordinating complex crane operations |
| **Control System** | Machine Learning Module | - | - | Adaptive optimization | Operational optimization based on historical data |
| **Control System** | Behavior Tree Module | - | - | Structured control frameworks | Predictable operational sequences |
| **Control System** | Visual-Language-Action Model | - | - | Natural language to behavior tree converter | Converts natural-language instructions to XML behavior trees |

## Component Specifications

### Physical Dimensions

| Property | Value |
|----------|-------|
| Folded Footprint | ≤ 1.2 m × 1.2 m |
| Boom Sections | 6-section telescopic design |
| Support Legs | 4 extensible outriggers (5 joints each) |
| Base Type | Tracked undercarriage |
| Max Payload | 3000 kg |

### Power Systems

- **High-Capacity Battery System**: 72V 300Ah LFP (Lithium Iron Phosphate) battery
- **Electric Motor Actuators**: All extensible components use electric actuators
- **Intelligent Charging System**: Battery charging and management
- **Power Management**: Power distribution for electric actuators
- **No Hydraulic Systems**: Fully electric - no hydraulic fluid systems required

### 5-Joint Outrigger System

Each of the 4 outriggers features 5 articulated joints:

| Joint | Function | Range | Control Type |
|-------|----------|-------|--------------|
| **Joint 1** | Base rotation relative to crane chassis | 360° rotation | Electric servo motor with position feedback |
| **Joint 2** | Primary outrigger section extension/retraction | Variable extension length | Electric linear actuator or ball-screw mechanism |
| **Joint 3** | Secondary section for extended reach | Additional extension capability | Electric actuator synchronized with Joint 2 |
| **Joint 4** | Horizontal angle adjustment for ground adaptation | ±45° horizontal pivot | Electric pivot actuator |
| **Joint 5** | Final ground contact point with adaptive flex | Vertical adjustment and flex compensation | Electric actuator with force feedback for ground contact |

**Benefits:**
- Automatic deployment with self-leveling
- High maneuverability for optimal stability
- Uneven terrain adaptation (5 joints allow adaptation to highly irregular ground surfaces)
- Precise leveling for maintaining crane stability on slopes
- Compact retraction (folds into minimal space when retracted)

### Control Interface

- **Network Connectivity**: Cloud-based coordination and remote monitoring
- **Client Device Support**: Mobile devices and control interfaces
- **Display Terminals**: Centralized monitoring and visualization
- **ROS 2 Integration**: Full ROS 2 topic and service support

## Key Differences from Hydraulic Variant

| Feature | Electric Variant | Hydraulic Variant |
|---------|----------------|-------------------|
| **Actuation** | Electric motor actuators | Hydraulic cylinders |
| **Control Precision** | Very High (direct motor control) | High (hydraulic valve control) |
| **Maintenance** | Minimal (no fluids) | Requires hydraulic fluid and seals |
| **Noise Level** | Low | Moderate |
| **Response Speed** | Very Fast | Fast |
| **Outriggers** | 5-joint automatic electric outriggers | Traditional hydraulic outriggers |
| **Terrain Adaptation** | Advanced (5 joints per outrigger) | Limited |
| **Positioning Accuracy** | Very High | High |
| **Installation** | Simpler (battery + motors) | Requires hydraulic reservoir |

## Deliverables
