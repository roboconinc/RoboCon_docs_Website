# Servicer Tracked Arms - BORUNTE BRTIRXZ1515A

**Internal Implementation Documentation**  
This page contains manufacturer-specific information about the robotic arms used in the RoboCon Servicer Tracked 15kg and Servicer Wheeled 15kg models.

## Manufacturer Information

**Manufacturer**: BORUNTE  
**Model**: BRTIRXZ1515A  
**Name**: 15kg Six axis robot  
**Robot Version**: 10 version

> **Note**: Specifications and exterior may be changed without further notice due to product improvements.

## BRTIRXZ1515A Robot Profile

### Introduction

BRTIRXZ1515A is a six-axis cooperative robot with drag-teaching function independently developed by BORUNTE. It features:
- Maximum load: 15kg
- Maximum arm length: 1500mm
- Collision detection
- 3D Visual Recognition
- Track reproduction

**Key Characteristics:**
- Safe and efficient
- Intelligent and easy to use
- Flexible and light
- Economical and reliable
- Low power consumption

The robot is designed for high-density flexible production lines and meets the needs of:
- Product packaging
- Injection molding
- Loading and unloading
- Assembly operations
- Human-machine collaborative work

**Protection Grade**: IP65 (dust-proof and water-proof)  
**Repeat Positioning Accuracy**: ±0.08mm

### Application Cases

1. Human-machine collaboration
2. Injection molding
3. Handling
4. Assembling
5. Loading and unloading
6. Packaging

## BRTIRXZ1515A Robot Specifications

### Basic Specifications

| Parameter | Value |
|-----------|-------|
| **Model** | BRTIRXZ1515A |
| **Axes** | 6 axis |
| **Loading Ability** | 15kg |
| **IP Code** | IP65 (dust-proof and water-proof) |
| **Repeated Positioning Accuracy** | ±0.08mm |
| **Maximum Arm Span** | 1500mm |
| **Weight** | 63kg |

### Maximum Joint Speeds

| Joint | Maximum Speed |
|-------|---------------|
| J1 | 120°/s |
| J2 | 113°/s |
| J3 | 106°/s |
| J4 | 181°/s |
| J5 | 181°/s |
| J6 | 181°/s |

### Motion Range

| Joint | Range |
|-------|-------|
| J1 | ±180° |
| J2 | ±180° |
| J3 | -65° ~ +250° |
| J4 | ±180° |
| J5 | ±180° |
| J6 | ±360° |

### Allowable Torque

| Joint | Torque |
|------|-------|
| J4 | 33.6 N·m |
| J5 | 33.6 N·m |
| J6 | 42 N·m |

### Allowable Moment of Inertia

| Joint | Moment of Inertia |
|------|------------------|
| J4 | 0.096 kg·m² |
| J5 | 0.096 kg·m² |
| J6 | 0.04 kg·m² |

### Mounting Environment

| Parameter | Value |
|-----------|-------|
| **Operating Temperature** | 0°C ~ 40°C |
| **Relative Humidity** | 20~80%RH (No condensation) |
| **Air Pressure** | 0.5~0.7 MPa |
| **Power Supply** | 220V ±10%, 50Hz ±1% |

### Core Components

| Component | Brand |
|-----------|-------|
| **Operating System** | BORUNTE |
| **Casting Body** | BORUNTE |
| **Servo Motor** | BORUNTE |
| **Reducer** | BORUNTE |
| **Electrical Component** | BORUNTE |

### Additional Information

- **Oil Content**: None
- **Vulnerable Parts**: None
- **Robot Accompany Materials**: USB

## Integration with RoboCon Servicer

The BRTIRXZ1515A arms are integrated into the RoboCon Servicer Tracked 15kg and Servicer Wheeled 15kg models as dual-arm systems, providing:

- **Dual 6-DOF Configuration**: Two independent BRTIRXZ1515A arms
- **15kg Payload per Arm**: Total 30kg dual-arm capacity
- **1500mm Reach**: Maximum arm span for each arm
- **High Precision**: ±0.08mm repeat positioning accuracy

## ROS 2 Integration

The arms are controlled via ROS 2 packages:
- `arm_borunte_brtirus2030a_driver` - Driver package
- `arm_borunte_brtirus2030a_controller` - Controller package
- `arm_borunte_brtirus2030a_moveit` - MoveIt integration (optional)

### ROS 2 Topics

- Arm joint states and commands
- Controller status and feedback
- MoveIt planning and execution (when enabled)

### Communication

- **Interface**: Serial communication (libserial)
- **Protocol**: BORUNTE proprietary protocol

## Technical Notes

> **Important**: The rated power, specifications, external dimensions, etc. of this product are subject to modification without prior notice.

> Technical data and illustrations are for supply reference only and reserve the right to make changes.

---

**This is internal implementation documentation. For public-facing product information, see:**
- [RoboCon Servicer Tracked 15kg](/docs/robots/servicer-tracked)
- [Hardware: Robotic Arms](/implementation/hardware/robotic-arms)

