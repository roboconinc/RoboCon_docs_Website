# BORUNTE BRTIRUS2550A

**Model**: BRTIRUS2550A  
**Manufacturer**: BORUNTE

> **Note**: Specifications and exterior may be changed without further notice due to product improvements.

## Robot Profile

### Introduction

BRTIRUS2550A is a 6-DOF articulated robotic arm developed by BORUNTE. This arm is used in RoboCon Sheather Tracked and RoboCon Sheather Wheeled models, mounted on top of the lift platform to provide extended reach for sheathing operations.

**Reference**: [BORUNTE BRTIRUS2550A Product Page](https://www.borunte.top/products/1-19bb73a057ca41a69d648843772f60ca)

### Application Cases

The BRTIRUS2550A is designed for:
- Precise manipulation tasks
- Intelligent sheathing operations
- Extended reach applications
- Mounted platform operations

## Technical Specifications

### Basic Specifications

| Parameter | Value |
|-----------|-------|
| **Model** | BRTIRUS2550A |
| **Axes** | 6 axis |
| **Type** | Articulated robotic arm |

### Joint Configuration

| Joint | Description | Range |
|-------|-------------|-------|
| **Joint 1** | Base rotation (shoulder rotation) | ±180° |
| **Joint 2** | Shoulder pitch | ±90° |
| **Joint 3** | Elbow pitch | 0° to 180° |
| **Joint 4** | Wrist roll | ±180° |
| **Joint 5** | Wrist pitch | ±90° |
| **Joint 6** | End effector rotation | ±180° or continuous |

## ROS 2 Integration

The BRTIRUS2550A arm is integrated with ROS 2:

- **Driver Package**: `arm_borunte_brtirus2550a_driver`
- **Controller Package**: `arm_borunte_brtirus2550a_controller`
- **Communication**: Serial communication (libserial)

### ROS 2 Topics

- `/arm_brtirus2550a/joint_trajectory` - Joint trajectory control
- `/joint_states` - Joint state feedback

## Integration with RoboCon

The BRTIRUS2550A is used in:
- **RoboCon Sheather Tracked**: Mounted on lift platform
- **RoboCon Sheather Wheeled**: Mounted on lift platform

The arm provides precise manipulation capabilities for intelligent sheathing operations with extended reach from the lift platform.

---

**Disclaimer**: The rated power, specifications, external dimensions, etc. of this product are subject to modification without prior notice. Technical data and illustrations are for supply reference only and reserve the right to make changes.

