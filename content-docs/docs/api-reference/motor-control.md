# Motor Control API

The Motor Control API provides low-level control for motors and drivers via CAN bus communication.

## Overview

The ROBOCON SDK supports multiple motor types:

- **IDS830ABS** - Linear actuator motor drivers (CAN bus)
- **Golden Motor EZA48400** - Wheel motors for left/right drive (CAN bus)
- **Qiangdeli Actuators** - Multiple linear actuators (1/2/3/4/5)

## CAN Bus Setup

Before using motor control, configure the CAN bus:

```bash
# Setup CAN bus (500kbps standard)
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# Install CAN utilities
sudo apt install can-utils python3-can

# Test CAN bus
candump can0
```

> **See Also**: [CAN Bus Hardware IDs](../can-bus.md) - Complete reference for all CAN bus node IDs and hardware configuration

## IDS830ABS Motor Driver

The IDS830ABS is a CAN bus-based linear actuator controller for precise positioning.

### Installation

```bash
pip install python-can
```

### Parameters

| Parameter | Description | Example |
|-----------|-------------|---------|
| `channel` | CAN interface name | `can0` |
| `bitrate` | CAN bus speed | `500000` (500 kbps) |
| `node_id` | CANopen node ID | `0x01` (see [CAN Bus Hardware IDs](../can-bus.md) for complete ID reference) |
| `pulses_per_revolution` | Encoder pulses per revolution | `10000` (default) |
| `reducer_ratio` | Gear reduction ratio | `1.0` (direct drive) |
| `reset_encoder_on_start` | Reset encoder at startup | `False` |
| `max_linear_mm` | Maximum linear travel (mm) | `55` |
| `lead_mm_per_rev` | Linear distance per motor rev (mm) | `5.0` |
| `mm_per_radian` | Conversion factor (mm/rad) | Check physical robot |
| `motor_rpm` | Maximum motor speed (RPM) | Motor specification |
| `motor_acceleration` | Acceleration (factor of 65ms) | `5` = 325ms |
| `motor_deceleration` | Deceleration (factor of 65ms) | `5` = 325ms |
| `direction` | Rotation direction | `1` or `-1` |

### Usage

```python
import rclpy
from motor_driver_ids830abs.motor_driver_ids830abs_node import MotorDriverIDS830ABSNode

# Launch node
ros2 run motor_driver_ids830abs motor_driver_ids830abs_node

# Or via launch file
ros2 launch robot_mini_crane_bringup real_robot.launch.py
```

### ROS 2 Topics

- Control: `/motor_driver_ids830abs/command` (position, velocity, torque)
- Status: `/motor_driver_ids830abs/status` (current position, velocity, state)

### Initial Configuration

1. **Set `reset_encoder_on_start` to `True`** when actuator is at factory zero position
2. **Run the node once** to reset encoder
3. **Set `reset_encoder_on_start` to `False`** for normal operation

### Physical Connection

- Center position of actuator = zero steering angle
- Factory position (unextended) = one limit
- Use `direction` parameter to match rotation (1 for anti-clockwise, -1 for clockwise)

## Golden Motor EZA48400 (Wheel Motors)

The Golden Motor EZA48400 controllers provide wheel motor control for left/right drive via CAN bus.

### Initial Configuration

Before using the motors, configure them using the EZKontrol app:

1. **Set CAN Protocol:**
   - Open EZKontrol app
   - Settings → CAN communication protocol → Set to `102` (500k bitrate)

2. **Set CAN IDs:**
   - Left Motor: Settings → Part number → `240` (0xF0)
   - Right Motor: Settings → Part number → `239` (0xEF)

3. **Set PID Parameters:**
   - Settings → Kp rotor speed → `1` (for both motors)

### Usage

```python
# Launch real robot (includes wheel motors)
ros2 launch robot_oservicer_bringup real_robot.launch.py

# Test with teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_joy
```

### Parameters

| Parameter | Description | Example |
|-----------|-------------|---------|
| `channel` | CAN interface name | `can0` |
| `bitrate` | CAN bus speed | `500000` |
| `node_id` | CAN ID (left: 0xEF, right: 0xF0) | `0xEF` or `0xF0` (see [CAN Bus Hardware IDs](../can-bus.md)) |
| `reducer_ratio` | Gear reduction ratio | Motor specification |

### ROS 2 Integration

The wheel motors integrate with Ackermann controller:

- Subscribes to: `/cmd_vel` (Twist messages)
- Converts to left/right wheel velocities
- Publishes motor status via custom topics

## Qiangdeli Actuators

Multiple linear actuators (1/2/3/4/5) for various robot functions.

### Usage

```python
# Launch with robot bringup
ros2 launch robot_mini_crane_bringup real_robot.launch.py
```

## Low-Level Control

### Direct CAN Bus Control

```python
import can

# Connect to CAN bus
bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=500000)

# Send motor command
msg = can.Message(
    arbitration_id=0x01,  # Node ID
    data=[0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF],
    is_extended_id=False
)
bus.send(msg)
```

## ROS 2 Topics

### Motor Control Topics

| Motor Type | Command Topic | Status Topic |
|------------|---------------|--------------|
| IDS830ABS | `/motor_driver_ids830abs/command` | `/motor_driver_ids830abs/status` |
| Golden Motor | `/cmd_vel` (via Ackermann) | Motor-specific status topics |

## Safety Features

- Position limits enforced
- Velocity limits configured
- CAN bus timeout detection
- Emergency stop capability

## Next Steps

- [Basic Motion Routines](../motor-control/basic-motion-routines.md) - High-level motion patterns
- [Low-Level Control](../motor-control/low-level-control.md) - Advanced control
- [Examples](../motor-control/examples.md) - Complete examples
