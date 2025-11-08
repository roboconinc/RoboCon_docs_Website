# CAN Bus Hardware IDs

Complete reference for all CAN Bus hardware IDs used in ROBOCON robots.

## Overview

ROBOCON robots use CAN bus (Controller Area Network) for communication with motors, actuators, sensors, and other hardware components. The CAN bus operates at **500kbps** for most devices, with some specialized sensors using **1Mbps**.

**CAN Bus Interface:**
- Device: `/dev/can0`
- Standard Bitrate: 500000 bps (500 kbps)
- Extended IDs: Used for some motor protocols (Golden Motor)

## CAN Bus Setup

```bash
# Setup CAN bus interface
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# Install CAN utilities
sudo apt install can-utils python3-can

# Monitor CAN bus traffic
candump can0

# Send test message
cansend can0 123#DEADBEEF
```

## Hardware ID Reference

### Motor Drivers

#### IDS830ABS Linear Actuator Drivers

**Type:** CANopen-based linear actuator controller  
**Bitrate:** 500 kbps  
**Protocol:** CANopen (standard IDs)  
**Default Node ID:** `0x01` (configurable)

| Device | CAN Node ID | Usage | Package |
|--------|-------------|-------|---------|
| Actuator 1 | `0x01` | Default/first actuator | `motor_driver_ids830abs` |
| Actuator 2 | `0x02` | Second actuator (if present) | `motor_driver_ids830abs` |
| Actuator 3 | `0x03` | Third actuator (if present) | `motor_driver_ids830abs` |
| Actuator 4 | `0x04` | Fourth actuator (if present) | `motor_driver_ids830abs` |
| Actuator 5 | `0x05` | Fifth actuator (if present) | `motor_driver_ids830abs` |

**Note:** Each IDS830ABS driver can be configured with a unique node ID from `0x01` to `0x7F`. The default is `0x01`.

**Response ID:** Node ID + `0x100` (e.g., `0x01` → response at `0x101`)

**Configuration:**
```yaml
motor_driver_ids830abs_node:
  ros__parameters:
    channel: can0
    bitrate: 500000
    node_id: 0x01  # Configurable: 0x01-0x7F
```

#### Golden Motor EZA48400 Wheel Motors

**Type:** CAN-based wheel motor controllers  
**Bitrate:** 500 kbps  
**Protocol:** Extended CAN IDs  
**Fixed Node IDs:** `0xEF` (left), `0xF0` (right)

| Motor | CAN Node ID | Decimal | Usage | Package |
|-------|------------|---------|-------|---------|
| Left Wheel | `0xEF` | 239 | Left drive motor | `golden_motor_eza48400` |
| Right Wheel | `0xF0` | 240 | Right drive motor | `golden_motor_eza48400` |

**CAN ID Format (Extended):**
- Command ID: `0x0C01_0000 | (node_id << 8) | 0xD0`
  - Left: `0x0C01EFD0`
  - Right: `0x0C01F0D0`
- Handshake RX ID: `0x1801_00_00 | (0xD0 << 8) | node_id`
  - Left: `0x1801D0EF`
  - Right: `0x1801D0F0`
- Feedback Part 1: `0x1801_00_00 | (0xD0 << 8) | node_id`
- Feedback Part 2: `0x1802_00_00 | (0xD0 << 8) | node_id`

**Configuration:**
```yaml
golden_motor_eza48400_node:
  ros__parameters:
    channel: can0
    bitrate: 500000
    reducer_ratio: 12.0
```

### LiDAR Sensors

#### Benewake TF03-180 LiDAR

**Type:** Long-range CAN bus LiDAR  
**Bitrate:** 1 Mbps (1,000,000 bps)  
**Range:** 0.1m - 180m  
**Protocol:** Proprietary CAN protocol

| Device | CAN ID | Bitrate | Usage | Package |
|--------|--------|---------|-------|---------|
| TF03-180 LiDAR | Vendor-specific | 1 Mbps | Long-range distance measurement | `lidar_benewake_tf03_180` |

**CAN Bus Setup for TF03-180:**
```bash
# Setup CAN bus at 1Mbps for TF03-180
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
```

**Note:** TF03-180 uses vendor-specific CAN IDs. Refer to the Benewake TF03-180 datasheet for detailed protocol information.

### Other CAN Bus Devices

Additional CAN bus devices may be present depending on robot configuration. Common ranges:

- **Reserved Range:** `0x000` - `0x07F` (Standard CAN IDs, often used for CANopen devices)
- **Motor Drivers:** `0x01` - `0x7F` (IDS830ABS and similar)
- **Extended IDs:** Used by Golden Motor EZA48400 and other proprietary protocols

## CAN Bus ID Ranges

### Standard CAN IDs (11-bit)

| Range | Usage | Devices |
|-------|-------|---------|
| `0x000` - `0x07F` | CANopen base ID range | IDS830ABS motor drivers, standard devices |
| `0x080` - `0x0FF` | Reserved/User | Available for custom devices |
| `0x100` - `0x17F` | CANopen response IDs | Responses from devices (node_id + 0x100) |
| `0x180` - `0x1FF` | Reserved | Extended protocol responses |

### Extended CAN IDs (29-bit)

Extended CAN IDs are used for proprietary protocols:

- **Golden Motor EZA48400:** `0x0C01_0000` - `0x0C01_FFFF` range
  - Command: `0x0C01_0000 | (node_id << 8) | 0xD0`
  - Feedback: `0x1801_0000` - `0x1802_FFFF` range

## Robot-Specific CAN Bus Configurations

### Mini Crane

**Motors:**
- IDS830ABS actuators: `0x01` - `0x05` (depending on configuration)

### Mini Loader

**Motors:**
- Golden Motor EZA48400: Left `0xEF`, Right `0xF0`
- IDS830ABS actuators: `0x01`+ (if equipped)

### Mini Excavator

**Motors:**
- Golden Motor EZA48400: Left `0xEF`, Right `0xF0`
- IDS830ABS actuators: `0x01`+ (boom, arm, bucket control)

### Transporter

**Motors:**
- Golden Motor EZA48400: Left `0xEF`, Right `0xF0`
- IDS830ABS actuators: `0x01`+ (load handling)

### RoboCon Servicer Tracked / RoboCon Servicer Wheeled

**Motors:**
- Golden Motor EZA48400: Left `0xEF`, Right `0xF0`
- IDS830ABS actuators: `0x01`+ (if equipped)

### RoboCon Sheather Tracked / RoboCon Sheather Wheeled

**Motors:**
- Golden Motor EZA48400: Left `0xEF`, Right `0xF0`
- IDS830ABS actuators: `0x01`+ (arm control, end effectors)
- BRTIRUS2550A arm joints (via serial communication, not CAN)

## CAN Bus Diagnostics

### Monitor CAN Traffic

```bash
# Monitor all CAN messages
candump can0

# Monitor with timestamps
candump can0 -t z

# Monitor specific CAN ID
candump can0 | grep "0EF"

# Monitor extended IDs
candump can0 -e
```

### Send Test Messages

```bash
# Send standard ID message
cansend can0 123#DEADBEEF

# Send extended ID message
cansend can0 12345678#DEADBEEF

# Send to specific device (e.g., IDS830ABS node 0x01)
cansend can0 01#00001A0000000000
```

### Check CAN Interface Status

```bash
# Check CAN interface
ip link show can0

# View CAN statistics
cat /proc/net/can/stats

# Check if CAN interface is up
ifconfig can0
```

## CAN Bus Troubleshooting

### Common Issues

**Issue: CAN interface not detected**
```bash
# Check if CAN interface exists
ls /dev/can*

# Load CAN modules
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev
sudo modprobe vcan  # Virtual CAN for testing
```

**Issue: No CAN messages received**
```bash
# Verify CAN bus is up
sudo ip link set can0 up

# Check bitrate matches device requirements
sudo ip link set can0 type can bitrate 500000

# Check for errors
ip -s link show can0
```

**Issue: CAN ID conflicts**
- Ensure each device has a unique CAN node ID
- For IDS830ABS: Configure node IDs in parameter files
- For Golden Motor: Fixed at 0xEF (left) and 0xF0 (right)

### Testing CAN Bus Communication

```python
import can

# Create CAN bus interface
bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=500000)

# Send test message
msg = can.Message(arbitration_id=0x01, data=[0x00, 0x1A, 0x00, 0x00, 0x01, 0x53, 0x00, 0x00])
bus.send(msg)

# Receive messages
for msg in bus:
    print(f"ID: {hex(msg.arbitration_id)}, Data: {msg.data.hex()}")
```

## CAN Bus Best Practices

1. **Unique IDs:** Always ensure each device has a unique CAN node ID
2. **Bitrate Consistency:** All devices on the same CAN bus must use the same bitrate
3. **Termination:** Ensure CAN bus is properly terminated (120Ω resistors at both ends)
4. **Error Handling:** Monitor CAN bus for error frames and implement proper error handling
5. **Configuration:** Store CAN node IDs in parameter files (YAML) for easy configuration
6. **Testing:** Use `candump` and `cansend` for debugging CAN bus communication

## Related Documentation

- [Motor Control API](../api-reference/motor-control.md) - Motor control via CAN bus
- [Sensor Interfaces](../api-reference/sensor-interfaces.md) - CAN bus sensors (TF03-180)
- [ROS 2 Custom Messages](../ros2/custom-messages.md) - CAN bus message types
- [Deployment: Raspberry Pi 5 Setup](../deployment/raspberry-pi5-setup.md) - CAN bus hardware setup

## Next Steps

- [Motor Control API](../api-reference/motor-control.md) - Control motors via CAN bus
- [Sensor Interfaces](../api-reference/sensor-interfaces.md) - CAN bus sensor integration
- [Raspberry Pi 5 Setup](../deployment/raspberry-pi5-setup.md) - Hardware configuration

