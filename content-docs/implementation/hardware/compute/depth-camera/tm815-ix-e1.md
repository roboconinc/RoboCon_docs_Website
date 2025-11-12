# Depth Camera TM815-IX-E1

## Overview

The TM815-IX-E1 is a 3D depth camera from Percipio designed for industrial applications. It uses active stereo technology to provide depth maps, color images, infrared images, and point clouds. This camera is part of the FM815-IX-E1 series and is suitable for RoboCon robot systems requiring high-precision depth sensing.

## Documentation

Documentation files available:
- `Depth Camera_TM815-IX-E1_Manual.pdf`
- `Depth Camera_TM815-IX-E1_英文.pdf`

*Documentation located in: Depth Camera_TM815-IX-E1/Docs/*

## Specifications

### FM815-IX-E1 Specifications

| Index | Specifications |
|-------|----------------|
| Technology Principle | Active Stereo |
| Working Distance | 800 mm ~ 3000 mm |
| Field of View | Near Field: 850 mm x 710 mm @ 800 mm (H/V: Approx. 6°)<br />Far Field: 3465 mm x 2670 mm @ 3000 mm (H/V: Approx. 6°) |
| Accuracy | Z: 1.56 mm @ 1000 mm<br />XY: 3.20 mm @ 1000 mm |
| Frame Rate @ Resolution (Depth) | 5 fps @ 1280 x 960<br />5 fps @ 640 x 480<br />5 fps @ 320 x 240 |
| Frame Rate @ Resolution @ Image Format (Color) | 4 fps @ 2560 x 1920 @ YUYV<br />6 fps @ 2560 x 1920 @ CSI BAYER12GBRG<br />8 fps @ 1920 x 1440 @ YUYV<br />16 fps @ 1280 x 960 @ YUYV<br />25 fps @ 640 x 480 @ YUYV |
| Power Supply | External DC Power Supply: 24V (±10%)<br />PoE Power Supply: IEEE802.3 at/af PoE |
| Dimensions (excluding ports) | 145 mm x 35 mm x 90 mm |
| Weight | 620 g |
| Power Consumption | 5.0W ~ 11.3W |
| Ingress Protection | IP65 |
| Temperature | Operating: -10°C ~ 50°C<br />Storage: -20°C ~ 55°C |

## Hardware Installation

### Connectors & Cables

#### Power & Trigger Connector

The camera features an M12 A-Code connector for power and trigger signals.

| Pin Number | Name | Descriptions |
|------------|------|--------------|
| 1 | TRIG_OUT 1 | Trigger output signal 1 (rising-edge) |
| 2 | P_24V | Power (camera) |
| 3 | P_GND | GND (camera) |
| 4 | TRIG_POWER | Power (trigger circuit) |
| 5 | TRIG_GND | GND (trigger circuit) |
| 6 | TRIG_IN 2 | Trigger input signal 2 (falling-edge) |
| 7 | TRIG_IN 1 | Trigger input signal 1 (rising-edge) |
| 8 | TRIG_OUT 2 | Trigger output signal 2 (falling-edge) |

#### Data Connector

The camera uses an M12 X-Code connector for Gigabit Ethernet data transmission. The provided cable is an M12 X-Code to RJ45 Gigabit Ethernet cable, with the RJ45 end complying with the EIA/TIA 568B standard.

### Indicator Lights

The camera initialization process takes approximately 40 seconds. During initialization, the PWR indicator light stays on constantly.

| Name | Color | Descriptions |
|------|-------|--------------|
| PWR (Camera Status Indicator) | Red | Flashing at 1Hz: The camera is working normally.<br />Flashing at > 1Hz: The camera firmware has encountered an initialization error.<br />Constantly on: The camera is currently in a system freeze state.<br />Constantly off: The camera is either not powered on or in a system freeze state. |
| ETH (Network Connection Indicator) | Green | Constantly on: The camera is working in Gigabit Ethernet mode.<br />Constantly off: The camera is not working in Gigabit Ethernet mode. |
| ACT (Network Transmission Indicator) | Yellow | Flashing: Data is being transmitted.<br />Constantly on: No data is being transmitted. |

### Camera Installation

Percipio does not include a mounting bracket for the camera. When installing the camera, choose a suitable mounting hole based on your specific requirements. The camera's casing is designed with built-in heat dissipation capabilities, so no additional cooling measures are necessary. Ensure good ventilation around the camera and that the metal mounting surface is in contact with the equipment during installation to optimize heat dissipation.

### Power Supply & Network Connection

#### Connection Method 1: External DC Power Supply

1. **Network Connection:**
   - Connect the M12 X-Code end of the Gigabit Ethernet cable to the camera's data connector.
   - Insert the RJ45 end of the Gigabit Ethernet cable into the RJ45 network port of the industrial computer (host computer).

2. **Network Configuration:**
   - Percipio cameras are configured with dynamic IP addresses by default, enabling them to automatically obtain an IP.
   - If a static IP address needs to be set, use the SDK or Percipio Viewer software.

3. **Power Supply:**
   - Recommended specification: 24V (±10%) external DC power supply.
   - Connect the M12 A-Code end of the trigger power cable to the camera's power & trigger connector.
   - Connect Pin Number 2 (Power Positive Wire) and Pin Number 3 (Power Ground Wire) to the external DC power supply.

#### Connection Method 2: PoE Switch

1. **Network Connection:**
   - Connect the M12 X-Code end of the Gigabit Ethernet cable to the camera's data connector and the RJ45 end to a port on the Gigabit Ethernet switch.
   - Use another RJ45 Gigabit Ethernet cable to connect the industrial computer (host computer) to the switch.

2. **Power Supply:**
   - Ensure the switch is a PoE switch compliant with IEEE802.3 at/af standards.
   - The PoE switch can both power the camera and transmit data.
   - When both external DC power supply and PoE switch power supply are available, the camera will give priority to using the external DC power supply.

## Hardware Trigger Connection

The camera supports two hardware trigger input/output channels (rising edge and falling edge). Hardware triggering allows the camera to capture images upon receiving an external trigger signal.

### Electrical Specifications for Hardware Triggering

| Index | Minimum (V) | Typical (V) | Maximum (V) |
|-------|-------------|-------------|--------------|
| TRIG_POWER Voltage | 11.4 | — | 25.2 |
| TRIG_OUT High Voltage | 11.4 | — | 25.2 |
| TRIG_OUT Low Voltage | -0.3 | 0 | 0.4 |
| TRIG_IN High Voltage | 11.4 | — | 25.2 |
| TRIG_IN Low Voltage | -0.3 | 0 | 0.4 |

### External Trigger Input Signal Requirements

- **Rising-edge trigger:** High pulse square wave signal required, with the rising-edge being effective. Pulse width: 10 to 30 milliseconds. Signal rise time should not exceed 5 microseconds.
- **Falling-edge trigger:** Low pulse square wave signal required, with the falling-edge being effective. Pulse width: 10 to 30 milliseconds. Signal fall time should not exceed 5 microseconds.
- **Trigger frequency:** Must not exceed the device's processing capability (i.e., the frame rate in continuous mode). Otherwise, the camera will discard the trigger signals without processing them.

## How to Run the Camera

Users can use **Percipio Viewer**, a proprietary image viewing software developed by Percipio, to preview the camera's output in real-time, including depth maps, color images, infrared images, and point clouds. Additionally, users can control the camera through Percipio's SDK and a series of APIs.

### Download Links

- **Percipio Viewer:** https://en.percipio.xyz/downloadcenter/
- **SDK:** https://en.percipio.xyz/downloadcenter/

### Tutorial Links

- **Percipio Viewer User Guide:** https://doc.percipio.xyz/cam/latest/en/viewer-en.html
- **SDK and API Documentation:** https://doc.percipio.xyz/cam/latest/en/index.html
- **Troubleshooting Guide:** https://doc.percipio.xyz/cam/latest/en/troubleshooting/index-en.html

## Safety

### Safety Precautions

- Read this user manual carefully and understand how to use this product correctly before operation.
- This product should be installed, connected, used, and maintained by qualified adult technicians.
- Do not place flammable, explosive, or other dangerous items near the camera.
- Avoid collisions, throwing, or dropping the camera.
- Prevent metal objects, dust, paper, sawdust, and other foreign materials from entering the camera.
- Do not use the camera in environments with extreme temperatures.
- Avoid using the camera in corrosive environments.
- Do not point the lens directly at the sun or other strong light sources for extended periods.
- Operate the camera at altitudes below 2,000 meters above sea level.
- Install the camera in a well-ventilated and open area.
- Do not use a power supply with a voltage higher than the standard power supply voltage.

### Pre-use Inspection

Before each use, carefully inspect the camera to ensure it is in normal working condition. Check for any signs of damage, water ingress, unusual odors, smoke emissions, or missing/damaged screws. If any issues are detected, immediately cut off the power and discontinue use.

## Service and Maintenance

### Service

FM815-IX-E1 is a precision optical instrument with no user-serviceable parts inside. Do not disassemble the camera.

### Maintenance (Cleaning)

- Use lint-free cloth and alcohol or water to wipe the camera window (glass panel) to keep it clean.
- Avoid using gasoline or other corrosive and volatile solvents.
- Regularly clean the dust on the camera surface to ensure efficient heat dissipation.

### Maintenance (Storage)

- Do not immerse the camera in water or place it in a high-humidity environment.
- Store the camera in a cool, dry, and well-ventilated indoor location.
- Storage temperature range: -20°C to 55°C.
- Disconnect the camera from the power supply before storage.
- Do not point the camera lens directly at the sun or other strong light sources for an extended period.

## Compliance

The FM815-IX-E1 3D camera complies with the following standards:

- **European Union EMC Standards:** EN 61000-3-2: 2014, EN 61000-3-4: 2013, EN IEC 61000-6-2: 2019, EN IEC 61000-6-4: 2019
- **American Standards:** ANSI C63.4-2014 and FCC Code CFR47 PART15B (2022)
- **China RoHS:** IEC 62321-3-1:2013, IEC 62321-4:2013+A1:2017, IEC 62321-5:2013, IEC 62321-6:2015, IEC 62321-7-1:2015, IEC 62321-7-2:2017, IEC 62321-8:2017
- **Korean KC Certification:** KS C 9832:2023, KS C 9835:2019, KS C 9610-4-2, KS C 9610-4-3, KS C 9610-4-4, KS C 9610-4-6

## Integration

This depth camera is used in RoboCon robot systems for:
- 3D scene reconstruction
- Object detection and tracking
- Depth sensing for navigation
- Visual perception tasks

## ROS 2 Integration

The TM815-IX-E1 depth camera integrates with ROS 2 for depth data publishing and processing. The camera can publish depth maps, color images, and point clouds through ROS 2 topics for integration with robotic perception pipelines.
