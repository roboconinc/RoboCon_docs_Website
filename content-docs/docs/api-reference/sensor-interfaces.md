# Sensor Interfaces

API for accessing and interfacing with robot sensors on ROBOCON platforms.

## Overview

The ROBOCON SDK provides interfaces for multiple sensor types:

- **LiDAR Sensors**: Hinson SE-1035, DE-4511 (Ethernet), Benewake TF03-180 (CAN bus)
- **IMU**: BW MINS50 via RS485
- **Depth Cameras**: OAK-D S2, TM815 IX E1
- **Energy Meters**: Accrel AMC16, AMC16Z-FDK24
- **Pressure Sensors**: PS-1L-NV with R4IVB02 analog reader

## LiDAR Sensors

### Hinson SE-1035 LiDAR

Ethernet-based LiDAR sensor for 360° scanning.

**Hardware Configuration:**
- Power: 24V (brown: +24V, blue: ground)
- Communication: Ethernet TCP/IP
- Default IP: 192.168.1.88
- Default Port: 8080

**ROS 2 Topics:**
- `/lidar_hinson_se_1035/scan` - `sensor_msgs/LaserScan` (0-361° range)

**ROS 2 Services:**
- `/HinsSESrv` - `lidar_hinson_interfaces/HinsSrv` (Area detection)

**Launch:**
```bash
ros2 launch lidar_hinson_SE_1035 hins_se_launch.py
```

**Monitor:**
```bash
ros2 topic echo /lidar_hinson_se_1035/scan
ros2 service call /HinsSESrv lidar_hinson_interfaces/srv/HinsSrv "{channel: 0}"
```

### Hinson DE-4511 LiDAR

Ethernet-based LiDAR similar to SE-1035.

**Launch:**
```bash
ros2 launch lidar_hinson_DE_4511 hins_de_launch.py
```

**Topics:**
- `/lidar_hinson_de_4511/scan` - `sensor_msgs/LaserScan`

### Benewake TF03-180 LiDAR

CAN bus-based long-range LiDAR (0.1m - 180m range).

**Hardware Configuration:**
- Communication: CAN bus (1Mbps default)
- Range: 0.1m - 180m
- Accuracy: ±2cm
- Update Rate: 1000Hz

**CAN Bus Setup:**
```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
sudo apt install can-utils python3-can
```

> **See Also**: [CAN Bus Hardware IDs](../can-bus.md) - CAN bus configuration and hardware ID reference

**ROS 2 Topics:**
- `/tf03_180/range` - `sensor_msgs/Range` (Distance measurement)
- `/tf03_180/status` - `lidar_benewake_tf03_180_msgs/LidarStatus`

**Launch:**
```bash
ros2 launch lidar_benewake_tf03_180_can_interface lidar_benewake_tf03_180_can_interface_launch.py
```

**Monitor:**
```bash
ros2 topic echo /tf03_180/range
ros2 topic echo /tf03_180/status
```

## IMU Sensor

### BW MINS50 IMU

High-frequency IMU sensor via RS485.

**Hardware Configuration:**
- Connection: RS485 port `/dev/ttySC1` (2nd RS485 port)
- Baudrate: 115200 (changed from default 9600)
- Output Frequency: 50 Hz

**ROS 2 Topics:**
- `/imu/data` - `sensor_msgs/Imu` (ENU frame, yaw increases anti-clockwise)

**Launch:**
```bash
ros2 run sensor_bw_mins50 sensor_bw_mins50_node
# Or via robot launch
ros2 launch robot_oservicer_bringup real_robot.launch.py
```

**Monitor:**
```bash
ros2 topic echo /imu/data
```

**GUI Display:**
The IMU data is displayed in the hardware monitor GUI under the IMU panel.

## Depth Cameras

### OAK-D S2 Depth Camera

RGB-D camera with on-camera YOLOv11 inference.

**Hardware Requirements:**
- USB 3.0 port
- Luxonis OAK-D S2 camera

**Installation:**
```bash
# Install dependencies (careful with numpy)
python3 -m pip install "numpy<2.0.0"
sudo apt install ros-jazzy-depthai-ros ros-jazzy-cv-bridge python3-opencv
```

**ROS 2 Topics:**
- `/camera/rgb` - `sensor_msgs/Image` (RGB image)
- `/camera/depth` - `sensor_msgs/Image` (Depth image)
- `/camera/pointcloud` - `sensor_msgs/PointCloud2` (3D point cloud)
- `/depth_camera_oak_d_s2/detections` - `vision_msgs/Detection2DArray` (YOLOv11 detections)
- `/depth_camera_oak_d_s2/masks` - `sensor_msgs/Image` (Segmentation masks)

**Launch:**
```bash
ros2 launch depth_camera_oak_d_s2 oak_d_s2_launch.py
```

**YOLOv11 Features:**
- On-camera inference (Myriad X VPU)
- Instance segmentation
- Real-time object detection
- Low latency (~15ms inference time)

### TM815 IX E1 Depth Camera

Alternative depth camera option.

**Topics:**
- `/camera/color/image_raw` - `sensor_msgs/Image`

## Energy Meters

### Accrel AMC16 Energy Meter

6-channel energy monitoring via RS485.

**Configuration:**
- Modbus Address: 3
- RS485 Port: `/dev/ttySC0`
- Set via LCD: Menu → Password (0001) → Communication → MODBUS → Address → 003

**ROS 2 Topics:**
- `/amc16/status` - `robot_custom_interfaces/msg/AMC16Status`

**Monitor:**
```bash
ros2 topic echo /amc16/status
```

### Accrel AMC16Z-FDK24 Energy Meter

24-channel energy monitoring via RS485.

**Configuration:**
- Modbus Address: 6
- Baudrate: 9600
- Use script to change address:
```bash
python3 src/Robocon-OS/src/energy_meter_acrel_amc16z_fdk24/energy_meter_acrel_amc16z_fdk24/change_address_amc16z_fdk24.py \
  --port /dev/ttyUSB0 --current_address 1 --new_address 6 \
  --current_baudrate 57600 --new_baudrate 9600
```

**ROS 2 Topics:**
- `/amc16z_fdk24/status` - `robot_custom_interfaces/msg/AMC16ZFDK24Status`

**Monitor:**
```bash
ros2 topic echo /amc16z_fdk24/status
```

**GUI Display:**
24-channel panel showing voltage, current, and power for all channels.

## Pressure Sensor

### PS-1L-NV Pressure Sensor with R4IVB02 Reader

Analog pressure sensor with Modbus interface.

**Configuration:**
1. Connect only R4IVB02 to `/dev/ttySC0`
2. Change Modbus address to 4:
```bash
python3 src/Robocon-OS/src/r4ivb02_analog_voltage_reader/r4ivb02_analog_voltage_reader/change_adddress_r4ivb02.py \
  --port /dev/ttySC0 --current_address 1 --new_address 4
```

**ROS 2 Topics:**
- `/r4ivb02_reader/status` - `robot_custom_interfaces/msg/R4IVB02Status` (Analog voltages)
- `/ps1lnv_sensor/pressure` - `std_msgs/msg/Float32` (Pressure in kPa)

**Launch:**
```bash
ros2 run pressure_sensor_ps_1l_nv pressure_sensor_ps_1l_nv_node
ros2 run serial_sc0_master serial_sc0_master_node
```

**Monitor:**
```bash
ros2 topic echo /r4ivb02_reader/status
ros2 topic echo /ps1lnv_sensor/pressure
```

## Control Systems

### JPF4816 Fan Speed Controller

4-channel fan controller via RS485.

**Configuration:**
- Modbus Address: 2
- Set via LCD: Mode → A01 → Up key → A02 → OK

**ROS 2 Topics:**
- `/jpf4816_fan_controller/status` - `robot_custom_interfaces/msg/JPF4816Status`

### N4ROC04 Relay Controller

4-channel relay controller via RS485.

**Configuration:**
- Modbus Address: 1 (default)

**ROS 2 Topics:**
- `/relay_controller/status` - `robot_custom_interfaces/msg/N4ROC04RelayStatus`
- `/relay_controller/command` - `std_msgs/msg/UInt8MultiArray` (Control relays)

**Control via GUI:**
Relay control panel in hardware monitor GUI.

### PLC 10IOA12

12 Digital Inputs and 12 Digital Outputs via RS485.

**Configuration:**
- Modbus Address: 5
- Set via board switches: Switch 1 and 3 ON, rest OFF (binary 101 = 5)

**ROS 2 Topics:**
- `/plc10ioa12/status` - `robot_custom_interfaces/msg/PLC10IOA12Status`

**Relation Modes:**
- Mode 0: DI and DO unrelated (manual control)
- Mode 1: Self-locking (default)
- Mode 5: Output = Input

## Serial SC0 Master

Master node for RS485 devices on first serial port.

**Dependencies:**
```bash
python3 -m pip install pyserial pymodbus
```

**Devices Handled:**
- AMC16 energy meter (address 3)
- JPF4816 fan controller (address 2)
- N4ROC04 relay (address 1)
- AMC16Z-FDK24 energy meter (address 6)
- R4IVB02 analog reader (address 4)
- PLC 10IOA12 (address 5)

**Launch:**
```bash
ros2 run serial_sc0_master serial_sc0_master_node
```

## ROS 2 Integration

All sensors publish to standard ROS 2 topics and can be monitored via:

```bash
ros2 topic list
ros2 topic echo <topic_name>
```

## Hardware Monitor GUI

Monitor all sensors via the hardware monitor GUI:

```bash
# Setup display (Raspberry Pi 5)
export DISPLAY=:0
export QT_QPA_PLATFORM=wayland

# Launch GUI
ros2 run robocon_hw_monitor_gui robocon_hw_monitor_gui_node
```

The GUI provides panels for:
- IMU data
- Energy meters (AMC16, AMC16Z-FDK24)
- Fan controller
- Relay controller
- PLC status
- Battery charger
- Hardware status overview

## Next Steps

- [Architecture Overview](../architecture/robocon-os.md) - System architecture
- [ROS 2 Integration](../ros2/nodes-and-topics.md) - ROS 2 patterns
- [Deployment](../deployment/robocon-hardware.md) - Hardware deployment
