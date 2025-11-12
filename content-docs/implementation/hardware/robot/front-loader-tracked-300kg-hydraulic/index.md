# RoboCon Front Loader Tracked 300kg Hydraulic

This folder contains asset deliverables for the RoboCon Front Loader Tracked 300kg Hydraulic robot.

**Folder Name**: robot_Loader TH300

## Asset List

The following table lists all major components, parts, model numbers, manufacturers, and brands used in the RoboCon Front Loader Tracked 300kg Hydraulic:

| Component Category | Component Name | Model Number / Part Number | Manufacturer / Brand | Specification | Application |
|-------------------|----------------|---------------------------|---------------------|---------------|-------------|
| **Power System** | Main Battery Pack | 72V 148Ah LiFePO4 | Ant Cloud Intelligent Equipment | 10.6 kWh capacity | Main power supply |
| **Power System** | Battery Charger | EPC602-4840-EP-01 | - | 240V/30A or 120V/12A | Onboard charging system |
| **Power System** | Charging Inlet | NEMA SS2-50P | Guangzhou Bosslyn Electric Co., Ltd. | 50A 125/250V | Power inlet connector |
| **Motor** | Drive Motor | 5.5 kW DC Permanent Magnet Motor | Jining Yongli Electric Co., Ltd. | 5.5kW rated, 3000 rpm | Base movement and mobility |
| **Motor** | Wheel Motor (Left) | Golden Motor EZA48400 | Golden Motor | CAN Bus Node ID: 0xEF | Left track drive |
| **Motor** | Wheel Motor (Right) | Golden Motor EZA48400 | Golden Motor | CAN Bus Node ID: 0xF0 | Right track drive |
| **Motor Driver** | CAN Motor Driver | IDS830ABS | - | CAN-based linear actuator controller | Bucket/lift control, steering |
| **Hydraulic System** | Hydraulic Pump | DLH1 series | Ningbo Deli Hydraulics Co., Ltd. / Ant Cloud | 40 L/min flow rate | Hydraulic system power |
| **Hydraulic System** | Valve Block | 6-way 12VDC proportional valve | Ant Cloud (internal brand) | 12VDC proportional | Hydraulic control |
| **Hydraulic System** | Oil Cooler | - | Jining Kadi Hydraulic Equipment Co., Ltd. | Aluminum fin cooler | Hydraulic oil cooling |
| **Hydraulic System** | Hydraulic Oil | #46 Hydraulic Oil | - | ISO VG 46 | Hydraulic fluid |
| **Control System** | Main Controller (MCU) | CFB108 | Cyber-MI (Shenzhen Cyber Industrial) | CAN Interface | Main control unit |
| **Control System** | Remote Controller | Ant Cloud 2.4 GHz | Ant Cloud Proprietary | 2.4 GHz dual-joystick | Remote control (up to 100m range) |
| **Control System** | Remote Receiver | Ant Cloud Proprietary | Ant Cloud Proprietary | 12V DC input | Remote control receiver |
| **Control System** | PLC | 2AO-8AI-8DI-8DO 24V | - | 2 Analog Outputs, 8 Analog Inputs, 8 Digital Inputs, 8 Digital Outputs | Control system I/O |
| **Pan-Tilt** | Pan-Tilt Camera Mount | J-PT-720 / J-PT-760 | Tianjin Jiajie Shengchuang Technology Co., Ltd. (JEC Century) | 1.7 kg load capacity, 4 kg total weight | Camera positioning and stabilization |
| **Camera** | Integrated Camera | ZN2133 | Tianjin Jiajie Shengchuang Technology Co., Ltd. (JEC) | 33× 2 MP Starlight Network Camera Module, 1/2.8" CMOS Sensor | Visual inspection and monitoring |
| **Depth Camera** | Depth Sensor (Front) | SE-1035 | HINSON Robotics (Shenzhen Hinson Intelligent Systems Co., Ltd.) | Up to 10m range, 12V DC @ 5W | 3D object mapping and collision avoidance (front) |
| **Depth Camera** | Depth Sensor (Rear) | SE-1035 | HINSON Robotics (Shenzhen Hinson Intelligent Systems Co., Ltd.) | Up to 10m range, 12V DC @ 5W | 3D object mapping and collision avoidance (rear) |
| **Depth Camera** | Stereo Depth AI Module (Front) | OAK-D-S2 | Luxonis Inc. (USA) | 4K AI-accelerated, Intel Movidius Myriad X VPU | 3D environment perception (front) |
| **Depth Camera** | Stereo Depth AI Module (Rear) | OAK-D-S2 | Luxonis Inc. (USA) | 4K AI-accelerated, Intel Movidius Myriad X VPU | 3D environment perception (rear) |
| **IMU Sensor** | 9-Axis IMU | BW-MINS50 | BWSensing | 9-axis (accelerometer, gyroscope, magnetometer) | Orientation and motion sensing |
| **Pressure Sensor** | Hydraulic Pressure Sensor | PS-1L-NV | - | Negative pressure sensor | Hydraulic pressure monitoring |
| **Electrical** | Main Harness & Connectors | - | Shenzhen Junda Cable Systems | Waterproof IP67 connectors | Electrical wiring system |
| **Electrical** | Lighting & Signal | - | Ant Cloud OEM | 24V LED working lights | Illumination system |
| **Compute** | Main GPU | Radeon™ RX 7900 XTX | AMD, USA | 24GB GDDR6, 96 Compute Units | Real-time visual inference and GPU compute |
| **Compute** | AI Co-Processor (Training) | A100 / L40S | NVIDIA Cloud | Cloud-based training | Fine-tuning hybrid YOLO and DeepSeek models |
| **Compute** | Edge Compute Unit | Custom Module | ROBOCON INC + Ant Cloud Intelligent Equipment | Custom integration | Houses GPU, AI co-processors, camera interface boards |
| **Compute** | ROS 2 Integration Kit | Custom Module | ROBOCON INC (California, USA) | Custom integration | Interfaces CAN BUS with ROS 2 Nodes |
| **Chassis** | Chassis Frame | - | Ant Cloud Intelligent Equipment Co., Ltd. | Q235 Structural Steel, powder-coated | Main structural frame |
| **Chassis** | Bucket | - | Ant Cloud Intelligent Equipment Co., Ltd. | 950mm width, 0.1 m³ volume | Material handling bucket |
| **Chassis** | Arms | - | Ant Cloud Intelligent Equipment Co., Ltd. | - | Lift arm assembly |
| **Chassis** | Tracks | - | - | 180 × 72 × 34 mm | Tracked undercarriage |
| **Paint** | Paint System | ROBOCON Red | - | Hex #d62b27 (Powder coat + UV Clear) | Exterior finish |

## Component Manufacturers

| Component Category | Manufacturer | Location |
|-------------------|--------------|----------|
| **Chassis Frame, Bucket, Arms** | Ant Cloud Intelligent Equipment Co., Ltd. | Jining, Shandong, China |
| **Hydraulic Assembly** | Deli Hydraulics / Ant Cloud | Ningbo / Jining, China |
| **Motor and Battery Assembly** | Ant Cloud Intelligent Equipment Co., Ltd. | Jining, Shandong, China |
| **Electrical System Harnessing** | Shenzhen Junda Cable Systems | Shenzhen, China |
| **Control System Integration** | ROBOCON INC | Saratoga, California, USA |
| **Final Assembly, Software Integration** | ROBOCON INC | California, USA |
| **Final QA & Testing** | ROBOCON INC | California, USA |

## Primary Manufacturer

**Primary Manufacturer:** Ant Cloud Intelligent Equipment (Shandong) Co., Ltd.  
📍 99 Tongji Road, High-Tech Zone, Jining City, Shandong Province, China

**Final Assembly & System Integration:**  
ROBOCON INC  
📍 Oakland, California, USA

## Deliverables
