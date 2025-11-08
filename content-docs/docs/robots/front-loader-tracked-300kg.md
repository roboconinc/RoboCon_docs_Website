# RoboCon Front Loader Tracked 300kg

**Brand/Make:** RoboCon  
**Model Family:** Front Loader  
**Base Option:** Tracked  
**Max Payload:** 300kg

The RoboCon Front Loader Tracked 300kg is an autonomous electric skid steer loader designed for construction applications and robotics integration. Built on the ROBOCON ecosystem, it combines robust mechanical design with advanced AI capabilities for autonomous material handling operations.

## Product Specifications

### General Information

| Parameter | Specification |
|-----------|--------------|
| **Product Type** | Autonomous Electric Skid Steer Loader |
| **Category** | Compact Electric Loader for Construction & Robotics Integration |
| **Brand** | ROBOCON™ |
| **Model** | RoboCon Mini Loader |
| **Base Price** | $43,000.00 USD |

### Power System

| Parameter | Specification |
|-----------|--------------|
| **Power System** | 72V Electric Drive |
| **Battery Type** | Lithium-Ion (LiFePO4) |
| **Battery Capacity** | 10.6 kWh (148 Ah @ 72V nominal) |
| **Runtime** | 7 hrs (normal use), 3 hrs (heavy use) |
| **Charging Voltage** | 120/240 VAC |
| **Charging Current** | 12A (120V) / 30A (240V) |
| **Charging Connector** | NEMA SS2-50P (ETL Listed, Guangzhou Bosslyn Electric Co., Ltd.) |
| **Charging Method** | Onboard AC-DC Converter (Ant Cloud proprietary) |
| **Charging Time** | Approx. 3 hours (240V) |

### Mechanical & Hydraulic Specifications

| Parameter | Specification |
|-----------|--------------|
| **Max Lift Capacity** | 660 lbs (300 kg) |
| **Max Lift Height** | 7.1 ft (2170 mm) |
| **Max Unloading Height** | 4.4 ft (1345 mm) |
| **Bucket Width** | 3.1 ft (950 mm) |
| **Bucket Volume** | 3.5 ft³ (0.1 m³) |
| **Unloading Angle** | 47.6° |
| **Hydraulic System Pressure** | 18 MPa |
| **Hydraulic Flow** | 40 L/min |
| **Hydraulic Oil Type** | #46 Hydraulic Oil (ISO VG 46) |
| **Ground Clearance** | 90 mm |
| **Tipping Angle / Repose** | 25° |
| **Drive Motor Type** | 5.5 kW DC Permanent Magnet Motor |
| **Motor Speed** | 3000 rpm |
| **Pump Brand** | Ningbo Deli Hydraulics Co., Ltd. (DLH1 series, Ant Cloud integration) |
| **Valve Block** | 6-way 12VDC proportional valve, Ant Cloud internal brand |
| **Oil Cooler** | Aluminum fin cooler, Jining Kadi Hydraulic Equipment Co., Ltd. |

### Dimensions & Weight

| Parameter | Specification |
|-----------|--------------|
| **Length × Width × Height** | 2250 × 880 × 1400 mm |
| **Track Size** | 180 × 72 × 34 mm |
| **Machine Weight** | 1320 lbs (600 kg) |
| **Turning Radius** | Zero-turn skid steering |
| **Frame Material** | Q235 Structural Steel, powder-coated |
| **Paint Code** | ROBOCON Red – Hex #d62b27 (Powder coat + UV Clear) |

### Control & Interface

| Component | Specification |
|-----------|--------------|
| **Control System** | Cyber-MI CFB108 Controller (Shenzhen Cyber Industrial) |
| **Communication** | CAN BUS (Open, supports ROS 2 integration) |
| **Remote Controller** | Proprietary Ant Cloud 2.4 GHz dual-joystick controller |
| **Range** | Up to 100 meters line of sight |
| **Receiver Voltage** | 12V DC |
| **Control Channels** | 6-way proportional control |
| **Interface Protocol** | OpenCAN for RoboCon Network |
| **Onboard Camera Mount** | JEC J-PT-760 Pan-Tilt compatible (VESA holes pre-drilled) |

### Electrical System Components

| Component | Manufacturer | Specification |
|-----------|--------------|--------------|
| **Main Battery Pack** | Ant Cloud Intelligent Equipment | 72V 148Ah LiFePO4 |
| **DC Motor** | Jining Yongli Electric Co., Ltd. | 5.5kW rated |
| **Onboard Charger** | Guangzhou Bosslyn Electric Co., Ltd. | 240V/30A or 120V/12A |
| **Main Controller (MCU)** | Cyber-MI CFB108 | CAN Interface |
| **Remote Receiver** | Ant Cloud Proprietary | 12V DC input |
| **Main Harness & Connectors** | Shenzhen Junda Cable Systems | Waterproof IP67 connectors |
| **Lighting & Signal** | Ant Cloud OEM | 24V LED working lights |

### Component Manufacturers

| Component | Manufacturer | Location |
|-----------|--------------|----------|
| **Chassis Frame, Bucket, Arms** | Ant Cloud Intelligent Equipment Co., Ltd. | Jining, Shandong, China |
| **Hydraulic Assembly** | Deli Hydraulics / Ant Cloud | Ningbo / Jining |
| **Motor and Battery Assembly** | Ant Cloud Intelligent Equipment Co., Ltd. | Jining, Shandong |
| **Electrical System Harnessing** | Shenzhen Junda Cable Systems | Shenzhen, China |
| **Control System Integration** | ROBOCON INC | Saratoga, California, USA |
| **Final Assembly, Software Integration** | ROBOCON INC | California, USA |
| **Final QA & Testing** | ROBOCON INC | California, USA |

### AI Autonomous Sensors and Compute

| Component | Manufacturer | Model / Specification | Description |
|-----------|--------------|----------------------|-------------|
| **Main GPU** | AMD, USA | Radeon™ RX 7900 XTX | 24GB GDDR6, 96 Compute Units, 4K+ graphics acceleration. Used for real-time visual inference and high-efficiency GPU compute for RoboCon's hybrid YOLO + DeepSeek model. |
| **AI Co-Processor (Inference)** | NVIDIA Cloud | A100 / L40S (training cloud) | Used for fine-tuning hybrid YOLO foundation model and DeepSeek-R1-Distill-Llama-8B-GGUF for autonomous operation logic. |
| **Pan-Tilt Platform** | Tianjin Jiajie Shengchuang Technology Co., Ltd. (JEC Century) | JEC J-PT-720 / J-PT-760 | Aluminum alloy pan-tilt unit with 1.7 kg load capacity and 4 kg total weight. Designed for mid-range scene monitoring. Power: 12 V DC, low power consumption. Indoor/outdoor rated with 1-year warranty. Dimensions: 43 × 34 × 22 cm. |
| **Integrated Camera** | Tianjin Jiajie Shengchuang Technology Co., Ltd. | JEC ZN2133 – 33× 2 MP Starlight Network Camera Module | 1/2.8" CMOS Sensor, 33× Optical Zoom + 16× Digital Zoom. 0.001 Lux Color Sensitivity, Full HD 1920×1080 @ 30 fps. H.265/H.264/MJPEG compression, 3D noise reduction, 120 dB WDR. ONVIF Profile S/G support. Supports area intrusion and motion detection. Power consumption: 2.5 W (IR 4.5 W max). Dimensions: 97.5 × 61.5 × 50 mm. Weight: 268 g. |
| **Depth Cameras** | HINSON Robotics (Shenzhen Hinson Intelligent Systems Co., Ltd.) | SE-1035 x2 | Dual-depth sensor units mounted front and rear for 3D object mapping and collision avoidance. Each unit provides RGB-D data via GigE and RS-485 with onboard ToF processor for multi-sensor fusion. Range: up to 10 m. Power: 12 V DC @ 5 W. |
| **Stereo Depth AI Modules** | Luxonis Inc. (USA) | OAK-D-S2 (x2) | 4K AI-accelerated stereo depth cameras based on Intel Movidius Myriad X VPU. On-device neural inference with ROS 2 integration. USB 3.1 interface for RoboCon Network connectivity. FOV: 81° (H) × 52° (V). Depth range: 0.3–10 m. Rated for -20 °C to +50 °C. |
| **ROS 2 Integration Kit** | ROBOCON INC (California, USA) | Custom Module | Interfaces RoboCon Network CAN BUS with ROS 2 Nodes for sensor fusion, mapping, and real-time AI control integration. |
| **Main Compute Integration** | ROBOCON INC + Ant Cloud Intelligent Equipment | Custom Edge Compute Unit | Houses the Radeon GPU, AI co-processors, and camera interface boards. Runs RoboCon AI Stack v1.5 with hybrid YOLO + DeepSeek inference pipeline optimized for construction autonomy. |

### Integrated System Features

- **AI Core**: Hybrid YOLO Vision + DeepSeek R1 Inference Node optimized for Radeon architecture
- **Compute Environment**: Ubuntu 22.04 / ROS 2 Humble / PyTorch 2.2 / OpenVINO toolkit
- **Data Interfaces**: CAN 2.0B, Ethernet, USB 3.1 Gen 1 (5 Gbps), and 12 V DC power distribution bus
- **Cooling**: Active dual-fan system integrated into loader chassis with aluminum heatsink module
- **AI Safety Logic**: Autonomous object avoidance and depth-based terrain profiling enabled by dual HINSON SE-1035 and dual Luxonis OAK-D-S2 stereo vision modules

All sensor nodes are calibrated during final assembly at ROBOCON INC in Oakland, CA, ensuring seamless integration into the RoboCon Network for autonomous site operation.

### Optional Attachments

- Forklift Fork Attachment
- 4-in-1 Bucket
- Quick-Change Mount Adapter

### Final Assembly & Quality Control

**Primary Manufacturer:** Ant Cloud Intelligent Equipment (Shandong) Co., Ltd.  
📍 99 Tongji Road, High-Tech Zone, Jining City, Shandong Province, China

**Final Assembly & System Integration:**  
ROBOCON INC  
📍 Oakland, California, USA

**Includes:** Electrical QA, ROS 2 system integration, and firmware flashing.

**Onboard Charger Port:** Upgraded to NEMA SS2-50P standard for U.S. compliance.

**Firmware:** RoboCon Network integration, tested with hybrid YOLO + DeepSeek inference node.

### Charging Inlet Reference

**Referenced Component:** NEMA SS2-50P Power Inlet  
**Manufacturer:** Guangzhou Bosslyn Electric Co., Ltd.  
**Electrical Rating:** 50A 125/250V  
**Material:** PVC housing, nickel-plated brass terminals.

## Overview

The RoboCon Front Loader Tracked 300kg features:
- **Compact Design**: Optimized for small-scale operations
- **Material Handling**: Capable of lifting and transporting materials
- **Maneuverability**: Efficient movement in confined spaces
- **Standard ROBOCON Components**: Uses common hardware drivers and sensors

## Hardware Drivers

The Mini Loader uses a subset of the ROBOCON hardware ecosystem, typically including:

### Motors and Actuators

#### Golden Motor EZA48400
- **Package**: `golden_motor_eza48400`
- **Type**: Wheel motors for left/right drive
- **CAN Bus**: 500kbps, Node IDs: 0xEF (left), 0xF0 (right)
- **Application**: Base movement and mobility
- **ROS 2 Topics**:
  - `/base/robot_base_cmd_pub_` (Float32MultiArray) - Command
  - `/base/wheel_motors_feedback` (Float32MultiArray) - Feedback
  - `/left_wheel_golden_motor_status` (GoldenMotorEZA48400Status)
  - `/right_wheel_golden_motor_status` (GoldenMotorEZA48400Status)

#### Motor Driver IDS830ABS
- **Package**: `motor_driver_ids830abs`
- **Type**: CAN-based linear actuator controller
- **Application**: Bucket/lift control, steering
- **ROS 2 Topics**:
  - `/motor_driver_ids830abs/command` (Float32MultiArray)
  - `/ids830abs_status` (IDS830ABSStatus)

### Base Controller

#### Base Ackermann Controller
- **Package**: `base_ackermann_controller`
- **Type**: Differential drive base controller
- **Application**: Base movement control
- **ROS 2 Topics**:
  - `/cmd_vel` (geometry_msgs/Twist) - Velocity command
  - `/base/ackermann_controller/reference_unstamped` - Reference velocity

### Sensors

#### Sensor_BW-MINS50 (IMU)
- **Package**: `sensor_bw_mins50`
- **Type**: 9-axis IMU
- **Application**: Orientation and motion sensing
- **ROS 2 Topics**: `/imu/data` (sensor_msgs/Imu)

#### Serial Master SC0
- **Package**: `serial_sc0_master`
- **Type**: Serial communication master
- **Application**: Communication with sensors and peripherals

### Power Management

#### Battery Charger EPC602 4840 EP 01
- **Package**: `battery_charger_epc602_4840_ep_01`
- **Type**: Intelligent battery charger
- **ROS 2 Topics**: `/battery_charger_epc602_status`

#### Pressure Sensor PS-1L-NV
- **Package**: `pressure_sensor_ps_1l_nv`
- **Type**: Pressure sensor
- **Application**: Hydraulic pressure monitoring
- **ROS 2 Topics**: `/ps1lnv_sensor/pressure` (std_msgs/Float32)

## ROS 2 Development Details

### Bringup Package

**Package Name**: `robot_mini_loader_bringup`

The bringup package initializes and coordinates all ROS 2 nodes and hardware drivers required for the RoboCon Front Loader Tracked robot. The robot's boot sequence automatically detects the hardware configuration and launches this bringup package.

### Launching the Robot

```bash
ros2 launch robot_mini_loader_bringup real_robot.launch.py
```

### What the Bringup Activates

The `robot_mini_loader_bringup` package's `real_robot.launch.py` launch file activates the following ROS 2 system components:

#### 1. **Base Mobility System**
- **Package**: `base_ackermann_controller`
- **Node**: Base motion controller
- **Activates**: Tracked base control, velocity command processing
- **ROS 2 Topics**: `/cmd_vel` (geometry_msgs/Twist)

#### 2. **Loader Mechanism Control**
- **Package**: `motor_driver_ids830abs`
- **Nodes**: Bucket lift, dump, and steering actuators
- **Activates**: CAN bus control for loader operations
- **ROS 2 Topics**: `/motor_driver_ids830abs/command`, `/ids830abs_status`

#### 3. **Motor Control System**
- **Package**: `golden_motor_eza48400`
- **Nodes**: Left and right track motor drivers
- **Activates**: CAN bus motor control for tracked mobility

#### 4. **Sensor System**
- **Package**: `sensor_bw_mins50`
- **Node**: IMU sensor node
- **Activates**: Orientation and motion sensing

#### 5. **Power Management**
- **Package**: `battery_charger_epc602_4840_ep_01`
- **Node**: Battery charger monitor
- **Activates**: Battery status monitoring and charging management

### ROS 2 Topic Network

**Command Topics:**
- `/cmd_vel` (geometry_msgs/Twist) - Base velocity commands
- `/motor_driver_ids830abs/command` (std_msgs/Float32MultiArray) - Loader mechanism control

**Feedback Topics:**
- `/imu/data` (sensor_msgs/Imu) - IMU orientation data
- `/base/wheel_motors_feedback` (std_msgs/Float32MultiArray) - Track motor feedback
- `/ids830abs_status` (IDS830ABSStatus) - Actuator status

## Competitive Analysis

For detailed competitive comparisons, see the [RoboCon Front Loader Tracked 300kg Comparison](./front-loader-tracked-300kg-comparison.md) page.

## Sample Code

### C++

#### Controlling Base Movement

```cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class MiniLoaderControl : public rclcpp::Node
{
public:
    MiniLoaderControl() : Node("mini_loader_control")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MiniLoaderControl::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // Example: Move forward at 0.5 m/s
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = 0.5;
        twist.angular.z = 0.0;
        cmd_vel_pub_->publish(twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MiniLoaderControl>());
    rclcpp::shutdown();
    return 0;
}
```

#### Controlling Lift Actuator

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <robot_custom_interfaces/msg/ids830abs_status.hpp>

class LiftControl : public rclcpp::Node
{
public:
    LiftControl() : Node("lift_control")
    {
        lift_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/motor_driver_ids830abs/command", 10);
        
        lift_status_sub_ = this->create_subscription<robot_custom_interfaces::msg::IDS830ABSStatus>(
            "/ids830abs_status", 10,
            std::bind(&LiftControl::lift_status_callback, this, std::placeholders::_1));
    }

private:
    void lift_status_callback(const robot_custom_interfaces::msg::IDS830ABSStatus::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Lift Position: %.2f mm", msg->position_mm);
    }

    void raise_lift(float height_mm)
    {
        auto cmd = std_msgs::msg::Float32MultiArray();
        cmd.data = {height_mm, 0.0f, 0.0f}; // position, velocity, torque
        lift_cmd_pub_->publish(cmd);
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr lift_cmd_pub_;
    rclcpp::Subscription<robot_custom_interfaces::msg::IDS830ABSStatus>::SharedPtr lift_status_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LiftControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### C

#### Basic Movement Control

```c
#include <rclc/rclc.h>
#include <geometry_msgs/msg/twist.h>

int main(int argc, char * argv[])
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_init(argc, argv, "mini_loader_control", &allocator, 
              RCL_DEFAULT_DOMAIN_ID, NULL, false);
    
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "mini_loader_control", "", &allocator));
    
    rcl_publisher_t cmd_vel_pub;
    RCCHECK(rclc_publisher_init_default(
        &cmd_vel_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));
    
    geometry_msgs__msg__Twist msg;
    msg.linear.x = 0.5;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
    
    rcl_publish(&cmd_vel_pub, &msg, NULL);
    
    rcl_publisher_fini(&cmd_vel_pub, &node);
    rcl_node_fini(&node);
    
    return 0;
}
```

### Python 3

#### Controlling Base Movement

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MiniLoaderControl(Node):
    def __init__(self):
        super().__init__('mini_loader_control')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def timer_callback(self):
        # Example: Move forward at 0.5 m/s
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MiniLoaderControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Monitoring Wheel Motors

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from robot_custom_interfaces.msg import GoldenMotorEZA48400Status

class WheelMotorMonitor(Node):
    def __init__(self):
        super().__init__('wheel_motor_monitor')
        
        self.left_motor_sub = self.create_subscription(
            GoldenMotorEZA48400Status,
            '/left_wheel_golden_motor_status',
            self.left_motor_callback,
            10
        )
        
        self.right_motor_sub = self.create_subscription(
            GoldenMotorEZA48400Status,
            '/right_wheel_golden_motor_status',
            self.right_motor_callback,
            10
        )
    
    def left_motor_callback(self, msg):
        self.get_logger().info(
            f'Left Motor - Speed: {msg.speed_rpm} RPM, '
            f'Voltage: {msg.voltage:.2f}V, '
            f'Current: {msg.phase_current:.2f}A'
        )
    
    def right_motor_callback(self, msg):
        self.get_logger().info(
            f'Right Motor - Speed: {msg.speed_rpm} RPM, '
            f'Voltage: {msg.voltage:.2f}V, '
            f'Current: {msg.phase_current:.2f}A'
        )

def main(args=None):
    rclpy.init(args=args)
    node = WheelMotorMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Controlling Lift Mechanism

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from robot_custom_interfaces.msg import IDS830ABSStatus

class LiftControl(Node):
    def __init__(self):
        super().__init__('lift_control')
        self.lift_cmd_pub = self.create_publisher(
            Float32MultiArray,
            '/motor_driver_ids830abs/command',
            10
        )
        self.lift_status_sub = self.create_subscription(
            IDS830ABSStatus,
            '/ids830abs_status',
            self.lift_status_callback,
            10
        )
    
    def lift_status_callback(self, msg):
        self.get_logger().info(f'Lift Position: {msg.position_mm:.2f} mm')
    
    def raise_lift(self, height_mm):
        """Raise lift to specified height in millimeters"""
        cmd = Float32MultiArray()
        cmd.data = [float(height_mm), 0.0, 0.0]  # position, velocity, torque
        self.lift_cmd_pub.publish(cmd)
        self.get_logger().info(f'Commanding lift to {height_mm} mm')

def main(args=None):
    rclpy.init(args=args)
    node = LiftControl()
    
    # Example: Raise lift to 30cm
    node.raise_lift(300.0)
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Monitoring Pressure Sensor

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class PressureMonitor(Node):
    def __init__(self):
        super().__init__('pressure_monitor')
        self.pressure_sub = self.create_subscription(
            Float32,
            '/ps1lnv_sensor/pressure',
            self.pressure_callback,
            10
        )
    
    def pressure_callback(self, msg):
        self.get_logger().info(f'Hydraulic Pressure: {msg.data:.2f} units')

def main(args=None):
    rclpy.init(args=args)
    node = PressureMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Patents

For comprehensive patent analysis related to skid-steer loaders and compact track loaders, including detailed comparisons with the RoboCon Front Loader Tracked 300kg, see the [Patents Analysis](./front-loader-tracked-300kg-patents.md) page.

## Next Steps

- [API Reference](../api-reference/motor-control.md) - Detailed motor control APIs
- [ROS 2 Integration](../ros2/nodes-and-topics.md) - ROS 2 topics and nodes
- [Deployment](../deployment/runtime-configuration.md) - Runtime configuration
- [Competitive Comparison](./front-loader-tracked-300kg-comparison.md) - Compare with competing products

