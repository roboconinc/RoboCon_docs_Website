# RoboCon Demolisher Tracked

**Brand/Make:** RoboCon  
**Model Family:** Demolisher  
**Base Option:** Tracked  
**Machine Weight:** 990 kg

The RoboCon Demolisher Tracked (based on ANTS MR110) is a remote control demolition robot designed for industrial buildings, house renovation, and slag removal operations. It features strong power, flexible working arms, and precise operation capabilities, making it ideal for hazardous work environments and demolition tasks.

## Product Specifications

### General Information

| Parameter | Specification |
|-----------|--------------|
| **Product Type** | Remote Control Demolition Robot / Unmanned Mini Excavator |
| **Category** | Compact Demolition Robot for Construction & Hazardous Operations |
| **Brand** | ROBOCON™ (based on ANTS MR110 platform) |
| **Model** | RoboCon Demolisher Tracked |
| **Base Platform** | Ant Cloud Intelligent Equipment (Shandong) Co., Ltd. MR110 |
| **Certification** | CE, ISO, EPA, EURO5, 2006/42/EC, EMC |

### Power System

| Parameter | Specification |
|-----------|--------------|
| **Power** | 15 kW |
| **Engine Brand** | Kubota |
| **Power Options** | Lithium battery and external power supply options |
| **Battery Type** | Lithium-ion battery system |
| **External Power** | Supports external power supply for extended operations |
| **Adaptability** | Optimized for complex environments and hazardous locations |

### Mechanical & Hydraulic Specifications

| Parameter | Specification |
|-----------|--------------|
| **Machine Weight** | 990 kg |
| **Bucket Capacity** | 0.1~0.5 m³ (variable bucket options) |
| **Hydraulic Pump Brand** | Eaton |
| **Hydraulic Valve Brand** | Eaton |
| **Hydraulic Cylinder Brand** | Eaton |
| **Climbing Inclination, Max** | 30° |
| **Rated Speed** | 4.3 km/h (2.7 mph) |
| **Slewing Speed** | 10 seconds/360° |

### Dimensions & Weight

| Parameter | Specification |
|-----------|--------------|
| **Machine Weight** | 990 kg |
| **Moving Type** | Crawler Excavator (Tracked) |
| **Stability** | Low center of gravity design for enhanced stability |

### Performance Specifications

| Parameter | Specification |
|-----------|--------------|
| **Arm Structure** | Three-stage mechanical arm structure |
| **Power Characteristics** | Strong power output for demolition operations |
| **Flexibility** | Flexible working arms for precise operation |
| **Precision** | High-precision control for demolition tasks |

### Control & Interface

| Component | Specification |
|-----------|--------------|
| **Control System** | Remote Control System |
| **Signal Code** | 1.4G frequency |
| **Remote Monitoring** | Remote monitoring system (unique selling point) |
| **Control Interface** | CAN BUS (Open, supports ROS 2 integration) |
| **Interface Protocol** | OpenCAN for RoboCon Network |

### Key Features

- **Three-Stage Mechanical Arm**: Enhanced reach and flexibility for demolition operations
- **Low Center of Gravity**: Designed for stability during heavy-duty operations
- **Versatile Attachments**: Supports various accessories including:
  - Hydraulic breakers
  - Grabbing devices
  - Custom demolition tools
- **Dual Power Options**: Lithium battery and external power supply for flexibility
- **Remote Monitoring**: Advanced remote monitoring system for operational oversight
- **Hazardous Environment Capability**: Designed for work in dangerous locations
- **Certified Safety**: CE, ISO, EPA, EURO5, 2006/42/EC, EMC certified

### Applications

- **Industrial Building Demolition**: Demolition of industrial structures
- **House Renovation**: Controlled demolition for renovation projects
- **Slag Removal**: Material handling and removal operations
- **Hazardous Work Environments**: Operations in dangerous or inaccessible locations
- **Cave Mining**: Underground mining and excavation support
- **Blasting Operations**: Support for controlled blasting and demolition

### Noise & Environmental

| Parameter | Specification |
|-----------|--------------|
| **Sound Power Level** | 65 dB(a) |
| **Emissions** | EURO5 compliant (when equipped with appropriate engine) |
| **Environmental Impact** | Zero-emissions operation with electric power option |

## Hardware Drivers

The RoboCon Demolisher Tracked uses ROBOCON hardware components optimized for demolition tasks:

### Motors and Actuators

#### Motor Driver IDS830ABS
- **Package**: `motor_driver_ids830abs`
- **Type**: CAN-based linear actuator controller
- **Application**: Boom, stick, and bucket control for demolition operations
- **Hydraulic Control**: Manages Eaton hydraulic system components
- **ROS 2 Topics**:
  - `/motor_driver_ids830abs/command` (Float32MultiArray)
  - `/ids830abs_status` (IDS830ABSStatus)

#### Golden Motor EZA48400 (Track Drive)
- **Package**: `golden_motor_eza48400`
- **Type**: CAN-based track drive motors
- **CAN Bus**: 500kbps, Node IDs: 0xEF (left), 0xF0 (right)
- **Application**: Track/base movement for positioning
- **ROS 2 Topics**:
  - `/base/robot_base_cmd_pub_` (Float32MultiArray)
  - `/base/wheel_motors_feedback` (Float32MultiArray)

### Base Controller

#### Base Ackermann Controller
- **Package**: `base_ackermann_controller`
- **Type**: Differential drive base controller
- **Application**: Base movement control
- **Speed Control**: Up to 4.3 km/h rated speed
- **ROS 2 Topics**:
  - `/cmd_vel` (geometry_msgs/Twist)

### Sensors

#### Sensor_BW-MINS50 (IMU)
- **Package**: `sensor_bw_mins50`
- **Type**: 9-axis IMU
- **Application**: Orientation sensing for stability monitoring during demolition
- **ROS 2 Topics**: `/imu/data` (sensor_msgs/Imu)

#### Pressure Sensor PS-1L-NV
- **Package**: `pressure_sensor_ps_1l_nv`
- **Type**: Pressure sensor
- **Application**: Hydraulic pressure monitoring for Eaton system
- **ROS 2 Topics**: `/ps1lnv_sensor/pressure` (std_msgs/Float32)

### Power Management

#### Battery Charger System
- **Type**: Lithium battery charging system
- **Application**: Battery management and charging
- **External Power**: Supports external power supply connection
- **ROS 2 Topics**: Battery status and power management topics

### Remote Control System

#### 1.4G Remote Control
- **Signal Code**: 1.4G frequency
- **Type**: Industrial remote control system
- **Application**: Remote operation and teleoperation
- **Features**: Remote monitoring system integration
- **Range**: Extended range for hazardous environment operations

### Attachment Interface

#### Quick-Change Attachment System
- **Type**: Hydraulic quick-change system
- **Supported Attachments**:
  - Hydraulic breakers
  - Grabbing devices
  - Custom demolition tools
  - Various bucket sizes (0.1~0.5 m³)
- **ROS 2 Integration**: Attachment control via CAN BUS

## Component Manufacturers

Based on product specifications and Alibaba supplier information:

| Component | Manufacturer/Supplier | Specification |
|-----------|----------------------|---------------|
| **Base Platform** | Ant Cloud Intelligent Equipment (Shandong) Co., Ltd. | MR110 remote control demolition robot platform |
| **Hydraulic Pump** | Eaton | Industrial hydraulic pump system |
| **Hydraulic Valve** | Eaton | Industrial hydraulic valve system |
| **Hydraulic Cylinder** | Eaton | Industrial hydraulic cylinder system |
| **Engine** | Kubota | 15 kW engine system |
| **Battery System** | Lithium-ion battery manufacturer | Lithium battery and external power supply options |
| **Remote Control System** | Industrial Remote Control Manufacturer | 1.4G signal code, remote monitoring |
| **Control System Integration** | ROBOCON INC | ROS 2 integration, CAN BUS, system integration |
| **Final Assembly** | ROBOCON INC | Electrical QA, ROS 2 system integration, firmware flashing |

**Supplier Information:**
- **Primary Manufacturer**: Ant Cloud Intelligent Equipment (Shandong) Co., Ltd.
- **Location**: Jining, Shandong, China
- **Factory Founded**: 2019
- **Patents Awarded**: 2
- **Collaborating Factories**: 58
- **Main Markets**: United States (35%), France (23%), Germany (13%), Australia (5%), Portugal (5%), Other (19%)
- **Final Assembly**: ROBOCON INC, Oakland, California, USA

**Certifications:**
- CE (Conformité Européenne)
- ISO standards compliance
- EPA (Environmental Protection Agency)
- EURO5 emissions standard
- 2006/42/EC (European Machinery Directive)
- EMC (Electromagnetic Compatibility)

**Warranty:**
- **Overall Warranty**: 1 Year
- **Core Components Warranty**: 3 years
- **Service**: Free replacement parts available

## ROS 2 Development Details

### Bringup Package

**Package Name**: `robot_demolisher_bringup`

The bringup package initializes and coordinates all ROS 2 nodes and hardware drivers required for the RoboCon Demolisher Tracked robot. The robot's boot sequence automatically detects the hardware configuration and launches this bringup package.

### Launching the Robot

```bash
ros2 launch robot_demolisher_bringup real_robot.launch.py
```

### What the Bringup Activates

The `robot_demolisher_bringup` package's `real_robot.launch.py` launch file activates the following ROS 2 system components:

#### 1. **Demolition Arm System**
- **Package**: `motor_driver_ids830abs`
- **Nodes**: Boom, stick, and bucket actuators
- **Activates**: CAN bus control for demolition arm operations
- **Hydraulic Control**: Manages Eaton hydraulic system (pump, valve, cylinders)
- **ROS 2 Topics**: `/motor_driver_ids830abs/command`, `/ids830abs_status`

#### 2. **Base Mobility System**
- **Package**: `base_ackermann_controller`
- **Node**: Base motion controller
- **Activates**: Tracked base control, velocity command processing
- **Speed Control**: Up to 4.3 km/h rated speed
- **Climbing Capability**: Up to 30° inclination
- **ROS 2 Topics**: `/cmd_vel` (geometry_msgs/Twist)

#### 3. **Motor Control System**
- **Package**: `golden_motor_eza48400`
- **Nodes**: Left and right track motor drivers
- **Activates**: CAN bus motor control for tracked mobility
- **Motor**: Track drive motors for crawler excavator base

#### 4. **Sensor System**
- **Package**: `sensor_bw_mins50`
- **Node**: IMU sensor node
- **Activates**: Orientation and motion sensing for stability monitoring during demolition operations
- **ROS 2 Topics**: `/imu/data` (sensor_msgs/Imu)

#### 5. **Pressure Monitoring**
- **Package**: `pressure_sensor_ps_1l_nv`
- **Node**: Hydraulic pressure sensor
- **Activates**: Real-time hydraulic pressure monitoring for Eaton system
- **ROS 2 Topics**: `/ps1lnv_sensor/pressure` (std_msgs/Float32)

#### 6. **Power Management**
- **Package**: Battery management system
- **Node**: Battery charger monitor
- **Activates**: Battery status monitoring and charging management
- **Features**: External power supply support

#### 7. **Remote Control System**
- **Package**: Remote control interface
- **Node**: Remote control receiver (1.4G signal)
- **Activates**: Remote operation capability for hazardous environment operations
- **Features**: Remote monitoring system integration

#### 8. **Attachment Control**
- **Package**: Quick-change attachment controller
- **Node**: Attachment interface manager
- **Activates**: Control for hydraulic breakers, grabbing devices, and custom attachments
- **Features**: Quick-change system for various demolition tools

### ROS 2 Topic Network

**Command Topics:**
- `/cmd_vel` (geometry_msgs/Twist) - Base velocity commands
- `/motor_driver_ids830abs/command` (std_msgs/Float32MultiArray) - Demolition arm control (boom, stick, bucket)
- `/attachment/command` (varies by attachment type) - Attachment control commands

**Feedback Topics:**
- `/imu/data` (sensor_msgs/Imu) - IMU orientation data for stability monitoring
- `/base/wheel_motors_feedback` (std_msgs/Float32MultiArray) - Track motor feedback
- `/ids830abs_status` (IDS830ABSStatus) - Actuator status (boom, stick, bucket positions)
- `/ps1lnv_sensor/pressure` (std_msgs/Float32) - Hydraulic pressure feedback
- `/battery/status` (varies) - Battery status and power management
- `/remote_control/status` (varies) - Remote control system status

## Sample Code

### C++ Example

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

class DemolisherController : public rclcpp::Node {
public:
    DemolisherController() : Node("demolisher_controller") {
        // Publishers
        arm_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/motor_driver_ids830abs/command", 10);
        base_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        
        // Timer for control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DemolisherController::control_loop, this));
    }

private:
    void control_loop() {
        // Control demolition arm
        auto arm_cmd = std_msgs::msg::Float32MultiArray();
        arm_cmd.data = {300.0f, 200.0f, 150.0f}; // Boom, stick, bucket positions
        arm_pub_->publish(arm_cmd);
        
        // Control base movement
        auto base_cmd = geometry_msgs::msg::Twist();
        base_cmd.linear.x = 0.5; // Forward movement (m/s)
        base_pub_->publish(base_cmd);
    }
    
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr arm_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DemolisherController>());
    rclcpp::shutdown();
    return 0;
}
```

### Python 3 Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class DemolisherController(Node):
    def __init__(self):
        super().__init__('demolisher_controller')
        
        # Publishers
        self.arm_pub = self.create_publisher(
            Float32MultiArray, 
            '/motor_driver_ids830abs/command', 
            10)
        self.base_pub = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10)
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
    
    def control_loop(self):
        # Control demolition arm
        arm_cmd = Float32MultiArray()
        arm_cmd.data = [300.0, 200.0, 150.0]  # Boom, stick, bucket positions
        self.arm_pub.publish(arm_cmd)
        
        # Control base movement
        base_cmd = Twist()
        base_cmd.linear.x = 0.5  # Forward movement (m/s)
        self.base_pub.publish(base_cmd)

def main(args=None):
    rclpy.init(args=args)
    controller = DemolisherController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Safety Considerations

- **Hazardous Environment Operation**: Designed for remote operation in dangerous locations
- **Stability Monitoring**: IMU sensors provide real-time stability feedback
- **Pressure Monitoring**: Hydraulic pressure sensors ensure safe operation limits
- **Remote Monitoring**: Advanced remote monitoring system for operational oversight
- **Certified Safety Standards**: CE, ISO, EPA, EURO5, 2006/42/EC, EMC certified
- **Low Center of Gravity**: Enhanced stability design for heavy-duty operations

## Overview

The RoboCon Demolisher Tracked features:
- **Demolition Capability**: Three-stage mechanical arm structure for strong power and flexible operation
- **Precise Control**: Fine-grained control for demolition tasks with remote operation
- **Versatile Attachments**: Supports hydraulic breakers, grabbing devices, and custom tools
- **Hazardous Environment Ready**: Designed for work in dangerous or inaccessible locations
- **Stability Design**: Low center of gravity for enhanced stability during operations
- **Standard ROBOCON Components**: Uses common hardware drivers and sensors
- **Remote Monitoring**: Advanced remote monitoring system for operational oversight

## See Also

- [Robot Models Overview](/docs/robots/robot-models)
- [Front Loader Tracked 300kg](/docs/robots/front-loader-tracked-300kg)
- [Excavator Tracked](/docs/robots/excavator-tracked)
- [Behavior Tree Low-Level Nodes](/docs/api-reference/behavior-tree-low-level-nodes)

