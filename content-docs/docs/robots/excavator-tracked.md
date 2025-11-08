# RoboCon Excavator Tracked

**Brand/Make:** RoboCon  
**Model Family:** Excavator  
**Base Option:** Tracked  
**Estimated Max Payload:** ~150-200kg (based on compact design and specifications)

The RoboCon Excavator Tracked is a compact, autonomous electric excavator designed for digging and earth-moving operations. Built on the ROBOCON ecosystem, it combines compact dimensions with advanced AI capabilities for autonomous excavation tasks.

## Product Specifications

### General Information

| Parameter | Specification |
|-----------|--------------|
| **Product Type** | Autonomous Electric Compact Excavator |
| **Category** | Compact Electric Excavator for Construction & Robotics Integration |
| **Brand** | ROBOCON™ |
| **Model** | RoboCon Mini Excavator |
| **Base Price** | ~$15,000 USD (estimated, varies by configuration) |

### Power System

| Parameter | Specification |
|-----------|--------------|
| **Power System** | 72V Electric Drive |
| **Motor Type** | Permanent Magnet Brushless Motor |
| **Motor Power** | 3.5 kW (7 kW peak power) |
| **Battery Type** | Ternary Lithium Battery (CATL battery cell) |
| **Battery Capacity** | 148 Ah @ 72V (10.656 kWh nominal) |
| **Runtime** | 6-8 hours (working time) |
| **Charging Voltage** | 110-240 VAC (wide voltage support) |
| **Charging Current** | Maximum 50A (dual-line fast charging) |
| **Charging Method** | Dual-line fast charging |
| **Charging Time** | 3-4 hours |
| **Work While Charging** | Yes, supports operation while charging |

### Mechanical & Hydraulic Specifications

| Parameter | Specification |
|-----------|--------------|
| **Hydraulic System Pressure** | 18 MPa |
| **Hydraulic Flow** | 20 L/min |
| **Maximum Digging Depth** | 860 mm |
| **Maximum Digging Height** | 2,520 mm |
| **Maximum Digging Radius** | 2,610 mm |
| **Maximum Loading/Unloading Height** | 1,800 mm |
| **Bucket Capacity** | 0.02 m³ |
| **Ground Clearance** | 90 mm (minimum) / 2,690 mm (maximum) |
| **Engine/Motor Speed** | 3,000 rpm |
| **Track Speed** | 1-13 rpm |
| **Rear Turning Radius** | 340 mm |

### Dimensions & Weight

| Parameter | Specification |
|-----------|--------------|
| **Length × Width × Height** | 2,220 × 660 × 1,040 mm (operating) |
| **Transport Length** | 1,500 mm (folded/compact) |
| **Track Size** | 130 × 72 × 29 mm (alternative: 180 × 86 × 39 mm) |
| **Chain Length** | 93 mm |
| **Machine Width** | 660 mm (alternative: 840 mm mentioned) |
| **Machine Weight** | ~400-500 kg (estimated based on compact design and battery capacity) |

**Note:** Weight estimate based on comparison with Front Loader Tracked 300kg (600 kg machine weight with 10.6 kWh battery) and accounting for compact excavator design with similar 72V system and 148 Ah battery.

### Performance Specifications

| Parameter | Specification |
|-----------|--------------|
| **Maximum Digging Depth** | 860 mm |
| **Maximum Digging Height** | 2,520 mm |
| **Maximum Digging Radius** | 2,610 mm |
| **Bucket Capacity** | 0.02 m³ |
| **Estimated Lift Capacity** | ~150-200 kg (estimated based on compact design, 18 MPa hydraulic system, and 20 L/min flow rate) |

**Payload Estimation Method:**
- Front Loader Tracked 300kg: 300 kg payload with 18 MPa pressure, 40 L/min flow
- Excavator: 18 MPa pressure, 20 L/min flow (50% of loader)
- Compact design and smaller bucket (0.02 m³ vs loader's 0.1 m³) suggests proportionally lower payload
- Estimated payload: 150-200 kg based on hydraulic capacity and compact design

### Control & Interface

| Component | Specification |
|-----------|--------------|
| **Control System** | CAN BUS (Open, supports ROS 2 integration) |
| **Remote Controller** | Industrial dedicated remote control |
| **Operation Delay** | Less than 0.02 seconds (20 milliseconds) |
| **Remote Control Range** | 300-500 meters |
| **Interface Protocol** | OpenCAN for RoboCon Network |
| **Communication** | CAN BUS with ROS 2 integration |

### Key Features

- **Boom Side Swing Function**: Allows side-to-side boom movement for enhanced maneuverability
- **Autonomous Operation**: ROS 2-based autonomous excavation capabilities
- **Work While Charging**: Can operate while connected to charging system
- **Compact Design**: Narrow 660mm width enables operation in confined spaces
- **Foldable Transport**: Reduces to 1,500mm length for transport
- **Industrial Remote Control**: Low-latency remote operation capability
- **Standard ROBOCON Components**: Uses common hardware drivers and sensors

## Overview

The RoboCon Excavator Tracked features:
- **Digging Capability**: Hydraulic boom, stick, and bucket system for precise excavation
- **Precise Control**: Fine-grained control for excavation tasks with autonomous operation
- **Compact Design**: Suitable for small-scale excavation work in confined spaces
- **Standard ROBOCON Components**: Uses common hardware drivers and sensors

## Hardware Drivers

The Mini Excavator uses ROBOCON hardware components optimized for excavation tasks:

### Motors and Actuators

#### Permanent Magnet Brushless Motor
- **Type**: Permanent Magnet Brushless Motor
- **Power**: 3.5 kW continuous / 7 kW peak
- **Voltage**: 72V
- **Speed**: 3,000 rpm
- **Application**: Track drive system, hydraulic pump drive
- **Manufacturer/Component**: Industrial grade brushless motor

#### Motor Driver IDS830ABS
- **Package**: `motor_driver_ids830abs`
- **Type**: CAN-based linear actuator controller
- **Application**: Boom, stick, and bucket control
- **Hydraulic Control**: Manages hydraulic actuators for excavation operations
- **ROS 2 Topics**:
  - `/motor_driver_ids830abs/command` (Float32MultiArray)
  - `/ids830abs_status` (IDS830ABSStatus)

#### Golden Motor EZA48400 (Track Drive)
- **Package**: `golden_motor_eza48400`
- **Type**: CAN-based track drive motors
- **CAN Bus**: 500kbps, Node IDs: 0xEF (left), 0xF0 (right)
- **Application**: Track/base movement
- **Speed Control**: 1-13 rpm track speed
- **ROS 2 Topics**:
  - `/base/robot_base_cmd_pub_` (Float32MultiArray)
  - `/base/wheel_motors_feedback` (Float32MultiArray)

### Base Controller

#### Base Ackermann Controller
- **Package**: `base_ackermann_controller`
- **Type**: Differential drive base controller
- **Application**: Base movement control
- **ROS 2 Topics**:
  - `/cmd_vel` (geometry_msgs/Twist)

### Sensors

#### Sensor_BW-MINS50 (IMU)
- **Package**: `sensor_bw_mins50`
- **Type**: 9-axis IMU
- **Application**: Orientation sensing for stability
- **ROS 2 Topics**: `/imu/data` (sensor_msgs/Imu)

#### Pressure Sensor PS-1L-NV
- **Package**: `pressure_sensor_ps_1l_nv`
- **Type**: Pressure sensor
- **Application**: Hydraulic pressure monitoring
- **ROS 2 Topics**: `/ps1lnv_sensor/pressure` (std_msgs/Float32)

#### Serial Master SC0
- **Package**: `serial_sc0_master`
- **Type**: Serial communication master
- **Application**: Communication with sensors

### Power Management

#### Ternary Lithium Battery System
- **Battery Type**: Ternary Lithium Battery (Li-NMC)
- **Battery Cell**: CATL (Contemporary Amperex Technology Co. Limited) battery cells
- **Capacity**: 148 Ah @ 72V (10.656 kWh nominal)
- **Voltage**: 72V DC system
- **Warranty**: 3-year warranty
- **Application**: Primary power source for electric excavator

#### Dual-Line Fast Charging System
- **Charging Method**: Dual-line fast charging
- **Input Voltage**: 110-240V AC (wide voltage support)
- **Maximum Input Current**: 50A
- **Charging Time**: 3-4 hours
- **Work While Charging**: Yes, supports operation while connected to charger
- **Charging Interface**: Industrial charging connector with power and voltage display

#### Battery Charger EPC602 4840 EP 01
- **Package**: `battery_charger_epc602_4840_ep_01`
- **Type**: Intelligent battery charger
- **ROS 2 Topics**: `/battery_charger_epc602_status`

### Remote Control System

#### Industrial Remote Control
- **Type**: Industrial dedicated remote control
- **Operation Delay**: Less than 0.02 seconds
- **Range**: 300-500 meters
- **Application**: Remote operation and teleoperation
- **Features**: Boom side swing function control, low-latency operation

## Component Manufacturers

Based on product specifications and conversation data:

| Component | Manufacturer/Supplier | Specification |
|-----------|----------------------|---------------|
| **Battery Cells** | CATL (Contemporary Amperex Technology Co. Limited) | Ternary Lithium (Li-NMC), 148 Ah @ 72V, 3-year warranty |
| **Motor** | Industrial Permanent Magnet Brushless Motor | 3.5 kW continuous / 7 kW peak, 72V, 3,000 rpm |
| **Hydraulic System** | Industrial Hydraulic Components | 18 MPa system pressure, 20 L/min flow |
| **Remote Control System** | Industrial Remote Control Manufacturer | Dedicated industrial remote, less than 0.02s delay, 300-500m range |
| **Charging System** | Industrial Charging Equipment | Dual-line fast charging, 50A max, 110-240V, with power/voltage display |
| **Control System Integration** | ROBOCON INC | ROS 2 integration, CAN BUS, system integration |
| **Final Assembly** | ROBOCON INC | Electrical QA, ROS 2 system integration, firmware flashing |

**Supplier Information:**
- Primary discussions with Amy Ants / Amy Lu (likely from Ant Cloud Intelligent Equipment or similar supplier)
- Battery cells sourced from CATL (Contemporary Amperex Technology Co. Limited) - major Chinese battery manufacturer
- Final assembly and integration: ROBOCON INC, Oakland, California, USA

**Note:** Additional component manufacturers and suppliers may be confirmed through further product documentation or supplier verification.

## ROS 2 Development Details

### Bringup Package

**Package Name**: `robot_mini_excavator_bringup`

The bringup package initializes and coordinates all ROS 2 nodes and hardware drivers required for the RoboCon Excavator Tracked robot. The robot's boot sequence automatically detects the hardware configuration and launches this bringup package.

### Launching the Robot

```bash
ros2 launch robot_mini_excavator_bringup real_robot.launch.py
```

### What the Bringup Activates

The `robot_mini_excavator_bringup` package's `real_robot.launch.py` launch file activates the following ROS 2 system components:

#### 1. **Excavator Arm System**
- **Package**: `motor_driver_ids830abs`
- **Nodes**: Boom, stick, and bucket actuators
- **Activates**: CAN bus control for excavator arm operations
- **Hydraulic Control**: Manages 18 MPa hydraulic system with 20 L/min flow
- **ROS 2 Topics**: `/motor_driver_ids830abs/command`, `/ids830abs_status`

#### 2. **Base Mobility System**
- **Package**: `base_ackermann_controller`
- **Node**: Base motion controller
- **Activates**: Tracked base control, velocity command processing
- **Track Speed**: 1-13 rpm speed control
- **ROS 2 Topics**: `/cmd_vel` (geometry_msgs/Twist)

#### 3. **Motor Control System**
- **Package**: `golden_motor_eza48400`
- **Nodes**: Left and right track motor drivers
- **Activates**: CAN bus motor control for tracked mobility
- **Motor**: Permanent magnet brushless motor (3.5 kW / 7 kW peak)

#### 4. **Sensor System**
- **Package**: `sensor_bw_mins50`
- **Node**: IMU sensor node
- **Activates**: Orientation and motion sensing for precision digging and stability monitoring

#### 5. **Power Management**
- **Package**: `battery_charger_epc602_4840_ep_01`
- **Node**: Battery charger monitor
- **Activates**: Battery status monitoring and charging management
- **Charging**: Dual-line fast charging support (up to 50A, 110-240V)

#### 6. **Remote Control System**
- **Package**: Industrial remote control interface
- **Node**: Remote control receiver
- **Activates**: Remote operation capability (300-500m range, less than 0.02s delay)
- **Features**: Industrial dedicated remote control for excavation operations

### ROS 2 Topic Network

**Command Topics:**
- `/cmd_vel` (geometry_msgs/Twist) - Base velocity commands
- `/motor_driver_ids830abs/command` (std_msgs/Float32MultiArray) - Excavator arm control (boom, stick, bucket)

**Feedback Topics:**
- `/imu/data` (sensor_msgs/Imu) - IMU orientation data for stability monitoring
- `/base/wheel_motors_feedback` (std_msgs/Float32MultiArray) - Track motor feedback
- `/ids830abs_status` (IDS830ABSStatus) - Actuator status (boom, stick, bucket positions)

## Sample Code

### C++

#### Coordinated Excavation Control

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <robot_custom_interfaces/msg/ids830abs_status.hpp>
#include <geometry_msgs/msg/twist.hpp>

class ExcavatorControl : public rclcpp::Node
{
public:
    ExcavatorControl() : Node("excavator_control")
    {
        // Boom, arm, and bucket actuators
        boom_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/boom_motor/command", 10);
        arm_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/arm_motor/command", 10);
        bucket_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/bucket_motor/command", 10);
        
        // Base movement
        base_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        
        // Pressure monitoring
        pressure_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/ps1lnv_sensor/pressure", 10,
            std::bind(&ExcavatorControl::pressure_callback, this, std::placeholders::_1));
    }

private:
    void pressure_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Hydraulic Pressure: %.2f", msg->data);
    }

    void move_boom(float position_mm)
    {
        auto cmd = std_msgs::msg::Float32MultiArray();
        cmd.data = {position_mm, 0.0f, 0.0f};
        boom_cmd_pub_->publish(cmd);
    }

    void move_arm(float position_mm)
    {
        auto cmd = std_msgs::msg::Float32MultiArray();
        cmd.data = {position_mm, 0.0f, 0.0f};
        arm_cmd_pub_->publish(cmd);
    }

    void move_bucket(float position_mm)
    {
        auto cmd = std_msgs::msg::Float32MultiArray();
        cmd.data = {position_mm, 0.0f, 0.0f};
        bucket_cmd_pub_->publish(cmd);
    }

    void move_base(float linear_x, float angular_z)
    {
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = linear_x;
        twist.angular.z = angular_z;
        base_cmd_pub_->publish(twist);
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr boom_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr arm_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr bucket_cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pressure_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExcavatorControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### C

#### Basic Actuator Control

```c
#include <rclc/rclc.h>
#include <std_msgs/msg/float32_multi_array.h>

int main(int argc, char * argv[])
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_init(argc, argv, "excavator_control", &allocator, 
              RCL_DEFAULT_DOMAIN_ID, NULL, false);
    
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "excavator_control", "", &allocator));
    
    rcl_publisher_t boom_cmd_pub;
    RCCHECK(rclc_publisher_init_default(
        &boom_cmd_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/boom_motor/command"));
    
    std_msgs__msg__Float32MultiArray msg;
    msg.data.size = 3;
    msg.data.data[0] = 250.0f; // position in mm
    msg.data.data[1] = 0.0f;   // velocity
    msg.data.data[2] = 0.0f;   // torque
    
    rcl_publish(&boom_cmd_pub, &msg, NULL);
    
    rcl_publisher_fini(&boom_cmd_pub, &node);
    rcl_node_fini(&node);
    
    return 0;
}
```

### Python 3

#### Coordinated Excavation Control

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist

class ExcavatorControl(Node):
    def __init__(self):
        super().__init__('excavator_control')
        
        # Boom, arm, and bucket actuators
        self.boom_cmd_pub = self.create_publisher(
            Float32MultiArray, '/boom_motor/command', 10)
        self.arm_cmd_pub = self.create_publisher(
            Float32MultiArray, '/arm_motor/command', 10)
        self.bucket_cmd_pub = self.create_publisher(
            Float32MultiArray, '/bucket_motor/command', 10)
        
        # Base movement
        self.base_cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Pressure monitoring
        self.pressure_sub = self.create_subscription(
            Float32, '/ps1lnv_sensor/pressure',
            self.pressure_callback, 10)
    
    def pressure_callback(self, msg):
        self.get_logger().info(f'Hydraulic Pressure: {msg.data:.2f}')
    
    def move_boom(self, position_mm):
        """Move boom to specified position in millimeters"""
        cmd = Float32MultiArray()
        cmd.data = [float(position_mm), 0.0, 0.0]
        self.boom_cmd_pub.publish(cmd)
    
    def move_arm(self, position_mm):
        """Move arm to specified position in millimeters"""
        cmd = Float32MultiArray()
        cmd.data = [float(position_mm), 0.0, 0.0]
        self.arm_cmd_pub.publish(cmd)
    
    def move_bucket(self, position_mm):
        """Move bucket to specified position in millimeters"""
        cmd = Float32MultiArray()
        cmd.data = [float(position_mm), 0.0, 0.0]
        self.bucket_cmd_pub.publish(cmd)
    
    def move_base(self, linear_x, angular_z):
        """Move base with specified linear and angular velocities"""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.base_cmd_pub.publish(twist)
    
    def perform_dig_cycle(self):
        """Perform a complete digging cycle"""
        self.get_logger().info("Starting dig cycle...")
        
        # 1. Position for digging
        self.move_boom(300.0)
        self.move_arm(200.0)
        self.move_bucket(150.0)
        rclpy.sleep(self, 2.0)
        
        # 2. Lower bucket
        self.move_bucket(100.0)
        rclpy.sleep(self, 1.0)
        
        # 3. Retract arm to scoop
        self.move_arm(250.0)
        rclpy.sleep(self, 2.0)
        
        # 4. Raise bucket
        self.move_bucket(200.0)
        rclpy.sleep(self, 1.0)
        
        # 5. Lift boom
        self.move_boom(400.0)
        rclpy.sleep(self, 2.0)
        
        self.get_logger().info("Dig cycle complete")

def main(args=None):
    rclpy.init(args=args)
    node = ExcavatorControl()
    
    # Example: Perform a dig cycle
    node.perform_dig_cycle()
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Monitoring Hydraulic Pressure

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
        self.max_pressure = 1000.0  # Maximum safe pressure
    
    def pressure_callback(self, msg):
        pressure = msg.data
        self.get_logger().info(f'Hydraulic Pressure: {pressure:.2f} units')
        
        if pressure > self.max_pressure:
            self.get_logger().warn(
                f'Warning: Pressure exceeds safe limit! ({pressure:.2f} > {self.max_pressure})'
            )

def main(args=None):
    rclpy.init(args=args)
    node = PressureMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Reading IMU for Stability

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class StabilityMonitor(Node):
    def __init__(self):
        super().__init__('stability_monitor')
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        self.max_tilt = 15.0  # Maximum safe tilt in degrees
    
    def quaternion_to_euler(self, q):
        """Convert quaternion to roll, pitch, yaw in degrees"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
    
    def imu_callback(self, msg):
        roll, pitch, yaw = self.quaternion_to_euler(msg.orientation)
        
        self.get_logger().info(
            f'Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°'
        )
        
        if abs(roll) > self.max_tilt or abs(pitch) > self.max_tilt:
            self.get_logger().warn(
                f'Warning: Excessive tilt detected! '
                f'(Roll: {roll:.2f}°, Pitch: {pitch:.2f}°)'
            )

def main(args=None):
    rclpy.init(args=args)
    node = StabilityMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Patents

For comprehensive patent analysis related to excavators, autonomous construction equipment, and competitive patent landscapes, including detailed comparisons with the RoboCon Excavator Tracked, see the [Patents Analysis](./excavator-tracked-patents.md) page.

## Competitive Analysis

For detailed competitive comparisons with other excavator products, see the [RoboCon Excavator Tracked Comparison](./excavator-tracked-comparison.md) page.

## Next Steps

- [API Reference](../api-reference/motor-control.md) - Detailed motor control APIs
- [ROS 2 Integration](../ros2/nodes-and-topics.md) - ROS 2 topics and nodes
- [Deployment](../deployment/runtime-configuration.md) - Runtime configuration
- [Competitive Comparison](./excavator-tracked-comparison.md) - Compare with competing products
- [Patents Analysis](./excavator-tracked-patents.md) - Detailed patent analysis and conflict assessment

