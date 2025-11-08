# RoboCon Transporter Tracked 1000kg

The RoboCon Transporter Tracked 1000kg is a tracked material transport robot designed for moving materials and equipment efficiently.

## Overview

The RoboCon Transporter - Tracked features:
- **Material Transport**: Designed for carrying loads
- **Maneuverability**: Efficient movement for transport tasks
- **Load Handling**: Capable of carrying various materials
- **Standard ROBOCON Components**: Uses common hardware drivers and sensors
- **Tracked Base**: Tracked undercarriage for stability and terrain capability

## Hardware Drivers

The RoboCon Transporter - Tracked uses ROBOCON hardware components optimized for transport tasks:

### Motors and Actuators

#### Golden Motor EZA48400
- **Package**: `golden_motor_eza48400`
- **Type**: Wheel motors for base movement
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
- **Application**: Load handling mechanisms, steering
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

## ROS 2 Development Details

### Bringup Package

**Package Name**: `Robot_Transporter_bringup`

The bringup package initializes and coordinates all ROS 2 nodes and hardware drivers required for the RoboCon Transporter Tracked robot. The robot's boot sequence automatically detects the hardware configuration and launches this bringup package.

### Launching the Robot

```bash
ros2 launch Robot_Transporter_bringup real_robot.launch.py
```

### What the Bringup Activates

The `Robot_Transporter_bringup` package's `real_robot.launch.py` launch file activates the following ROS 2 system components:

#### 1. **Base Mobility System**
- **Package**: `base_ackermann_controller`
- **Node**: Base motion controller
- **Activates**: Tracked base control, velocity command processing
- **ROS 2 Topics**: `/cmd_vel` (geometry_msgs/Twist)

#### 2. **Motor Control System**
- **Package**: `golden_motor_eza48400`
- **Nodes**: Left and right track motor drivers
- **Activates**: CAN bus motor control for tracked mobility, motor feedback
- **Package**: `motor_driver_ids830abs`
- **Node**: Load handling mechanism controller
- **Activates**: Actuator control for material handling via CAN bus

#### 3. **Sensor System**
- **Package**: `sensor_bw_mins50`
- **Node**: IMU sensor node
- **Activates**: Orientation and motion sensing
- **Package**: `serial_sc0_master`
- **Node**: Serial communication master
- **Activates**: Communication with sensors and peripherals

#### 4. **Power Management**
- **Package**: `battery_charger_epc602_4840_ep_01`
- **Node**: Battery charger monitor
- **Activates**: Battery status monitoring and charging management

### ROS 2 Topic Network

**Command Topics:**
- `/cmd_vel` (geometry_msgs/Twist) - Base velocity commands
- `/motor_driver_ids830abs/command` (std_msgs/Float32MultiArray) - Load handling control

**Feedback Topics:**
- `/imu/data` (sensor_msgs/Imu) - IMU orientation data
- `/base/wheel_motors_feedback` (std_msgs/Float32MultiArray) - Track motor feedback
- `/battery_charger_epc602_status` - Battery status

## Sample Code

### C++

#### Basic Transport Control

```cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class TransporterControl : public rclcpp::Node
{
public:
    TransporterControl() : Node("transporter_control")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        
        load_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/motor_driver_ids830abs/command", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TransporterControl::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // Move forward
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = 0.5;
        twist.angular.z = 0.0;
        cmd_vel_pub_->publish(twist);
    }

    void control_load_mechanism(float position_mm)
    {
        auto cmd = std_msgs::msg::Float32MultiArray();
        cmd.data = {position_mm, 0.0f, 0.0f};
        load_cmd_pub_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr load_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransporterControl>());
    rclcpp::shutdown();
    return 0;
}
```

### C

#### Movement Control

```c
#include <rclc/rclc.h>
#include <geometry_msgs/msg/twist.h>

int main(int argc, char * argv[])
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_init(argc, argv, "transporter_control", &allocator, 
              RCL_DEFAULT_DOMAIN_ID, NULL, false);
    
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "transporter_control", "", &allocator));
    
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

#### Basic Transport Control

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class TransporterControl(Node):
    def __init__(self):
        super().__init__('transporter_control')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.load_cmd_pub = self.create_publisher(
            Float32MultiArray, '/motor_driver_ids830abs/command', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def timer_callback(self):
        # Move forward at 0.5 m/s
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
    
    def control_load_mechanism(self, position_mm):
        """Control load handling mechanism"""
        cmd = Float32MultiArray()
        cmd.data = [float(position_mm), 0.0, 0.0]
        self.load_cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TransporterControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Monitoring Battery Status

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_custom_interfaces.msg import BatteryChargerEPC602Status

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self.battery_sub = self.create_subscription(
            BatteryChargerEPC602Status,
            '/battery_charger_epc602_status',
            self.battery_callback,
            10
        )
    
    def battery_callback(self, msg):
        self.get_logger().info(
            f'Battery - Voltage: {msg.voltage:.2f}V, '
            f'Current: {msg.current:.2f}A, '
            f'Charge: {msg.charge_level:.1f}%, '
            f'Charging: {msg.is_charging}'
        )
        
        if msg.charge_level < 20.0:
            self.get_logger().warn('Low battery! Consider charging.')

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Next Steps

- [API Reference](../api-reference/motor-control.md) - Detailed motor control APIs
- [ROS 2 Integration](../ros2/nodes-and-topics.md) - ROS 2 topics and nodes
- [Deployment](../deployment/runtime-configuration.md) - Runtime configuration

