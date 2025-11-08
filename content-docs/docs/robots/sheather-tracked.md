# RoboCon Sheather Tracked

The RoboCon Sheather Tracked is a tracked sheathing robot variant designed for construction sheathing operations. Available in 50kg and 60kg payload variants.

## Model Specifications

- **Year**: 2025
- **Make**: RoboCon
- **Model**: Sheather Tracked (50kg or 60kg)
- **Payload Capacity**: 50kg or 60kg (depending on variant)
- **Base Configuration**: Tracked mobility system
- **Lift Platform**: Vertical lift system with articulated arm
- **Articulated Arm**: BORUNTE BRTIRUS2550A 6-DOF robotic arm
- **Runtime**: 8+ hours per battery pack

## Overview

The RoboCon Sheather Tracked features:
- **Sheathing Operations**: Specialized for construction sheathing tasks
- **Precise Control**: Fine-grained control for material placement
- **Mobile Base**: Tracked base for navigation and positioning
- **Lift Platform**: Vertical lift system for extended reach capability
- **Articulated Arm**: BORUNTE BRTIRUS2550A 6-DOF robotic arm mounted on lift platform
- **Standard ROBOCON Components**: Uses common hardware drivers and sensors

**Reference**: [BORUNTE BRTIRUS2550A Product Page](https://www.borunte.top/products/1-19bb73a057ca41a69d648843772f60ca)

## Hardware Drivers

The RoboCon Sheather Tracked uses ROBOCON hardware components optimized for sheathing tasks:

### Lift Platform

#### Lift Extension System
- **Type**: Vertical lift mechanism
- **Application**: Raises and lowers the arm platform for extended reach
- **ROS 2 Topics**:
  - `/lift/extend_cmd` (std_msgs/Float64) - Extension command in meters
  - `/lift/position_status` (std_msgs/Float64) - Current lift position

### Articulated Arm

#### BORUNTE BRTIRUS2550A Robotic Arm
- **Package**: `arm_borunte_brtirus2550a_driver`
- **Type**: 6-DOF articulated robotic arm
- **Mounting**: Mounted on top of the lift platform
- **Application**: Precise manipulation for sheathing operations
- **ROS 2 Integration**: 
  - Uses `arm_borunte_brtirus2550a_controller` for control
  - Joint trajectory control via `/arm_brtirus2550a/joint_trajectory`
  - Joint states published on `/joint_states`
- **Communication**: Serial communication (libserial)
- **Reference**: [BORUNTE BRTIRUS2550A Product Page](https://www.borunte.top/products/1-19bb73a057ca41a69d648843772f60ca)

**Joint Configuration:**
- **Joint 1**: Base rotation (shoulder rotation) - ±180°
- **Joint 2**: Shoulder pitch - ±90°
- **Joint 3**: Elbow pitch - 0° to 180°
- **Joint 4**: Wrist roll - ±180°
- **Joint 5**: Wrist pitch - ±90°
- **Joint 6**: End effector rotation - ±180° or continuous

### Motors and Actuators

#### Golden Motor EZA48400
- **Package**: `golden_motor_eza48400`
- **Type**: Wheel motors for base movement
- **CAN Bus**: 500kbps, Node IDs: 0xEF (left), 0xF0 (right)
- **Application**: Base movement and positioning
- **ROS 2 Topics**:
  - `/base/robot_base_cmd_pub_` (Float32MultiArray)
  - `/base/wheel_motors_feedback` (Float32MultiArray)
  - `/left_wheel_golden_motor_status` (GoldenMotorEZA48400Status)
  - `/right_wheel_golden_motor_status` (GoldenMotorEZA48400Status)

#### Motor Driver IDS830ABS
- **Package**: `motor_driver_ids830abs`
- **Type**: CAN-based linear actuator controller
- **Application**: Sheathing tool control, material handling
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
- **Application**: Orientation and motion sensing for precise positioning
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

**Package Name**: `Robot_oSheather_bringup`

The bringup package initializes and coordinates all ROS 2 nodes and hardware drivers required for the RoboCon Sheather Tracked robot. The robot's boot sequence automatically detects the hardware configuration and launches this bringup package.

### Launching the Robot

```bash
ros2 launch Robot_oSheather_bringup real_robot.launch.py
```

### What the Bringup Activates

The `Robot_oSheather_bringup` package's `real_robot.launch.py` launch file activates the following ROS 2 system components:

#### 1. **Lift Platform System**
- **Package**: Lift extension controller (robot-specific)
- **Node**: Lift control node
- **Activates**: Vertical lift mechanism control, lift position feedback
- **ROS 2 Topics**: `/lift/extend_cmd`, `/lift/position_status`

#### 2. **Articulated Arm System**
- **Package**: `arm_borunte_brtirus2550a_controller`
- **Node**: BRTIRUS2550A arm controller
- **Activates**: 6-DOF arm joint control, trajectory execution, joint state feedback
- **ROS 2 Topics**: `/arm_brtirus2550a/joint_trajectory`, `/joint_states`

#### 3. **Base Mobility System**
- **Package**: `base_ackermann_controller`
- **Node**: Base motion controller
- **Activates**: Tracked base control, velocity command processing
- **ROS 2 Topics**: `/cmd_vel` (geometry_msgs/Twist)

#### 4. **Motor Control System**
- **Package**: `golden_motor_eza48400`
- **Nodes**: Left and right track motor drivers
- **Activates**: CAN bus motor control for tracked mobility
- **Package**: `motor_driver_ids830abs`
- **Node**: Sheathing tool actuator controller
- **Activates**: Tool mechanism control via CAN bus

#### 5. **Sensor System**
- **Package**: `sensor_bw_mins50`
- **Node**: IMU sensor node
- **Activates**: Orientation and motion sensing for precise positioning
- **Package**: `serial_sc0_master`
- **Node**: Serial communication master
- **Activates**: Communication with sensors and peripherals

#### 6. **Power Management**
- **Package**: `battery_charger_epc602_4840_ep_01`
- **Node**: Battery charger monitor
- **Activates**: Battery status monitoring and charging management

### ROS 2 Topic Network

**Command Topics:**
- `/cmd_vel` (geometry_msgs/Twist) - Base velocity commands
- `/arm_brtirus2550a/joint_trajectory` (trajectory_msgs/JointTrajectory) - Arm motion commands
- `/lift/extend_cmd` (std_msgs/Float64) - Lift extension commands
- `/motor_driver_ids830abs/command` (std_msgs/Float32MultiArray) - Tool control

**Feedback Topics:**
- `/joint_states` (sensor_msgs/JointState) - Arm and lift joint states
- `/lift/position_status` (std_msgs/Float64) - Current lift height
- `/imu/data` (sensor_msgs/Imu) - IMU orientation data
- `/base/wheel_motors_feedback` (std_msgs/Float32MultiArray) - Track motor feedback

## Sample Code

### Python 3

#### Sheathing Operation Control

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from robot_custom_interfaces.msg import IDS830ABSStatus

class SheatherControl(Node):
    def __init__(self):
        super().__init__('sheather_control')
        
        # Base control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Sheathing tool control
        self.tool_cmd_pub = self.create_publisher(
            Float32MultiArray,
            '/motor_driver_ids830abs/command',
            10
        )
        
        # Tool status monitoring
        self.tool_status_sub = self.create_subscription(
            IDS830ABSStatus,
            '/ids830abs_status',
            self.tool_status_callback,
            10
        )
    
    def tool_status_callback(self, msg):
        self.get_logger().info(f'Tool Position: {msg.position_mm:.2f} mm')
    
    def position_base(self, linear_x, angular_z):
        """Position the base"""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)
    
    def control_sheathing_tool(self, position_mm):
        """Control the sheathing tool"""
        cmd = Float32MultiArray()
        cmd.data = [float(position_mm), 0.0, 0.0]
        self.tool_cmd_pub.publish(cmd)
        self.get_logger().info(f'Commanding tool to {position_mm} mm')
    
    def perform_sheathing_operation(self):
        """Perform a complete sheathing operation"""
        self.get_logger().info("Starting sheathing operation...")
        
        # 1. Position base to target location
        self.position_base(0.3, 0.0)
        rclpy.sleep(self, 2.0)
        self.position_base(0.0, 0.0)
        
        # 2. Extend tool to apply sheathing
        self.control_sheathing_tool(200.0)
        rclpy.sleep(self, 1.5)
        
        # 3. Maintain position for application
        rclpy.sleep(self, 1.0)
        
        # 4. Retract tool
        self.control_sheathing_tool(50.0)
        rclpy.sleep(self, 1.5)
        
        # 5. Move to next position
        self.position_base(0.2, 0.0)
        rclpy.sleep(self, 2.0)
        self.position_base(0.0, 0.0)
        
        self.get_logger().info("Sheathing operation complete")

def main(args=None):
    rclpy.init(args=args)
    node = SheatherControl()
    
    # Perform sheathing operation
    node.perform_sheathing_operation()
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Next Steps

- [API Reference](../api-reference/motor-control.md) - Detailed motor control APIs
- [ROS 2 Integration](../ros2/nodes-and-topics.md) - ROS 2 topics and nodes
- [Deployment](../deployment/runtime-configuration.md) - Runtime configuration

