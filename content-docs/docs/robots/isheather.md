# RoboCon Sheather Wheeled (Robot_iSheather_bringup)

The RoboCon Sheather Wheeled is an intelligent sheathing robot variant with enhanced AI capabilities for autonomous construction sheathing operations. It uses wheeled base mobility instead of tracks.

## Overview

The RoboCon Sheather Wheeled features:
- **Intelligent Sheathing**: AI-enhanced sheathing operations with autonomous decision-making
- **Precise Control**: Fine-grained control with sensor feedback
- **Mobile Base**: Wheeled base for intelligent navigation and optimal positioning
- **Lift Platform**: Vertical lift system for extended reach capability
- **Articulated Arm**: BORUNTE BRTIRUS2550A 6-DOF robotic arm mounted on lift platform
- **Advanced Sensors**: Comprehensive sensor suite for environment perception
- **AI Integration**: Integration with DeepSeek LLM for intelligent planning and execution

**Reference**: [BORUNTE BRTIRUS2550A Product Page](https://www.borunte.top/products/1-19bb73a057ca41a69d648843772f60ca)

## Hardware Drivers

The RoboCon Sheather Wheeled uses similar hardware to RoboCon Sheather Tracked with additional intelligent features:

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
- **Application**: Precise manipulation for intelligent sheathing operations
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
- **Application**: Base movement and intelligent positioning
- **ROS 2 Topics**:
  - `/base/robot_base_cmd_pub_` (Float32MultiArray)
  - `/base/wheel_motors_feedback` (Float32MultiArray)
  - `/left_wheel_golden_motor_status` (GoldenMotorEZA48400Status)
  - `/right_wheel_golden_motor_status` (GoldenMotorEZA48400Status)

#### Motor Driver IDS830ABS
- **Package**: `motor_driver_ids830abs`
- **Type**: CAN-based linear actuator controller
- **Application**: Sheathing tool control with intelligent feedback
- **ROS 2 Topics**:
  - `/motor_driver_ids830abs/command` (Float32MultiArray)
  - `/ids830abs_status` (IDS830ABSStatus)

### Base Controller

#### Base Ackermann Controller
- **Package**: `base_ackermann_controller`
- **Type**: Differential drive base controller
- **Application**: Intelligent base movement control
- **ROS 2 Topics**:
  - `/cmd_vel` (geometry_msgs/Twist) - Velocity command

### Sensors

#### Sensor_BW-MINS50 (IMU)
- **Package**: `sensor_bw_mins50`
- **Type**: 9-axis IMU
- **Application**: Precise orientation sensing for intelligent positioning
- **ROS 2 Topics**: `/imu/data` (sensor_msgs/Imu)

#### Vision Systems (if equipped)
- **Cameras**: Depth cameras and RGB cameras for visual perception
- **LiDAR**: For mapping and intelligent navigation

#### Serial Master SC0
- **Package**: `serial_sc0_master`
- **Type**: Serial communication master
- **Application**: Communication with sensors and peripherals

### Power Management

#### Battery Charger EPC602 4840 EP 01
- **Package**: `battery_charger_epc602_4840_ep_01`
- **Type**: Intelligent battery charger
- **ROS 2 Topics**: `/battery_charger_epc602_status`

### Monitoring

#### RoboCon Hardware Monitor GUI
- **Package**: `robocon_hw_monitor_gui`
- **Type**: Hardware monitoring GUI
- **Application**: Real-time hardware status monitoring

## Launching the Robot

```bash
ros2 launch Robot_iSheather_bringup real_robot.launch.py
```

## Sample Code

### C++

#### Intelligent Sheathing Operation

```cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <robot_custom_interfaces/msg/ids830abs_status.hpp>
#include <sensor_msgs/msg/imu.hpp>

class IntelligentSheatherControl : public rclcpp::Node
{
public:
    IntelligentSheatherControl() : Node("intelligent_sheather_control")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        
        tool_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/motor_driver_ids830abs/command", 10);
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&IntelligentSheatherControl::imu_callback, this, std::placeholders::_1));
        
        tool_status_sub_ = this->create_subscription<robot_custom_interfaces::msg::IDS830ABSStatus>(
            "/ids830abs_status", 10,
            std::bind(&IntelligentSheatherControl::tool_status_callback, this, std::placeholders::_1));
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Use IMU for intelligent positioning decisions
        current_orientation_ = msg->orientation;
    }

    void tool_status_callback(const robot_custom_interfaces::msg::IDS830ABSStatus::SharedPtr msg)
    {
        // Monitor tool position for intelligent control
        current_tool_position_ = msg->position_mm;
    }

    void intelligent_positioning()
    {
        // AI-based positioning using sensor feedback
        auto twist = geometry_msgs::msg::Twist();
        // Intelligent positioning logic here
        cmd_vel_pub_->publish(twist);
    }

    void intelligent_sheathing()
    {
        // AI-based sheathing with adaptive control
        auto cmd = std_msgs::msg::Float32MultiArray();
        // Intelligent tool control logic here
        tool_cmd_pub_->publish(cmd);
    }

    geometry_msgs::msg::Quaternion current_orientation_;
    float current_tool_position_ = 0.0f;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr tool_cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<robot_custom_interfaces::msg::IDS830ABSStatus>::SharedPtr tool_status_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IntelligentSheatherControl>());
    rclcpp::shutdown();
    return 0;
}
```

### C

#### Basic Control

```c
#include <rclc/rclc.h>
#include <geometry_msgs/msg/twist.h>

int main(int argc, char * argv[])
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_init(argc, argv, "isheather_control", &allocator, 
              RCL_DEFAULT_DOMAIN_ID, NULL, false);
    
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "isheather_control", "", &allocator));
    
    rcl_publisher_t cmd_vel_pub;
    RCCHECK(rclc_publisher_init_default(
        &cmd_vel_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));
    
    geometry_msgs__msg__Twist msg;
    msg.linear.x = 0.3;
    msg.angular.z = 0.0;
    
    rcl_publish(&cmd_vel_pub, &msg, NULL);
    
    rcl_publisher_fini(&cmd_vel_pub, &node);
    rcl_node_fini(&node);
    
    return 0;
}
```

### Python 3

#### Intelligent Sheathing Operation

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from robot_custom_interfaces.msg import IDS830ABSStatus
from sensor_msgs.msg import Imu
import math

class IntelligentSheatherControl(Node):
    def __init__(self):
        super().__init__('intelligent_sheather_control')
        
        # Base control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Tool control
        self.tool_cmd_pub = self.create_publisher(
            Float32MultiArray,
            '/motor_driver_ids830abs/command',
            10
        )
        
        # Sensors
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        
        self.tool_status_sub = self.create_subscription(
            IDS830ABSStatus,
            '/ids830abs_status',
            self.tool_status_callback,
            10
        )
        
        self.current_orientation = None
        self.current_tool_position = 0.0
    
    def imu_callback(self, msg):
        """Update orientation for intelligent positioning"""
        self.current_orientation = msg.orientation
    
    def tool_status_callback(self, msg):
        """Monitor tool position for intelligent control"""
        self.current_tool_position = msg.position_mm
    
    def quaternion_to_yaw(self, q):
        """Extract yaw from quaternion"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))
    
    def intelligent_positioning(self, target_yaw_deg):
        """AI-enhanced positioning using IMU feedback"""
        if self.current_orientation is None:
            return False
        
        current_yaw = self.quaternion_to_yaw(self.current_orientation)
        error = target_yaw_deg - current_yaw
        
        # Normalize error
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        
        twist = Twist()
        
        if abs(error) > 2.0:
            # Proportional control with damping
            twist.angular.z = 0.3 * (error / 180.0)
            twist.linear.x = 0.0  # Stop forward motion while rotating
        else:
            twist.angular.z = 0.0
            twist.linear.x = 0.3  # Move forward when aligned
        
        self.cmd_vel_pub.publish(twist)
        return abs(error) <= 2.0
    
    def intelligent_tool_control(self, target_position_mm):
        """AI-enhanced tool control with adaptive positioning"""
        error = target_position_mm - self.current_tool_position
        
        # Adaptive control based on error
        if abs(error) > 50.0:
            # Large error - fast movement
            velocity = 10.0
        elif abs(error) > 10.0:
            # Medium error - moderate movement
            velocity = 5.0
        else:
            # Small error - slow movement for precision
            velocity = 1.0
        
        cmd = Float32MultiArray()
        cmd.data = [float(target_position_mm), float(velocity), 0.0]
        self.tool_cmd_pub.publish(cmd)
        
        return abs(error) < 5.0
    
    def perform_intelligent_sheathing_sequence(self, positions):
        """Perform intelligent sheathing at multiple positions"""
        self.get_logger().info("Starting intelligent sheathing sequence...")
        
        for i, (yaw_deg, tool_position_mm) in enumerate(positions):
            self.get_logger().info(
                f'Position {i+1}/{len(positions)}: '
                f'Yaw={yaw_deg}°, Tool={tool_position_mm}mm'
            )
            
            # 1. Intelligent positioning
            positioned = False
            for _ in range(100):  # 10 seconds timeout
                positioned = self.intelligent_positioning(yaw_deg)
                if positioned:
                    break
                rclpy.sleep(self, 0.1)
            
            if not positioned:
                self.get_logger().warn(f'Failed to position at step {i+1}')
                continue
            
            rclpy.sleep(self, 0.5)  # Stabilize
            
            # 2. Intelligent tool extension
            for _ in range(50):  # 5 seconds timeout
                if self.intelligent_tool_control(tool_position_mm):
                    break
                rclpy.sleep(self, 0.1)
            
            rclpy.sleep(self, 1.0)  # Hold position
            
            # 3. Retract tool
            self.intelligent_tool_control(50.0)
            rclpy.sleep(self, 2.0)
        
        self.get_logger().info("Intelligent sheathing sequence complete")

def main(args=None):
    rclpy.init(args=args)
    node = IntelligentSheatherControl()
    
    # Wait for sensors
    rclpy.spin_once(node, timeout_sec=1.0)
    
    # Define sheathing positions (yaw_angle, tool_position)
    positions = [
        (0.0, 200.0),
        (45.0, 200.0),
        (90.0, 200.0),
        (135.0, 200.0),
    ]
    
    # Perform intelligent sheathing
    node.perform_intelligent_sheathing_sequence(positions)
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### AI-Enhanced Adaptive Control

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from robot_custom_interfaces.msg import IDS830ABSStatus
import numpy as np

class AdaptiveToolControl(Node):
    def __init__(self):
        super().__init__('adaptive_tool_control')
        
        self.tool_cmd_pub = self.create_publisher(
            Float32MultiArray,
            '/motor_driver_ids830abs/command',
            10
        )
        
        self.tool_status_sub = self.create_subscription(
            IDS830ABSStatus,
            '/ids830abs_status',
            self.tool_status_callback,
            10
        )
        
        self.position_history = []
        self.velocity_history = []
        self.timer = self.create_timer(0.1, self.adaptive_control)
    
    def tool_status_callback(self, msg):
        """Store position and velocity for adaptive control"""
        self.position_history.append(msg.position_mm)
        self.velocity_history.append(msg.velocity_mm_per_s)
        
        # Keep only recent history
        if len(self.position_history) > 20:
            self.position_history.pop(0)
            self.velocity_history.pop(0)
    
    def adaptive_control(self):
        """Adaptive control using historical data"""
        if len(self.position_history) < 5:
            return
        
        # Calculate position variance
        positions = np.array(self.position_history[-10:])
        position_variance = np.var(positions)
        
        # Calculate velocity trend
        velocities = np.array(self.velocity_history[-10:])
        velocity_trend = np.mean(np.diff(velocities))
        
        # Adaptive control logic
        target_position = 200.0
        current_position = self.position_history[-1]
        error = target_position - current_position
        
        # Adjust control parameters based on variance and trend
        if position_variance > 10.0:
            # High variance - reduce gain for stability
            kp = 0.5
        else:
            # Low variance - normal gain
            kp = 1.0
        
        if abs(velocity_trend) > 5.0:
            # High velocity change - add damping
            kd = 0.3
        else:
            kd = 0.1
        
        # Calculate adaptive velocity
        velocity = kp * error - kd * velocity_trend
        
        cmd = Float32MultiArray()
        cmd.data = [float(target_position), float(velocity), 0.0]
        self.tool_cmd_pub.publish(cmd)
        
        self.get_logger().debug(
            f'Adaptive Control - Error: {error:.2f}mm, '
            f'Variance: {position_variance:.2f}, '
            f'Velocity: {velocity:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveToolControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Next Steps

- [API Reference](../api-reference/motor-control.md) - Detailed motor control APIs
- [ROS 2 Integration](../ros2/nodes-and-topics.md) - ROS 2 topics and nodes
- [AI Programs](../ai-programs/overview.md) - AI program development
- [Deployment](../deployment/runtime-configuration.md) - Runtime configuration

