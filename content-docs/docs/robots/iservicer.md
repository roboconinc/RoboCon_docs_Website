# RoboCon Servicer Wheeled (Robot_iServicer_bringup)

The RoboCon Servicer Wheeled is an intelligent service robot variant with enhanced AI capabilities for advanced service and maintenance tasks. It uses wheeled base mobility instead of tracks.

## Overview

The RoboCon Servicer Wheeled features:
- **Intelligent Operation**: Enhanced AI capabilities for autonomous service tasks
- **Robotic Arm**: Manipulation capabilities for complex service operations
- **Mobile Base**: Navigation for service tasks
- **Advanced Sensors**: Comprehensive sensor suite for environment perception
- **AI Integration**: Integration with DeepSeek LLM and vision systems

## Hardware Drivers

The RoboCon Servicer Wheeled uses similar hardware to RoboCon Servicer Tracked with additional intelligent features:

### Robotic Arm

#### Arm Borunte BRTIRUS2030A
- **Package**: `arm_borunte_brtirus2030a_driver`
- **Type**: 6-DOF robotic arm controller
- **Application**: Manipulation and service tasks
- **ROS 2 Integration**: 
  - Uses `arm_borunte_brtirus2030a_controller` for control
  - MoveIt integration available via `arm_borunte_brtirus2030a_moveit`
- **ROS 2 Topics**:
  - Arm joint states and commands
  - Controller status and feedback

### Base Movement

#### Base Ackermann Controller
- **Package**: `base_ackermann_controller`
- **Type**: Differential drive base controller
- **Application**: Base movement and navigation
- **ROS 2 Topics**:
  - `/cmd_vel` (geometry_msgs/Twist) - Velocity command

#### Twist Mux
- **Package**: `twist_mux`
- **Type**: Velocity command multiplexer
- **Application**: Prioritizes velocity commands from multiple sources

### Motors

#### Golden Motor EZA48400
- **Package**: `golden_motor_eza48400`
- **Type**: Wheel motors for base movement
- **CAN Bus**: 500kbps, Node IDs: 0xEF (left), 0xF0 (right)
- **Application**: Base drive motors
- **ROS 2 Topics**:
  - `/base/robot_base_cmd_pub_` (Float32MultiArray)
  - `/base/wheel_motors_feedback` (Float32MultiArray)
  - `/left_wheel_golden_motor_status` (GoldenMotorEZA48400Status)
  - `/right_wheel_golden_motor_status` (GoldenMotorEZA48400Status)

#### Motor Driver IDS830ABS
- **Package**: `motor_driver_ids830abs`
- **Type**: CAN-based linear actuator controller
- **Application**: Auxiliary actuators
- **ROS 2 Topics**:
  - `/motor_driver_ids830abs/command` (Float32MultiArray)
  - `/ids830abs_status` (IDS830ABSStatus)

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

#### Vision Systems (if equipped)
- **Cameras**: Depth cameras and RGB cameras for visual perception
- **LiDAR**: For mapping and navigation

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
ros2 launch Robot_iServicer_bringup real_robot.launch.py
```

## Sample Code

### C++

#### Intelligent Service Task with Arm and Base

```cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/imu.hpp>

class IntelligentServicerControl : public rclcpp::Node
{
public:
    IntelligentServicerControl() : Node("intelligent_servicer_control")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        
        arm_cmd_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/arm_controller/joint_trajectory", 10);
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&IntelligentServicerControl::imu_callback, this, std::placeholders::_1));
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Use IMU data for intelligent navigation decisions
        RCLCPP_DEBUG(this->get_logger(), "IMU data received");
    }

    void execute_intelligent_task()
    {
        // 1. Navigate using IMU feedback
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = 0.4;
        twist.angular.z = 0.0;
        cmd_vel_pub_->publish(twist);
        
        // 2. Position arm for service
        // ... arm control logic
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IntelligentServicerControl>());
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
    rclc_init(argc, argv, "iservicer_control", &allocator, 
              RCL_DEFAULT_DOMAIN_ID, NULL, false);
    
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "iservicer_control", "", &allocator));
    
    rcl_publisher_t cmd_vel_pub;
    RCCHECK(rclc_publisher_init_default(
        &cmd_vel_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));
    
    geometry_msgs__msg__Twist msg;
    msg.linear.x = 0.4;
    msg.angular.z = 0.0;
    
    rcl_publish(&cmd_vel_pub, &msg, NULL);
    
    rcl_publisher_fini(&cmd_vel_pub, &node);
    rcl_node_fini(&node);
    
    return 0;
}
```

### Python 3

#### Intelligent Service Task

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Imu

class IntelligentServicerControl(Node):
    def __init__(self):
        super().__init__('intelligent_servicer_control')
        
        # Base control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Arm control
        self.arm_cmd_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        
        # IMU for intelligent navigation
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        self.imu_data = None
    
    def imu_callback(self, msg):
        """Store IMU data for intelligent decision making"""
        self.imu_data = msg
    
    def navigate_intelligently(self):
        """Navigate using IMU feedback"""
        if self.imu_data is None:
            return
        
        # Use IMU data to adjust navigation
        twist = Twist()
        
        # Simple example: adjust based on orientation
        if abs(self.imu_data.orientation.z) > 0.5:
            # Compensate for tilt
            twist.angular.z = -0.1 * self.imu_data.orientation.z
        else:
            twist.linear.x = 0.4
        
        self.cmd_vel_pub.publish(twist)
    
    def execute_service_task(self):
        """Execute an intelligent service task"""
        self.get_logger().info("Starting intelligent service task...")
        
        # Navigate to target
        for _ in range(50):  # 5 seconds at 10Hz
            self.navigate_intelligently()
            rclpy.sleep(self, 0.1)
        
        # Stop base
        self.move_base(0.0, 0.0)
        
        # Position arm
        self.move_arm([0.5, 0.3, -0.2, 0.8, 0.4, 0.0])
        rclpy.sleep(self, 3.0)
        
        # Perform service
        self.move_arm([0.5, 0.4, -0.1, 0.9, 0.5, 0.2])
        rclpy.sleep(self, 2.0)
        
        # Retract
        self.move_arm([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        self.get_logger().info("Intelligent service task complete")
    
    def move_base(self, linear_x, angular_z):
        """Move base with specified velocities"""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)
    
    def move_arm(self, joint_positions):
        """Move arm to specified joint positions"""
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in joint_positions]
        point.time_from_start.sec = 2
        
        trajectory.points = [point]
        self.arm_cmd_pub.publish(trajectory)

def main(args=None):
    rclpy.init(args=args)
    node = IntelligentServicerControl()
    
    # Wait for IMU data
    rclpy.spin_once(node, timeout_sec=1.0)
    
    # Execute task
    node.execute_service_task()
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### AI-Enhanced Navigation

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import numpy as np

class AINavigation(Node):
    def __init__(self):
        super().__init__('ai_navigation')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        
        self.orientation_history = []
        self.timer = self.create_timer(0.1, self.navigate)
    
    def imu_callback(self, msg):
        """Store orientation for pattern recognition"""
        self.orientation_history.append(msg.orientation)
        # Keep only recent history
        if len(self.orientation_history) > 10:
            self.orientation_history.pop(0)
    
    def navigate(self):
        """AI-enhanced navigation using sensor data"""
        if len(self.orientation_history) < 5:
            return
        
        # Simple AI: detect patterns in orientation
        recent = self.orientation_history[-5:]
        
        # Calculate average orientation change
        avg_change = np.mean([
            abs(recent[i].z - recent[i-1].z) 
            for i in range(1, len(recent))
        ])
        
        twist = Twist()
        
        if avg_change > 0.1:
            # High orientation change - reduce speed
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            self.get_logger().info("Reducing speed due to orientation instability")
        else:
            # Stable orientation - normal speed
            twist.linear.x = 0.5
            twist.angular.z = 0.0
        
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = AINavigation()
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

