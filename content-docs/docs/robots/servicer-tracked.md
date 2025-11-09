# RoboCon Servicer Tracked 15kg

The RoboCon Servicer Tracked 15kg is a tracked service robot variant designed for general service and maintenance tasks. With dual arms and advanced manipulation capabilities, it delivers superhumanoid performance in industrial environments.

## Model Specifications

- **Year**: 2025
- **Make**: RoboCon
- **Model**: Servicer Tracked 15kg
- **Payload Capacity**: 15kg per arm
- **Base Configuration**: Tracked mobility system
- **Arms**: Dual 6-DOF robotic arms
- **Runtime**: 8+ hours per battery pack

## Overview

The RoboCon Servicer Tracked features:
- **Robotic Arms**: Dual 6-DOF robotic arms for manipulation tasks
- **Mobile Base**: Tracked base for all-terrain navigation
- **Sensor Suite**: IMU and pressure sensors for operation monitoring
- **Service Capabilities**: Designed for service and maintenance operations

## Hardware Drivers

### Robotic Arms

#### Dual 6-DOF Robotic Arms
- **Type**: Dual 6-DOF robotic arm system
- **Application**: Manipulation and service tasks
- **Payload**: 15kg per arm (30kg total dual-arm capacity)
- **Reach**: 1500mm maximum arm span
- **ROS 2 Integration**: 
  - Full ROS 2 control interface
  - MoveIt integration available for advanced motion planning
- **Communication**: Serial communication interface
- **ROS 2 Topics**:
  - Arm joint states and commands
  - Controller status and feedback

> **For detailed manufacturer specifications and technical information, see:** [Implementation: Servicer Arms](/docs/implementation/robots/servicer-tracked-arms)

### Base Movement

#### Base Ackermann Controller
- **Package**: `base_ackermann_controller`
- **Type**: Differential drive base controller
- **Application**: Base movement and navigation
- **ROS 2 Topics**:
  - `/cmd_vel` (geometry_msgs/Twist) - Input command
  - `/base/ackermann_controller/reference_unstamped` - Reference velocity

#### Twist Mux
- **Package**: `twist_mux`
- **Type**: Velocity command multiplexer
- **Application**: Prioritizes velocity commands from multiple sources
- **Configuration**: `/params/twist_mux.yaml`

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
- **Application**: Auxiliary actuators (if equipped)
- **ROS 2 Topics**:
  - `/motor_driver_ids830abs/command` (Float32MultiArray)
  - `/ids830abs_status` (IDS830ABSStatus)

### Sensors

#### Sensor_BW-MINS50 (IMU)
- **Package**: `sensor_bw_mins50`
- **Type**: 9-axis IMU (accelerometer, gyroscope, magnetometer)
- **Application**: Orientation and motion sensing
- **ROS 2 Topics**: `/imu/data` (sensor_msgs/Imu)
- **Parameters**: `/params/sensor_bw_mins50_params.yaml`
  - `baud_rate`: 115200
  - `output_freq`: 50 Hz
  - `port`: /dev/ttySC1

#### Serial Master SC0
- **Package**: `serial_sc0_master`
- **Type**: Serial communication master
- **Application**: Communication with sensors and peripherals
- **Parameters**: `/params/serial_sc0_master_params.yaml`

#### Pressure Sensor PS-1L-NV
- **Package**: `pressure_sensor_ps_1l_nv`
- **Type**: Pressure sensor
- **Application**: Pressure monitoring
- **ROS 2 Topics**: `/ps1lnv_sensor/pressure` (std_msgs/Float32)

### Power Management

#### Battery Charger EPC602 4840 EP 01
- **Package**: `battery_charger_epc602_4840_ep_01`
- **Type**: Intelligent battery charger
- **ROS 2 Topics**: `/battery_charger_epc602_status`

### Monitoring

#### RoboCon Hardware Monitor GUI
- **Package**: `robocon_hw_monitor_gui`
- **Type**: Hardware monitoring GUI
- **Application**: Real-time hardware status monitoring and diagnostics

## Launching the Robot

```bash
ros2 launch robot_oservicer_bringup real_robot.launch.py
```

The launch file brings up:
- Dual arm controllers
- MoveIt (optional, commented out by default)
- Base controller
- Twist mux for velocity command priority
- Serial master
- Hardware monitor GUI
- Motor drivers
- Sensors (IMU, pressure)
- Battery charger

## Comparison with Similar Robots

### RoboCon Servicer Tracked 15kg vs. RoboForce TITAN

The RoboCon Servicer Tracked 15kg shares many design principles with the [RoboForce TITAN](https://www.roboforce.ai/product), a general-purpose AI robot designed for industrial applications.

**Similarities:**
- **Dual Arm Configuration**: Both robots feature dual robotic arms for bimanual manipulation
- **Industrial Focus**: Designed for industrial and logistics applications
- **Payload Capacity**: Similar payload handling capabilities (RoboCon: 15kg per arm, TITAN: 40kg total)
- **Physical AI**: Both leverage AI-driven manipulation and autonomous capabilities
- **Modular Design**: Both feature modular hardware systems for customization
- **Field-Proven Autonomy**: Both designed for deployment in real-world industrial environments

**Key Differences:**
- **Base Configuration**: RoboCon Servicer Tracked uses tracked mobility; TITAN features flexible base configurations
- **Height/Reach**: TITAN reaches 2.10m maximum height with 1100mm arm reachability; RoboCon Servicer optimized for different workspace requirements
- **Payload Distribution**: TITAN handles 40kg total payload; RoboCon Servicer designed for 15kg per arm (30kg total dual-arm capacity)
- **Software Stack**: RoboCon Servicer built on ROBOCON OS with ROS 2; TITAN uses RoboForce's proprietary Physical AI platform
- **Marketplace Integration**: RoboCon Servicer supports ROBOCON marketplace for server installation; TITAN uses different integration approach

**Best Use Cases:**
- **RoboCon Servicer Tracked**: Service tasks, maintenance operations, tracked mobility requirements, ROS 2 ecosystem integration
- **RoboForce TITAN**: General-purpose industrial manipulation, taller workspaces, higher payload requirements

### RoboCon Servicer Tracked 15kg vs. Dexterity Mech

The RoboCon Servicer Tracked 15kg aligns with the capabilities of [Dexterity Mech](https://www.dexterity.ai/mech), an industrial superhumanoid robot designed for logistics and material handling.

**Similarities:**
- **Dual Arm Design**: Both feature dual robotic arms with extensive reach capabilities
- **Superhumanoid Performance**: Both designed to operate in human workspaces with enhanced capabilities
- **Industrial Applications**: Optimized for industrial and logistics tasks (truck loading, palletizing, depalletizing)
- **Force Control**: Both incorporate force control and touch sensing for dexterous manipulation
- **Multi-Use Capability**: Both designed to move between tasks and workstations
- **Industrial Strength**: Built for durability with high MTBF (Mean Time Between Failures) specifications

**Key Differences:**
- **Arm Span**: Dexterity Mech achieves 5.4m armspan with 60kg lift capacity; RoboCon Servicer designed for different workspace requirements
- **Base Mobility**: Dexterity Mech features a rover base with four-wheel steerable system; RoboCon Servicer uses tracked base
- **Payload Capacity**: Mech lifts 60kg total; RoboCon Servicer handles 15kg per arm (30kg total dual-arm)
- **Vertical Reach**: Mech extends beyond 8 feet (2.4m); RoboCon Servicer optimized for different height requirements
- **Operating System**: RoboCon Servicer uses ROBOCON OS (ROS 2); Mech uses Dexterity's proprietary control system
- **Dual Vacuum Pumps**: Mech includes dual vacuum pumps for material handling; RoboCon Servicer uses different end effector systems

**Best Use Cases:**
- **RoboCon Servicer Tracked**: Tracked mobility for rough terrain, service and maintenance, ROS 2 development, lower payload applications
- **Dexterity Mech**: Truck loading, high-reach palletizing, larger payload requirements, warehouse operations requiring 8+ foot reach

**Technical Comparison:**

| Feature | RoboCon Servicer Tracked 15kg | Dexterity Mech | RoboForce TITAN |
|---------|------------------------------|----------------|-----------------|
| **Arms** | Dual 6-DOF | Dual arms with unique shoulder | Dual arms |
| **Payload** | 15kg per arm (30kg total) | 60kg total | 40kg total |
| **Arm Span** | Optimized for service tasks | 5.4m | Variable |
| **Base** | Tracked | Rover (4-wheel steerable) | Flexible |
| **Reach** | Service workspace optimized | >8 feet vertical | 1100mm arm reach |
| **OS** | ROBOCON OS (ROS 2) | Dexterity OS | RoboForce Physical AI |
| **Mobility** | All-terrain tracked | Indoor/warehouse rover | Flexible configurations |
| **Special Features** | ROS 2 marketplace, CAN bus | Force control, dual vacuum | Physical AI, modular |

## Sample Code

### C++

#### Controlling the Arm

```cpp
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class ArmControl : public rclcpp::Node
{
public:
    ArmControl() : Node("arm_control")
    {
        arm_cmd_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/arm_controller/joint_trajectory", 10);
        
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&ArmControl::joint_state_callback, this, std::placeholders::_1));
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received joint states: %zu joints", msg->name.size());
    }

    void move_arm_to_position(const std::vector<double>& positions)
    {
        auto trajectory = trajectory_msgs::msg::JointTrajectory();
        trajectory.joint_names = {
            "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
        };
        
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = positions;
        point.time_from_start.sec = 2;
        
        trajectory.points.push_back(point);
        arm_cmd_pub_->publish(trajectory);
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmControl>();
    
    // Example: Move arm to a specific position
    std::vector<double> target_positions = {0.5, 0.3, -0.2, 0.8, 0.4, 0.0};
    node->move_arm_to_position(target_positions);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

#### Base Movement Control

```cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class BaseControl : public rclcpp::Node
{
public:
    BaseControl() : Node("base_control")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&BaseControl::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = 0.3;
        twist.angular.z = 0.1;
        cmd_vel_pub_->publish(twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BaseControl>());
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
    rclc_init(argc, argv, "oservicer_control", &allocator, 
              RCL_DEFAULT_DOMAIN_ID, NULL, false);
    
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "oservicer_control", "", &allocator));
    
    rcl_publisher_t cmd_vel_pub;
    RCCHECK(rclc_publisher_init_default(
        &cmd_vel_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));
    
    geometry_msgs__msg__Twist msg;
    msg.linear.x = 0.3;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.1;
    
    rcl_publish(&cmd_vel_pub, &msg, NULL);
    
    rcl_publisher_fini(&cmd_vel_pub, &node);
    rcl_node_fini(&node);
    
    return 0;
}
```

### Python 3

#### Coordinated Arm and Base Control

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class ServicerControl(Node):
    def __init__(self):
        super().__init__('servicer_control')
        
        # Arm control
        self.arm_cmd_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        
        # Base control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Joint state monitoring
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
    
    def joint_state_callback(self, msg):
        self.get_logger().info(f'Joint states: {len(msg.name)} joints')
    
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
        self.get_logger().info(f'Commanded arm to position: {joint_positions}')
    
    def move_base(self, linear_x, angular_z):
        """Move base with specified velocities"""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)
    
    def perform_service_task(self):
        """Perform a coordinated service task"""
        self.get_logger().info("Starting service task...")
        
        # 1. Move base to position
        self.move_base(0.5, 0.0)
        rclpy.sleep(self, 3.0)
        
        # 2. Stop base
        self.move_base(0.0, 0.0)
        
        # 3. Move arm to pick position
        self.move_arm([0.5, 0.3, -0.2, 0.8, 0.4, 0.0])
        rclpy.sleep(self, 3.0)
        
        # 4. Perform task (e.g., grab object)
        self.move_arm([0.5, 0.4, -0.1, 0.9, 0.5, 0.2])
        rclpy.sleep(self, 2.0)
        
        # 5. Retract arm
        self.move_arm([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        rclpy.sleep(self, 2.0)
        
        self.get_logger().info("Service task complete")

def main(args=None):
    rclpy.init(args=args)
    node = ServicerControl()
    
    # Example: Perform a service task
    node.perform_service_task()
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Monitoring IMU Data

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class IMUMonitor(Node):
    def __init__(self):
        super().__init__('imu_monitor')
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
    
    def imu_callback(self, msg):
        self.get_logger().info(
            f'Orientation: w={msg.orientation.w:.3f} '
            f'x={msg.orientation.x:.3f} '
            f'y={msg.orientation.y:.3f} '
            f'z={msg.orientation.z:.3f}'
        )
        self.get_logger().info(
            f'Linear Accel: x={msg.linear_acceleration.x:.3f} '
            f'y={msg.linear_acceleration.y:.3f} '
            f'z={msg.linear_acceleration.z:.3f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = IMUMonitor()
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
from robot_custom_interfaces.msg import GoldenMotorEZA48400Status

class MotorMonitor(Node):
    def __init__(self):
        super().__init__('motor_monitor')
        
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
            f'Current: {msg.phase_current:.2f}A, '
            f'Temp: {msg.controller_temp:.1f}°C'
        )
    
    def right_motor_callback(self, msg):
        self.get_logger().info(
            f'Right Motor - Speed: {msg.speed_rpm} RPM, '
            f'Voltage: {msg.voltage:.2f}V, '
            f'Current: {msg.phase_current:.2f}A, '
            f'Temp: {msg.controller_temp:.1f}°C'
        )

def main(args=None):
    rclpy.init(args=args)
    node = MotorMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Next Steps

- [API Reference](../api-reference/motor-control.md) - Detailed motor control APIs
- [ROS 2 Integration](../ros2/nodes-and-topics.md) - ROS 2 topics and nodes
- [Deployment](../deployment/runtime-configuration.md) - Runtime configuration

