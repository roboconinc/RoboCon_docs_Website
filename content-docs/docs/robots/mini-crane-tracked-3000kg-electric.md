# RoboCon Mini Crane Tracked 3000kg - Electric

**Brand/Make:** RoboCon  
**Model Family:** Mini Crane  
**Base Option:** Tracked  
**Max Payload:** 3000kg  
**Actuation Type:** Electric

The RoboCon Mini Crane Tracked 3000kg - Electric is a revolutionary foldable crane assembly featuring full electric actuators instead of hydraulic systems. This variant uses electric motor actuators for all boom and leg movements, providing precise control, simplified maintenance, and enhanced maneuverability with automatic and highly maneuverable outriggers featuring 5 joints on each outrigger.

## Overview

The Mini Crane Electric variant offers:

- **Electric Motor Actuators**: Precision electric motors for all extensible components
- **Enhanced Control**: Fine-grained positioning and velocity control
- **Reduced Maintenance**: No hydraulic fluid systems or seals
- **Quieter Operation**: Lower noise levels compared to hydraulic systems
- **Advanced Outriggers**: Automatic and highly maneuverable outriggers with 5 joints per outrigger
- **Precise Positioning**: High-resolution electric actuators for accurate positioning

## Key Innovations

### 1. Full Electric Actuation System

Unlike the hydraulic variant, the Electric Mini Crane uses electric motor actuators for:

- **Boom Extension/Retraction**: Electric linear actuators or ball-screw mechanisms
- **Outrigger Control**: Electric actuators for all 5 joints on each outrigger
- **Rotation Control**: Electric servo motors for base rotation
- **End Effector Control**: Electric actuators for all end module operations

### 2. Advanced 5-Joint Outrigger System

Each outrigger features **5 articulated joints** that provide:

#### Joint 1: Base Rotation
- **Function**: Rotates outrigger base relative to crane chassis
- **Range**: 360° rotation capability
- **Control**: Electric servo motor with position feedback

#### Joint 2: Primary Extension/Flexion
- **Function**: Main outrigger section extension/retraction
- **Range**: Variable extension length
- **Control**: Electric linear actuator or ball-screw mechanism

#### Joint 3: Secondary Extension/Flexion
- **Function**: Secondary section for extended reach
- **Range**: Additional extension capability
- **Control**: Electric actuator synchronized with Joint 2

#### Joint 4: Horizontal Pivot
- **Function**: Horizontal angle adjustment for ground adaptation
- **Range**: ±45° horizontal pivot
- **Control**: Electric pivot actuator

#### Joint 5: Ground Contact/Flex
- **Function**: Final ground contact point with adaptive flex
- **Range**: Vertical adjustment and flex compensation
- **Control**: Electric actuator with force feedback for ground contact

**Benefits of 5-Joint Outriggers:**
- **Automatic Deployment**: Self-leveling and automatic ground adaptation
- **High Maneuverability**: Complex positioning for optimal stability
- **Uneven Terrain Adaptation**: 5 joints allow adaptation to highly irregular ground surfaces
- **Precise Leveling**: Fine control for maintaining crane stability on slopes
- **Compact Retraction**: Folds into minimal space when retracted

### 3. Foldable Compact Design

- **Compact Footprint**: Folds to a footprint equal to or less than **1.2 m × 1.2 m** for efficient storage and transport
- **Rapid Deployment**: Quick transition between storage and operational configurations
- **Telescoping Boom**: Multi-section telescopic boom with electric actuator extension/retraction
- **Retractable Legs**: Plurality of extensible legs (with 5 joints each) that deploy for stability and retract for mobility

### 4. Configurable End Module System

The end module is **automatically configurable** to operate as multiple attachment types:

#### Hook Assembly
- Utilizes rope or cable connections to hold panels and loads
- Provides rotational freedom during lifting operations
- Electric winch control for rope management

#### Vacuum-Suction Assembly
- Removably attaches to panels through pneumatic suction
- Electric vacuum pump control
- Ideal for flat, smooth surfaces like construction panels

#### Fork Assembly
- Lifts panels through mechanical engagement
- Electric fork positioning actuators
- Precision positioning for material placement

#### End-Effector Platform
The end module can accommodate multiple **removably attached end-effectors**:
- **Robotic Arms**: Multi-degree-of-freedom manipulation with electric joint control
- **Rotary Drills**: Electric drill motors with speed and torque control
- **Screwdrivers**: Electric drive with automated fastening and torque control
- **Nail Drivers**: Electric impact mechanisms for construction fastening
- **Grippers**: Electric articulated fingers for grasping objects

### 5. Intelligent Control System

#### System Controller Architecture

The Mini Crane Electric features an advanced system controller with:

- **Computational Processor**: Primary processing unit for command inputs, sensor data processing, and operational control
- **Computational Planner**: Advanced planning capabilities for coordinating complex crane operations
- **Machine Learning Module**: Adaptive operational optimization based on historical performance data
- **Behavior Tree Module**: Structured control frameworks for predictable operational sequences

#### Visual-Language-Action Model

- Converts **natural-language instructions** into behavior trees (XML format)
- Enables intuitive control through spoken or written commands
- Automatically generates operational sequences from high-level task descriptions

#### Multi-Sensor Perception

- **2D-LiDAR Sensors**: Environment mapping and obstacle detection
- **3D-LiDAR Sensors**: Detailed 3D environmental data for navigation and task execution
- **Vision Systems**: Target assembly data and visual feedback
- **IMU Sensors**: Orientation and motion sensing

### 6. Coordinated Deployment Sequence

The Mini Crane Electric follows an intelligent deployment and retraction sequence:

1. **Receiving Commands**: System receives operational commands
2. **Outrigger Deployment**: 5-joint outriggers automatically deploy with terrain adaptation
3. **Boom Extension**: Electric actuators extend extensible boom lift to operational position
4. **Task Execution**: Crane performs lifting, positioning, or manipulation operations
5. **Retraction**: Prior to mobility, boom retracts, then outriggers retract in coordinated sequence

This sequence ensures safety and prevents component damage during transitions.

### 7. Multi-Crane Coordination

The Mini Crane Electric can coordinate with other foldable crane assemblies:

- **Cooperative Lifting**: Multiple cranes work together on large loads
- **Task Synchronization**: Coordinated movements between crane units
- **Dynamic Task Allocation**: Real-time coordination based on capabilities and positioning
- **Inter-Crane Communication**: Exchange of operational status and planning data

## Technical Specifications

### Physical Dimensions

| Property | Value |
|----------|-------|
| Folded Footprint | ≤ 1.2 m × 1.2 m |
| Boom Sections | 6-section telescopic design |
| Support Legs | 4 extensible outriggers (5 joints each) |
| Base Type | Tracked undercarriage |
| Max Payload | 3000 kg |

### Power Systems

#### Mini Crane Electric
- High-capacity battery system (72V 300Ah LFP or similar)
- Electric motor actuators for all extensible components
- Intelligent charging system
- Power management for electric actuators
- No hydraulic fluid systems required

### Control Interface

- **Network Connectivity**: Cloud-based coordination and remote monitoring
- **Client Device Support**: Mobile devices and control interfaces
- **Display Terminals**: Centralized monitoring and visualization
- **ROS 2 Integration**: Full ROS 2 topic and service support

## Hardware Drivers

### Motors and Actuators

#### Electric Boom Actuators
- **Package**: `electric_boom_actuator`
- **Type**: Electric linear actuators or ball-screw mechanisms
- **Voltage**: 72V DC or 380V AC (depending on actuator type)
- **Application**: Boom extension and retraction
- **Control**: Position, velocity, and force/torque control
- **ROS 2 Topics**:
  - `/boom_actuator/command` (trajectory_msgs/JointTrajectory)
  - `/boom_actuator/feedback` (sensor_msgs/JointState)

#### Electric Outrigger Actuators (5 Joints Per Outrigger)
- **Package**: `electric_outrigger_5joint`
- **Type**: Multi-joint electric outrigger system
- **Joints**: 5 electric actuators per outrigger
  - Joint 1: Base rotation (electric servo)
  - Joint 2: Primary extension (electric linear actuator)
  - Joint 3: Secondary extension (electric linear actuator)
  - Joint 4: Horizontal pivot (electric pivot actuator)
  - Joint 5: Ground contact/flex (electric actuator with force feedback)
- **Application**: Automatic deployment, terrain adaptation, stability control
- **ROS 2 Topics**:
  - `/outrigger/{id}/joint_trajectory` (trajectory_msgs/JointTrajectory)
  - `/outrigger/{id}/joint_state` (sensor_msgs/JointState)
  - `/outrigger/{id}/force_feedback` (geometry_msgs/Wrench)

#### Motor_Golden 144VDC 20kW
- **Package**: `motor_golden_144vdc_20kw`
- **Type**: High-power DC motor
- **Voltage**: 144V DC
- **Power**: 20kW
- **Application**: Primary drive motor for crane base mobility
- **ROS 2 Topics**:
  - `/golden_motor_left/status`
  - `/golden_motor_right/status`

#### Motor_EMDA440H2JD0 380V 4400W
- **Package**: `motor_emda440h2jd0_380v_4400w`
- **Type**: AC servo motor
- **Voltage**: 380V AC
- **Power**: 4400W
- **Application**: Boom slewing control, rotation platforms

#### Electric End Effector Actuators
- **Package**: `electric_end_effector_actuator`
- **Type**: Various electric actuators depending on end-effector type
- **Application**: Hook winch, vacuum pump, fork positioning, tool actuators
- **ROS 2 Topics**: Varies by end-effector type

### Pan-Tilt and Rotary Systems

#### Pantilt_JEC J-PT-760
- **Package**: `pantilt_jec_jpt_760`
- **Type**: Pan-tilt camera mount
- **Application**: Camera positioning and stabilization
- **ROS 2 Topics**: `/pantilt_jec_jpt_760/status`

#### Rotary Platform_3F DK120
- **Package**: `rotary_platform_3f_dk120`
- **Type**: Rotary platform (electric)
- **Application**: Base rotation for crane positioning

#### Rotary Platform_Newgear PTN085-18
- **Package**: `rotary_platform_newgear_ptn085_18`
- **Type**: High-precision rotary platform (electric)
- **Application**: Fine positioning and orientation control

### Sensors

#### Lidar_Hinson SE-1035
- **Package**: `lidar_hinson_se_1035`
- **Type**: 2D LiDAR sensor
- **Range**: Up to 10m
- **Application**: Obstacle detection and environment mapping
- **ROS 2 Topics**: `/lidar_hinson_se_1035/scan` (sensor_msgs/LaserScan)

#### Lidar_TF03-180
- **Package**: `lidar_benewake_tf03_180`
- **Type**: Single-point LiDAR
- **Range**: Up to 180m
- **Application**: Long-range distance measurement
- **ROS 2 Topics**: `/tf03_180/range`

#### Sensor_BW-MINS50 (IMU)
- **Package**: `sensor_bw_mins50`
- **Type**: 9-axis IMU (accelerometer, gyroscope, magnetometer)
- **Application**: Orientation and motion sensing
- **ROS 2 Topics**: `/imu/data` (sensor_msgs/Imu)

#### Force/Torque Sensors (Outrigger Joint 5)
- **Package**: `ft_sensor_outrigger`
- **Type**: Force/torque sensors
- **Application**: Ground contact force feedback for outrigger Joint 5
- **ROS 2 Topics**: `/outrigger/{id}/joint5/force` (geometry_msgs/Wrench)

### Power Management

#### Battery_LFP 72V 300Ah
- **Package**: `battery_lfp_72v_300ah`
- **Type**: Lithium Iron Phosphate battery
- **Voltage**: 72V
- **Capacity**: 300Ah
- **Application**: Primary power source for all electric actuators

#### Battery Charger EPC602 4840 EP 01
- **Package**: `battery_charger_epc602_4840_ep_01`
- **Type**: Intelligent battery charger
- **ROS 2 Topics**: `/battery_charger_epc602_status` (robot_custom_interfaces/BatteryChargerEPC602Status)

### Control Systems

#### PLC_2AO-8AI-8DI-8DO 24V
- **Package**: `plc_2ao_8ai_8di_8do_24v`
- **Type**: Programmable Logic Controller
- **I/O**: 2 Analog Outputs, 8 Analog Inputs, 8 Digital Inputs, 8 Digital Outputs
- **ROS 2 Topics**: `/plc_2ao_8ai_8di_8do_24v/status`

#### PLC 10IOA12 (12DI-12DO)
- **Package**: `plc_10ioa12_12di_12do_24v`
- **Type**: Programmable Logic Controller
- **I/O**: 12 Digital Inputs, 12 Digital Outputs
- **ROS 2 Topics**: `/plc10ioa12/status` (robot_custom_interfaces/PLC10IOA12Status)

### Vision Systems

#### Camera_JEC ZN2133
- **Package**: `camera_jec_zn2133`
- **Type**: Industrial camera
- **Application**: Visual inspection and monitoring
- **ROS 2 Topics**: `/camera_jec_zn2133/image_raw` (sensor_msgs/Image)

#### Depth Camera TM815 IX E1
- **Package**: `depth_camera_tm815_ix_e1`
- **Type**: 3D depth camera
- **Application**: 3D environment perception and object detection
- **ROS 2 Topics**:
  - `/camera/color/image_raw` (sensor_msgs/Image)
  - `/camera/depth/image_raw` (sensor_msgs/Image)
  - `/camera/pointcloud` (sensor_msgs/PointCloud2)

### Additional Components

- **Energy Meters**: `energy_meter_acrel_amc16_dett`, `energy_meter_acrel_amc16z_fdk24`
- **Fan Controller**: `controller_jpf4816_fan_speed`
- **Relay Controller**: `relay_xinlihui_n4roc04_24v`
- **Analog Voltage Reader**: `r4ivb02_analog_voltage_reader`
- **Electric Actuator Controllers**: Various CAN-based and serial-based controllers for electric actuators

## ROS 2 Development Details

### Bringup Package

**Package Name**: `robot_mini_crane_bringup`

The bringup package initializes and coordinates all ROS 2 nodes and hardware drivers required for the RoboCon Mini Crane Tracked 3000kg - Electric robot. The robot's boot sequence automatically detects the hardware configuration and launches the appropriate bringup variant.

### Launching the Robot

#### Electric Variant

```bash
ros2 launch robot_mini_crane_bringup robot_mini_crane_electric.launch.py
```

#### Coordinated Multi-Crane

```bash
ros2 launch robot_mini_crane_bringup robot_mini_crane_coordinated.launch.py
```

### What the Bringup Activates

The `robot_mini_crane_bringup` package activates a comprehensive set of ROS 2 nodes for electric crane operation:

#### 1. **Electric Crane Control System**
- **Boom Control**: Electric linear actuators or ball-screw mechanisms for extension/retraction
- **Rotation Platform**: Electric servo motors for crane base rotation
- **Outrigger System**: 5-joint electric outrigger deployment and positioning with automatic terrain adaptation
- **End Effector Control**: Electric actuators for sucker, fork, or other attachment control
- **Electric Actuator Management**: Coordinated control of all electric actuators

#### 2. **5-Joint Outrigger Control System**
- **Package**: `electric_outrigger_5joint`
- **Nodes**: Individual joint controllers for each of the 5 joints per outrigger
- **Joints Per Outrigger**:
  - Joint 1: Base rotation controller
  - Joint 2: Primary extension controller
  - Joint 3: Secondary extension controller
  - Joint 4: Horizontal pivot controller
  - Joint 5: Ground contact/flex controller with force feedback
- **Activates**: Automatic deployment, terrain adaptation, stability control
- **Features**: Coordinated joint control for complex outrigger positioning

#### 3. **Motor Control System**
- **Package**: Various electric motor drivers
- **Nodes**: Electric motor controllers for base mobility, rotation, and auxiliary functions
- **Activates**: Coordinated electric motor control

#### 4. **Sensor System**
- **Package**: `sensor_bw_mins50` (IMU)
- **Package**: `lidar_hinson_SE_1035` (LiDAR)
- **Package**: `lidar_tf03_180` (Ranging LiDAR)
- **Package**: `ft_sensor_outrigger` (Force/torque sensors for outrigger ground contact)
- **Activates**: Spatial awareness, obstacle detection, load monitoring, terrain sensing

#### 5. **Vision System**
- **Package**: `camera_jec_zn2133` (RGB camera)
- **Package**: `camera_tm815_ix_e1` (Depth camera)
- **Activates**: Visual feedback, depth perception, material identification

#### 6. **Power Management**
- **Package**: `battery_charger_epc602_4840_ep_01`
- **Package**: Energy meter monitoring
- **Activates**: Battery status, power consumption tracking, actuator power management

#### 7. **Monitoring and Diagnostics**
- **Package**: `robocon_hw_monitor_gui`
- **Activates**: Real-time hardware status, diagnostics dashboard, actuator monitoring

### ROS 2 Topic Network

**Command Topics:**
- `/boom_actuator/command` (trajectory_msgs/JointTrajectory) - Boom extension control
- `/outrigger/{id}/joint_trajectory` (trajectory_msgs/JointTrajectory) - Outrigger joint control
- `/end_module/configure` (service) - End module configuration
- `/base/cmd_vel` (geometry_msgs/Twist) - Base movement control

**Feedback Topics:**
- `/imu/data` (sensor_msgs/Imu) - IMU orientation data
- `/lidar_hinson_se_1035/scan` (sensor_msgs/LaserScan) - LiDAR scan data
- `/boom_actuator/feedback` (sensor_msgs/JointState) - Boom actuator status
- `/outrigger/{id}/joint_state` (sensor_msgs/JointState) - Outrigger joint positions
- `/outrigger/{id}/joint5/force` (geometry_msgs/Wrench) - Ground contact force feedback
- `/battery_charger_epc602_status` (robot_custom_interfaces/BatteryChargerEPC602Status) - Battery status

## Sample Code

### Controlling 5-Joint Outrigger System

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Wrench

class Outrigger5JointControl(Node):
    def __init__(self):
        super().__init__('outrigger_5joint_control')
        
        # Publishers for each outrigger (4 outriggers)
        self.outrigger_pubs = {}
        for i in range(4):
            self.outrigger_pubs[i] = self.create_publisher(
                JointTrajectory,
                f'/outrigger/{i}/joint_trajectory',
                10
            )
        
        # Subscribers for joint states
        self.joint_state_subs = {}
        for i in range(4):
            self.joint_state_subs[i] = self.create_subscription(
                JointState,
                f'/outrigger/{i}/joint_state',
                lambda msg, idx=i: self.joint_state_callback(msg, idx),
                10
            )
        
        # Subscribers for force feedback (Joint 5)
        self.force_subs = {}
        for i in range(4):
            self.force_subs[i] = self.create_subscription(
                Wrench,
                f'/outrigger/{i}/joint5/force',
                lambda msg, idx=i: self.force_callback(msg, idx),
                10
            )
    
    def joint_state_callback(self, msg: JointState, outrigger_id: int):
        """Callback for outrigger joint state feedback"""
        joint_names = ['joint1_base_rotation', 'joint2_primary_extension', 
                       'joint3_secondary_extension', 'joint4_horizontal_pivot', 
                       'joint5_ground_contact']
        positions = dict(zip(msg.name, msg.position))
        
        self.get_logger().info(
            f'Outrigger {outrigger_id} positions: '
            f'J1={positions.get(joint_names[0], 0):.2f}°, '
            f'J2={positions.get(joint_names[1], 0):.2f}m, '
            f'J3={positions.get(joint_names[2], 0):.2f}m, '
            f'J4={positions.get(joint_names[3], 0):.2f}°, '
            f'J5={positions.get(joint_names[4], 0):.2f}m'
        )
    
    def force_callback(self, msg: Wrench, outrigger_id: int):
        """Callback for ground contact force feedback"""
        force_z = msg.force.z
        if force_z > 0:
            self.get_logger().info(
                f'Outrigger {outrigger_id} ground contact force: {force_z:.2f}N'
            )
    
    def deploy_outrigger(self, outrigger_id: int, 
                        joint1_rot: float, joint2_ext: float, joint3_ext: float,
                        joint4_pivot: float, joint5_contact: float):
        """Deploy outrigger with 5-joint positioning"""
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'joint1_base_rotation',
            'joint2_primary_extension',
            'joint3_secondary_extension',
            'joint4_horizontal_pivot',
            'joint5_ground_contact'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = [joint1_rot, joint2_ext, joint3_ext, joint4_pivot, joint5_contact]
        point.velocities = [0.0] * 5
        point.time_from_start.sec = 5  # 5 second duration
        
        trajectory.points = [point]
        
        self.outrigger_pubs[outrigger_id].publish(trajectory)
        self.get_logger().info(f"Deploying outrigger {outrigger_id} with 5-joint positioning")

def main(args=None):
    rclpy.init(args=args)
    node = Outrigger5JointControl()
    
    # Example: Deploy all 4 outriggers automatically
    # Each outrigger uses 5 joints for terrain adaptation
    node.deploy_outrigger(0, 45.0, 0.5, 0.3, 0.0, 0.1)  # Outrigger 0
    node.deploy_outrigger(1, 135.0, 0.5, 0.3, 0.0, 0.1)  # Outrigger 1
    node.deploy_outrigger(2, 225.0, 0.5, 0.3, 0.0, 0.1)  # Outrigger 2
    node.deploy_outrigger(3, 315.0, 0.5, 0.3, 0.0, 0.1)  # Outrigger 3
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Controlling Electric Boom Extension

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ElectricBoomControl(Node):
    def __init__(self):
        super().__init__('electric_boom_control')
        
        self.boom_cmd_pub = self.create_publisher(
            JointTrajectory,
            '/boom_actuator/command',
            10
        )
        
        self.boom_feedback_sub = self.create_subscription(
            JointState,
            '/boom_actuator/feedback',
            self.boom_feedback_callback,
            10
        )
    
    def boom_feedback_callback(self, msg: JointState):
        """Callback for boom actuator feedback"""
        if len(msg.position) > 0:
            self.get_logger().info(f'Boom extension: {msg.position[0]:.2f}m')
    
    def extend_boom(self, target_height_m: float):
        """Extend boom to target height using electric actuator"""
        trajectory = JointTrajectory()
        trajectory.joint_names = ['boom_extension']
        
        point = JointTrajectoryPoint()
        point.positions = [target_height_m]
        point.velocities = [0.1]  # 0.1 m/s extension velocity
        point.time_from_start.sec = int(target_height_m / 0.1)  # Duration based on velocity
        
        trajectory.points = [point]
        
        self.boom_cmd_pub.publish(trajectory)
        self.get_logger().info(f"Extending boom to {target_height_m}m using electric actuator")

def main(args=None):
    rclpy.init(args=args)
    node = ElectricBoomControl()
    node.extend_boom(3.0)  # Extend to 3 meters
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Automatic Outrigger Terrain Adaptation

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Wrench
import numpy as np

class AutomaticOutriggerAdaptation(Node):
    def __init__(self):
        super().__init__('automatic_outrigger_adaptation')
        
        # Publishers for outrigger control
        self.outrigger_pubs = {}
        for i in range(4):
            self.outrigger_pubs[i] = self.create_publisher(
                JointTrajectory,
                f'/outrigger/{i}/joint_trajectory',
                10
            )
        
        # Force feedback subscribers
        self.force_subs = {}
        self.outrigger_forces = {}
        for i in range(4):
            self.outrigger_forces[i] = 0.0
            self.force_subs[i] = self.create_subscription(
                Wrench,
                f'/outrigger/{i}/joint5/force',
                lambda msg, idx=i: self.force_callback(msg, idx),
                10
            )
        
        # Timer for automatic adaptation
        self.timer = self.create_timer(0.5, self.adaptation_timer_callback)
    
    def force_callback(self, msg: Wrench, outrigger_id: int):
        """Update ground contact force for outrigger"""
        self.outrigger_forces[outrigger_id] = msg.force.z
    
    def adaptation_timer_callback(self):
        """Automatically adjust outriggers based on terrain and forces"""
        target_force = 500.0  # Target force per outrigger (N)
        
        for i in range(4):
            current_force = self.outrigger_forces[i]
            
            # Adjust Joint 5 (ground contact) based on force feedback
            if current_force < target_force * 0.8:
                # Increase ground contact (extend Joint 5)
                self.adjust_outrigger_joint5(i, 0.05)  # Extend 5cm
            elif current_force > target_force * 1.2:
                # Decrease ground contact (retract Joint 5)
                self.adjust_outrigger_joint5(i, -0.05)  # Retract 5cm
    
    def adjust_outrigger_joint5(self, outrigger_id: int, adjustment: float):
        """Adjust Joint 5 for automatic terrain adaptation"""
        # This would typically read current positions first,
        # then adjust only Joint 5 while keeping others constant
        self.get_logger().info(
            f'Auto-adjusting outrigger {outrigger_id} Joint 5 by {adjustment:.3f}m'
        )

def main(args=None):
    rclpy.init(args=args)
    node = AutomaticOutriggerAdaptation()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Comparison: Hydraulic vs Electric

| Feature | Mini Crane Hydraulic | Mini Crane Electric |
|---------|---------------------|---------------------|
| **Actuation** | Hydraulic cylinders | Electric motor actuators |
| **Control Precision** | High (hydraulic valve control) | Very High (direct motor control) |
| **Maintenance** | Requires hydraulic fluid and seals | Minimal (no fluids) |
| **Noise Level** | Moderate | Low |
| **Response Speed** | Fast | Very Fast |
| **Load Capacity** | Maximum | High (slightly lower peak) |
| **Energy Efficiency** | Good | Excellent |
| **Cold Weather** | May require fluid heating | No special requirements |
| **Outriggers** | Traditional hydraulic outriggers | 5-joint automatic electric outriggers |
| **Terrain Adaptation** | Limited | Advanced (5 joints per outrigger) |
| **Positioning Accuracy** | High | Very High |
| **Installation** | Requires hydraulic reservoir | Simpler (battery + motors) |

**Key Advantages of Electric Variant:**
- **5-Joint Outriggers**: Automatic and highly maneuverable outriggers with superior terrain adaptation
- **Precise Control**: Fine-grained positioning and velocity control for all actuators
- **Simplified Maintenance**: No hydraulic fluid systems, seals, or filters
- **Quieter Operation**: Lower noise levels ideal for urban and indoor applications
- **Enhanced Maneuverability**: Complex outrigger positioning for optimal stability on uneven terrain

## Applications

The Mini Crane Electric is designed for diverse applications including:

- **Construction**: Panel lifting, material positioning, tool deployment
- **Manufacturing**: Assembly operations, component placement
- **Warehousing**: Load handling, pallet operations
- **Infrastructure**: Maintenance tasks, equipment positioning
- **Urban Environments**: Quiet operation suitable for noise-sensitive areas
- **Indoor Operations**: No hydraulic fluid concerns for indoor use
- **Uneven Terrain**: Advanced 5-joint outriggers for challenging ground conditions
- **Multi-Crane Operations**: Cooperative lifting of large loads

## Safety Features

- **Automatic Retraction**: Boom and outriggers retract before mobility operations
- **Load Monitoring**: Real-time load sensing and overload protection
- **Automatic Leveling**: 5-joint outriggers automatically adapt to terrain
- **Force Feedback**: Ground contact force monitoring for safe deployment
- **Emergency Stop**: Comprehensive emergency stop systems
- **Sensor Fusion**: Multiple sensor systems for safe operation
- **Electric Safety**: Overcurrent protection and safe shutdown procedures

## Patent Information

The Mini Crane Electric variant incorporates several patented innovations including the 5-joint outrigger system and electric actuation methods. Patent information is available in the main Mini Crane documentation.

## Next Steps
- [Mini Crane Hydraulic](./mini-crane-tracked-3000kg-hydraulic.md) - Hydraulic variant documentation
- [API Reference](../api-reference/motor-control.md) - Detailed motor control APIs
- [Behavior Trees](../api-reference/behavior-trees.md) - Behavior tree control structures
- [ROS 2 Integration](../ros2/nodes-and-topics.md) - ROS 2 topics and nodes
- [Multi-Robot Communication](../api-reference/multi-robot-communication.md) - Multi-crane coordination
- [Deployment](../deployment/runtime-configuration.md) - Runtime configuration

