# ROS 2 Topics and DDS Reference

Complete reference for all ROS 2 topics, DDS (Cyclone DDS) architecture, and topic modeling in ROBOCON SDK.

## Overview

ROBOCON SDK uses **ROS 2 Jazzy** with **Cyclone DDS** as the underlying Data Distribution Service (DDS) middleware. This document provides a comprehensive reference to all topics, their message types, QoS settings, and the relationship between ROS 2's topic abstraction and DDS's data-centric publish/subscribe model.

## Understanding ROS 2 Topics vs DDS Topics

### ROS 2 Topics (Application Layer)

**ROS 2 Topics** are high-level named channels that enable asynchronous publish/subscribe communication between nodes:

- **Purpose**: Named data streams (e.g., `/cmd_vel`, `/odom`)
- **Abstraction Level**: Application-level (ROS 2 API)
- **Characteristics**:
  - Strongly-typed messages (e.g., `geometry_msgs/Twist`)
  - Quality of Service (QoS) policies
  - Automatic discovery via DDS
  - Many-to-many communication (multiple publishers/subscribers)
  - Topic namespacing (`/namespace/topic_name`)

**Example:**
```python
# ROS 2 Topic
self.publisher = self.create_publisher(
    Twist,                    # Message type
    '/cmd_vel',              # Topic name
    10                       # Queue depth
)
```

### DDS Topics (Middleware Layer)

**DDS Topics** are the underlying data distribution mechanism:

- **Purpose**: Data-centric publish/subscribe at middleware level
- **Implementation**: Cyclone DDS in ROBOCON OS
- **Characteristics**:
  - Automatic participant discovery
  - Quality of Service (QoS) policies (Reliability, Durability, History)
  - Type-safe data exchange
  - Real-time performance optimizations
  - Network transparency (works across machines)

**Mapping:**
```
ROS 2 Topic (/cmd_vel) 
    ↓
RMW (ROS Middleware Wrapper)
    ↓
DDS Topic (Cyclone DDS)
    ↓
Network/Shared Memory
```

### How ROS 2 Maps to DDS

ROS 2 uses the **RMW (ROS Middleware)** layer to abstract DDS:

1. **ROS 2 Node** → **DDS DomainParticipant**
2. **ROS 2 Topic** → **DDS Topic** + **DDS Publisher/Subscriber**
3. **ROS 2 Message** → **DDS Type** (via IDL/RTPS)
4. **ROS 2 QoS** → **DDS QoS Policies**

**RMW Implementation:**
```bash
# ROBOCON OS uses Cyclone DDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Quality of Service (QoS) Policies

QoS policies control communication behavior in both ROS 2 and DDS:

### Key QoS Profiles

#### 1. Sensor Data (Default for sensors)

```python
from rclpy.qos import qos_profile_sensor_data

qos_profile_sensor_data = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)
```

- **Reliability**: BEST_EFFORT (drops old messages if queue full)
- **Durability**: VOLATILE (doesn't keep messages for late joiners)
- **Use Case**: High-frequency sensor data (IMU, LiDAR scans)

#### 2. Services Default

```python
from rclpy.qos import qos_profile_services_default

qos_profile_services_default = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)
```

- **Reliability**: RELIABLE (guarantees delivery)
- **Durability**: VOLATILE
- **Use Case**: Commands, state updates

#### 3. Parameters Default

```python
from rclpy.qos import qos_profile_parameters

qos_profile_parameters = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=1000
)
```

#### 4. System Default

```python
from rclpy.qos import qos_profile_system_default

qos_profile_system_default = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)
```

- **Durability**: TRANSIENT_LOCAL (keeps messages for late joiners)
- **Use Case**: TF transforms, static maps

### DDS QoS Policies (Cyclone DDS)

DDS provides additional QoS policies beyond ROS 2's abstraction:

1. **Reliability**:
   - `RELIABLE`: Guarantees message delivery
   - `BEST_EFFORT`: Best-effort delivery (no retransmission)

2. **Durability**:
   - `VOLATILE`: Messages not kept for late joiners
   - `TRANSIENT_LOCAL`: Keeps messages in publisher's history
   - `TRANSIENT`: Keeps messages in service's history
   - `PERSISTENT`: Keeps messages persistently

3. **History**:
   - `KEEP_LAST`: Keep last N messages
   - `KEEP_ALL`: Keep all messages

4. **Lifespan**: Maximum age of data samples

5. **Deadline**: Maximum expected time between messages

6. **Liveliness**: Participant liveliness contract

### Configuring QoS in ROBOCON

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Custom QoS for motor commands
motor_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
    deadline=Duration(seconds=0.1)  # 100ms deadline
)

self.motor_pub = self.create_publisher(
    MotorCommand,
    '/robocon/motor/command',
    motor_qos
)
```

## Complete Topic Reference

### Standard ROS 2 Topics

#### Navigation and Control

| Topic | Message Type | Description | QoS | Frequency |
|-------|-------------|------------|-----|-----------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands for base movement | RELIABLE | Variable |
| `/odom` | `nav_msgs/Odometry` | Odometry data (position, velocity) | BEST_EFFORT | 50-100 Hz |
| `/tf` | `tf2_msgs/TFMessage` | Transform tree | TRANSIENT_LOCAL | Variable |
| `/tf_static` | `tf2_msgs/TFMessage` | Static transforms | TRANSIENT_LOCAL | Once |

#### Sensor Data

| Topic | Message Type | Description | QoS | Frequency |
|-------|-------------|------------|-----|-----------|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR scan data | BEST_EFFORT | 10-30 Hz |
| `/camera/image_raw` | `sensor_msgs/Image` | Camera RGB images | BEST_EFFORT | 30 Hz |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Depth camera images | BEST_EFFORT | 30 Hz |
| `/camera/color/image_raw` | `sensor_msgs/Image` | Color camera images | BEST_EFFORT | 30 Hz |
| `/imu/data` | `sensor_msgs/Imu` | IMU data (orientation, acceleration) | BEST_EFFORT | 50-200 Hz |
| `/gps/fix` | `sensor_msgs/NavSatFix` | GPS position data | BEST_EFFORT | 1-10 Hz |
| `/joint_states` | `sensor_msgs/JointState` | Joint positions, velocities, efforts | BEST_EFFORT | 50-100 Hz |

#### Navigation 2 Topics

| Topic | Message Type | Description | QoS | Frequency |
|-------|-------------|------------|-----|-----------|
| `/map` | `nav_msgs/OccupancyGrid` | Map occupancy grid | TRANSIENT_LOCAL | On update |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | Global costmap | TRANSIENT_LOCAL | On update |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | Local costmap | TRANSIENT_LOCAL | On update |
| `/plan` | `nav_msgs/Path` | Current navigation path | RELIABLE | On replan |

### ROBOCON-Specific Topics

#### Motor Control

| Topic | Message Type | Description | QoS | Frequency |
|-------|-------------|------------|-----|-----------|
| `/robocon/motor/command` | `robocon_interfaces/MotorCommand` | Motor control commands | RELIABLE | 50-100 Hz |
| `/robocon/motor/state` | `robocon_interfaces/MotorState` | Motor state feedback | BEST_EFFORT | 50-100 Hz |
| `/robocon/motor/status` | `robocon_interfaces/MotorStatus` | Motor status information | RELIABLE | 10 Hz |
| `/base/robot_base_cmd_pub_` | `std_msgs/Float32MultiArray` | Base motor commands | RELIABLE | 50 Hz |
| `/base/wheel_motors_feedback` | `std_msgs/Float32MultiArray` | Wheel motor feedback | BEST_EFFORT | 50 Hz |

#### Sensor Aggregation

| Topic | Message Type | Description | QoS | Frequency |
|-------|-------------|------------|-----|-----------|
| `/robocon/sensor/data` | `robocon_interfaces/SensorData` | Aggregated sensor readings | BEST_EFFORT | 10 Hz |
| `/robocon/sensor/fusion` | `robocon_interfaces/SensorFusion` | Fused sensor data | BEST_EFFORT | 50 Hz |

#### TSBT-VLA System

| Topic | Message Type | Description | QoS | Frequency |
|-------|-------------|------------|-----|-----------|
| `/robocon/llm/command` | `std_msgs/String` | Natural language commands | RELIABLE | On command |
| `/robocon/llm/decision` | `robocon_interfaces/LLMDecision` | LLM behavior tree output | RELIABLE | On decision |
| `/robocon/llm/text_world` | `std_msgs/String` | Text world scene description | RELIABLE | 1-5 Hz |
| `/robocon/llm/sensor_text` | `std_msgs/String` | Temporal sensor logs | BEST_EFFORT | 10 Hz |

#### Multi-Robot Communication

| Topic | Message Type | Description | QoS | Frequency |
|-------|-------------|------------|-----|-----------|
| `/robocon/multi_robot/discovery` | `robocon_interfaces/DiscoveryBeacon` | Robot discovery beacons | BEST_EFFORT | 1 Hz |
| `/robocon/multi_robot/action_suggestion` | `robocon_interfaces/ActionSuggestion` | Action suggestion packets | RELIABLE | Variable |
| `/robocon/multi_robot/vote_intent` | `robocon_interfaces/VoteIntent` | Vote intent responses | RELIABLE | On vote |
| `/robocon/multi_robot/vote_ballot` | `robocon_interfaces/VoteBallot` | Vote ballots | RELIABLE | On vote |
| `/robocon/multi_robot/motion_ledger` | `robocon_interfaces/MotionLedger` | Motion ledger updates | RELIABLE | 10 Hz |
| `/robocon/multi_robot/goal_ledger` | `robocon_interfaces/GoalLedger` | Goal ledger updates | RELIABLE | On update |
| `/robocon/multi_robot/environment` | `robocon_interfaces/EnvironmentUpdate` | Environment server updates | RELIABLE | 1-5 Hz |

#### Hardware-Specific Topics

| Topic | Message Type | Description | QoS | Frequency |
|-------|-------------|------------|-----|-----------|
| `/robocon/led/command` | `robocon_interfaces/LEDCommand` | LED control commands | RELIABLE | On command |
| `/robocon/fan/command` | `robocon_interfaces/FanCommand` | Fan speed control | RELIABLE | On command |
| `/robocon/speaker/output` | `std_msgs/String` | Speaker text output | RELIABLE | On output |
| `/robocon/dig/command` | `std_msgs/String` | Digging operation commands | RELIABLE | On command |
| `/robocon/dig/status` | `std_msgs/String` | Digging operation status | RELIABLE | 10 Hz |
| `/robocon/actuator/command` | `robocon_interfaces/ActuatorCommand` | Actuator movement commands | RELIABLE | 50 Hz |

### LiDAR-Specific Topics

| Topic | Message Type | Description | QoS | Frequency |
|-------|-------------|------------|-----|-----------|
| `/lidar_hinson_se_1035/scan` | `sensor_msgs/LaserScan` | Hinson SE-1035 LiDAR scan | BEST_EFFORT | 10-30 Hz |
| `/lidar_hinson_de_4511/scan` | `sensor_msgs/LaserScan` | Hinson DE-4511 LiDAR scan | BEST_EFFORT | 10-30 Hz |
| `/tf03_180/range` | `sensor_msgs/Range` | Benewake TF03-180 distance | BEST_EFFORT | 1000 Hz |
| `/tf03_180/status` | `lidar_benewake_tf03_180_msgs/LidarStatus` | TF03-180 status | RELIABLE | 10 Hz |

### Energy and Power Management

| Topic | Message Type | Description | QoS | Frequency |
|-------|-------------|------------|-----|-----------|
| `/amc16/status` | `robot_custom_interfaces/AMC16Status` | AMC16 energy meter status | RELIABLE | 10 Hz |
| `/amc16z_fdk24/status` | `robot_custom_interfaces/AMC16ZFDK24Status` | AMC16Z-FDK24 status | RELIABLE | 10 Hz |
| `/battery_charger_epc602_status` | `robot_custom_interfaces/BatteryChargerStatus` | Battery charger status | RELIABLE | 1 Hz |

### Control Systems

| Topic | Message Type | Description | QoS | Frequency |
|-------|-------------|------------|-----|-----------|
| `/relay_controller/status` | `robot_custom_interfaces/N4ROC04RelayStatus` | Relay controller status | RELIABLE | 10 Hz |
| `/relay_controller/command` | `std_msgs/UInt8MultiArray` | Relay control commands | RELIABLE | On command |
| `/jpf4816_fan_controller/status` | `robot_custom_interfaces/JPF4816Status` | Fan controller status | RELIABLE | 10 Hz |
| `/plc10ioa12/status` | `robot_custom_interfaces/PLC10IOA12Status` | PLC I/O status | RELIABLE | 10 Hz |

### Motor Driver Topics

| Topic | Message Type | Description | QoS | Frequency |
|-------|-------------|------------|-----|-----------|
| `/motor_driver_ids830abs/command` | `std_msgs/Float32MultiArray` | IDS830ABS commands | RELIABLE | 50 Hz |
| `/ids830abs_status` | `robot_custom_interfaces/IDS830ABSStatus` | IDS830ABS status | RELIABLE | 50 Hz |
| `/left_wheel_golden_motor_status` | `robot_custom_interfaces/GoldenMotorEZA48400Status` | Left wheel motor status | RELIABLE | 50 Hz |
| `/right_wheel_golden_motor_status` | `robot_custom_interfaces/GoldenMotorEZA48400Status` | Right wheel motor status | RELIABLE | 50 Hz |

## DDS Domain Configuration

### Cyclone DDS Configuration

ROBOCON OS uses Cyclone DDS with specific domain and participant settings:

**Configuration File** (`CYCLONEDDS_URI`):

```xml
<!-- cyclonedds.xml -->
<CycloneDDS>
    <Domain>
        <Id>0</Id>
        <General>
            <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
            <AllowMulticast>true</AllowMulticast>
            <MaxMessageSize>65500</MaxMessageSize>
            <FragmentSize>63000</FragmentSize>
        </General>
        <Internal>
            <MinSocketBufferSize>64KB</MinSocketBufferSize>
            <MaxSocketBufferSize>16MB</MaxSocketBufferSize>
        </Internal>
    </Domain>
</CycloneDDS>
```

**Environment Variables:**
```bash
# Set Cyclone DDS as RMW implementation
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Configure Cyclone DDS
export CYCLONEDDS_URI=file:///path/to/cyclonedds.xml

# DDS Domain ID (default: 0)
export ROS_DOMAIN_ID=0
```

### DDS Domain Participant Discovery

Cyclone DDS automatically discovers participants on the same domain:

1. **Discovery Phase**:
   - Multicast announcements (if enabled)
   - Participant endpoint discovery
   - Topic discovery

2. **Matching Phase**:
   - Publisher/Subscriber matching
   - QoS compatibility checking
   - Endpoint pairing

3. **Communication Phase**:
   - Direct data exchange
   - Reliable/Best-effort transmission

### Multi-Machine Communication

For distributed systems (Raspberry Pi 5 + AI Computer):

```bash
# On Raspberry Pi 5
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# On AI Computer
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Topics automatically shared via network
```

## Message Type Definitions

### Standard ROS 2 Messages

ROBOCON uses standard ROS 2 message types from:

- `geometry_msgs` - Geometry primitives (Twist, Pose, Point)
- `sensor_msgs` - Sensor data (Image, LaserScan, Imu, JointState)
- `nav_msgs` - Navigation messages (Odometry, Path, OccupancyGrid)
- `std_msgs` - Standard primitives (String, Float32MultiArray)
- `tf2_msgs` - Transform messages

### ROBOCON Custom Messages

Custom message packages for ROBOCON-specific interfaces:

- `robocon_interfaces` - ROBOCON-specific messages
- `robot_custom_interfaces` - Robot hardware interfaces
- `lidar_benewake_tf03_180_msgs` - TF03-180 LiDAR messages
- `lidar_hinson_interfaces` - Hinson LiDAR interfaces

## Topic Naming Conventions

### ROS 2 Topic Namespace Rules

1. **Absolute Names**: Start with `/` (e.g., `/cmd_vel`)
2. **Relative Names**: No leading `/` (relative to node's namespace)
3. **Private Names**: Start with `~` (private to node)

### ROBOCON Naming Pattern

```
/robocon/<category>/<specific_topic>
```

Examples:
- `/robocon/motor/command`
- `/robocon/sensor/data`
- `/robocon/llm/decision`
- `/robocon/multi_robot/discovery`

## Monitoring and Debugging Topics

### List All Topics

```bash
# List all active topics
ros2 topic list

# List topics with types
ros2 topic list -t

# List topics with message count
ros2 topic list -v
```

### Inspect Topic Information

```bash
# Show topic info
ros2 topic info /cmd_vel

# Show message type
ros2 interface show geometry_msgs/msg/Twist

# Show topic frequency
ros2 topic hz /cmd_vel

# Echo topic messages
ros2 topic echo /cmd_vel
```

### Monitor DDS (Cyclone DDS)

```bash
# Install Cyclone DDS tools
sudo apt install cyclonedds-tools

# List DDS participants
ddsperf discovery

# Monitor DDS traffic
ddsperf monitor
```

### Visualize Topic Graph

```bash
# Generate topic graph
ros2 run rqt_graph rqt_graph

# Or use command line
ros2 run rqt_topic rqt_topic
```

## Performance Optimization

### Topic Performance Considerations

1. **Message Size**: Large messages (images) should use BEST_EFFORT QoS
2. **Publish Frequency**: Match sensor update rates
3. **Queue Depth**: Balance memory vs. latency
4. **QoS Selection**: Use appropriate reliability for data type

### Cyclone DDS Optimization

**For High-Frequency Sensors:**
```xml
<CycloneDDS>
    <Domain>
        <Internal>
            <SocketReceiveBufferSize>4MB</SocketReceiveBufferSize>
        </Internal>
    </Domain>
</CycloneDDS>
```

**For Wireless Networks:**
```python
# Use BEST_EFFORT for sensor data over wireless
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=5  # Small queue to reduce latency
)
```

## Troubleshooting

### Common Issues

#### Topic Not Publishing

```bash
# Check if node is running
ros2 node list

# Check if topic exists
ros2 topic list | grep <topic_name>

# Check node's publishers
ros2 node info <node_name>
```

#### QoS Mismatch

```
ERROR: Incompatible QoS profiles
```

**Solution**: Match QoS profiles between publisher and subscriber:
```python
# Publisher
pub = node.create_publisher(Twist, '/cmd_vel', qos_profile_sensor_data)

# Subscriber (must match)
sub = node.create_subscription(Twist, '/cmd_vel', callback, qos_profile_sensor_data)
```

#### DDS Discovery Issues

```bash
# Verify DDS domain ID matches
echo $ROS_DOMAIN_ID

# Check Cyclone DDS configuration
echo $CYCLONEDDS_URI

# Verify network connectivity
ping <other_machine_ip>
```

### Debug Commands

```bash
# Monitor topic bandwidth
ros2 topic bw /camera/image_raw

# Check topic latency
ros2 topic delay /cmd_vel

# Show topic statistics
ros2 topic stats /odom
```

## Best Practices

1. **Use Appropriate QoS**: Match QoS to data characteristics
2. **Topic Namespacing**: Use `/robocon/` prefix for ROBOCON topics
3. **Message Types**: Prefer standard ROS 2 messages when possible
4. **Monitoring**: Regularly check topic frequency and bandwidth
5. **Network Optimization**: Configure Cyclone DDS for your network
6. **Error Handling**: Check QoS compatibility before publishing

## References

- [ROS 2 Topics Documentation](https://docs.ros.org/en/jazzy/Concepts/About-Topics.html)
- [ROS 2 QoS Documentation](https://docs.ros.org/en/jazzy/Concepts/About-Quality-of-Service-Settings.html)
- [Cyclone DDS Documentation](https://github.com/eclipse-cyclonedds/cyclonedds)
- [DDS Specification](https://www.omg.org/spec/DDS/)
- [ROS 2 Jazzy API Reference](https://docs.ros.org/en/jazzy/)

## Next Steps

- [Nodes and Topics](../ros2/nodes-and-topics.md) - Creating ROS 2 nodes
- [Publishers and Subscribers](../ros2/publishers-and-subscribers.md) - Working with topics
- [Services and Actions](../ros2/services-and-actions.md) - Synchronous communication
- [ROS 2 Overview](../architecture/ros2-overview.md) - ROS 2 architecture
- [Sensor Interfaces](sensor-interfaces.md) - Sensor topic details

