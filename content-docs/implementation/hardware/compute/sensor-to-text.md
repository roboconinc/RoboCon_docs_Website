# Track: Sensor to Text

This track converts temporal sensor data into text logs (Sensor Text) that the Large Language Model uses for safety checks, energy management, and motion feasibility reasoning.

## Overview

The Sensor to Text track monitors various sensor sources, aggregates temporal data, and converts it into structured text format that provides context to the LLM about:
- System health (battery, temperature, energy)
- Motion feasibility (torque, current, encoder states)
- Safety status (sensor readings, limits, warnings)
- Operational constraints (power levels, thermal limits)

## Processing Flow

``mermaid
flowchart TD
    subgraph Sources[Sensor Sources]
        Energy[Energy Sensor]
        Temp[Internal Temperature]
        Battery[Battery Voltage]
        Torque[Arm Joint Torque]
        Current[Motor Current]
        Laser[Laser Distance]
        Other[Other Sensors]
    end
    
    Energy --> Topics[Sensor Topics<br/>ROS 2 Topics]
    Temp --> Topics
    Battery --> Topics
    Torque --> Topics
    Current --> Topics
    Laser --> Topics
    Other --> Topics
    
    Topics --> Converter[Text Converter<br/>â€¢ Temporal Logs<br/>â€¢ Key-Value Pairs<br/>â€¢ Timestamped]
    
    Converter --> SensorText[Sensor Text<br/>Sent to LLM]
    
    style Sources fill:#e1f5ff
    style Topics fill:#fff4e1
    style Converter fill:#ffe1f5
    style SensorText fill:#f5e1ff
``

## Sensor Categories

### Path Optimization Sensors

These sensors help the LLM optimize paths and plan energy-efficient actions:

#### Energy Sensor

**Purpose**: Monitor power consumption and energy usage

**Sensor Topic**: `/energy/sensor` (custom message)

**Data**: 
- Current power draw (Watts)
- Total energy consumed (kWh)
- Energy efficiency metrics

**Text Output**:
``
[EnergyConsumption] = 2.3kW
[TotalEnergyUsed] = 15.7kWh
[Efficiency] = 78%
``

#### Internal Temperature

**Purpose**: Monitor system thermal state

**Sensor Topic**: `/temperature/internal` (sensor_msgs/Temperature)

**Data**:
- CPU temperature
- GPU temperature
- Controller temperature
- Motor temperatures

**Text Output**:
``
[InternalTemp] = 52.3Â°C
[ControllerTemp] = 48.7Â°C
[ThermalStatus] = normal
``

#### Battery Voltage

**Purpose**: Monitor battery state and power availability

**Sensor Topic**: `/battery/voltage` (std_msgs/Float32)

**Data**:
- Battery voltage (Volts)
- Battery state of charge (%)
- Charge/discharge rate
- Health status

**Text Output**:
``
[BatteryVoltage] = 48.2V
[BatterySoC] = 78%
[ChargeRate] = 0.5A
[BatteryHealth] = good
``

### Interaction Sensors

#### Microphone

**Purpose**: Audio input for voice commands

**Sensor Topic**: `/audio/microphone` (audio_msgs/AudioData)

**Note**: Audio data is processed through Voice-to-Text in the User Input to Action track, not Sensor to Text.

#### Phone App / Tablet / Laptop / Desktop

**Purpose**: User interface for command input

**Note**: These interfaces provide chat/text input processed in the User Input to Action track, not Sensor to Text.

### Localization Sensors

These sensors are primarily used in the 3D World to Text track but may also provide data for Sensor Text:

#### LIDAR

**Purpose**: Spatial mapping and localization

**Sensor Topic**: `/lidar/scan` (sensor_msgs/LaserScan)

**Sensor Text Usage**: Distance measurements, obstacle proximity

#### Depth Camera

**Purpose**: Depth perception and visual localization

**Sensor Topic**: `/depth/image_raw` (sensor_msgs/Image)

**Sensor Text Usage**: Depth statistics, range information

#### GPS

**Purpose**: Global positioning

**Sensor Topic**: `/gps/position` (sensor_msgs/NavSatFix)

**Sensor Text Usage**: Location coordinates, accuracy metrics

### Odometry Sensors

These sensors are primarily used in the 3D World to Text track but provide motion data:

#### Wheel Encoders

**Purpose**: Base motion tracking

**Sensor Topic**: `/odom` (nav_msgs/Odometry)

**Sensor Text Usage**: Speed, distance traveled, motion status

#### Motor Encoders

**Purpose**: Joint position tracking

**Sensor Topic**: `/joint_states` (sensor_msgs/JointState)

**Sensor Text Usage**: Joint positions, velocities, torques

#### IMUs

**Purpose**: Orientation and acceleration sensing

**Sensor Topic**: `/imu/data` (sensor_msgs/Imu)

**Sensor Text Usage**: Orientation, acceleration, motion constraints

### Motion Feasibility Sensors

#### Arm Joint Torque

**Purpose**: Monitor joint loading and motion feasibility

**Sensor Topic**: `/arm/joint_torque` (sensor_msgs/JointState)

**Data**: Torque for each joint (Newton-meters)

**Text Output**:
``
[ArmJointTorque] = [55.4Nm, 54.8Nm, 56.0Nm, 52.1Nm, 53.7Nm]
[MaxTorque] = 60Nm
[TorqueLimit] = 95%
``

#### Motor Current

**Purpose**: Monitor motor loading and power consumption

**Sensor Topic**: `/motor/current` (Float32MultiArray)

**Data**: Current for each motor (Amperes)

**Text Output**:
``
[MotorCurrent] = [12.3A, 11.8A, 13.1A, 12.5A]
[MaxCurrent] = 15A
[CurrentLimit] = 82%
``

#### Laser Distance

**Purpose**: Proximity sensing and collision avoidance

**Sensor Topic**: `/laser/distance` (Float32MultiArray)

**Data**: Distance measurements (meters)

**Text Output**:
``
[LaserDistance] = [0.5m, 0.49m, 0.48m, 0.52m]
[MinDistance] = 0.48m
[SafetyThreshold] = 0.3m
``

## Sensor Text Format

Sensor Text is formatted as temporal logs with key-value pairs:

**Example Sensor Text**:
``
[Timestamp] = 2025-01-15T10:23:45.123Z
[ArmJointTorque] = [55.4Nm, 54.8Nm, 56.0Nm]
[BatteryTemp] = 52.3Â°C
[BatteryVoltage] = 48.2V
[EnergyConsumption] = 2.3kW
[LaserDistance] = [0.5m, 0.49m, 0.48m]
[MotorCurrent] = [12.3A, 11.8A, 13.1A]
[SystemStatus] = normal
[EnergyRemaining] = 78%
[ThermalStatus] = normal
[SafetyStatus] = all_systems_go
``

**Characteristics**:
- **Timestamped entries**: Each log entry includes timestamp
- **Temporal window**: Last N ticks (configurable, typically 10-20 ticks)
- **Structured format**: Key-value pairs for easy parsing
- **Context-aware**: Only includes relevant sensors for current operation

## Text Converter Process

The Text Converter:

1. **Subscribes** to relevant sensor topics
2. **Buffers** temporal data (last N ticks)
3. **Formats** data into key-value text pairs
4. **Filters** irrelevant or redundant information
5. **Outputs** Sensor Text string

**Configuration**:
``yaml
tsbt_vla:
  sensor_text:
    window_size: 10  # Last N ticks to include
    update_rate: 1.0  # Hz - Update frequency
    
    include_torque: true
    include_temperature: true
    include_distance: true
    include_battery: true
    include_energy: true
    
    filters:
      min_relevance: 0.1  # Minimum relevance threshold
      max_entries: 20  # Maximum log entries
``

## Integration with LLM

Sensor Text provides critical context for the LLM to:

1. **Safety Checks**: 
   - Verify system health before executing actions
   - Check thermal limits
   - Validate battery levels
   - Monitor proximity sensors

2. **Energy Management**:
   - Optimize paths for energy efficiency
   - Plan operations within power budgets
   - Schedule charging operations

3. **Motion Feasibility**:
   - Check if joints can handle required torques
   - Verify motor current limits
   - Validate motion constraints

4. **Operational Constraints**:
   - Respect temperature limits
   - Maintain battery thresholds
   - Follow safety margins

**Example LLM Prompt Section**:
``
Sensor Text:
[ArmJointTorque] = [55.4Nm, 54.8Nm, 56.0Nm]
[BatteryTemp] = 52.3Â°C
[EnergyConsumption] = 2.3kW
[SystemStatus] = normal
[EnergyRemaining] = 78%

Generate a behavior tree that completes the task while:
- Respecting joint torque limits (max 60Nm)
- Maintaining battery temperature below 60Â°C
- Staying within 3kW power budget
- Ensuring energy remaining stays above 50%
``

**See**: [Large Language Model Processing](llm-processing.md) for how Sensor Text is integrated into LLM prompts.

## ROS 2 Topics

**Input Topics** (Sensor Sources):
- `/energy/sensor` - Energy consumption data
- `/temperature/internal` - Internal temperature
- `/battery/voltage` - Battery state
- `/arm/joint_torque` - Joint torques
- `/motor/current` - Motor currents
- `/laser/distance` - Proximity sensors
- `/odom` - Odometry data
- `/imu/data` - IMU data
- `/joint_states` - Joint states

**Output Topics**:
- `/tsbt_vla/sensor_text` (std_msgs/String) - Sensor Text output

**Services**:
- `/tsbt_vla/get_sensor_text` - Request current Sensor Text

## Configuration Examples

### Minimal Sensor Text (Safety Only)
``yaml
sensor_text:
  window_size: 5
  include_torque: false
  include_temperature: true
  include_distance: true
  include_battery: true
  include_energy: false
``

### Comprehensive Sensor Text (Full Monitoring)
``yaml
sensor_text:
  window_size: 20
  include_torque: true
  include_temperature: true
  include_distance: true
  include_battery: true
  include_energy: true
  include_localization: true
  include_odometry: true
``

## Next Steps

- [3D World to Text Track](3d-world-to-text.md) - Scene understanding
- [User Input to Action Track](user-input-to-action.md) - Natural language input
- [Large Language Model Processing](llm-processing.md) - How Sensor Text is used by LLM
- [Behavior Tree Node Reference](../../behavior-tree-node-reference.md) - Node reference

