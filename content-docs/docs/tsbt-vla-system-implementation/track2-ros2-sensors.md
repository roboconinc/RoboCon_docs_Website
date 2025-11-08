# Track 2 Implementation: ROS 2 Sensor Integration

This document covers the implementation of Track 2 (Sensor to Text) using **ROS 2** topics and messages.

## Overview

The Sensor to Text track subscribes to ROS 2 sensor topics, aggregates temporal data, and converts it into structured text logs that provide context to the LLM for safety checks, energy management, and motion feasibility reasoning.

## ROS 2 Integration

### Node Structure

**Package**: `robocon_sensor_text`

**Node Name**: `sensor_text_node`

### Python Implementation

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String
from sensor_msgs.msg import Temperature, JointState
from nav_msgs.msg import Odometry
from collections import deque
from datetime import datetime

class SensorTextNode(Node):
    def __init__(self):
        super().__init__('sensor_text_node')
        
        # Configuration
        self.window_size = self.declare_parameter('window_size', 10).value
        self.update_rate = self.declare_parameter('update_rate', 1.0).value
        
        # Temporal buffer
        self.sensor_buffer = {
            'arm_joint_torque': deque(maxlen=self.window_size),
            'battery_temp': deque(maxlen=self.window_size),
            'battery_voltage': deque(maxlen=self.window_size),
            'energy_consumption': deque(maxlen=self.window_size),
            'laser_distance': deque(maxlen=self.window_size),
            'motor_current': deque(maxlen=self.window_size),
        }
        
        # Subscriptions
        self.subscriptions = {}
        self.subscriptions['arm_joint_torque'] = self.create_subscription(
            JointState,
            '/arm/joint_torque',
            self.arm_joint_torque_callback,
            10
        )
        
        self.subscriptions['battery_temp'] = self.create_subscription(
            Temperature,
            '/battery/temperature',
            self.battery_temp_callback,
            10
        )
        
        self.subscriptions['battery_voltage'] = self.create_subscription(
            Float32,
            '/battery/voltage',
            self.battery_voltage_callback,
            10
        )
        
        self.subscriptions['energy_consumption'] = self.create_subscription(
            Float32,
            '/energy/consumption',
            self.energy_consumption_callback,
            10
        )
        
        self.subscriptions['laser_distance'] = self.create_subscription(
            Float32MultiArray,
            '/laser/distance',
            self.laser_distance_callback,
            10
        )
        
        self.subscriptions['motor_current'] = self.create_subscription(
            Float32MultiArray,
            '/motor/current',
            self.motor_current_callback,
            10
        )
        
        # Publisher
        self.sensor_text_pub = self.create_publisher(
            String,
            '/tsbt_vla/sensor_text',
            10
        )
        
        # Timer for periodic publishing
        self.timer = self.create_timer(1.0 / self.update_rate, self.publish_sensor_text)
        
        self.get_logger().info('Sensor Text Node Started')
    
    def arm_joint_torque_callback(self, msg):
        # Extract torques from joint states
        torques = msg.effort  # Effort represents torque
        self.sensor_buffer['arm_joint_torque'].append({
            'timestamp': datetime.now().isoformat(),
            'values': list(torques)
        })
    
    def battery_temp_callback(self, msg):
        self.sensor_buffer['battery_temp'].append({
            'timestamp': datetime.now().isoformat(),
            'value': msg.temperature
        })
    
    def battery_voltage_callback(self, msg):
        self.sensor_buffer['battery_voltage'].append({
            'timestamp': datetime.now().isoformat(),
            'value': msg.data
        })
    
    def energy_consumption_callback(self, msg):
        self.sensor_buffer['energy_consumption'].append({
            'timestamp': datetime.now().isoformat(),
            'value': msg.data
        })
    
    def laser_distance_callback(self, msg):
        self.sensor_buffer['laser_distance'].append({
            'timestamp': datetime.now().isoformat(),
            'values': list(msg.data)
        })
    
    def motor_current_callback(self, msg):
        self.sensor_buffer['motor_current'].append({
            'timestamp': datetime.now().isoformat(),
            'values': list(msg.data)
        })
    
    def publish_sensor_text(self):
        # Format sensor data as text
        sensor_text = self.format_sensor_text()
        
        # Publish
        msg = String()
        msg.data = sensor_text
        self.sensor_text_pub.publish(msg)
    
    def format_sensor_text(self):
        """Convert sensor buffer to text format"""
        lines = []
        lines.append(f"[Timestamp] = {datetime.now().isoformat()}")
        
        # Arm Joint Torque
        if self.sensor_buffer['arm_joint_torque']:
            latest = self.sensor_buffer['arm_joint_torque'][-1]
            values = latest['values']
            values_str = ', '.join([f'{v:.1f}Nm' for v in values])
            lines.append(f"[ArmJointTorque] = [{values_str}]")
        
        # Battery Temperature
        if self.sensor_buffer['battery_temp']:
            latest = self.sensor_buffer['battery_temp'][-1]
            lines.append(f"[BatteryTemp] = {latest['value']:.1f}°C")
        
        # Battery Voltage
        if self.sensor_buffer['battery_voltage']:
            latest = self.sensor_buffer['battery_voltage'][-1]
            lines.append(f"[BatteryVoltage] = {latest['value']:.1f}V")
        
        # Energy Consumption
        if self.sensor_buffer['energy_consumption']:
            latest = self.sensor_buffer['energy_consumption'][-1]
            lines.append(f"[EnergyConsumption] = {latest['value']:.1f}kW")
        
        # Laser Distance
        if self.sensor_buffer['laser_distance']:
            latest = self.sensor_buffer['laser_distance'][-1]
            values = latest['values']
            values_str = ', '.join([f'{v:.2f}m' for v in values])
            lines.append(f"[LaserDistance] = [{values_str}]")
        
        # Motor Current
        if self.sensor_buffer['motor_current']:
            latest = self.sensor_buffer['motor_current'][-1]
            values = latest['values']
            values_str = ', '.join([f'{v:.1f}A' for v in values])
            lines.append(f"[MotorCurrent] = [{values_str}]")
        
        # System Status
        lines.append("[SystemStatus] = normal")
        
        # Energy Remaining (computed from battery voltage)
        if self.sensor_buffer['battery_voltage']:
            voltage = self.sensor_buffer['battery_voltage'][-1]['value']
            # Simple estimation (actual calculation would be more complex)
            energy_remaining = min(100, max(0, (voltage / 52.0) * 100))
            lines.append(f"[EnergyRemaining] = {energy_remaining:.0f}%")
        
        return '\n'.join(lines)

def main(args=None):
    rclpy.init(args=args)
    node = SensorTextNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### C++ Implementation

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <deque>
#include <sstream>

class SensorTextNode : public rclcpp::Node {
public:
    SensorTextNode() : Node("sensor_text_node") {
        window_size_ = this->declare_parameter("window_size", 10).as_int();
        update_rate_ = this->declare_parameter("update_rate", 1.0).as_double();
        
        // Subscriptions
        arm_joint_torque_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/arm/joint_torque", 10,
            std::bind(&SensorTextNode::arm_joint_torque_callback, this, std::placeholders::_1)
        );
        
        battery_temp_sub_ = this->create_subscription<sensor_msgs::msg::Temperature>(
            "/battery/temperature", 10,
            std::bind(&SensorTextNode::battery_temp_callback, this, std::placeholders::_1)
        );
        
        battery_voltage_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/battery/voltage", 10,
            std::bind(&SensorTextNode::battery_voltage_callback, this, std::placeholders::_1)
        );
        
        // Publisher
        sensor_text_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/tsbt_vla/sensor_text", 10
        );
        
        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_)),
            std::bind(&SensorTextNode::publish_sensor_text, this)
        );
    }

private:
    void arm_joint_torque_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Store in buffer
        if (arm_joint_torque_buffer_.size() >= window_size_) {
            arm_joint_torque_buffer_.pop_front();
        }
        arm_joint_torque_buffer_.push_back(*msg);
    }
    
    void battery_temp_callback(const sensor_msgs::msg::Temperature::SharedPtr msg) {
        if (battery_temp_buffer_.size() >= window_size_) {
            battery_temp_buffer_.pop_front();
        }
        battery_temp_buffer_.push_back(*msg);
    }
    
    void battery_voltage_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        if (battery_voltage_buffer_.size() >= window_size_) {
            battery_voltage_buffer_.pop_front();
        }
        battery_voltage_buffer_.push_back(*msg);
    }
    
    void publish_sensor_text() {
        std::string sensor_text = format_sensor_text();
        
        auto msg = std_msgs::msg::String();
        msg.data = sensor_text;
        sensor_text_pub_->publish(msg);
    }
    
    std::string format_sensor_text() {
        std::stringstream ss;
        
        // Timestamp
        auto now = this->now();
        ss << "[Timestamp] = " << now.seconds() << "\n";
        
        // Arm Joint Torque
        if (!arm_joint_torque_buffer_.empty()) {
            auto latest = arm_joint_torque_buffer_.back();
            ss << "[ArmJointTorque] = [";
            for (size_t i = 0; i < latest.effort.size(); ++i) {
                if (i > 0) ss << ", ";
                ss << latest.effort[i] << "Nm";
            }
            ss << "]\n";
        }
        
        // Battery Temperature
        if (!battery_temp_buffer_.empty()) {
            auto latest = battery_temp_buffer_.back();
            ss << "[BatteryTemp] = " << latest.temperature << "°C\n";
        }
        
        // Battery Voltage
        if (!battery_voltage_buffer_.empty()) {
            auto latest = battery_voltage_buffer_.back();
            ss << "[BatteryVoltage] = " << latest.data << "V\n";
        }
        
        // System Status
        ss << "[SystemStatus] = normal\n";
        
        return ss.str();
    }
    
    int window_size_;
    double update_rate_;
    
    std::deque<sensor_msgs::msg::JointState> arm_joint_torque_buffer_;
    std::deque<sensor_msgs::msg::Temperature> battery_temp_buffer_;
    std::deque<std_msgs::msg::Float32> battery_voltage_buffer_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_joint_torque_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr battery_temp_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_voltage_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sensor_text_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

## ROS 2 Topics

### Input Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/arm/joint_torque` | `sensor_msgs/JointState` | Arm joint torques |
| `/battery/temperature` | `sensor_msgs/Temperature` | Battery temperature |
| `/battery/voltage` | `std_msgs/Float32` | Battery voltage |
| `/energy/consumption` | `std_msgs/Float32` | Energy consumption (kW) |
| `/laser/distance` | `std_msgs/Float32MultiArray` | Laser distance measurements |
| `/motor/current` | `std_msgs/Float32MultiArray` | Motor current values |
| `/odom` | `nav_msgs/Odometry` | Odometry data (optional) |
| `/imu/data` | `sensor_msgs/Imu` | IMU data (optional) |

### Output Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/tsbt_vla/sensor_text` | `std_msgs/String` | Formatted sensor text |

### ROS 2 Services

| Service | Request | Response | Description |
|---------|---------|----------|-------------|
| `/tsbt_vla/get_sensor_text` | Empty | `std_msgs/String` | Request current sensor text |

## Sensor Text Format

### Output Example

```
[Timestamp] = 2025-01-15T10:23:45.123Z
[ArmJointTorque] = [55.4Nm, 54.8Nm, 56.0Nm]
[BatteryTemp] = 52.3°C
[BatteryVoltage] = 48.2V
[EnergyConsumption] = 2.3kW
[LaserDistance] = [0.5m, 0.49m, 0.48m]
[MotorCurrent] = [12.3A, 11.8A, 13.1A]
[SystemStatus] = normal
[EnergyRemaining] = 78%
```

## Configuration

```yaml
sensor_text:
  window_size: 10  # Number of samples to buffer
  update_rate: 1.0  # Hz - Update frequency
  
  sensors:
    arm_joint_torque:
      enabled: true
      topic: "/arm/joint_torque"
    battery_temp:
      enabled: true
      topic: "/battery/temperature"
    battery_voltage:
      enabled: true
      topic: "/battery/voltage"
    energy_consumption:
      enabled: true
      topic: "/energy/consumption"
    laser_distance:
      enabled: true
      topic: "/laser/distance"
    motor_current:
      enabled: true
      topic: "/motor/current"
```

## Integration with LLM

The Sensor Text output is integrated into LLM prompts:

```python
# In LLM processing node
sensor_text_sub = self.create_subscription(
    String,
    '/tsbt_vla/sensor_text',
    self.sensor_text_callback,
    10
)

def sensor_text_callback(self, msg):
    self.current_sensor_text = msg.data
    # Include in LLM prompt
    prompt = f"""
High-Level Action: {high_level_action}

Text World:
{text_world}

Sensor Text:
{self.current_sensor_text}

Generate a Behavior Tree XML subtree.
"""
```

## Launch File

```xml
<launch>
    <node pkg="robocon_sensor_text" 
          exec="sensor_text_node" 
          name="sensor_text_node">
        <param name="window_size" value="10"/>
        <param name="update_rate" value="1.0"/>
    </node>
</launch>
```

## Next Steps

- [Track 3: ROS 2 User Input](track3-ros2-user-input.md) - User input processing
- [Track 4: DeepSeek LLM](track4-deepseek-llm.md) - LLM integration
- [Sensor to Text Track](../../tsbt-vla-system/sensor-to-text.md) - Overall track overview

