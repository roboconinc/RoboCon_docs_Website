# Behavior Tree Low-Level Nodes

Low-level nodes represent hard-coded functionality or close-to-hard-coded operations. These nodes directly interface with hardware actuators and sensors, providing precise control over robot components. They execute deterministic operations with minimal inference or planning.

> **Note on Robot Compatibility**: Low-level nodes are hardware-specific. Each node section indicates which robots support that functionality. Nodes may not be available on all robot platforms.

## Boom Control Nodes

> **Robot Compatibility**: Mini Crane

### `Boom_Extend`

Extends the boom to a specified length. This node directly controls the boom extension mechanism, sending commands to the hydraulic or electromechanical actuator.

**Description**: Controls the linear extension of the crane boom. The extension value is specified in meters from the base position. This is a synchronous action that blocks until the boom reaches the target extension or times out.

**Controls**: 
- Boom extension actuator
- Extension position feedback
- Safety interlocks

**Arguments:**
- `Extention` (Meters) - Extension distance in meters
  - Example: `10`

**XML Example:**
```xml
<Boom_Extend Extention="10"/>
```

**C++ Example (BehaviorTree.CPP v4.6+):**
```cpp
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>

class BoomExtendNode : public BT::StatefulActionNode
{
public:
    BoomExtendNode(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("boom_extend_node");
        cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
            "/boom/extend_cmd", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<double>("Extention")};
    }

    BT::NodeStatus onStart() override
    {
        double extension;
        if (!getInput("Extention", extension))
        {
            RCLCPP_ERROR(node_->get_logger(), "Missing input: Extention");
            return BT::NodeStatus::FAILURE;
        }

        // Publish extension command
        std_msgs::msg::Float64 msg;
        msg.data = extension;
        cmd_pub_->publish(msg);

        RCLCPP_INFO(node_->get_logger(), 
            "Boom extension command: %.2f meters", extension);
        
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        // Check if boom has reached target position
        // This would typically check feedback from the boom position sensor
        // For this example, we'll simulate completion after a delay
        
        // In real implementation, check actual position feedback
        if (/* position reached */) {
            return BT::NodeStatus::SUCCESS;
        }
        
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        RCLCPP_WARN(node_->get_logger(), "Boom extension halted");
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_pub_;
};

// Register with factory
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<BoomExtendNode>("Boom_Extend");
}
```

---

### `Boom_Lift`

Lifts the boom by a specified extension distance. Controls the vertical lifting mechanism of the boom.

**Description**: Controls the boom's vertical elevation. The lift extension is specified in meters from the current position. This action synchronously moves the boom up or down.

**Controls**:
- Boom lift actuator (hydraulic or electric)
- Vertical position feedback
- Load monitoring sensors

**Arguments:**
- `Extention` (Meters) - Lift extension distance in meters
  - Example: `0.4`

**XML Example:**
```xml
<Boom_Lift Extention="0.4"/>
```

**C++ Example (BehaviorTree.CPP v4.6+):**
```cpp
class BoomLiftNode : public BT::StatefulActionNode
{
public:
    BoomLiftNode(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("boom_lift_node");
        cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
            "/boom/lift_cmd", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<double>("Extention")};
    }

    BT::NodeStatus onStart() override
    {
        double extension;
        if (!getInput("Extention", extension))
        {
            return BT::NodeStatus::FAILURE;
        }

        std_msgs::msg::Float64 msg;
        msg.data = extension;
        cmd_pub_->publish(msg);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        // Check position feedback
        if (/* lift position reached */) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_pub_;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<BoomLiftNode>("Boom_Lift");
}
```

---

### `Boom_Slewing`

Rotates the boom by a specified angle around its base. Controls the slewing (rotation) mechanism.

**Description**: Rotates the boom horizontally around the crane base. The rotation angle is specified in degrees, with positive values typically representing clockwise rotation (may vary by configuration).

**Controls**:
- Slewing motor/actuator
- Rotation encoder feedback
- Slewing brake mechanism

**Arguments:**
- `Rotation` (Degrees) - Rotation angle in degrees
  - Example: `270`

**XML Example:**
```xml
<Boom_Slewing Rotation="270"/>
```

**C++ Example (BehaviorTree.CPP v4.6+):**
```cpp
class BoomSlewingNode : public BT::StatefulActionNode
{
public:
    BoomSlewingNode(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("boom_slewing_node");
        cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
            "/boom/slew_cmd", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<double>("Rotation")};
    }

    BT::NodeStatus onStart() override
    {
        double rotation;
        if (!getInput("Rotation", rotation))
        {
            return BT::NodeStatus::FAILURE;
        }

        std_msgs::msg::Float64 msg;
        msg.data = rotation * M_PI / 180.0; // Convert to radians
        cmd_pub_->publish(msg);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (/* rotation completed */) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_pub_;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<BoomSlewingNode>("Boom_Slewing");
}
```

---


## End Effector (Sucker) Attachment Nodes

> **Robot Compatibility**: RoboCon Mini Crane
> 
> **Note**: End Effector - Sucker Attachment Nodes are for the sucker attachment for the RoboCon Mini Crane.

### `EndEffectorSuckerExtend`

Extends the sucker end effector linearly.

**Description**: Controls the linear extension of a sucker-type end effector. This is typically used for reaching objects before engaging suction.

**Controls**:
- End effector extension actuator
- Extension position feedback

**Arguments:**
- `Extention` (Meters) - Extension distance in meters
  - Example: `0.4`

**XML Example:**
```xml
<EndEffectorSuckerExtend Extention="0.4"/>
```

**C++ Example (BehaviorTree.CPP v4.6+):**
```cpp
class EndEffectorSuckerExtendNode : public BT::StatefulActionNode
{
public:
    EndEffectorSuckerExtendNode(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("sucker_extend_node");
        cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
            "/end_effector/extend_cmd", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<double>("Extention")};
    }

    BT::NodeStatus onStart() override
    {
        double extension;
        if (!getInput("Extention", extension))
        {
            return BT::NodeStatus::FAILURE;
        }

        std_msgs::msg::Float64 msg;
        msg.data = extension;
        cmd_pub_->publish(msg);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (/* extension complete */) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_pub_;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<EndEffectorSuckerExtendNode>("EndEffectorSuckerExtend");
}
```

---

### `EndEffectorSuckerRotate`

Rotates the sucker end effector around its axis.

**Description**: Rotates the sucker to orient it properly before engaging with an object.

**Controls**:
- End effector rotation motor
- Rotation encoder feedback

**Arguments:**
- `Rotation` (Degrees) - Rotation angle in degrees
  - Example: `270`

**XML Example:**
```xml
<EndEffectorSuckerRotate Rotation="270"/>
```

---

### `EndEffectorSuckerSlew`

Slews (rotates) the sucker end effector around a different axis.

**Description**: Provides additional rotational control for the sucker end effector, typically around a perpendicular axis to the main rotation.

**Controls**:
- Slew rotation actuator
- Slew angle feedback

**Arguments:**
- `Rotation` (Degrees) - Slew rotation angle in degrees
  - Example: `270`

**XML Example:**
```xml
<EndEffectorSuckerSlew Rotation="270"/>
```

---

### `EndEffectorSuckerSuck`

Controls sucker suction on/off state.

**Description**: Activates or deactivates the vacuum/suction system of the end effector. This is typically a simple on/off control.

**Controls**:
- Vacuum pump/suction system
- Pressure sensors
- Suction valves

**Arguments:**
- `Suction` (Integer) - Suction state (1 = on, 0 = off)
  - Example: `1`

**XML Example:**
```xml
<EndEffectorSuckerSuck Suction="1"/>
```

**C++ Example (BehaviorTree.CPP v4.6+):**
```cpp
class EndEffectorSuckerSuckNode : public BT::SyncActionNode
{
public:
    EndEffectorSuckerSuckNode(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("sucker_suck_node");
        cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
            "/end_effector/suction_cmd", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<int>("Suction")};
    }

    BT::NodeStatus tick() override
    {
        int suction;
        if (!getInput("Suction", suction))
        {
            return BT::NodeStatus::FAILURE;
        }

        std_msgs::msg::Bool msg;
        msg.data = (suction == 1);
        cmd_pub_->publish(msg);

        RCLCPP_INFO(node_->get_logger(), 
            "Suction %s", msg.data ? "ON" : "OFF");

        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cmd_pub_;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<EndEffectorSuckerSuckNode>("EndEffectorSuckerSuck");
}
```

---

### `EndEffectorSuckerTilt`

Tilts the sucker end effector.

**Description**: Adjusts the tilt angle of the sucker to better engage with surfaces at various angles.

**Controls**:
- Tilt actuator
- Tilt angle feedback

**Arguments:**
- `Extention` (Meters) - Tilt extension distance in meters
  - Example: `0.4`

**XML Example:**
```xml
<EndEffectorSuckerTilt Extention="0.4"/>
```

---

## Outrigger Nodes

> **Robot Compatibility**: RoboCon Mini Crane
> 
> **Note**: Outrigger nodes work only for the RoboCon Mini Crane.

### `Outrigger_Section2_Extend`

Extends outrigger section 2 to a specified length.

**Description**: Extends the second section of one of the crane's outriggers to provide stability. Multiple outriggers can be controlled independently.

**Controls**:
- Outrigger extension actuator
- Extension position feedback
- Load sensors on outrigger foot

**Arguments:**
- `Extention` (Meters) - Extension distance in meters
  - Example: `0.35`
- `OutriggerID` (Integer) - Outrigger identifier (typically 1-4)
  - Example: `1`

**XML Example:**
```xml
<Outrigger_Section2_Extend Extention="0.35" OutriggerID="1"/>
```

**C++ Example (BehaviorTree.CPP v4.6+):**
```cpp
#include <std_msgs/msg/int32.hpp>

class OutriggerExtendNode : public BT::StatefulActionNode
{
public:
    OutriggerExtendNode(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("outrigger_extend_node");
        cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3>(
            "/outrigger/extend_cmd", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("Extention"),
            BT::InputPort<int>("OutriggerID")
        };
    }

    BT::NodeStatus onStart() override
    {
        double extension;
        int outrigger_id;
        
        if (!getInput("Extention", extension) || 
            !getInput("OutriggerID", outrigger_id))
        {
            return BT::NodeStatus::FAILURE;
        }

        geometry_msgs::msg::Vector3 msg;
        msg.x = extension;        // Extension distance
        msg.y = outrigger_id;     // Outrigger ID
        msg.z = 0.0;
        
        cmd_pub_->publish(msg);

        RCLCPP_INFO(node_->get_logger(), 
            "Extending outrigger %d to %.2f meters", outrigger_id, extension);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (/* extension complete */) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr cmd_pub_;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<OutriggerExtendNode>("Outrigger_Section2_Extend");
}
```

---

### `Outrigger_FootTilt`

Tilts an outrigger foot to compensate for uneven terrain.

**Description**: Adjusts the tilt angle of an outrigger foot to maintain full ground contact on uneven surfaces.

**Controls**:
- Outrigger foot tilt actuator
- Tilt angle feedback
- Ground contact sensors

**Arguments:**
- `Extention` (Meters) - Tilt extension distance in meters
  - Example: `0.01`
- `OutriggerID` (Integer) - Outrigger identifier
  - Example: `1`

**XML Example:**
```xml
<Outrigger_FootTilt Extention="0.01" OutriggerID="1"/>
```

---

### `Outrigger_Rotate`

Rotates an outrigger.

**Description**: Rotates an outrigger around its mounting point to position it for deployment.

**Controls**:
- Outrigger rotation actuator
- Rotation angle feedback

**Arguments:**
- `Rotation` (Degrees) - Rotation angle in degrees
  - Example: `90`
- `OutriggerID` (Integer) - Outrigger identifier
  - Example: `1`

**XML Example:**
```xml
<Outrigger_Rotate Rotation="90" OutriggerID="1"/>
```

---

### `Outrigger_Section1_Flex`

Flexes outrigger section 1.

**Description**: Flexes or adjusts the first section of a multi-stage outrigger.

**Controls**:
- Section 1 extension actuator
- Section engagement locks
- Position feedback

**Arguments:**
- `Extention` (Meters) - Extension distance in meters
  - Example: `0.35`
- `OutriggerID` (Integer) - Outrigger identifier
  - Example: `1`

**XML Example:**
```xml
<Outrigger_Section1_Flex Extention="0.35" OutriggerID="1"/>
```

---

### `Outrigger_Section2_Flex`

Flexes outrigger section 2.

**Description**: Flexes or adjusts the second section of a multi-stage outrigger.

**Controls**:
- Section 2 extension actuator
- Section engagement locks
- Position feedback

**Arguments:**
- `Extention` (Meters) - Extension distance in meters
  - Example: `0.35`
- `OutriggerID` (Integer) - Outrigger identifier
  - Example: `1`

**XML Example:**
```xml
<Outrigger_Section2_Flex Extention="0.35" OutriggerID="1"/>
```

---

## Pulley Nodes

> **Robot Compatibility**: RoboCon Mini Crane

### `Pulley_Lift`

Controls pulley lift position.

**Description**: Controls the winch/pulley system that raises and lowers loads. The distance represents the vertical position of the hook or load attachment point.

**Controls**:
- Winch motor/pulley actuator
- Cable length encoder
- Load weight sensors

**Arguments:**
- `Distance` (Meters) - Pulley end position in meters
  - Example: `4`
  - **Note**: 0 for fully retracted, positive values extend the pulley downward

**XML Example:**
```xml
<Pulley_Lift Distance="4"/>
```

**C++ Example (BehaviorTree.CPP v4.6+):**
```cpp
class PulleyLiftNode : public BT::StatefulActionNode
{
public:
    PulleyLiftNode(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("pulley_lift_node");
        cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
            "/pulley/lift_cmd", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<double>("Distance")};
    }

    BT::NodeStatus onStart() override
    {
        double distance;
        if (!getInput("Distance", distance))
        {
            return BT::NodeStatus::FAILURE;
        }

        std_msgs::msg::Float64 msg;
        msg.data = distance;
        cmd_pub_->publish(msg);

        RCLCPP_INFO(node_->get_logger(), 
            "Setting pulley lift position to %.2f meters", distance);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (/* pulley reached target position */) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_pub_;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<PulleyLiftNode>("Pulley_Lift");
}
```

---

## Travel Nodes

### `Travel_MotorLeft`

Controls the golden motor on the left track for robots with tracked and differential mode of travel.

> **Robot Compatibility**: 
> - RoboCon Mini Crane (tracked undercarriage)
> - RoboCon Mini Excavator (tracked undercarriage)
> - RoboCon Servicer Tracked (tracked/differential drive)
> - RoboCon Sheather Tracked (tracked/differential drive)
> - RoboCon Transporter Tracked (tracked/differential drive)

**Description**: Directly controls the left track/wheel motor for differential drive robots. This node sends speed commands directly to the motor controller via CAN bus. For coordinated movement, use both left and right motors together or use the high-level `Travel_Plan` node.

**Controls**:
- Left track/wheel motor (Golden Motor EZA48400)
- CAN bus communication
- Motor feedback (speed, current, temperature)

**Hardware Details:**
- **Motor**: Golden Motor EZA48400
- **CAN Node ID**: 0xF0 (240 decimal)
- **CAN Bitrate**: 500 kbps
- **Control Interface**: CAN bus via `/base/robot_base_cmd_pub_` topic (Float32MultiArray)
- **Speed Command**: Sent in RPM (output shaft speed)
- **Direction**: Positive RPM values rotate the motor forward (left track forward for forward motion)

**Arguments:**
- `RotationSpeed` (RPM) - Rotation speed in RPM (output shaft speed)
  - Example: `1000`
  - Range: Motor-dependent (typically -3000 to +3000 RPM for EZA48400)
  - Negative values reverse direction

**Driver Implementation:**
The golden motor driver (`golden_motor_eza48400`) converts the RPM command to:
- CAN bus speed command protocol (EZA48400 format)
- Speed control mode with phase current limit
- Reducer ratio compensation (if configured)

**ROS 2 Integration:**
- **Command Topic**: `/base/robot_base_cmd_pub_` (std_msgs/Float32MultiArray)
  - Format: `[right_radps, left_radps]` - Angular velocity in rad/s
  - The node internally converts rad/s to RPM for the motor controller
- **Feedback Topic**: `/left_wheel_golden_motor_status` (robot_custom_interfaces/GoldenMotorEZA48400Status)
- **Status Topic**: `/base/wheel_motors_feedback` (std_msgs/Float32MultiArray)

**XML Example:**
```xml
<Travel_MotorLeft RotationSpeed="1000"/>
```

**C++ Example (BehaviorTree.CPP v4.6+):**
```cpp
#include <std_msgs/msg/float32_multi_array.hpp>

class TravelMotorLeftNode : public BT::SyncActionNode
{
public:
    TravelMotorLeftNode(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("travel_motor_left_node");
        cmd_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/base/robot_base_cmd_pub_", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<double>("RotationSpeed")};
    }

    BT::NodeStatus tick() override
    {
        double rpm;
        if (!getInput("RotationSpeed", rpm))
        {
            return BT::NodeStatus::FAILURE;
        }

        // Convert RPM to rad/s for the command topic
        // Assuming gear ratio is handled by motor controller
        double rad_per_sec = rpm * 2.0 * M_PI / 60.0;

        std_msgs::msg::Float32MultiArray msg;
        msg.data.resize(2);
        msg.data[0] = 0.0;  // Right motor (not controlled by this node)
        msg.data[1] = rad_per_sec;  // Left motor angular velocity in rad/s

        cmd_pub_->publish(msg);

        RCLCPP_INFO(node_->get_logger(), 
            "Left motor command: %.2f RPM (%.2f rad/s)", rpm, rad_per_sec);

        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_pub_;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<TravelMotorLeftNode>("Travel_MotorLeft");
}
```

**Note**: For coordinated movement, use both `Travel_MotorLeft` and `Travel_MotorRight` together, or use the high-level `Travel_Plan` node which handles differential drive control automatically.

---

### `Travel_MotorRight`

Controls the golden motor on the right track for robots with tracked and differential mode of travel.

> **Robot Compatibility**: 
> - RoboCon Mini Crane (tracked undercarriage)
> - RoboCon Mini Excavator (tracked undercarriage)
> - RoboCon Servicer Tracked (tracked/differential drive)
> - RoboCon Sheather Tracked (tracked/differential drive)
> - RoboCon Transporter Tracked (tracked/differential drive)

---

### `Track_LeftExtend`

Extends the left track outward to widen the base of the tracked vehicle.

> **Robot Compatibility**:
> - RoboCon Mini Crane Tracked
> - RoboCon Servicer Tracked
> - RoboCon Sheather Tracked
> - RoboCon Transporter Tracked

**Description**: Extends the left track outward from the vehicle body to increase the width of the base. This provides additional stability, especially on uneven terrain or when lifting heavy loads.

**Controls**:
- Left track extension actuator
- Extension position feedback
- Load sensors

**Arguments:**
- `Extention` (Meters) - Extension distance in meters
  - Example: `0.5`

**XML Example:**
```xml
<Track_LeftExtend Extention="0.5"/>
```

**C++ Example (BehaviorTree.CPP v4.6+):**
```cpp
class TrackLeftExtendNode : public BT::StatefulActionNode
{
public:
    TrackLeftExtendNode(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("track_left_extend_node");
        cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
            "/track/left_extend_cmd", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<double>("Extention")};
    }

    BT::NodeStatus onStart() override
    {
        double extension;
        if (!getInput("Extention", extension))
        {
            return BT::NodeStatus::FAILURE;
        }

        std_msgs::msg::Float64 msg;
        msg.data = extension;
        cmd_pub_->publish(msg);

        RCLCPP_INFO(node_->get_logger(), 
            "Left track extension: %.2f meters", extension);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (/* extension complete */) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_pub_;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<TrackLeftExtendNode>("Track_LeftExtend");
}
```

---

### `Track_RightExtend`

Extends the right track outward to widen the base of the tracked vehicle.

> **Robot Compatibility**:
> - RoboCon Mini Crane Tracked
> - RoboCon Servicer Tracked
> - RoboCon Sheather Tracked
> - RoboCon Transporter Tracked

**Description**: Extends the right track outward from the vehicle body to increase the width of the base. This provides additional stability, especially on uneven terrain or when lifting heavy loads.

**Controls**:
- Right track extension actuator
- Extension position feedback
- Load sensors

**Arguments:**
- `Extention` (Meters) - Extension distance in meters
  - Example: `0.5`

**XML Example:**
```xml
<Track_RightExtend Extention="0.5"/>
```

**C++ Example (BehaviorTree.CPP v4.6+):**
```cpp
class TrackRightExtendNode : public BT::StatefulActionNode
{
public:
    TrackRightExtendNode(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("track_right_extend_node");
        cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
            "/track/right_extend_cmd", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<double>("Extention")};
    }

    BT::NodeStatus onStart() override
    {
        double extension;
        if (!getInput("Extention", extension))
        {
            return BT::NodeStatus::FAILURE;
        }

        std_msgs::msg::Float64 msg;
        msg.data = extension;
        cmd_pub_->publish(msg);

        RCLCPP_INFO(node_->get_logger(), 
            "Right track extension: %.2f meters", extension);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (/* extension complete */) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_pub_;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<TrackRightExtendNode>("Track_RightExtend");
}
```

---

## End Effector (Fork) Attachment Nodes

> **Robot Compatibility**: RoboCon Mini Crane
> 
> **Note**: End Effector - Fork Attachment Nodes are for the fork attachment for the RoboCon Mini Crane.

### `EndEffector_Fork_Slew`

Rotates the fork assembly.

**Description**: Rotates the entire fork assembly around its mounting axis. This allows the forks to be oriented for proper engagement with pallets or materials.

**Controls**:
- Fork slew actuator
- Rotation encoder feedback

**Arguments:**
- `Rotation` (Degrees) - Slew rotation angle in degrees
  - Example: `45`

**XML Example:**
```xml
<EndEffector_Fork_Slew Rotation="45"/>
```

---

### `EndEffector_Fork_Tilt`

Tilts the fork assembly.

**Description**: Tilts the fork assembly forward or backward to adjust the angle for engaging with materials or maintaining load stability during transport.

**Controls**:
- Fork tilt actuator
- Tilt angle feedback

**Arguments:**
- `Tilt` (Degrees) - Tilt angle in degrees
  - Example: `15`
  - Positive values typically tilt forward (away from vehicle)
  - Negative values tilt backward (toward vehicle)

**XML Example:**
```xml
<EndEffector_Fork_Tilt Tilt="15"/>
```

---

### `EndEffector_Fork_Slide`

Moves the whole fork assembly side to side.

**Description**: Translates the entire fork assembly horizontally, allowing it to shift left or right for proper alignment with pallets or load positioning.

**Controls**:
- Fork slide actuator
- Slide position feedback

**Arguments:**
- `Distance` (Meters) - Slide distance in meters
  - Example: `0.2`
  - Positive values slide right (when facing forward)
  - Negative values slide left

**XML Example:**
```xml
<EndEffector_Fork_Slide Distance="0.2"/>
```

---

### `EndEffector_Fork_ProngLeft`

Moves the left prong of the fork side to side.

**Description**: Independently adjusts the left fork prong horizontally. This allows for fine-tuning the fork spacing or compensating for misalignment.

**Controls**:
- Left prong slide actuator
- Left prong position feedback

**Arguments:**
- `Distance` (Meters) - Prong slide distance in meters
  - Example: `0.1`

**XML Example:**
```xml
<EndEffector_Fork_ProngLeft Distance="0.1"/>
```

---

### `EndEffector_Fork_ProngRight`

Moves the right prong of the fork side to side.

**Description**: Independently adjusts the right fork prong horizontally. This allows for fine-tuning the fork spacing or compensating for misalignment.

**Controls**:
- Right prong slide actuator
- Right prong position feedback

**Arguments:**
- `Distance` (Meters) - Prong slide distance in meters
  - Example: `0.1`

**XML Example:**
```xml
<EndEffector_Fork_ProngRight Distance="0.1"/>
```

---

## Lift Control Nodes

> **Robot Compatibility**: RoboCon Sheather Tracked, RoboCon Sheather Wheeled
> 
> **Note**: Lift control nodes work only for the RoboCon Sheather robots with lift platforms.

### `Lift_Extend`

Extends or retracts the lift platform to a specified height.

**Description**: Controls the vertical extension of the lift platform that supports the articulated arm. This allows the entire arm assembly to be raised or lowered for better reach and positioning.

**Controls**:
- Lift extension actuator (hydraulic or electromechanical)
- Extension position feedback
- Load monitoring sensors
- Safety interlocks

**Arguments:**
- `Extention` (Meters) - Extension distance in meters from base position
  - Example: `1.5`
  - Positive values extend upward
  - Negative values retract downward
  - Zero returns to base position

**XML Example:**
```xml
<Lift_Extend Extention="1.5"/>
```

**C++ Example (BehaviorTree.CPP v4.6+):**
```cpp
#include <std_msgs/msg/float64.hpp>

class LiftExtendNode : public BT::StatefulActionNode
{
public:
    LiftExtendNode(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("lift_extend_node");
        cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
            "/lift/extend_cmd", 10);
        
        status_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
            "/lift/position_status", 10,
            std::bind(&LiftExtendNode::position_callback, this, std::placeholders::_1));
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<double>("Extention")};
    }

    BT::NodeStatus onStart() override
    {
        double extension;
        if (!getInput("Extention", extension))
        {
            RCLCPP_ERROR(node_->get_logger(), "Missing input: Extention");
            return BT::NodeStatus::FAILURE;
        }

        std_msgs::msg::Float64 msg;
        msg.data = extension;
        cmd_pub_->publish(msg);

        target_position_ = extension;
        RCLCPP_INFO(node_->get_logger(), 
            "Lift extension command: %.2f meters", extension);
        
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        // Check if lift has reached target position (within 0.01m tolerance)
        if (std::abs(current_position_ - target_position_) < 0.01)
        {
            return BT::NodeStatus::SUCCESS;
        }
        
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        RCLCPP_WARN(node_->get_logger(), "Lift extension halted");
    }

private:
    void position_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        current_position_ = msg->data;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr status_sub_;
    double target_position_ = 0.0;
    double current_position_ = 0.0;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<LiftExtendNode>("Lift_Extend");
}
```

---

## BRTIRUS2550A Arm Joint Control Nodes

> **Robot Compatibility**: RoboCon Sheather Tracked, RoboCon Sheather Wheeled
> 
> **Note**: BRTIRUS2550A arm nodes are for controlling the Borunte BRTIRUS2550A 6-DOF articulated arm mounted on the lift platform. The arm is mounted on top of the lift, providing extended reach for sheathing operations.
> 
> **Reference**: [BORUNTE BRTIRUS2550A Product Page](https://www.borunte.top/products/1-19bb73a057ca41a69d648843772f60ca)

### `Arm_BRTIRUS2550A_Joint1`

Controls the first joint (base rotation) of the BRTIRUS2550A arm.

**Description**: Rotates the base of the arm horizontally around the vertical axis. This is the shoulder rotation joint that provides the arm's primary horizontal reach capability.

**Controls**:
- Joint 1 servo motor
- Joint position encoder
- Joint velocity feedback

**Arguments:**
- `Angle` (Degrees) - Target joint angle in degrees
  - Example: `45.0`
  - Range: Typically -180° to +180° (may vary by configuration)

**XML Example:**
```xml
<Arm_BRTIRUS2550A_Joint1 Angle="45.0"/>
```

**C++ Example (BehaviorTree.CPP v4.6+):**
```cpp
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class ArmBRTIRUS2550AJoint1Node : public BT::StatefulActionNode
{
public:
    ArmBRTIRUS2550AJoint1Node(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("arm_brtirus2550a_joint1_node");
        cmd_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/arm_brtirus2550a/joint_trajectory", 10);
        
        joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&ArmBRTIRUS2550AJoint1Node::joint_state_callback, this, std::placeholders::_1));
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<double>("Angle")};
    }

    BT::NodeStatus onStart() override
    {
        double angle;
        if (!getInput("Angle", angle))
        {
            RCLCPP_ERROR(node_->get_logger(), "Missing input: Angle");
            return BT::NodeStatus::FAILURE;
        }

        // Convert degrees to radians
        double angle_rad = angle * M_PI / 180.0;

        auto trajectory = trajectory_msgs::msg::JointTrajectory();
        trajectory.joint_names = {"brtirus2550a_joint1"};
        
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {angle_rad};
        point.time_from_start.sec = 2;
        
        trajectory.points.push_back(point);
        cmd_pub_->publish(trajectory);

        target_angle_ = angle_rad;
        RCLCPP_INFO(node_->get_logger(), 
            "BRTIRUS2550A Joint 1 command: %.2f degrees", angle);
        
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        // Check if joint has reached target angle (within 5 degrees tolerance)
        if (std::abs(current_angle_ - target_angle_) < (5.0 * M_PI / 180.0))
        {
            return BT::NodeStatus::SUCCESS;
        }
        
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        RCLCPP_WARN(node_->get_logger(), "BRTIRUS2550A Joint 1 movement halted");
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Find joint1 in the joint states
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i] == "brtirus2550a_joint1" && i < msg->position.size())
            {
                current_angle_ = msg->position[i];
                break;
            }
        }
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    double target_angle_ = 0.0;
    double current_angle_ = 0.0;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<ArmBRTIRUS2550AJoint1Node>("Arm_BRTIRUS2550A_Joint1");
}
```

---

### `Arm_BRTIRUS2550A_Joint2`

Controls the second joint (shoulder pitch) of the BRTIRUS2550A arm.

**Description**: Controls the shoulder pitch joint that lifts or lowers the upper arm relative to the base. This provides vertical reach capability.

**Arguments:**
- `Angle` (Degrees) - Target joint angle in degrees
  - Example: `-30.0`
  - Range: Typically -90° to +90°

**XML Example:**
```xml
<Arm_BRTIRUS2550A_Joint2 Angle="-30.0"/>
```

---

### `Arm_BRTIRUS2550A_Joint3`

Controls the third joint (elbow pitch) of the BRTIRUS2550A arm.

**Description**: Controls the elbow joint that bends the forearm relative to the upper arm. This provides additional vertical reach and manipulation capability.

**Arguments:**
- `Angle` (Degrees) - Target joint angle in degrees
  - Example: `90.0`
  - Range: Typically 0° to +180°

**XML Example:**
```xml
<Arm_BRTIRUS2550A_Joint3 Angle="90.0"/>
```

---

### `Arm_BRTIRUS2550A_Joint4`

Controls the fourth joint (wrist roll) of the BRTIRUS2550A arm.

**Description**: Rotates the wrist around the forearm axis. This provides orientation control for the end effector.

**Arguments:**
- `Angle` (Degrees) - Target joint angle in degrees
  - Example: `180.0`
  - Range: Typically -180° to +180°

**XML Example:**
```xml
<Arm_BRTIRUS2550A_Joint4 Angle="180.0"/>
```

---

### `Arm_BRTIRUS2550A_Joint5`

Controls the fifth joint (wrist pitch) of the BRTIRUS2550A arm.

**Description**: Tilts the wrist up or down relative to the forearm. This provides additional end effector orientation control.

**Arguments:**
- `Angle` (Degrees) - Target joint angle in degrees
  - Example: `-45.0`
  - Range: Typically -90° to +90°

**XML Example:**
```xml
<Arm_BRTIRUS2550A_Joint5 Angle="-45.0"/>
```

---

### `Arm_BRTIRUS2550A_Joint6`

Controls the sixth joint (end effector rotation) of the BRTIRUS2550A arm.

**Description**: Rotates the end effector or tool around the wrist axis. This provides final orientation control for tools and attachments.

**Arguments:**
- `Angle` (Degrees) - Target joint angle in degrees
  - Example: `90.0`
  - Range: Typically -180° to +180° or continuous rotation

**XML Example:**
```xml
<Arm_BRTIRUS2550A_Joint6 Angle="90.0"/>
```

---

## Camera Control Nodes

### `Camera_Adjust`

Adjusts camera settings. This is a low-level node since it does not require an LLM to determine how to set high resolution.

**Description**: Changes camera parameters such as resolution, exposure, or focus to optimize for current task requirements. This is a direct hardware control operation.

**Controls**:
- Camera parameter interface
- Image quality settings
- Camera hardware configuration

**Arguments:**
- `Camera_Mode` (Text) - Camera mode to set
  - Example: `High Resolution`

**XML Example:**
```xml
<Camera_Adjust Camera_Mode="High Resolution"/>
```

**C++ Example (BehaviorTree.CPP v4.6+):**
```cpp
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>

class CameraAdjustNode : public BT::SyncActionNode
{
public:
    CameraAdjustNode(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("camera_adjust_node");
        camera_mode_client_ = node_->create_client<sensor_msgs::srv::SetCameraInfo>(
            "/camera/set_mode");
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("Camera_Mode")};
    }

    BT::NodeStatus tick() override
    {
        std::string camera_mode;
        if (!getInput("Camera_Mode", camera_mode))
        {
            return BT::NodeStatus::FAILURE;
        }

        auto req = std::make_shared<sensor_msgs::srv::SetCameraInfo::Request>();
        req->camera_info.header.frame_id = "camera";
        // Set resolution based on mode
        if (camera_mode == "High Resolution")
        {
            req->camera_info.width = 3840;
            req->camera_info.height = 2160;
        }
        // ... other mode settings

        auto result = camera_mode_client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(node_, result) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(node_->get_logger(), 
            "Camera mode set to: %s", camera_mode.c_str());

        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<sensor_msgs::srv::SetCameraInfo>::SharedPtr camera_mode_client_;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<CameraAdjustNode>("Camera_Adjust");
}
```

---

## Next Steps

- [High-Level Nodes](../api-reference/behavior-tree-high-level-nodes.md) - High-level behavior tree nodes with inference and planning
- [Behavior Trees API](../api-reference/behavior-trees.md) - Complete BehaviorTree.CPP documentation
- [World_Find](world_find.md) - 3D scene object search node
- [Object_Pick](object_pick.md) - Complete pick operation node
- [Red Flag Detection Example](../ai-programs/examples/red-flag-detection.md) - Complete working example using these nodes
- [CAN Bus Hardware IDs](../can-bus.md) - CAN bus hardware reference

