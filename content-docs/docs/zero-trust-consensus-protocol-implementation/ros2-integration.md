# ROS 2 Integration

This document describes the ROS 2 message definitions, topics, and services used by the Zero-Trust Consensus Protocol implementation.

## Custom Message Definitions

### ActionSuggestion Message

```msg
# robocon_network_client/msg/ActionSuggestion.msg
string action_id
string proposer_id
string action_type
string target_location
float64 estimated_cost
string[] required_capabilities
builtin_interfaces/Time proposal_timestamp
string behavior_tree_node
string world_state_hash
```

### Vote Message

```msg
# robocon_network_client/msg/Vote.msg
string action_id
string voter_id
bool vote  # true for yes, false for no
builtin_interfaces/Time vote_timestamp
string reason
string signature
```

### VoteIntent Message

```msg
# robocon_network_client/msg/VoteIntent.msg
string action_id
string voter_id
bool intent  # true if willing to vote
builtin_interfaces/Time timestamp
```

### ConsensusResult Message

```msg
# robocon_network_client/msg/ConsensusResult.msg
string action_id
bool consensus_reached
uint32 total_votes
uint32 yes_votes
uint32 no_votes
builtin_interfaces/Time consensus_timestamp
string executor_id
```

### MotionEntry Message

```msg
# robocon_network_client/msg/MotionEntry.msg
string entry_id
string robot_id
builtin_interfaces/Time timestamp
geometry_msgs/Pose pose
float64 precision
float64 resolution
string shape_hash
string previous_hash
string current_hash
string signature
```

### GoalEntry Message

```msg
# robocon_network_client/msg/GoalEntry.msg
string entry_id
string robot_id
builtin_interfaces/Time timestamp
string natural_language_goal
string[] motion_entry_links
string previous_hash
string current_hash
string signature
```

### PeerInfo Message

```msg
# robocon_network_client/msg/PeerInfo.msg
string peer_id
string peer_type  # "robot", "tablet", "laptop"
string[] capabilities
geometry_msgs/Pose current_pose
builtin_interfaces/Time last_seen
bool is_online
string version
string signature
string public_key
```

### SensorDataRequest Message

```msg
# robocon_network_client/msg/SensorDataRequest.msg
string request_id
string robot_id
string sensor_type
builtin_interfaces/Time timestamp
string requester_id
```

### SensorData Message

```msg
# robocon_network_client/msg/SensorData.msg
string data_id
string robot_id
string sensor_type
builtin_interfaces/Time timestamp
uint8[] data
string data_format
float64 resolution
float64 precision
```

## ROS 2 Topics

### Consensus Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/consensus/action_suggestion` | `ActionSuggestion` | Pub/Sub | Action proposals from robots |
| `/consensus/vote` | `Vote` | Pub/Sub | Votes cast on actions |
| `/consensus/vote_intent` | `VoteIntent` | Pub/Sub | Intent to vote on actions |
| `/consensus/result` | `ConsensusResult` | Pub/Sub | Consensus results |

### Ledger Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/ledger/motion` | `MotionEntry` | Pub/Sub | Motion ledger entries |
| `/ledger/goal` | `GoalEntry` | Pub/Sub | Goal ledger entries |
| `/ledger/sync` | `LedgerSync` | Pub/Sub | Ledger synchronization messages |

### Discovery Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/discovery/beacon` | `PeerInfo` | Pub/Sub | Discovery beacons |
| `/discovery/response` | `PeerInfo` | Pub/Sub | Discovery responses |

### Security Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/security/verification` | `VerificationRequest` | Pub/Sub | Identity verification requests |
| `/security/motion_hash` | `MotionHashRequest` | Pub/Sub | Motion hash verification requests |

## ROS 2 Services

### Consensus Services

```srv
# robocon_network_client/srv/ProposeAction.srv
robocon_network_client/msg/ActionSuggestion action
---
robocon_network_client/msg/ConsensusResult result
```

```srv
# robocon_network_client/srv/GetConsensusStatus.srv
string action_id
---
robocon_network_client/msg/ConsensusResult result
```

### Ledger Services

```srv
# robocon_network_client/srv/GetMotionLedger.srv
string peer_id
---
robocon_network_client/msg/MotionEntry[] entries
```

```srv
# robocon_network_client/srv/GetGoalLedger.srv
string peer_id
---
robocon_network_client/msg/GoalEntry[] entries
```

### Discovery Services

```srv
# robocon_network_client/srv/DiscoverPeers.srv
uint32 timeout_ms
---
robocon_network_client/msg/PeerInfo[] peers
```

## QoS Settings

The protocol uses specific QoS settings for different message types:

### Reliable Transient Local

Used for consensus and ledger messages that need to be durable:

```cpp
rclcpp::QoS qos(10);
qos.reliable();
qos.transient_local();
```

### Best Effort

Used for high-frequency sensor data:

```cpp
rclcpp::QoS qos(10);
qos.best_effort();
```

### Reliable

Used for discovery and security messages:

```cpp
rclcpp::QoS qos(10);
qos.reliable();
```

## DDS Discovery Configuration

The protocol uses Cyclone DDS with specific discovery settings:

```xml
<!-- cyclonedds.xml -->
<CycloneDDS>
    <Domain>
        <General>
            <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
            <AllowMulticast>true</AllowMulticast>
        </General>
        <Discovery>
            <ExternalDomainId>0</ExternalDomainId>
            <ParticipantIndex>auto</ParticipantIndex>
            <Peers>
                <Peer Address="127.0.0.1"/>
            </Peers>
        </Discovery>
    </Domain>
</CycloneDDS>
```

## Topic Namespace Customization

Topics can be customized using ROS 2 namespaces:

```cpp
// Using namespace
std::string namespace = "/robocon/robot_001";
action_publisher_ = node_->create_publisher<ActionSuggestion>(
    namespace + "/consensus/action_suggestion",
    rclcpp::QoS(10).reliable().transient_local()
);
```

## Message Serialization

Messages are serialized using CDR (Common Data Representation) format by Cyclone DDS:

```cpp
// Custom serialization (if needed)
void serializeActionSuggestion(
    const ActionSuggestion& action,
    std::vector<uint8_t>& buffer
) {
    // CDR serialization handled by ROS 2
    // Custom serialization can be added here if needed
}
```

## Integration Example

```cpp
#include <rclcpp/rclcpp.hpp>
#include "robocon_network_client/msg/action_suggestion.hpp"
#include "robocon_network_client/msg/consensus_result.hpp"

class ConsensusNode : public rclcpp::Node {
public:
    ConsensusNode() : Node("consensus_node") {
        // Publisher
        action_publisher_ = create_publisher<ActionSuggestion>(
            "/consensus/action_suggestion",
            rclcpp::QoS(10).reliable().transient_local()
        );
        
        // Subscriber
        result_subscriber_ = create_subscription<ConsensusResult>(
            "/consensus/result",
            rclcpp::QoS(10).reliable().transient_local(),
            std::bind(&ConsensusNode::onConsensusResult, 
                      this, std::placeholders::_1)
        );
    }
    
private:
    void onConsensusResult(
        const ConsensusResult::SharedPtr msg
    ) {
        RCLCPP_INFO(get_logger(), 
                    "Consensus reached: %s, Executor: %s", 
                    msg->action_id.c_str(),
                    msg->executor_id.c_str());
    }
    
    rclcpp::Publisher<ActionSuggestion>::SharedPtr action_publisher_;
    rclcpp::Subscription<ConsensusResult>::SharedPtr result_subscriber_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConsensusNode>());
    rclcpp::shutdown();
    return 0;
}
```

## Building Messages

Add to your `CMakeLists.txt`:

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActionSuggestion.msg"
  "msg/Vote.msg"
  "msg/VoteIntent.msg"
  "msg/ConsensusResult.msg"
  "msg/MotionEntry.msg"
  "msg/GoalEntry.msg"
  "msg/PeerInfo.msg"
  "msg/SensorDataRequest.msg"
  "msg/SensorData.msg"
  "srv/ProposeAction.srv"
  "srv/GetConsensusStatus.srv"
  "srv/GetMotionLedger.srv"
  "srv/GetGoalLedger.srv"
  "srv/DiscoverPeers.srv"
  DEPENDENCIES geometry_msgs builtin_interfaces
)
```

## Key Implementation Details

1. **Message Types**: All protocol messages are defined as ROS 2 message types.

2. **QoS Settings**: Different QoS policies are used based on message importance and reliability requirements.

3. **DDS Integration**: Messages are transmitted using Cyclone DDS, which provides reliable, low-latency communication.

4. **Namespace Support**: Topics can be namespaced for multi-robot scenarios.

5. **Serialization**: Messages are automatically serialized/deserialized by ROS 2 using CDR format.

## Next Steps

- [Network Client Implementation](network-client.md) - Main interface
- [Consensus Manager Implementation](consensus-manager.md) - Voting protocol
- [Ledger Manager Implementation](ledger-manager.md) - Blockchain-style ledger

