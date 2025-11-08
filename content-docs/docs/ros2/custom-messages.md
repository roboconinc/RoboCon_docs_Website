# Custom Messages

Creating and using custom ROS 2 messages for ROBOCON SDK.

## Message Definitions

Define custom messages in `.msg` files within ROS 2 packages.

## Multi-Robot Communication Messages

The ROBOCON SDK includes comprehensive message types for the consensus-based multi-robot task allocation system.

### ActionSuggestion.msg

Proposed action for consensus-based task allocation:

```msg
string id                           # Unique suggestion ID
string proposer_id                  # Agent proposing action
string action_subtree_xml          # Behavior tree subtree XML
string world_state_hash            # Cryptographic hash of expected world-state change
string[] required_capabilities    # Required robot capabilities
robocon_multi_robot_msgs/ResourceRequirements resources
int64 timeout_ms                    # Consensus timeout in milliseconds
```

### VoteBallot.msg

Vote response from participating agents:

```msg
string suggestion_id                # ID of action suggestion being voted on
string voter_id                     # Agent casting vote
uint8 vote                          # 0=APPROVE, 1=REJECT, 2=ABSTAIN
string reasoning                    # Optional explanation
robocon_multi_robot_msgs/ActionSuggestion alternative  # Optional alternative
```

### VoteIntent.msg

Initial response indicating participation capability:

```msg
string suggestion_id                # ID of action suggestion
string responder_id                 # Agent responding
uint8 intent                        # 0=CAN_PARTICIPATE, 1=CANNOT_PARTICIPATE, 2=NEEDS_CAPABILITIES
string[] available_capabilities    # Capabilities available for this task
```

### VotingResult.msg

Result of consensus voting process:

```msg
string suggestion_id                # ID of action suggestion
bool consensus_achieved              # Whether consensus was reached
int32 approve_count                 # Number of approve votes
int32 reject_count                  # Number of reject votes
int32 abstain_count                 # Number of abstain votes
string tie_breaker_agent_id         # Agent ID used for tie-breaking (if tied)
builtin_interfaces/Time timestamp   # Time of consensus achievement
```

### MotionLedgerBlock.msg

Block in the blockchain-structured motion ledger:

```msg
string previous_hash                # Hash of previous block
string block_hash                   # Hash of this block
builtin_interfaces/Time timestamp   # Block creation time
string robot_id                     # Robot that created this block
robocon_multi_robot_msgs/MotionData[] motions  # Fine-grained motion data
string[] endorsements               # Digital signatures from verifying agents
```

### MotionData.msg

Fine-grained motion data recorded in ledger:

```msg
geometry_msgs/Pose pose             # Robot pose (position, orientation)
std_msgs/Float64MultiArray joint_positions  # Joint positions
std_msgs/Float64MultiArray actuator_states  # Actuator states
sensor_msgs/PointCloud2 lidar_data  # Optional LiDAR data
sensor_msgs/Image stereo_vision      # Optional stereo vision data
builtin_interfaces/Time timestamp   # Motion timestamp
```

### GoalEntry.msg

Natural language goal linked to motion ledger entries:

```msg
string id                           # Unique goal ID
string natural_language_description # Human-readable goal description
string[] motion_ledger_entry_ids    # Linked motion ledger entry IDs
builtin_interfaces/Time created_at  # Goal creation time
builtin_interfaces/Time completed_at  # Goal completion time
uint8 status                        # 0=PENDING, 1=IN_PROGRESS, 2=COMPLETED, 3=FAILED, 4=CANCELLED
```

### AgentInfo.msg

Information about discovered robotic agents:

```msg
string id                           # Unique agent ID
string manufacturer_code            # Robot manufacturer identifier
string machine_id                   # Machine-specific identifier
geometry_msgs/Pose current_pose     # Current position and orientation
string[] capabilities               # Available capabilities
string motion_ledger_hash           # Latest motion ledger hash
builtin_interfaces/Time last_seen   # Last discovery time
```

### ResourceRequirements.msg

Resource requirements for proposed actions:

```msg
float64 estimated_duration_seconds  # Estimated execution time
float64 cpu_usage_percent           # Required CPU usage
float64 memory_mb                   # Required memory in MB
float64 energy_consumption_wh        # Estimated energy consumption
string[] required_hardware          # Required hardware components
```

### ActionReplay.msg

Private action replay for identity verification:

```msg
string robot_id                     # Robot that performed actions
builtin_interfaces/Time start_time  # Replay start time
builtin_interfaces/Time end_time    # Replay end time
robocon_multi_robot_msgs/MotionData[] motion_sequence  # Sequence of motions
robocon_multi_robot_msgs/ValidationResult[] validations  # Validation results
```

## Using Custom Messages

### Python

```python
from robocon_multi_robot_msgs.msg import ActionSuggestion, VoteBallot
import uuid

# Create action suggestion
suggestion = ActionSuggestion()
suggestion.id = str(uuid.uuid4())
suggestion.proposer_id = "robot_001"
suggestion.action_subtree_xml = "<BehaviorTree>...</BehaviorTree>"
suggestion.world_state_hash = calculate_hash()
suggestion.timeout_ms = 5000

# Publish suggestion
publisher = node.create_publisher(
    ActionSuggestion,
    '/robocon/multi_robot/action_suggestion',
    10
)
publisher.publish(suggestion)
```

### C++

```cpp
#include <robocon_multi_robot_msgs/msg/action_suggestion.hpp>
#include <rclcpp/rclcpp.hpp>

// Create action suggestion
auto suggestion = robocon_multi_robot_msgs::msg::ActionSuggestion();
suggestion.id = generateUUID();
suggestion.proposer_id = "robot_001";
suggestion.action_subtree_xml = "<BehaviorTree>...</BehaviorTree>";
suggestion.world_state_hash = calculateHash();
suggestion.timeout_ms = 5000;

// Publish suggestion
auto publisher = node->create_publisher<
    robocon_multi_robot_msgs::msg::ActionSuggestion>(
    "/robocon/multi_robot/action_suggestion", 10);
publisher->publish(suggestion);
```

## Message Packages

Multi-robot communication messages are provided in the `robocon_multi_robot_msgs` package:

```bash
# Install package
ros2 pkg install robocon_multi_robot_msgs

# View message definitions
ros2 interface show robocon_multi_robot_msgs/msg/ActionSuggestion
```

## Topic Namespaces

Multi-robot communication uses the following topic namespaces:

- `/robocon/multi_robot/discovery` - Discovery beacons
- `/robocon/multi_robot/action_suggestion` - Action suggestion packets
- `/robocon/multi_robot/vote_intent` - Vote intent responses
- `/robocon/multi_robot/vote_ballot` - Vote ballots
- `/robocon/multi_robot/motion_ledger` - Motion ledger updates
- `/robocon/multi_robot/goal_ledger` - Goal ledger updates
- `/robocon/multi_robot/environment` - Environment server updates

## Next Steps

- [Multi-Robot Communication API](../api-reference/multi-robot-communication.md) - Complete API reference
- [Multi-Robot Communication Architecture](../architecture/multi-robot-communication.md) - Protocol overview
- [Publishers and Subscribers](publishers-and-subscribers.md) - ROS 2 communication patterns

