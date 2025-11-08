# Multi-Robot Communication API Reference

Complete API reference for the consensus-based multi-robot task allocation system in ROBOCON OS.

## Overview

The Multi-Robot Communication API provides functions and classes for:
- Discovery of nearby robotic agents
- Consensus-based voting for task allocation
- Identity verification through motion ledgers
- Behavior tree synchronization
- Environment server integration

## C++ API

### Discovery Protocol

#### DiscoveryProtocol Class

Manages discovery and identification of nearby robotic agents.

```cpp
#include <robocon_multi_robot/discovery_protocol.hpp>

class DiscoveryProtocol {
public:
    DiscoveryProtocol(rclcpp::Node::SharedPtr node);
    
    // Start discovery process
    void startDiscovery();
    
    // Stop discovery
    void stopDiscovery();
    
    // Get discovered agents
    std::vector<AgentInfo> getDiscoveredAgents() const;
    
    // Request agent capabilities
    bool requestCapabilities(const std::string& agent_id);
    
    // Set discovery callbacks
    void setDiscoveryCallback(
        std::function<void(const AgentInfo&)> callback
    );
};
```

**Usage Example:**

```cpp
#include <rclcpp/rclcpp.hpp>
#include <robocon_multi_robot/discovery_protocol.hpp>

class MyRobotNode : public rclcpp::Node {
public:
    MyRobotNode() : Node("my_robot") {
        discovery_ = std::make_shared<DiscoveryProtocol>(shared_from_this());
        
        discovery_->setDiscoveryCallback(
            [this](const AgentInfo& agent) {
                RCLCPP_INFO(this->get_logger(), 
                    "Discovered agent: %s", agent.id.c_str());
            }
        );
        
        discovery_->startDiscovery();
    }
    
private:
    std::shared_ptr<DiscoveryProtocol> discovery_;
};
```

#### AgentInfo Structure

```cpp
struct AgentInfo {
    std::string id;
    std::string manufacturer_code;
    std::string machine_id;
    geometry_msgs::msg::Pose current_pose;
    std::vector<std::string> capabilities;
    std::string motion_ledger_hash;
    std::chrono::system_clock::time_point last_seen;
};
```

### Voting Protocol

#### VotingProtocol Class

Manages consensus-based voting for task allocation.

```cpp
#include <robocon_multi_robot/voting_protocol.hpp>

class VotingProtocol {
public:
    VotingProtocol(rclcpp::Node::SharedPtr node);
    
    // Broadcast action suggestion
    bool broadcastActionSuggestion(
        const ActionSuggestion& suggestion,
        std::function<void(const VotingResult&)> callback
    );
    
    // Respond to action suggestion
    bool respondToSuggestion(
        const std::string& suggestion_id,
        VoteIntent intent
    );
    
    // Submit vote ballot
    bool submitVoteBallot(
        const std::string& suggestion_id,
        const VoteBallot& ballot
    );
    
    // Get voting configuration
    VotingConfig getConfig() const;
    void setConfig(const VotingConfig& config);
};

enum class VoteIntent {
    CAN_PARTICIPATE,
    CANNOT_PARTICIPATE,
    NEEDS_CAPABILITIES
};

enum class VoteValue {
    APPROVE,
    REJECT,
    ABSTAIN
};
```

**ActionSuggestion Structure:**

```cpp
struct ActionSuggestion {
    std::string id;                          // Unique suggestion ID
    std::string proposer_id;                 // Agent proposing action
    std::string action_subtree_xml;          // BT subtree XML
    std::string world_state_hash;            // Expected world-state change hash
    std::vector<std::string> required_capabilities;
    ResourceRequirements resources;
    std::chrono::milliseconds timeout_ms;    // Consensus timeout
};
```

**VoteBallot Structure:**

```cpp
struct VoteBallot {
    std::string suggestion_id;
    std::string voter_id;
    VoteValue vote;                          // APPROVE, REJECT, or ABSTAIN
    std::string reasoning;                   // Optional explanation
    ActionSuggestion alternative;             // Optional alternative suggestion
};
```

**VotingResult Structure:**

```cpp
struct VotingResult {
    std::string suggestion_id;
    bool consensus_achieved;
    int approve_count;
    int reject_count;
    int abstain_count;
    std::string tie_breaker_agent_id;        // If tied
    std::chrono::system_clock::time_point timestamp;
};
```

**Usage Example:**

```cpp
#include <robocon_multi_robot/voting_protocol.hpp>

class TaskAllocator {
public:
    TaskAllocator(rclcpp::Node::SharedPtr node) 
        : node_(node) {
        voting_ = std::make_shared<VotingProtocol>(node);
    }
    
    void proposeTask(const std::string& bt_xml) {
        ActionSuggestion suggestion;
        suggestion.id = generateUUID();
        suggestion.proposer_id = node_->get_name();
        suggestion.action_subtree_xml = bt_xml;
        suggestion.world_state_hash = calculateWorldStateHash();
        suggestion.timeout_ms = std::chrono::milliseconds(5000);
        
        voting_->broadcastActionSuggestion(
            suggestion,
            [this](const VotingResult& result) {
                if (result.consensus_achieved) {
                    insertActionSubtree(result.suggestion_id);
                } else {
                    handleVotingFailure(result);
                }
            }
        );
    }
    
private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<VotingProtocol> voting_;
};
```

### Motion Ledger

#### MotionLedger Class

Manages blockchain-structured motion ledger for identity verification.

```cpp
#include <robocon_multi_robot/motion_ledger.hpp>

class MotionLedger {
public:
    MotionLedger(rclcpp::Node::SharedPtr node);
    
    // Record motion
    bool recordMotion(const MotionData& motion);
    
    // Get ledger hash
    std::string getLatestHash() const;
    
    // Get ledger block
    MotionLedgerBlock getBlock(size_t index) const;
    
    // Request hash for verification
    std::vector<std::string> getHashChain(size_t num_blocks) const;
    
    // Verify ledger continuity
    bool verifyContinuity(
        const std::vector<std::string>& hash_chain
    ) const;
    
    // Generate action replay
    ActionReplay generateReplay(
        const std::string& start_time,
        const std::string& end_time
    ) const;
};
```

**MotionData Structure:**

```cpp
struct MotionData {
    geometry_msgs::msg::Pose pose;
    std::map<std::string, double> joint_positions;
    std::map<std::string, double> actuator_states;
    sensor_msgs::msg::PointCloud2 lidar_data;    // Optional
    sensor_msgs::msg::Image stereo_vision;        // Optional
    std::chrono::system_clock::time_point timestamp;
};
```

**MotionLedgerBlock Structure:**

```cpp
struct MotionLedgerBlock {
    std::string previous_hash;
    std::string block_hash;
    std::chrono::system_clock::time_point timestamp;
    std::string robot_id;
    std::vector<MotionData> motions;
    std::vector<std::string> endorsements;
};
```

**ActionReplay Structure:**

```cpp
struct ActionReplay {
    std::string robot_id;
    std::chrono::system_clock::time_point start_time;
    std::chrono::system_clock::time_point end_time;
    std::vector<MotionData> motion_sequence;
    std::vector<ValidationResult> validations;
};
```

**Usage Example:**

```cpp
#include <robocon_multi_robot/motion_ledger.hpp>

class RobotController {
public:
    RobotController(rclcpp::Node::SharedPtr node)
        : node_(node) {
        ledger_ = std::make_shared<MotionLedger>(node);
    }
    
    void executeMotion(const MotionCommand& cmd) {
        // Execute motion...
        MotionData data;
        data.pose = getCurrentPose();
        data.joint_positions = getJointPositions();
        data.timestamp = std::chrono::system_clock::now();
        
        ledger_->recordMotion(data);
    }
    
    std::string getVerificationHash() const {
        return ledger_->getLatestHash();
    }
    
private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<MotionLedger> ledger_;
};
```

### Goal Ledger

#### GoalLedger Class

Manages natural language goals linked to motion ledger entries.

```cpp
#include <robocon_multi_robot/goal_ledger.hpp>

class GoalLedger {
public:
    GoalLedger(rclcpp::Node::SharedPtr node);
    
    // Add goal
    std::string addGoal(
        const std::string& natural_language_description,
        const std::vector<std::string>& motion_ledger_entries
    );
    
    // Get goal
    GoalEntry getGoal(const std::string& goal_id) const;
    
    // Link to motion ledger
    bool linkToMotionLedger(
        const std::string& goal_id,
        const std::string& motion_entry_id
    );
    
    // Get goals by time range
    std::vector<GoalEntry> getGoals(
        const std::chrono::system_clock::time_point& start,
        const std::chrono::system_clock::time_point& end
    ) const;
};
```

**GoalEntry Structure:**

```cpp
struct GoalEntry {
    std::string id;
    std::string natural_language_description;
    std::vector<std::string> motion_ledger_entry_ids;
    std::chrono::system_clock::time_point created_at;
    std::chrono::system_clock::time_point completed_at;
    GoalStatus status;
};

enum class GoalStatus {
    PENDING,
    IN_PROGRESS,
    COMPLETED,
    FAILED,
    CANCELLED
};
```

**Usage Example:**

```cpp
#include <robocon_multi_robot/goal_ledger.hpp>

class GoalManager {
public:
    GoalManager(rclcpp::Node::SharedPtr node)
        : node_(node) {
        goal_ledger_ = std::make_shared<GoalLedger>(node);
    }
    
    std::string createGoal(const std::string& description) {
        return goal_ledger_->addGoal(description, {});
    }
    
    void linkMotionToGoal(
        const std::string& goal_id,
        const std::string& motion_id
    ) {
        goal_ledger_->linkToMotionLedger(goal_id, motion_id);
    }
    
private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<GoalLedger> goal_ledger_;
};
```

### Identity Verification

#### IdentityVerifier Class

Performs zero-trust identity verification through motion ledgers.

```cpp
#include <robocon_multi_robot/identity_verifier.hpp>

class IdentityVerifier {
public:
    IdentityVerifier(rclcpp::Node::SharedPtr node);
    
    // Verify agent identity
    bool verifyIdentity(
        const std::string& agent_id,
        IdentityVerificationLevel level = IdentityVerificationLevel::HASH_ONLY
    );
    
    // Request motion ledger hash
    bool requestMotionLedgerHash(
        const std::string& agent_id,
        std::function<void(const std::vector<std::string>&)> callback
    );
    
    // Request private action replay
    bool requestActionReplay(
        const std::string& agent_id,
        const std::string& start_time,
        const std::string& end_time,
        std::function<void(const ActionReplay&)> callback
    );
    
    // Perform morphology scan
    bool performMorphologyScan(
        const std::string& agent_id,
        std::function<void(const MorphologyData&)> callback
    );
    
    // Get trust level
    TrustLevel getTrustLevel(const std::string& agent_id) const;
    
    // Set trust level
    void setTrustLevel(
        const std::string& agent_id,
        TrustLevel level
    );
};

enum class IdentityVerificationLevel {
    HASH_ONLY,           // Level 1: Motion ledger hash only
    ACTION_REPLAY,       // Level 2: Private action replay
    MORPHOLOGY_SCAN      // Level 3: Multimodal morphology scanning
};

enum class TrustLevel {
    UNKNOWN,             // No verification
    LOW,                 // Hash verification passed
    MEDIUM,              // Action replay verified
    HIGH                 // Morphology scan verified
};
```

**MorphologyData Structure:**

```cpp
struct MorphologyData {
    geometry_msgs::msg::Vector3 dimensions;
    std::map<std::string, geometry_msgs::msg::Transform> joint_configurations;
    sensor_msgs::msg::PointCloud2 lidar_point_cloud;
    sensor_msgs::msg::Image stereo_vision_left;
    sensor_msgs::msg::Image stereo_vision_right;
    std::chrono::system_clock::time_point timestamp;
};
```

**Usage Example:**

```cpp
#include <robocon_multi_robot/identity_verifier.hpp>

class SecureTaskAllocator {
public:
    SecureTaskAllocator(rclcpp::Node::SharedPtr node)
        : node_(node) {
        verifier_ = std::make_shared<IdentityVerifier>(node);
    }
    
    void verifyBeforeTask(
        const std::string& agent_id,
        std::function<void(bool)> callback
    ) {
        verifier_->verifyIdentity(
            agent_id,
            IdentityVerificationLevel::ACTION_REPLAY
        );
        
        // Check trust level
        auto trust_level = verifier_->getTrustLevel(agent_id);
        if (trust_level >= TrustLevel::MEDIUM) {
            callback(true);
        } else {
            // Request higher verification
            verifier_->requestActionReplay(
                agent_id,
                "",
                "",
                [this, agent_id, callback](const ActionReplay& replay) {
                    if (validateReplay(replay)) {
                        verifier_->setTrustLevel(agent_id, TrustLevel::MEDIUM);
                        callback(true);
                    } else {
                        callback(false);
                    }
                }
            );
        }
    }
    
private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<IdentityVerifier> verifier_;
    
    bool validateReplay(const ActionReplay& replay) {
        // Validation logic
        return true;
    }
};
```

### Environment Server

#### EnvironmentServerClient Class

Interfaces with the environment server for 3D avatar management and spatial coordination.

```cpp
#include <robocon_multi_robot/environment_server_client.hpp>

class EnvironmentServerClient {
public:
    EnvironmentServerClient(rclcpp::Node::SharedPtr node);
    
    // Insert 3D avatar
    bool insertAvatar(
        const std::string& agent_id,
        const MorphologyData& morphology
    );
    
    // Update avatar pose
    bool updateAvatarPose(
        const std::string& agent_id,
        const geometry_msgs::msg::Pose& pose
    );
    
    // Get environment model
    EnvironmentModel getEnvironmentModel() const;
    
    // Check for collisions
    bool checkCollision(
        const geometry_msgs::msg::Pose& proposed_pose,
        const std::vector<std::string>& other_agent_ids
    ) const;
    
    // Get predicted positions
    std::map<std::string, geometry_msgs::msg::Pose> 
        getPredictedPositions(
            const std::chrono::milliseconds& time_horizon
        ) const;
};
```

**EnvironmentModel Structure:**

```cpp
struct EnvironmentModel {
    std::map<std::string, AgentAvatar> agent_avatars;
    nav_msgs::msg::OccupancyGrid occupancy_grid;
    std::vector<geometry_msgs::msg::Polygon> obstacles;
    std::chrono::system_clock::time_point last_updated;
};

struct AgentAvatar {
    std::string agent_id;
    geometry_msgs::msg::Pose current_pose;
    geometry_msgs::msg::Pose predicted_pose;
    MorphologyData morphology;
    std::vector<geometry_msgs::msg::Pose> predicted_trajectory;
};
```

## Python API

The Python API provides equivalent functionality:

```python
from robocon_multi_robot.discovery_protocol import DiscoveryProtocol
from robocon_multi_robot.voting_protocol import VotingProtocol
from robocon_multi_robot.motion_ledger import MotionLedger
from robocon_multi_robot.identity_verifier import IdentityVerifier
import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot')
        
        # Initialize components
        self.discovery = DiscoveryProtocol(self)
        self.voting = VotingProtocol(self)
        self.ledger = MotionLedger(self)
        self.verifier = IdentityVerifier(self)
        
        # Start discovery
        self.discovery.start_discovery()
        
        # Set callbacks
        self.discovery.set_discovery_callback(self.on_agent_discovered)
    
    def on_agent_discovered(self, agent_info):
        self.get_logger().info(f'Discovered agent: {agent_info.id}')
        
        # Verify identity
        trust_level = self.verifier.get_trust_level(agent_info.id)
        if trust_level.value >= TrustLevel.MEDIUM.value:
            self.get_logger().info('Agent verified, can proceed')
    
    def propose_task(self, bt_xml):
        suggestion = ActionSuggestion()
        suggestion.id = str(uuid.uuid4())
        suggestion.proposer_id = self.get_name()
        suggestion.action_subtree_xml = bt_xml
        
        self.voting.broadcast_action_suggestion(
            suggestion,
            self.on_voting_result
        )
    
    def on_voting_result(self, result):
        if result.consensus_achieved:
            self.get_logger().info('Consensus achieved!')
        else:
            self.get_logger().warn('Consensus failed')
```

## ROS 2 Message Types

### robocon_multi_robot_msgs

Custom message definitions for multi-robot communication.

**ActionSuggestion.msg:**
```msg
string id
string proposer_id
string action_subtree_xml
string world_state_hash
string[] required_capabilities
robocon_multi_robot_msgs/ResourceRequirements resources
int64 timeout_ms
```

**VoteBallot.msg:**
```msg
string suggestion_id
string voter_id
uint8 vote  # 0=APPROVE, 1=REJECT, 2=ABSTAIN
string reasoning
robocon_multi_robot_msgs/ActionSuggestion alternative
```

**MotionLedgerBlock.msg:**
```msg
string previous_hash
string block_hash
builtin_interfaces/Time timestamp
string robot_id
robocon_multi_robot_msgs/MotionData[] motions
string[] endorsements
```

**GoalEntry.msg:**
```msg
string id
string natural_language_description
string[] motion_ledger_entry_ids
builtin_interfaces/Time created_at
builtin_interfaces/Time completed_at
uint8 status  # 0=PENDING, 1=IN_PROGRESS, 2=COMPLETED, 3=FAILED, 4=CANCELLED
```

## Configuration

Configuration parameters can be set via ROS 2 parameters:

```yaml
multi_robot:
  voting:
    consensus_timeout_ms: 5000
    max_retry_attempts: 3
    consensus_threshold: 0.5
  
  discovery:
    bluetooth_scan_duration_ms: 2000
    wifi_broadcast_interval_ms: 5000
    discovery_timeout_ms: 10000
  
  identity:
    ledger_hash_algorithm: "SHA-256"
    min_trust_level_for_task: 1  # 0=UNKNOWN, 1=LOW, 2=MEDIUM, 3=HIGH
    progressive_verification_enabled: true
```

## Error Handling

All API functions return appropriate error codes:

```cpp
enum class MultiRobotError {
    SUCCESS,
    TIMEOUT,
    NETWORK_ERROR,
    VERIFICATION_FAILED,
    CONSENSUS_FAILED,
    INVALID_ARGUMENT
};
```

## Next Steps

- [Multi-Robot Communication Architecture](../architecture/multi-robot-communication.md) - System architecture overview
- [Behavior Trees API](behavior-trees.md) - Integration with behavior trees
- [ROS 2 Custom Messages](../ros2/custom-messages.md) - Complete message definitions

