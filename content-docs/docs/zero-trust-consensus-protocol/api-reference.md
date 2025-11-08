# Zero-Trust Consensus Protocol API Reference

Complete API documentation for the Zero-Trust Consensus Protocol implementation in the ROBOCON OS Network Client.

## NetworkClient

Main interface for DDS communication and message handling.

### Header

```cpp
#include "robocon_network_client/network_client.hpp"
```

### Constructor

```cpp
explicit NetworkClient(const std::string& robot_id, 
                      const std::string& config_path = "");
```

**Parameters:**
- `robot_id`: Unique identifier for this robot/device
- `config_path`: Path to configuration file (optional)

### Lifecycle Methods

#### `bool initialize()`

Initialize the network client and establish DDS connections.

**Returns:** `true` if initialization successful

#### `void shutdown()`

Shutdown the network client and cleanup resources.

### Sensor Data Operations

#### `SensorDataResponse sendSensorDataRequest(const SensorDataRequest& request, uint32_t timeout_ms = 5000)`

Send a sensor data request to a specific robot.

**Parameters:**
- `request`: The sensor data request structure
- `timeout_ms`: Timeout in milliseconds (default: 5000)

**Returns:** `SensorDataResponse` containing the response data

#### `void publishSensorData(const SensorData& data)`

Publish sensor data to the network.

**Parameters:**
- `data`: The sensor data to publish

#### `void setSensorDataRequestCallback(std::function<SensorData(const SensorDataRequest&)> callback)`

Set callback for handling incoming sensor data requests.

**Parameters:**
- `callback`: Function that handles sensor data requests and returns sensor data

### Discovery Operations

#### `std::vector<PeerInfo> discoverPeers(uint32_t timeout_ms = 5000)`

Discover peers on the network.

**Parameters:**
- `timeout_ms`: Discovery timeout in milliseconds (default: 5000)

**Returns:** Vector of discovered peer information

#### `PeerInfo getPeerInfo(const std::string& peer_id)`

Get information about a specific peer.

**Parameters:**
- `peer_id`: The peer's unique identifier

**Returns:** Peer information structure

#### `void announcePresence()`

Announce presence on the network by broadcasting discovery beacons.

### Consensus Operations

#### `ConsensusManager& getConsensusManager()`

Get the consensus manager for direct access to consensus operations.

**Returns:** Reference to consensus manager

#### `ConsensusResult proposeAction(const ActionSuggestion& action)`

Propose an action for consensus voting.

**Parameters:**
- `action`: The action suggestion to propose

**Returns:** Consensus result indicating voting outcome

#### `void voteOnAction(const std::string& action_id, bool vote)`

Vote on a proposed action.

**Parameters:**
- `action_id`: The action identifier
- `vote`: `true` for yes, `false` for no

### Ledger Operations

#### `LedgerManager& getLedgerManager()`

Get the ledger manager for direct access to ledger operations.

**Returns:** Reference to ledger manager

#### `void appendMotionLedger(const MotionEntry& entry)`

Append entry to motion ledger.

**Parameters:**
- `entry`: The motion entry to append

#### `void appendGoalLedger(const GoalEntry& entry)`

Append entry to goal ledger.

**Parameters:**
- `entry`: The goal entry to append

#### `std::vector<MotionEntry> getMotionLedger(const std::string& peer_id)`

Get motion ledger for a peer.

**Parameters:**
- `peer_id`: The peer's unique identifier (empty for local ledger)

**Returns:** Vector of motion entries

#### `std::vector<GoalEntry> getGoalLedger(const std::string& peer_id)`

Get goal ledger for a peer.

**Parameters:**
- `peer_id`: The peer's unique identifier (empty for local ledger)

**Returns:** Vector of goal entries

### Security Operations

#### `bool verifyPeerIdentity(const std::string& peer_id)`

Verify peer identity using zero-trust protocol.

**Parameters:**
- `peer_id`: The peer's unique identifier

**Returns:** `true` if identity verified

#### `bool verifyMotionHash(const std::string& peer_id, const std::string& hash)`

Verify motion hash for a peer.

**Parameters:**
- `peer_id`: The peer's unique identifier
- `hash`: The hash to verify

**Returns:** `true` if hash matches

### Monitoring

#### `SystemMetrics getMetrics() const`

Get system metrics including message counts, consensus rates, and performance data.

**Returns:** System metrics structure

#### `void setMetricsCallback(std::function<void(const SystemMetrics&)> callback)`

Set callback for metrics updates.

**Parameters:**
- `callback`: Function to handle metrics updates

#### `bool isConnected() const`

Check if client is connected to the network.

**Returns:** `true` if connected

## ConsensusManager

Handles consensus-based task allocation and voting.

### Header

```cpp
#include "robocon_network_client/consensus_manager.hpp"
```

### Constructor

```cpp
explicit ConsensusManager(const std::string& robot_id);
```

### Core Methods

#### `ConsensusResult proposeAction(const ActionSuggestion& action)`

Propose an action for consensus voting.

**Parameters:**
- `action`: The action suggestion containing action details

**Returns:** `ConsensusResult` with voting outcome

#### `void voteOnAction(const std::string& action_id, bool vote)`

Vote on a proposed action.

**Parameters:**
- `action_id`: The action identifier
- `vote`: `true` for yes, `false` for no

#### `void handleActionSuggestion(const ActionSuggestion& action)`

Handle incoming action suggestion from another robot.

**Parameters:**
- `action`: The action suggestion to handle

#### `void handleVote(const Vote& vote)`

Handle incoming vote from another robot.

**Parameters:**
- `vote`: The vote to handle

### Behavior Tree Operations

#### `void updateBehaviorTree(const BehaviorTreeNode& node)`

Update local behavior tree with a new or modified node.

**Parameters:**
- `node`: The behavior tree node to update

#### `void replicatePeerBehaviorTree(const std::string& peer_id, const BehaviorTree& tree)`

Replicate a peer's behavior tree for audit and coordination.

**Parameters:**
- `peer_id`: The peer's unique identifier
- `tree`: The behavior tree to replicate

### Verification

#### `bool verifyPeerIdentity(const std::string& peer_id)`

Verify peer identity before allowing participation in consensus.

**Parameters:**
- `peer_id`: The peer's unique identifier

**Returns:** `true` if identity verified

#### `bool verifyMotionHash(const std::string& peer_id, const std::string& hash)`

Verify motion hash for consensus participation.

**Parameters:**
- `peer_id`: The peer's unique identifier
- `hash`: The hash to verify

**Returns:** `true` if hash matches

### Status and Monitoring

#### `ConsensusResult getConsensusStatus(const std::string& action_id) const`

Get current consensus status for an action.

**Parameters:**
- `action_id`: The action identifier

**Returns:** Consensus result with current voting state

#### `std::vector<ActionSuggestion> getPendingActions() const`

Get all pending action suggestions.

**Returns:** Vector of pending actions

#### `void setConsensusCallback(std::function<void(const ConsensusResult&)> callback)`

Set callback for consensus results.

**Parameters:**
- `callback`: Function to handle consensus results

## LedgerManager

Manages motion and goal ledgers with blockchain-style structure.

### Header

```cpp
#include "robocon_network_client/ledger_manager.hpp"
```

### Constructor

```cpp
explicit LedgerManager(const std::string& robot_id);
```

### Motion Ledger Operations

#### `void appendMotionLedger(const MotionEntry& entry)`

Append entry to motion ledger with hash calculation.

**Parameters:**
- `entry`: The motion entry to append

#### `std::vector<MotionEntry> getMotionLedger(const std::string& peer_id) const`

Get motion ledger for a peer.

**Parameters:**
- `peer_id`: The peer's unique identifier (empty for local ledger)

**Returns:** Vector of motion entries in chronological order

#### `std::optional<MotionEntry> getMotionEntry(const std::string& entry_id) const`

Get motion ledger entry by ID.

**Parameters:**
- `entry_id`: The entry identifier

**Returns:** Motion entry if found

#### `std::optional<std::string> getLatestMotionHash(const std::string& peer_id) const`

Get latest motion hash for a peer.

**Parameters:**
- `peer_id`: The peer's unique identifier

**Returns:** Latest hash if available

#### `bool verifyMotionLedger(const std::string& peer_id) const`

Verify motion ledger integrity by checking hash chain.

**Parameters:**
- `peer_id`: The peer's unique identifier

**Returns:** `true` if ledger is valid

### Goal Ledger Operations

#### `void appendGoalLedger(const GoalEntry& entry)`

Append entry to goal ledger with hash calculation.

**Parameters:**
- `entry`: The goal entry to append

#### `std::vector<GoalEntry> getGoalLedger(const std::string& peer_id) const`

Get goal ledger for a peer.

**Parameters:**
- `peer_id`: The peer's unique identifier (empty for local ledger)

**Returns:** Vector of goal entries in chronological order

#### `std::optional<GoalEntry> getGoalEntry(const std::string& entry_id) const`

Get goal ledger entry by ID.

**Parameters:**
- `entry_id`: The entry identifier

**Returns:** Goal entry if found

#### `std::optional<std::string> getLatestGoalHash(const std::string& peer_id) const`

Get latest goal hash for a peer.

**Parameters:**
- `peer_id`: The peer's unique identifier

**Returns:** Latest hash if available

#### `bool verifyGoalLedger(const std::string& peer_id) const`

Verify goal ledger integrity by checking hash chain.

**Parameters:**
- `peer_id`: The peer's unique identifier

**Returns:** `true` if ledger is valid

### Cross-Ledger Operations

#### `void linkGoalToMotions(const std::string& goal_entry_id, const std::vector<std::string>& motion_entry_ids)`

Link goal entry to motion entries.

**Parameters:**
- `goal_entry_id`: The goal entry identifier
- `motion_entry_ids`: Vector of motion entry identifiers

#### `std::vector<MotionEntry> getMotionsForGoal(const std::string& goal_entry_id) const`

Get motion entries linked to a goal.

**Parameters:**
- `goal_entry_id`: The goal entry identifier

**Returns:** Vector of linked motion entries

#### `std::vector<GoalEntry> getGoalsForMotion(const std::string& motion_entry_id) const`

Get goal entries linked to a motion.

**Parameters:**
- `motion_entry_id`: The motion entry identifier

**Returns:** Vector of linked goal entries

### Utility Operations

#### `std::string generateHash(const std::string& data) const`

Generate SHA-256 hash for data.

**Parameters:**
- `data`: The data to hash

**Returns:** SHA-256 hash string

#### `std::string signEntry(const std::string& entry_data) const`

Sign entry with robot's private key.

**Parameters:**
- `entry_data`: The entry data to sign

**Returns:** Digital signature

#### `bool verifySignature(const std::string& entry_data, const std::string& signature, const std::string& public_key) const`

Verify signature.

**Parameters:**
- `entry_data`: The entry data
- `signature`: The signature to verify
- `public_key`: The public key for verification

**Returns:** `true` if signature is valid

#### `LedgerStats getStats() const`

Get ledger statistics.

**Returns:** `LedgerStats` structure with entry counts and peer information

#### `void setLedgerCallback(std::function<void(const std::string&, const std::string&)> callback)`

Set callback for ledger updates.

**Parameters:**
- `callback`: Function called when ledger is updated (peer_id, entry_type)

#### `bool exportLedger(const std::string& filename, const std::string& peer_id = "") const`

Export ledger to file.

**Parameters:**
- `filename`: The output filename
- `peer_id`: The peer's unique identifier (empty for all)

**Returns:** `true` if export successful

#### `bool importLedger(const std::string& filename)`

Import ledger from file.

**Parameters:**
- `filename`: The input filename

**Returns:** `true` if import successful

## SecurityManager

Implements zero-trust verification protocol.

### Header

```cpp
#include "robocon_network_client/security_manager.hpp"
```

### Constructor

```cpp
explicit SecurityManager(const std::string& robot_id);
```

### Methods

#### `bool verifyPeerIdentity(const std::string& peer_id)`

Verify peer identity using zero-trust protocol.

**Parameters:**
- `peer_id`: The peer's unique identifier

**Returns:** `true` if identity verified

#### `bool verifyMotionHash(const std::string& peer_id, const std::string& hash)`

Verify motion hash for a peer.

**Parameters:**
- `peer_id`: The peer's unique identifier
- `hash`: The hash to verify

**Returns:** `true` if hash matches

#### `std::string generateHash(const std::string& data)`

Generate hash for data.

**Parameters:**
- `data`: The data to hash

**Returns:** Hash string

#### `std::string signData(const std::string& data)`

Sign data with robot's private key.

**Parameters:**
- `data`: The data to sign

**Returns:** Digital signature

#### `bool verifySignature(const std::string& data, const std::string& signature, const std::string& public_key)`

Verify signature.

**Parameters:**
- `data`: The data
- `signature`: The signature to verify
- `public_key`: The public key for verification

**Returns:** `true` if signature is valid

## Data Structures

### Pose

```cpp
struct Pose {
    double x, y, z;           // Position
    double qx, qy, qz, qw;    // Orientation (quaternion)
};
```

### SensorDataRequest

```cpp
struct SensorDataRequest {
    std::string request_id;
    std::string robot_id;
    std::string sensor_type;
    std::chrono::system_clock::time_point timestamp;
    std::string requester_id;
};
```

### SensorData

```cpp
struct SensorData {
    std::string data_id;
    std::string robot_id;
    std::string sensor_type;
    std::chrono::system_clock::time_point timestamp;
    std::vector<uint8_t> data;
    std::string data_format;
    double resolution;
    double precision;
};
```

### PeerInfo

```cpp
struct PeerInfo {
    std::string peer_id;
    std::string peer_type;  // "robot", "tablet", "laptop"
    std::vector<std::string> capabilities;
    Pose current_pose;
    std::chrono::system_clock::time_point last_seen;
    bool is_online;
    std::string version;
    std::string signature;
};
```

### ActionSuggestion

```cpp
struct ActionSuggestion {
    std::string action_id;
    std::string proposer_id;
    std::string action_type;
    std::string target_location;
    double estimated_cost;
    std::vector<std::string> required_capabilities;
    std::chrono::system_clock::time_point proposal_timestamp;
    std::string behavior_tree_node;
    std::string world_state_hash;
};
```

### Vote

```cpp
struct Vote {
    std::string action_id;
    std::string voter_id;
    bool vote;  // true for yes, false for no
    std::chrono::system_clock::time_point vote_timestamp;
    std::string reason;
    std::string signature;
};
```

### ConsensusResult

```cpp
struct ConsensusResult {
    std::string action_id;
    bool consensus_reached;
    uint32_t total_votes;
    uint32_t yes_votes;
    uint32_t no_votes;
    std::chrono::system_clock::time_point consensus_timestamp;
    std::string executor_id;
};
```

### MotionEntry

```cpp
struct MotionEntry {
    std::string entry_id;
    std::string robot_id;
    std::chrono::system_clock::time_point timestamp;
    Pose pose;
    double precision;
    double resolution;
    std::string shape_hash;
    std::string previous_hash;
    std::string current_hash;
    std::string signature;
};
```

### GoalEntry

```cpp
struct GoalEntry {
    std::string entry_id;
    std::string robot_id;
    std::chrono::system_clock::time_point timestamp;
    std::string natural_language_goal;
    std::vector<std::string> motion_entry_links;
    std::string previous_hash;
    std::string current_hash;
    std::string signature;
};
```

### SystemMetrics

```cpp
struct SystemMetrics {
    uint64_t messages_sent;
    uint64_t messages_received;
    uint64_t consensus_attempts;
    uint64_t consensus_successes;
    double consensus_success_rate;
    double avg_response_time_ms;
    uint64_t peers_discovered;
    uint64_t ledger_entries;
    std::chrono::system_clock::time_point last_update;
};
```

## Error Handling

All methods that return `bool` indicate success with `true` and failure with `false`. Methods that return optional types use `std::optional` to indicate when a value is not available.

Exceptions may be thrown for:
- Invalid parameters
- Network failures
- Cryptographic operations failures
- Resource allocation failures

## Thread Safety

The Network Client classes are designed to be thread-safe. Multiple threads can safely call methods concurrently, with internal synchronization handled by the implementation.

## Next Steps

- [Discovery Flow](discovery-flow.md) - Detailed discovery protocol flow
- [Voting Flow](voting-flow.md) - Detailed voting protocol flow
- [Code Examples](examples.md) - Example code in C, C++, and Python 3

