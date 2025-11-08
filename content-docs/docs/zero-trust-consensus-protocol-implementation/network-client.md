# Network Client Implementation

The `NetworkClient` class is the main interface for the Zero-Trust Consensus Protocol. It coordinates all protocol components and provides a unified API for ROS 2 applications.

## Header File Structure

```cpp
// network_client.hpp
#ifndef ROBOCON_NETWORK_CLIENT_NETWORK_CLIENT_HPP
#define ROBOCON_NETWORK_CLIENT_NETWORK_CLIENT_HPP

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <vector>
#include <functional>

#include "robocon_network_client/consensus_manager.hpp"
#include "robocon_network_client/ledger_manager.hpp"
#include "robocon_network_client/security_manager.hpp"
#include "robocon_network_client/discovery_manager.hpp"
#include "robocon_network_client/msg/action_suggestion.hpp"
#include "robocon_network_client/msg/peer_info.hpp"
#include "robocon_network_client/msg/sensor_data.hpp"

namespace robocon {

class NetworkClient {
public:
    explicit NetworkClient(
        const std::string& robot_id,
        const std::string& config_path = "",
        rclcpp::Node::SharedPtr node = nullptr
    );
    
    ~NetworkClient();
    
    bool initialize();
    void shutdown();
    
    // Consensus operations
    ConsensusManager& getConsensusManager();
    ConsensusResult proposeAction(const ActionSuggestion& action);
    
    // Ledger operations
    LedgerManager& getLedgerManager();
    void appendMotionLedger(const MotionEntry& entry);
    void appendGoalLedger(const GoalEntry& entry);
    
    // Security operations
    SecurityManager& getSecurityManager();
    bool verifyPeerIdentity(const std::string& peer_id);
    
    // Discovery operations
    std::vector<PeerInfo> discoverPeers(uint32_t timeout_ms = 5000);
    void announcePresence();
    
    // Sensor data operations
    SensorDataResponse sendSensorDataRequest(
        const SensorDataRequest& request,
        uint32_t timeout_ms = 5000
    );
    
    // Status
    bool isConnected() const;
    SystemMetrics getMetrics() const;

private:
    std::string robot_id_;
    std::string config_path_;
    rclcpp::Node::SharedPtr node_;
    bool initialized_;
    
    std::unique_ptr<ConsensusManager> consensus_manager_;
    std::unique_ptr<LedgerManager> ledger_manager_;
    std::unique_ptr<SecurityManager> security_manager_;
    std::unique_ptr<DiscoveryManager> discovery_manager_;
    
    // ROS 2 Publishers
    rclcpp::Publisher<ActionSuggestion>::SharedPtr action_publisher_;
    rclcpp::Publisher<PeerInfo>::SharedPtr discovery_publisher_;
    
    // ROS 2 Subscribers
    rclcpp::Subscription<ActionSuggestion>::SharedPtr action_subscriber_;
    rclcpp::Subscription<PeerInfo>::SharedPtr discovery_subscriber_;
    
    // Callbacks
    void onActionSuggestionReceived(const ActionSuggestion::SharedPtr msg);
    void onDiscoveryBeaconReceived(const PeerInfo::SharedPtr msg);
    
    void setupROSPublishers();
    void setupROSSubscribers();
};

} // namespace robocon

#endif // ROBOCON_NETWORK_CLIENT_NETWORK_CLIENT_HPP
```

## Implementation

```cpp
// network_client.cpp
#include "robocon_network_client/network_client.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>

namespace robocon {

NetworkClient::NetworkClient(
    const std::string& robot_id,
    const std::string& config_path,
    rclcpp::Node::SharedPtr node
) : robot_id_(robot_id),
    config_path_(config_path),
    node_(node),
    initialized_(false)
{
    // Create ROS 2 node if not provided
    if (!node_) {
        node_ = std::make_shared<rclcpp::Node>(
            "network_client_" + robot_id_,
            rclcpp::NodeOptions().use_intra_process_comms(false)
        );
    }
    
    // Initialize managers
    consensus_manager_ = std::make_unique<ConsensusManager>(robot_id_, node_);
    ledger_manager_ = std::make_unique<LedgerManager>(robot_id_, node_);
    security_manager_ = std::make_unique<SecurityManager>(robot_id_, node_);
    discovery_manager_ = std::make_unique<DiscoveryManager>(robot_id_, node_);
}

NetworkClient::~NetworkClient() {
    if (initialized_) {
        shutdown();
    }
}

bool NetworkClient::initialize() {
    if (initialized_) {
        return true;
    }
    
    RCLCPP_INFO(node_->get_logger(), 
                "Initializing Network Client for robot: %s", 
                robot_id_.c_str());
    
    // Initialize ROS 2 publishers and subscribers
    setupROSPublishers();
    setupROSPSubscribers();
    
    // Initialize component managers
    if (!consensus_manager_->initialize()) {
        RCLCPP_ERROR(node_->get_logger(), 
                     "Failed to initialize ConsensusManager");
        return false;
    }
    
    if (!ledger_manager_->initialize()) {
        RCLCPP_ERROR(node_->get_logger(), 
                     "Failed to initialize LedgerManager");
        return false;
    }
    
    if (!security_manager_->initialize()) {
        RCLCPP_ERROR(node_->get_logger(), 
                     "Failed to initialize SecurityManager");
        return false;
    }
    
    if (!discovery_manager_->initialize()) {
        RCLCPP_ERROR(node_->get_logger(), 
                     "Failed to initialize DiscoveryManager");
        return false;
    }
    
    initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), 
                "Network Client initialized successfully");
    
    return true;
}

void NetworkClient::shutdown() {
    if (!initialized_) {
        return;
    }
    
    RCLCPP_INFO(node_->get_logger(), 
                "Shutting down Network Client");
    
    // Shutdown managers in reverse order
    discovery_manager_->shutdown();
    security_manager_->shutdown();
    ledger_manager_->shutdown();
    consensus_manager_->shutdown();
    
    initialized_ = false;
}

void NetworkClient::setupROSPublishers() {
    // Publisher for action suggestions
    action_publisher_ = node_->create_publisher<ActionSuggestion>(
        "/consensus/action_suggestion",
        rclcpp::QoS(10).reliable().transient_local()
    );
    
    // Publisher for discovery beacons
    discovery_publisher_ = node_->create_publisher<PeerInfo>(
        "/discovery/beacon",
        rclcpp::QoS(10).reliable().transient_local()
    );
}

void NetworkClient::setupROSSubscribers() {
    // Subscriber for action suggestions
    action_subscriber_ = node_->create_subscription<ActionSuggestion>(
        "/consensus/action_suggestion",
        rclcpp::QoS(10).reliable().transient_local(),
        std::bind(&NetworkClient::onActionSuggestionReceived, 
                  this, std::placeholders::_1)
    );
    
    // Subscriber for discovery beacons
    discovery_subscriber_ = node_->create_subscription<PeerInfo>(
        "/discovery/beacon",
        rclcpp::QoS(10).reliable().transient_local(),
        std::bind(&NetworkClient::onDiscoveryBeaconReceived, 
                  this, std::placeholders::_1)
    );
}

void NetworkClient::onActionSuggestionReceived(
    const ActionSuggestion::SharedPtr msg
) {
    // Forward to consensus manager
    consensus_manager_->handleActionSuggestion(*msg);
}

void NetworkClient::onDiscoveryBeaconReceived(
    const PeerInfo::SharedPtr msg
) {
    // Forward to discovery manager
    discovery_manager_->handleDiscoveryBeacon(*msg);
}

ConsensusManager& NetworkClient::getConsensusManager() {
    return *consensus_manager_;
}

ConsensusResult NetworkClient::proposeAction(
    const ActionSuggestion& action
) {
    if (!initialized_) {
        RCLCPP_ERROR(node_->get_logger(), 
                     "NetworkClient not initialized");
        return ConsensusResult{};
    }
    
    // Publish action suggestion to ROS 2 topic
    auto msg = std::make_shared<ActionSuggestion>();
    msg->action_id = action.action_id;
    msg->proposer_id = action.proposer_id;
    msg->action_type = action.action_type;
    msg->target_location = action.target_location;
    msg->estimated_cost = action.estimated_cost;
    msg->required_capabilities = action.required_capabilities;
    msg->behavior_tree_node = action.behavior_tree_node;
    
    action_publisher_->publish(*msg);
    
    // Use consensus manager to handle proposal
    return consensus_manager_->proposeAction(action);
}

LedgerManager& NetworkClient::getLedgerManager() {
    return *ledger_manager_;
}

void NetworkClient::appendMotionLedger(const MotionEntry& entry) {
    ledger_manager_->appendMotionLedger(entry);
}

void NetworkClient::appendGoalLedger(const GoalEntry& entry) {
    ledger_manager_->appendGoalLedger(entry);
}

SecurityManager& NetworkClient::getSecurityManager() {
    return *security_manager_;
}

bool NetworkClient::verifyPeerIdentity(const std::string& peer_id) {
    return security_manager_->verifyPeerIdentity(peer_id);
}

std::vector<PeerInfo> NetworkClient::discoverPeers(uint32_t timeout_ms) {
    return discovery_manager_->discoverPeers(timeout_ms);
}

void NetworkClient::announcePresence() {
    discovery_manager_->announcePresence();
}

bool NetworkClient::isConnected() const {
    return initialized_ && 
           discovery_manager_->isConnected();
}

SystemMetrics NetworkClient::getMetrics() const {
    SystemMetrics metrics;
    metrics.consensus_attempts = consensus_manager_->getConsensusAttempts();
    metrics.consensus_successes = consensus_manager_->getConsensusSuccesses();
    metrics.peers_discovered = discovery_manager_->getPeerCount();
    metrics.ledger_entries = ledger_manager_->getEntryCount();
    return metrics;
}

} // namespace robocon
```

## Usage Example

```cpp
#include "robocon_network_client/network_client.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char** argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // Create ROS 2 node
    auto node = std::make_shared<rclcpp::Node>("example_robot");
    
    // Create Network Client
    robocon::NetworkClient client("robot_001", "", node);
    
    // Initialize client
    if (!client.initialize()) {
        std::cerr << "Failed to initialize network client" << std::endl;
        return 1;
    }
    
    // Discover peers
    auto peers = client.discoverPeers(5000);
    std::cout << "Discovered " << peers.size() << " peers" << std::endl;
    
    // Propose an action
    robocon::ActionSuggestion action;
    action.action_id = "action_001";
    action.proposer_id = "robot_001";
    action.action_type = "move_sheathing";
    action.target_location = "building_B_floor_2_room_5";
    action.estimated_cost = 150.0;
    action.required_capabilities = {"manipulation", "navigation"};
    
    auto result = client.proposeAction(action);
    
    if (result.consensus_reached) {
        std::cout << "Consensus reached! Executor: " 
                  << result.executor_id << std::endl;
    }
    
    // Cleanup
    client.shutdown();
    rclcpp::shutdown();
    
    return 0;
}
```

## ROS 2 Node Integration

The `NetworkClient` can be used as a component within a ROS 2 node:

```cpp
#include "robocon_network_client/network_client.hpp"
#include <rclcpp/rclcpp.hpp>

class RobotControlNode : public rclcpp::Node {
public:
    RobotControlNode() : Node("robot_control") {
        // Initialize Network Client
        network_client_ = std::make_unique<robocon::NetworkClient>(
            "robot_001",
            "",
            shared_from_this()
        );
        
        network_client_->initialize();
        
        // Set up timer for periodic operations
        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RobotControlNode::timerCallback, this)
        );
    }
    
private:
    void timerCallback() {
        // Periodic operations
        network_client_->announcePresence();
        
        auto metrics = network_client_->getMetrics();
        RCLCPP_INFO(get_logger(), 
                    "Connected peers: %lu", 
                    metrics.peers_discovered);
    }
    
    std::unique_ptr<robocon::NetworkClient> network_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotControlNode>());
    rclcpp::shutdown();
    return 0;
}
```

## Key Implementation Details

1. **ROS 2 Node Integration**: The NetworkClient can use an existing ROS 2 node or create its own.

2. **DDS Topics**: All communication flows through ROS 2 topics, which are implemented using Cyclone DDS.

3. **Component Managers**: The NetworkClient coordinates four main managers:
   - ConsensusManager
   - LedgerManager
   - SecurityManager
   - DiscoveryManager

4. **Thread Safety**: The implementation is thread-safe and can be used from multiple threads.

5. **Error Handling**: All initialization methods return `bool` to indicate success/failure.

## Next Steps

- [Consensus Manager Implementation](consensus-manager.md) - Detailed voting protocol implementation
- [Ledger Manager Implementation](ledger-manager.md) - Blockchain-style ledger implementation
- [ROS 2 Messages and Topics](ros2-integration.md) - Custom message definitions

