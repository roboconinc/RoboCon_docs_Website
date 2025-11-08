# Discovery Manager Implementation

The `DiscoveryManager` class implements peer discovery and capability exchange for the Zero-Trust Consensus Protocol. It handles discovery beacons, peer responses, and capability announcements.

## Header File Structure

```cpp
// discovery_manager.hpp
#ifndef ROBOCON_NETWORK_CLIENT_DISCOVERY_MANAGER_HPP
#define ROBOCON_NETWORK_CLIENT_DISCOVERY_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <chrono>
#include <thread>

#include "robocon_network_client/msg/peer_info.hpp"

namespace robocon {

struct PeerInfo {
    std::string peer_id;
    std::string peer_type;  // "robot", "tablet", "laptop"
    std::vector<std::string> capabilities;
    Pose current_pose;
    std::chrono::system_clock::time_point last_seen;
    bool is_online;
    std::string version;
    std::string signature;
    std::string public_key;
};

class DiscoveryManager {
public:
    DiscoveryManager(
        const std::string& robot_id,
        rclcpp::Node::SharedPtr node
    );
    
    ~DiscoveryManager();
    
    bool initialize();
    void shutdown();
    
    // Discovery Operations
    std::vector<PeerInfo> discoverPeers(uint32_t timeout_ms = 5000);
    void announcePresence();
    void handleDiscoveryBeacon(const PeerInfo& beacon);
    
    // Peer Management
    PeerInfo getPeerInfo(const std::string& peer_id) const;
    std::vector<PeerInfo> getKnownPeers() const;
    bool isPeerKnown(const std::string& peer_id) const;
    void updatePeerInfo(const PeerInfo& info);
    void removePeer(const std::string& peer_id);
    
    // Capability Management
    void setCapabilities(const std::vector<std::string>& capabilities);
    std::vector<std::string> getCapabilities() const;
    
    // Status
    bool isConnected() const;
    size_t getPeerCount() const;
    void setDiscoveryCallback(
        std::function<void(const PeerInfo&)> callback
    );

private:
    std::string robot_id_;
    rclcpp::Node::SharedPtr node_;
    bool initialized_;
    
    std::unordered_map<std::string, PeerInfo> known_peers_;
    mutable std::mutex peers_mutex_;
    
    std::vector<std::string> local_capabilities_;
    std::string robot_version_;
    
    bool discovery_running_;
    std::thread discovery_thread_;
    
    std::function<void(const PeerInfo&)> discovery_callback_;
    
    // ROS 2 Publishers
    rclcpp::Publisher<PeerInfo>::SharedPtr beacon_publisher_;
    
    // ROS 2 Subscribers
    rclcpp::Subscription<PeerInfo>::SharedPtr beacon_subscriber_;
    rclcpp::Subscription<PeerInfo>::SharedPtr response_subscriber_;
    
    // Callbacks
    void onDiscoveryBeaconReceived(const PeerInfo::SharedPtr msg);
    void onDiscoveryResponseReceived(const PeerInfo::SharedPtr msg);
    
    void setupROSPublishers();
    void setupROSSubscribers();
    
    // Internal methods
    void discoveryLoop();
    void cleanupStalePeers();
    PeerInfo createLocalPeerInfo() const;
    
    static constexpr uint32_t BEACON_INTERVAL_MS = 5000;  // 5 seconds
    static constexpr uint32_t PEER_TIMEOUT_MS = 30000;    // 30 seconds
};

} // namespace robocon

#endif // ROBOCON_NETWORK_CLIENT_DISCOVERY_MANAGER_HPP
```

## Implementation

```cpp
// discovery_manager.cpp
#include "robocon_network_client/discovery_manager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <chrono>

namespace robocon {

DiscoveryManager::DiscoveryManager(
    const std::string& robot_id,
    rclcpp::Node::SharedPtr node
) : robot_id_(robot_id),
    node_(node),
    initialized_(false),
    robot_version_("1.0.0"),
    discovery_running_(false)
{
    // Default capabilities
    local_capabilities_ = {
        "navigation",
        "manipulation",
        "sensing",
        "communication"
    };
}

DiscoveryManager::~DiscoveryManager() {
    if (initialized_) {
        shutdown();
    }
}

bool DiscoveryManager::initialize() {
    if (initialized_) {
        return true;
    }
    
    RCLCPP_INFO(node_->get_logger(), 
                "Initializing DiscoveryManager for robot: %s", 
                robot_id_.c_str());
    
    setupROSPublishers();
    setupROSSubscribers();
    
    // Start discovery loop
    discovery_running_ = true;
    discovery_thread_ = std::thread(&DiscoveryManager::discoveryLoop, this);
    
    initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), 
                "DiscoveryManager initialized successfully");
    
    return true;
}

void DiscoveryManager::shutdown() {
    if (!initialized_) {
        return;
    }
    
    discovery_running_ = false;
    if (discovery_thread_.joinable()) {
        discovery_thread_.join();
    }
    
    std::lock_guard<std::mutex> lock(peers_mutex_);
    known_peers_.clear();
    initialized_ = false;
}

void DiscoveryManager::setupROSPublishers() {
    beacon_publisher_ = node_->create_publisher<PeerInfo>(
        "/discovery/beacon",
        rclcpp::QoS(10).reliable().transient_local()
    );
}

void DiscoveryManager::setupROSSubscribers() {
    beacon_subscriber_ = node_->create_subscription<PeerInfo>(
        "/discovery/beacon",
        rclcpp::QoS(10).reliable().transient_local(),
        std::bind(&DiscoveryManager::onDiscoveryBeaconReceived, 
                  this, std::placeholders::_1)
    );
    
    response_subscriber_ = node_->create_subscription<PeerInfo>(
        "/discovery/response",
        rclcpp::QoS(10).reliable().transient_local(),
        std::bind(&DiscoveryManager::onDiscoveryResponseReceived, 
                  this, std::placeholders::_1)
    );
}

void DiscoveryManager::announcePresence() {
    if (!initialized_) {
        return;
    }
    
    PeerInfo local_info = createLocalPeerInfo();
    
    // Publish beacon
    auto msg = std::make_shared<PeerInfo>();
    msg->peer_id = local_info.peer_id;
    msg->peer_type = local_info.peer_type;
    msg->capabilities = local_info.capabilities;
    msg->current_pose = local_info.current_pose;
    msg->version = local_info.version;
    msg->signature = local_info.signature;
    msg->public_key = local_info.public_key;
    msg->is_online = true;
    msg->last_seen = rclcpp::Clock().now();
    
    beacon_publisher_->publish(*msg);
    
    RCLCPP_DEBUG(node_->get_logger(), 
                 "Announced presence: %s", 
                 robot_id_.c_str());
}

std::vector<PeerInfo> DiscoveryManager::discoverPeers(
    uint32_t timeout_ms
) {
    if (!initialized_) {
        return {};
    }
    
    // Announce presence to trigger discovery
    announcePresence();
    
    // Wait for responses
    std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));
    
    // Clean up stale peers
    cleanupStalePeers();
    
    // Return known peers
    std::lock_guard<std::mutex> lock(peers_mutex_);
    std::vector<PeerInfo> peers;
    for (const auto& [peer_id, info] : known_peers_) {
        if (info.is_online) {
            peers.push_back(info);
        }
    }
    
    RCLCPP_INFO(node_->get_logger(), 
                "Discovered %zu peers", 
                peers.size());
    
    return peers;
}

void DiscoveryManager::handleDiscoveryBeacon(
    const PeerInfo& beacon
) {
    if (beacon.peer_id == robot_id_) {
        return;  // Ignore our own beacons
    }
    
    std::lock_guard<std::mutex> lock(peers_mutex_);
    
    // Update or add peer
    PeerInfo info = beacon;
    info.last_seen = std::chrono::system_clock::now();
    info.is_online = true;
    
    known_peers_[beacon.peer_id] = info;
    
    RCLCPP_INFO(node_->get_logger(), 
                "Discovered peer: %s (type: %s)", 
                beacon.peer_id.c_str(),
                beacon.peer_type.c_str());
    
    // Call discovery callback if set
    if (discovery_callback_) {
        discovery_callback_(info);
    }
    
    // Respond with our own info
    PeerInfo local_info = createLocalPeerInfo();
    auto response_msg = std::make_shared<PeerInfo>();
    response_msg->peer_id = local_info.peer_id;
    response_msg->peer_type = local_info.peer_type;
    response_msg->capabilities = local_info.capabilities;
    response_msg->current_pose = local_info.current_pose;
    response_msg->version = local_info.version;
    response_msg->signature = local_info.signature;
    response_msg->public_key = local_info.public_key;
    response_msg->is_online = true;
    response_msg->last_seen = rclcpp::Clock().now();
    
    // Publish response (in real implementation, this would be on a response topic)
    beacon_publisher_->publish(*response_msg);
}

void DiscoveryManager::onDiscoveryBeaconReceived(
    const PeerInfo::SharedPtr msg
) {
    // Convert ROS message to PeerInfo struct
    PeerInfo info;
    info.peer_id = msg->peer_id;
    info.peer_type = msg->peer_type;
    info.capabilities = msg->capabilities;
    info.current_pose = msg->current_pose;
    info.version = msg->version;
    info.signature = msg->signature;
    info.public_key = msg->public_key;
    info.is_online = msg->is_online;
    info.last_seen = std::chrono::system_clock::time_point(
        std::chrono::nanoseconds(msg->last_seen.nanosec)
    );
    
    handleDiscoveryBeacon(info);
}

void DiscoveryManager::onDiscoveryResponseReceived(
    const PeerInfo::SharedPtr msg
) {
    // Handle discovery response (similar to beacon)
    onDiscoveryBeaconReceived(msg);
}

PeerInfo DiscoveryManager::getPeerInfo(const std::string& peer_id) const {
    std::lock_guard<std::mutex> lock(peers_mutex_);
    auto it = known_peers_.find(peer_id);
    if (it != known_peers_.end()) {
        return it->second;
    }
    
    return PeerInfo{};  // Return empty peer info if not found
}

std::vector<PeerInfo> DiscoveryManager::getKnownPeers() const {
    std::lock_guard<std::mutex> lock(peers_mutex_);
    std::vector<PeerInfo> peers;
    for (const auto& [peer_id, info] : known_peers_) {
        peers.push_back(info);
    }
    return peers;
}

bool DiscoveryManager::isPeerKnown(const std::string& peer_id) const {
    std::lock_guard<std::mutex> lock(peers_mutex_);
    return known_peers_.find(peer_id) != known_peers_.end();
}

void DiscoveryManager::updatePeerInfo(const PeerInfo& info) {
    std::lock_guard<std::mutex> lock(peers_mutex_);
    known_peers_[info.peer_id] = info;
    known_peers_[info.peer_id].last_seen = std::chrono::system_clock::now();
}

void DiscoveryManager::removePeer(const std::string& peer_id) {
    std::lock_guard<std::mutex> lock(peers_mutex_);
    known_peers_.erase(peer_id);
}

void DiscoveryManager::setCapabilities(
    const std::vector<std::string>& capabilities
) {
    local_capabilities_ = capabilities;
}

std::vector<std::string> DiscoveryManager::getCapabilities() const {
    return local_capabilities_;
}

bool DiscoveryManager::isConnected() const {
    std::lock_guard<std::mutex> lock(peers_mutex_);
    return !known_peers_.empty();
}

size_t DiscoveryManager::getPeerCount() const {
    std::lock_guard<std::mutex> lock(peers_mutex_);
    size_t count = 0;
    for (const auto& [peer_id, info] : known_peers_) {
        if (info.is_online) {
            count++;
        }
    }
    return count;
}

void DiscoveryManager::setDiscoveryCallback(
    std::function<void(const PeerInfo&)> callback
) {
    discovery_callback_ = callback;
}

void DiscoveryManager::discoveryLoop() {
    while (discovery_running_ && rclcpp::ok()) {
        // Periodically announce presence
        announcePresence();
        
        // Clean up stale peers
        cleanupStalePeers();
        
        // Sleep until next beacon
        std::this_thread::sleep_for(
            std::chrono::milliseconds(BEACON_INTERVAL_MS)
        );
    }
}

void DiscoveryManager::cleanupStalePeers() {
    auto now = std::chrono::system_clock::now();
    
    std::lock_guard<std::mutex> lock(peers_mutex_);
    
    std::vector<std::string> to_remove;
    
    for (const auto& [peer_id, info] : known_peers_) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - info.last_seen
        );
        
        if (elapsed.count() > PEER_TIMEOUT_MS) {
            to_remove.push_back(peer_id);
        }
    }
    
    for (const auto& peer_id : to_remove) {
        known_peers_.erase(peer_id);
        RCLCPP_INFO(node_->get_logger(), 
                    "Removed stale peer: %s", 
                    peer_id.c_str());
    }
}

PeerInfo DiscoveryManager::createLocalPeerInfo() const {
    PeerInfo info;
    info.peer_id = robot_id_;
    info.peer_type = "robot";
    info.capabilities = local_capabilities_;
    info.current_pose = Pose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};  // Placeholder
    info.version = robot_version_;
    info.is_online = true;
    info.last_seen = std::chrono::system_clock::now();
    // In real implementation, signature and public_key would be set
    // from SecurityManager
    info.signature = "";
    info.public_key = "";
    
    return info;
}

} // namespace robocon
```

## Usage Example

```cpp
#include "robocon_network_client/discovery_manager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("discovery_example");
    
    robocon::DiscoveryManager discovery("robot_001", node);
    discovery.initialize();
    
    // Set capabilities
    discovery.setCapabilities({
        "navigation",
        "manipulation",
        "sensing",
        "communication",
        "lidar"
    });
    
    // Set discovery callback
    discovery.setDiscoveryCallback([](const robocon::PeerInfo& peer) {
        std::cout << "New peer discovered: " << peer.peer_id << std::endl;
        std::cout << "Type: " << peer.peer_type << std::endl;
        std::cout << "Capabilities: ";
        for (const auto& cap : peer.capabilities) {
            std::cout << cap << " ";
        }
        std::cout << std::endl;
    });
    
    // Discover peers
    auto peers = discovery.discoverPeers(10000);
    std::cout << "Discovered " << peers.size() << " peers" << std::endl;
    
    // Get known peers
    auto known_peers = discovery.getKnownPeers();
    for (const auto& peer : known_peers) {
        std::cout << "Known peer: " << peer.peer_id << std::endl;
    }
    
    // Keep running
    rclcpp::spin(node);
    
    discovery.shutdown();
    rclcpp::shutdown();
    return 0;
}
```

## Key Implementation Details

1. **Periodic Beacons**: Automatically sends discovery beacons at regular intervals.

2. **Peer Tracking**: Maintains a map of known peers with their capabilities and status.

3. **Stale Peer Cleanup**: Automatically removes peers that haven't been seen within a timeout period.

4. **Capability Exchange**: Exchanges capabilities during discovery to enable task allocation.

5. **Thread Safety**: Uses mutexes to protect shared state.

6. **ROS 2 Integration**: All discovery communication flows through ROS 2 topics.

## Next Steps

- [Network Client Implementation](network-client.md) - Main interface
- [Security Manager Implementation](security-manager.md) - Zero-trust verification
- [ROS 2 Messages and Topics](ros2-integration.md) - Message definitions

