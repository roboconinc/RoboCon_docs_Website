# Ledger Manager Implementation

The `LedgerManager` class implements the blockchain-style motion and goal ledgers with cryptographic hashing and digital signatures. It maintains tamper-evident records of robot movements and high-level goals.

## Header File Structure

```cpp
// ledger_manager.hpp
#ifndef ROBOCON_NETWORK_CLIENT_LEDGER_MANAGER_HPP
#define ROBOCON_NETWORK_CLIENT_LEDGER_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <optional>
#include <chrono>

#include "robocon_network_client/msg/motion_entry.hpp"
#include "robocon_network_client/msg/goal_entry.hpp"
#include "robocon_network_client/security_manager.hpp"

namespace robocon {

struct LedgerStats {
    uint64_t motion_entries;
    uint64_t goal_entries;
    uint64_t peer_count;
    std::chrono::system_clock::time_point last_update;
};

class LedgerManager {
public:
    LedgerManager(
        const std::string& robot_id,
        rclcpp::Node::SharedPtr node
    );
    
    ~LedgerManager();
    
    bool initialize();
    void shutdown();
    
    // Motion Ledger Operations
    void appendMotionLedger(const MotionEntry& entry);
    std::vector<MotionEntry> getMotionLedger(
        const std::string& peer_id = ""
    ) const;
    std::optional<MotionEntry> getMotionEntry(
        const std::string& entry_id
    ) const;
    std::optional<std::string> getLatestMotionHash(
        const std::string& peer_id = ""
    ) const;
    bool verifyMotionLedger(const std::string& peer_id = "") const;
    
    // Goal Ledger Operations
    void appendGoalLedger(const GoalEntry& entry);
    std::vector<GoalEntry> getGoalLedger(
        const std::string& peer_id = ""
    ) const;
    std::optional<GoalEntry> getGoalEntry(
        const std::string& entry_id
    ) const;
    std::optional<std::string> getLatestGoalHash(
        const std::string& peer_id = ""
    ) const;
    bool verifyGoalLedger(const std::string& peer_id = "") const;
    
    // Cross-Ledger Operations
    void linkGoalToMotions(
        const std::string& goal_entry_id,
        const std::vector<std::string>& motion_entry_ids
    );
    std::vector<MotionEntry> getMotionsForGoal(
        const std::string& goal_entry_id
    ) const;
    std::vector<GoalEntry> getGoalsForMotion(
        const std::string& motion_entry_id
    ) const;
    
    // Utility Operations
    std::string generateHash(const std::string& data) const;
    std::string signEntry(const std::string& entry_data) const;
    bool verifySignature(
        const std::string& entry_data,
        const std::string& signature,
        const std::string& public_key
    ) const;
    
    LedgerStats getStats() const;
    uint64_t getEntryCount() const;
    
    // ROS 2 Integration
    void syncPeerLedger(const std::string& peer_id);

private:
    std::string robot_id_;
    rclcpp::Node::SharedPtr node_;
    bool initialized_;
    
    // Local ledgers
    std::vector<MotionEntry> local_motion_ledger_;
    std::vector<GoalEntry> local_goal_ledger_;
    
    // Peer ledgers (robot_id -> ledger)
    std::unordered_map<std::string, std::vector<MotionEntry>> peer_motion_ledgers_;
    std::unordered_map<std::string, std::vector<GoalEntry>> peer_goal_ledgers_;
    
    // Cross-linking: goal_id -> motion_ids
    std::unordered_map<std::string, std::vector<std::string>> goal_to_motions_;
    // Cross-linking: motion_id -> goal_ids
    std::unordered_map<std::string, std::vector<std::string>> motion_to_goals_;
    
    mutable std::mutex ledger_mutex_;
    
    // Security manager for cryptographic operations
    std::shared_ptr<SecurityManager> security_manager_;
    
    // ROS 2 Publishers
    rclcpp::Publisher<MotionEntry>::SharedPtr motion_publisher_;
    rclcpp::Publisher<GoalEntry>::SharedPtr goal_publisher_;
    
    // ROS 2 Subscribers
    rclcpp::Subscription<MotionEntry>::SharedPtr motion_subscriber_;
    rclcpp::Subscription<GoalEntry>::SharedPtr goal_subscriber_;
    
    // Callbacks
    void onMotionEntryReceived(const MotionEntry::SharedPtr msg);
    void onGoalEntryReceived(const GoalEntry::SharedPtr msg);
    
    void setupROSPublishers();
    void setupROSSubscribers();
    
    // Internal hash calculation
    std::string calculateMotionHash(const MotionEntry& entry) const;
    std::string calculateGoalHash(const GoalEntry& entry) const;
    std::string calculateHashChain(
        const std::string& previous_hash,
        const std::string& current_data
    ) const;
};

} // namespace robocon

#endif // ROBOCON_NETWORK_CLIENT_LEDGER_MANAGER_HPP
```

## Implementation

```cpp
// ledger_manager.cpp
#include "robocon_network_client/ledger_manager.hpp"
#include <openssl/sha.h>
#include <openssl/evp.h>
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace robocon {

LedgerManager::LedgerManager(
    const std::string& robot_id,
    rclcpp::Node::SharedPtr node
) : robot_id_(robot_id),
    node_(node),
    initialized_(false)
{
    security_manager_ = std::make_shared<SecurityManager>(robot_id_, node_);
}

LedgerManager::~LedgerManager() {
    if (initialized_) {
        shutdown();
    }
}

bool LedgerManager::initialize() {
    if (initialized_) {
        return true;
    }
    
    RCLCPP_INFO(node_->get_logger(), 
                "Initializing LedgerManager for robot: %s", 
                robot_id_.c_str());
    
    if (!security_manager_->initialize()) {
        RCLCPP_ERROR(node_->get_logger(), 
                     "Failed to initialize SecurityManager");
        return false;
    }
    
    setupROSPublishers();
    setupROSSubscribers();
    
    initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), 
                "LedgerManager initialized successfully");
    
    return true;
}

void LedgerManager::shutdown() {
    if (!initialized_) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(ledger_mutex_);
    local_motion_ledger_.clear();
    local_goal_ledger_.clear();
    peer_motion_ledgers_.clear();
    peer_goal_ledgers_.clear();
    initialized_ = false;
}

void LedgerManager::setupROSPublishers() {
    motion_publisher_ = node_->create_publisher<MotionEntry>(
        "/ledger/motion",
        rclcpp::QoS(10).reliable().transient_local()
    );
    
    goal_publisher_ = node_->create_publisher<GoalEntry>(
        "/ledger/goal",
        rclcpp::QoS(10).reliable().transient_local()
    );
}

void LedgerManager::setupROSSubscribers() {
    motion_subscriber_ = node_->create_subscription<MotionEntry>(
        "/ledger/motion",
        rclcpp::QoS(10).reliable().transient_local(),
        std::bind(&LedgerManager::onMotionEntryReceived, 
                  this, std::placeholders::_1)
    );
    
    goal_subscriber_ = node_->create_subscription<GoalEntry>(
        "/ledger/goal",
        rclcpp::QoS(10).reliable().transient_local(),
        std::bind(&LedgerManager::onGoalEntryReceived, 
                  this, std::placeholders::_1)
    );
}

void LedgerManager::appendMotionLedger(const MotionEntry& entry) {
    if (!initialized_) {
        RCLCPP_ERROR(node_->get_logger(), 
                     "LedgerManager not initialized");
        return;
    }
    
    std::lock_guard<std::mutex> lock(ledger_mutex_);
    
    // Create new entry with hash calculation
    MotionEntry new_entry = entry;
    new_entry.robot_id = robot_id_;
    new_entry.timestamp = std::chrono::system_clock::now();
    
    // Get previous hash
    if (!local_motion_ledger_.empty()) {
        new_entry.previous_hash = local_motion_ledger_.back().current_hash;
    } else {
        new_entry.previous_hash = "0";  // Genesis hash
    }
    
    // Calculate current hash
    new_entry.current_hash = calculateMotionHash(new_entry);
    
    // Sign the entry
    std::string entry_data = serializeMotionEntry(new_entry);
    new_entry.signature = signEntry(entry_data);
    
    // Append to ledger
    local_motion_ledger_.push_back(new_entry);
    
    RCLCPP_INFO(node_->get_logger(), 
                "Appended motion entry: %s (hash: %s)", 
                new_entry.entry_id.c_str(),
                new_entry.current_hash.substr(0, 16).c_str());
    
    // Publish to ROS 2 topic
    auto msg = std::make_shared<MotionEntry>();
    msg->entry_id = new_entry.entry_id;
    msg->robot_id = new_entry.robot_id;
    msg->timestamp = rclcpp::Time(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            new_entry.timestamp.time_since_epoch()
        ).count()
    );
    msg->pose = new_entry.pose;
    msg->precision = new_entry.precision;
    msg->resolution = new_entry.resolution;
    msg->shape_hash = new_entry.shape_hash;
    msg->previous_hash = new_entry.previous_hash;
    msg->current_hash = new_entry.current_hash;
    msg->signature = new_entry.signature;
    
    motion_publisher_->publish(*msg);
}

void LedgerManager::appendGoalLedger(const GoalEntry& entry) {
    if (!initialized_) {
        RCLCPP_ERROR(node_->get_logger(), 
                     "LedgerManager not initialized");
        return;
    }
    
    std::lock_guard<std::mutex> lock(ledger_mutex_);
    
    // Create new entry with hash calculation
    GoalEntry new_entry = entry;
    new_entry.robot_id = robot_id_;
    new_entry.timestamp = std::chrono::system_clock::now();
    
    // Get previous hash
    if (!local_goal_ledger_.empty()) {
        new_entry.previous_hash = local_goal_ledger_.back().current_hash;
    } else {
        new_entry.previous_hash = "0";  // Genesis hash
    }
    
    // Calculate current hash
    new_entry.current_hash = calculateGoalHash(new_entry);
    
    // Sign the entry
    std::string entry_data = serializeGoalEntry(new_entry);
    new_entry.signature = signEntry(entry_data);
    
    // Append to ledger
    local_goal_ledger_.push_back(new_entry);
    
    RCLCPP_INFO(node_->get_logger(), 
                "Appended goal entry: %s (hash: %s)", 
                new_entry.entry_id.c_str(),
                new_entry.current_hash.substr(0, 16).c_str());
    
    // Publish to ROS 2 topic
    auto msg = std::make_shared<GoalEntry>();
    msg->entry_id = new_entry.entry_id;
    msg->robot_id = new_entry.robot_id;
    msg->timestamp = rclcpp::Time(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            new_entry.timestamp.time_since_epoch()
        ).count()
    );
    msg->natural_language_goal = new_entry.natural_language_goal;
    msg->motion_entry_links = new_entry.motion_entry_links;
    msg->previous_hash = new_entry.previous_hash;
    msg->current_hash = new_entry.current_hash;
    msg->signature = new_entry.signature;
    
    goal_publisher_->publish(*msg);
}

std::vector<MotionEntry> LedgerManager::getMotionLedger(
    const std::string& peer_id
) const {
    std::lock_guard<std::mutex> lock(ledger_mutex_);
    
    if (peer_id.empty() || peer_id == robot_id_) {
        return local_motion_ledger_;
    }
    
    auto it = peer_motion_ledgers_.find(peer_id);
    if (it != peer_motion_ledgers_.end()) {
        return it->second;
    }
    
    return {};
}

std::optional<MotionEntry> LedgerManager::getMotionEntry(
    const std::string& entry_id
) const {
    std::lock_guard<std::mutex> lock(ledger_mutex_);
    
    // Search local ledger
    for (const auto& entry : local_motion_ledger_) {
        if (entry.entry_id == entry_id) {
            return entry;
        }
    }
    
    // Search peer ledgers
    for (const auto& [peer_id, ledger] : peer_motion_ledgers_) {
        for (const auto& entry : ledger) {
            if (entry.entry_id == entry_id) {
                return entry;
            }
        }
    }
    
    return std::nullopt;
}

std::optional<std::string> LedgerManager::getLatestMotionHash(
    const std::string& peer_id
) const {
    std::lock_guard<std::mutex> lock(ledger_mutex_);
    
    const std::vector<MotionEntry>* ledger = nullptr;
    
    if (peer_id.empty() || peer_id == robot_id_) {
        ledger = &local_motion_ledger_;
    } else {
        auto it = peer_motion_ledgers_.find(peer_id);
        if (it != peer_motion_ledgers_.end()) {
            ledger = &it->second;
        }
    }
    
    if (ledger && !ledger->empty()) {
        return ledger->back().current_hash;
    }
    
    return std::nullopt;
}

bool LedgerManager::verifyMotionLedger(const std::string& peer_id) const {
    std::lock_guard<std::mutex> lock(ledger_mutex_);
    
    const std::vector<MotionEntry>* ledger = nullptr;
    
    if (peer_id.empty() || peer_id == robot_id_) {
        ledger = &local_motion_ledger_;
    } else {
        auto it = peer_motion_ledgers_.find(peer_id);
        if (it != peer_motion_ledgers_.end()) {
            ledger = &it->second;
        }
    }
    
    if (!ledger || ledger->empty()) {
        return false;
    }
    
    // Verify hash chain
    std::string previous_hash = "0";
    for (const auto& entry : *ledger) {
        if (entry.previous_hash != previous_hash) {
            RCLCPP_WARN(node_->get_logger(), 
                        "Hash chain broken at entry: %s", 
                        entry.entry_id.c_str());
            return false;
        }
        
        // Verify current hash
        std::string calculated_hash = calculateMotionHash(entry);
        if (calculated_hash != entry.current_hash) {
            RCLCPP_WARN(node_->get_logger(), 
                        "Hash mismatch at entry: %s", 
                        entry.entry_id.c_str());
            return false;
        }
        
        previous_hash = entry.current_hash;
    }
    
    return true;
}

void LedgerManager::linkGoalToMotions(
    const std::string& goal_entry_id,
    const std::vector<std::string>& motion_entry_ids
) {
    std::lock_guard<std::mutex> lock(ledger_mutex_);
    
    goal_to_motions_[goal_entry_id] = motion_entry_ids;
    
    // Reverse mapping
    for (const auto& motion_id : motion_entry_ids) {
        motion_to_goals_[motion_id].push_back(goal_entry_id);
    }
}

std::vector<MotionEntry> LedgerManager::getMotionsForGoal(
    const std::string& goal_entry_id
) const {
    std::lock_guard<std::mutex> lock(ledger_mutex_);
    
    std::vector<MotionEntry> motions;
    auto it = goal_to_motions_.find(goal_entry_id);
    if (it != goal_to_motions_.end()) {
        for (const auto& motion_id : it->second) {
            auto motion = getMotionEntry(motion_id);
            if (motion.has_value()) {
                motions.push_back(motion.value());
            }
        }
    }
    
    return motions;
}

std::string LedgerManager::calculateMotionHash(
    const MotionEntry& entry
) const {
    std::ostringstream oss;
    oss << entry.entry_id << ","
        << entry.robot_id << ","
        << entry.timestamp.time_since_epoch().count() << ","
        << entry.pose.x << "," << entry.pose.y << "," << entry.pose.z << ","
        << entry.pose.qx << "," << entry.pose.qy << "," 
        << entry.pose.qz << "," << entry.pose.qw << ","
        << entry.precision << ","
        << entry.resolution << ","
        << entry.shape_hash << ","
        << entry.previous_hash;
    
    return generateHash(oss.str());
}

std::string LedgerManager::calculateGoalHash(const GoalEntry& entry) const {
    std::ostringstream oss;
    oss << entry.entry_id << ","
        << entry.robot_id << ","
        << entry.timestamp.time_since_epoch().count() << ","
        << entry.natural_language_goal << ","
        << entry.previous_hash;
    
    // Include motion links
    for (const auto& motion_id : entry.motion_entry_links) {
        oss << "," << motion_id;
    }
    
    return generateHash(oss.str());
}

std::string LedgerManager::generateHash(const std::string& data) const {
    return security_manager_->generateHash(data);
}

std::string LedgerManager::signEntry(const std::string& entry_data) const {
    return security_manager_->signData(entry_data);
}

void LedgerManager::onMotionEntryReceived(
    const MotionEntry::SharedPtr msg
) {
    if (msg->robot_id == robot_id_) {
        return;  // Ignore our own entries
    }
    
    std::lock_guard<std::mutex> lock(ledger_mutex_);
    
    // Convert ROS message to MotionEntry struct
    MotionEntry entry;
    entry.entry_id = msg->entry_id;
    entry.robot_id = msg->robot_id;
    entry.timestamp = std::chrono::system_clock::time_point(
        std::chrono::nanoseconds(msg->timestamp.nanosec)
    );
    entry.pose = msg->pose;
    entry.precision = msg->precision;
    entry.resolution = msg->resolution;
    entry.shape_hash = msg->shape_hash;
    entry.previous_hash = msg->previous_hash;
    entry.current_hash = msg->current_hash;
    entry.signature = msg->signature;
    
    // Add to peer ledger
    peer_motion_ledgers_[msg->robot_id].push_back(entry);
    
    RCLCPP_INFO(node_->get_logger(), 
                "Received motion entry from %s: %s", 
                msg->robot_id.c_str(),
                entry.entry_id.c_str());
}

void LedgerManager::onGoalEntryReceived(const GoalEntry::SharedPtr msg) {
    if (msg->robot_id == robot_id_) {
        return;  // Ignore our own entries
    }
    
    std::lock_guard<std::mutex> lock(ledger_mutex_);
    
    // Convert ROS message to GoalEntry struct
    GoalEntry entry;
    entry.entry_id = msg->entry_id;
    entry.robot_id = msg->robot_id;
    entry.timestamp = std::chrono::system_clock::time_point(
        std::chrono::nanoseconds(msg->timestamp.nanosec)
    );
    entry.natural_language_goal = msg->natural_language_goal;
    entry.motion_entry_links = msg->motion_entry_links;
    entry.previous_hash = msg->previous_hash;
    entry.current_hash = msg->current_hash;
    entry.signature = msg->signature;
    
    // Add to peer ledger
    peer_goal_ledgers_[msg->robot_id].push_back(entry);
    
    RCLCPP_INFO(node_->get_logger(), 
                "Received goal entry from %s: %s", 
                msg->robot_id.c_str(),
                entry.entry_id.c_str());
}

LedgerStats LedgerManager::getStats() const {
    std::lock_guard<std::mutex> lock(ledger_mutex_);
    
    LedgerStats stats;
    stats.motion_entries = local_motion_ledger_.size();
    stats.goal_entries = local_goal_ledger_.size();
    stats.peer_count = peer_motion_ledgers_.size();
    stats.last_update = std::chrono::system_clock::now();
    
    return stats;
}

uint64_t LedgerManager::getEntryCount() const {
    std::lock_guard<std::mutex> lock(ledger_mutex_);
    return local_motion_ledger_.size() + local_goal_ledger_.size();
}

} // namespace robocon
```

## Usage Example

```cpp
#include "robocon_network_client/ledger_manager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("ledger_example");
    
    robocon::LedgerManager ledger("robot_001", node);
    ledger.initialize();
    
    // Create and append motion entry
    robocon::MotionEntry motion;
    motion.entry_id = "motion_001";
    motion.pose = robocon::Pose{10.5, 20.3, 0.0, 0.0, 0.0, 0.0, 1.0};
    motion.precision = 0.01;
    motion.resolution = 0.1;
    motion.shape_hash = "shape_hash_001";
    
    ledger.appendMotionLedger(motion);
    
    // Verify ledger integrity
    if (ledger.verifyMotionLedger("robot_001")) {
        std::cout << "Motion ledger verified" << std::endl;
    }
    
    // Get latest hash
    auto hash = ledger.getLatestMotionHash("robot_001");
    if (hash.has_value()) {
        std::cout << "Latest motion hash: " << hash.value() << std::endl;
    }
    
    // Create and append goal entry
    robocon::GoalEntry goal;
    goal.entry_id = "goal_001";
    goal.natural_language_goal = "Move sheathing to building B, floor 2, room 5";
    
    ledger.appendGoalLedger(goal);
    
    // Link goal to motions
    ledger.linkGoalToMotions("goal_001", {"motion_001", "motion_002"});
    
    // Get motions for goal
    auto motions = ledger.getMotionsForGoal("goal_001");
    std::cout << "Goal has " << motions.size() << " linked motions" << std::endl;
    
    // Get statistics
    auto stats = ledger.getStats();
    std::cout << "Motion entries: " << stats.motion_entries << std::endl;
    std::cout << "Goal entries: " << stats.goal_entries << std::endl;
    
    ledger.shutdown();
    rclcpp::shutdown();
    return 0;
}
```

## Key Implementation Details

1. **Hash Chain**: Each entry includes the hash of the previous entry, creating a tamper-evident chain.

2. **SHA-256 Hashing**: Uses SHA-256 for cryptographic hashing of ledger entries.

3. **Digital Signatures**: Each entry is signed using the robot's private key for authentication.

4. **Peer Ledger Sync**: Receives and stores ledgers from peer robots via ROS 2 topics.

5. **Cross-Linking**: Goals can be linked to multiple motion entries, creating hierarchical relationships.

6. **Integrity Verification**: Can verify the integrity of any ledger by checking the hash chain.

## Next Steps

- [Security Manager Implementation](security-manager.md) - Zero-trust verification implementation
- [Discovery Manager Implementation](discovery-manager.md) - Peer discovery implementation
- [ROS 2 Messages and Topics](ros2-integration.md) - Message definitions

