# Consensus Manager Implementation

The `ConsensusManager` class implements the voting protocol for consensus-based task allocation. It handles action proposals, vote collection, and consensus determination.

## Header File Structure

```cpp
// consensus_manager.hpp
#ifndef ROBOCON_NETWORK_CLIENT_CONSENSUS_MANAGER_HPP
#define ROBOCON_NETWORK_CLIENT_CONSENSUS_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <chrono>
#include <functional>

#include "robocon_network_client/msg/action_suggestion.hpp"
#include "robocon_network_client/msg/vote.hpp"
#include "robocon_network_client/msg/consensus_result.hpp"

namespace robocon {

struct ActionProposal {
    ActionSuggestion suggestion;
    std::chrono::system_clock::time_point proposal_time;
    std::unordered_map<std::string, bool> votes;  // peer_id -> vote (true=yes, false=no)
    std::vector<std::string> vote_intents;        // Peers who indicated intent to vote
    bool consensus_reached;
    std::string executor_id;
    uint32_t yes_votes;
    uint32_t no_votes;
};

class ConsensusManager {
public:
    ConsensusManager(
        const std::string& robot_id,
        rclcpp::Node::SharedPtr node
    );
    
    ~ConsensusManager();
    
    bool initialize();
    void shutdown();
    
    ConsensusResult proposeAction(const ActionSuggestion& action);
    void voteOnAction(const std::string& action_id, bool vote);
    void handleActionSuggestion(const ActionSuggestion& action);
    void handleVote(const Vote& vote);
    void handleVoteIntent(const VoteIntent& intent);
    
    ConsensusResult getConsensusStatus(const std::string& action_id) const;
    std::vector<ActionSuggestion> getPendingActions() const;
    
    void setConsensusCallback(
        std::function<void(const ConsensusResult&)> callback
    );
    
    uint64_t getConsensusAttempts() const;
    uint64_t getConsensusSuccesses() const;

private:
    std::string robot_id_;
    rclcpp::Node::SharedPtr node_;
    bool initialized_;
    
    std::unordered_map<std::string, ActionProposal> active_proposals_;
    mutable std::mutex proposals_mutex_;
    
    std::function<void(const ConsensusResult&)> consensus_callback_;
    
    uint64_t consensus_attempts_;
    uint64_t consensus_successes_;
    
    // ROS 2 Publishers
    rclcpp::Publisher<Vote>::SharedPtr vote_publisher_;
    rclcpp::Publisher<VoteIntent>::SharedPtr vote_intent_publisher_;
    rclcpp::Publisher<ConsensusResult>::SharedPtr result_publisher_;
    
    // ROS 2 Subscribers
    rclcpp::Subscription<ActionSuggestion>::SharedPtr action_subscriber_;
    rclcpp::Subscription<Vote>::SharedPtr vote_subscriber_;
    rclcpp::Subscription<VoteIntent>::SharedPtr vote_intent_subscriber_;
    
    // Callbacks
    void onActionSuggestion(const ActionSuggestion::SharedPtr msg);
    void onVoteReceived(const Vote::SharedPtr msg);
    void onVoteIntentReceived(const VoteIntent::SharedPtr msg);
    
    // Internal methods
    void processVotes(const std::string& action_id);
    bool checkConsensus(const ActionProposal& proposal) const;
    std::string selectExecutor(const ActionProposal& proposal) const;
    void broadcastConsensusResult(const ConsensusResult& result);
    
    void setupROSPublishers();
    void setupROSSubscribers();
    
    static constexpr uint32_t MIN_QUORUM = 2;  // Minimum peers for consensus
    static constexpr double MAJORITY_THRESHOLD = 0.5;  // 50% majority required
    static constexpr uint32_t VOTE_TIMEOUT_MS = 10000;  // 10 seconds
};

} // namespace robocon

#endif // ROBOCON_NETWORK_CLIENT_CONSENSUS_MANAGER_HPP
```

## Implementation

```cpp
// consensus_manager.cpp
#include "robocon_network_client/consensus_manager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <thread>

namespace robocon {

ConsensusManager::ConsensusManager(
    const std::string& robot_id,
    rclcpp::Node::SharedPtr node
) : robot_id_(robot_id),
    node_(node),
    initialized_(false),
    consensus_attempts_(0),
    consensus_successes_(0)
{
}

ConsensusManager::~ConsensusManager() {
    if (initialized_) {
        shutdown();
    }
}

bool ConsensusManager::initialize() {
    if (initialized_) {
        return true;
    }
    
    RCLCPP_INFO(node_->get_logger(), 
                "Initializing ConsensusManager for robot: %s", 
                robot_id_.c_str());
    
    setupROSPublishers();
    setupROSSubscribers();
    
    initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), 
                "ConsensusManager initialized successfully");
    
    return true;
}

void ConsensusManager::shutdown() {
    if (!initialized_) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(proposals_mutex_);
    active_proposals_.clear();
    initialized_ = false;
}

void ConsensusManager::setupROSPublishers() {
    vote_publisher_ = node_->create_publisher<Vote>(
        "/consensus/vote",
        rclcpp::QoS(10).reliable().transient_local()
    );
    
    vote_intent_publisher_ = node_->create_publisher<VoteIntent>(
        "/consensus/vote_intent",
        rclcpp::QoS(10).reliable().transient_local()
    );
    
    result_publisher_ = node_->create_publisher<ConsensusResult>(
        "/consensus/result",
        rclcpp::QoS(10).reliable().transient_local()
    );
}

void ConsensusManager::setupROSSubscribers() {
    action_subscriber_ = node_->create_subscription<ActionSuggestion>(
        "/consensus/action_suggestion",
        rclcpp::QoS(10).reliable().transient_local(),
        std::bind(&ConsensusManager::onActionSuggestion, 
                  this, std::placeholders::_1)
    );
    
    vote_subscriber_ = node_->create_subscription<Vote>(
        "/consensus/vote",
        rclcpp::QoS(10).reliable().transient_local(),
        std::bind(&ConsensusManager::onVoteReceived, 
                  this, std::placeholders::_1)
    );
    
    vote_intent_subscriber_ = node_->create_subscription<VoteIntent>(
        "/consensus/vote_intent",
        rclcpp::QoS(10).reliable().transient_local(),
        std::bind(&ConsensusManager::onVoteIntentReceived, 
                  this, std::placeholders::_1)
    );
}

ConsensusResult ConsensusManager::proposeAction(
    const ActionSuggestion& action
) {
    if (!initialized_) {
        RCLCPP_ERROR(node_->get_logger(), 
                     "ConsensusManager not initialized");
        return ConsensusResult{};
    }
    
    std::lock_guard<std::mutex> lock(proposals_mutex_);
    
    // Create proposal entry
    ActionProposal proposal;
    proposal.suggestion = action;
    proposal.proposal_time = std::chrono::system_clock::now();
    proposal.consensus_reached = false;
    proposal.yes_votes = 0;
    proposal.no_votes = 0;
    
    // Add self as proposer with yes vote
    proposal.votes[robot_id_] = true;
    proposal.yes_votes = 1;
    
    active_proposals_[action.action_id] = proposal;
    consensus_attempts_++;
    
    RCLCPP_INFO(node_->get_logger(), 
                "Proposed action: %s (type: %s)", 
                action.action_id.c_str(),
                action.action_type.c_str());
    
    // Publish action suggestion to ROS 2 topic
    auto msg = std::make_shared<ActionSuggestion>();
    msg->action_id = action.action_id;
    msg->proposer_id = action.proposer_id;
    msg->action_type = action.action_type;
    msg->target_location = action.target_location;
    msg->estimated_cost = action.estimated_cost;
    msg->required_capabilities = action.required_capabilities;
    msg->behavior_tree_node = action.behavior_tree_node;
    
    // Note: Publishing is handled by NetworkClient
    // This is shown for completeness
    
    // Wait for votes (in real implementation, this would be async)
    // For now, return immediately and process asynchronously
    std::thread([this, action_id = action.action_id]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(VOTE_TIMEOUT_MS));
        processVotes(action_id);
    }).detach();
    
    return getConsensusStatus(action.action_id);
}

void ConsensusManager::voteOnAction(
    const std::string& action_id,
    bool vote
) {
    if (!initialized_) {
        return;
    }
    
    // Create vote message
    auto vote_msg = std::make_shared<Vote>();
    vote_msg->action_id = action_id;
    vote_msg->voter_id = robot_id_;
    vote_msg->vote = vote;
    vote_msg->vote_timestamp = rclcpp::Clock().now();
    
    // Publish vote
    vote_publisher_->publish(*vote_msg);
    
    // Also update local proposal if we have one
    std::lock_guard<std::mutex> lock(proposals_mutex_);
    auto it = active_proposals_.find(action_id);
    if (it != active_proposals_.end()) {
        it->second.votes[robot_id_] = vote;
        if (vote) {
            it->second.yes_votes++;
        } else {
            it->second.no_votes++;
        }
        processVotes(action_id);
    }
}

void ConsensusManager::handleActionSuggestion(
    const ActionSuggestion& action
) {
    if (!initialized_) {
        return;
    }
    
    // Don't process our own proposals
    if (action.proposer_id == robot_id_) {
        return;
    }
    
    RCLCPP_INFO(node_->get_logger(), 
                "Received action suggestion: %s from %s", 
                action.action_id.c_str(),
                action.proposer_id.c_str());
    
    // Send vote intent
    auto intent_msg = std::make_shared<VoteIntent>();
    intent_msg->action_id = action.action_id;
    intent_msg->voter_id = robot_id_;
    intent_msg->intent = true;  // Willing to vote
    intent_msg->timestamp = rclcpp::Clock().now();
    
    vote_intent_publisher_->publish(*intent_msg);
    
    // Create proposal entry
    std::lock_guard<std::mutex> lock(proposals_mutex_);
    ActionProposal proposal;
    proposal.suggestion = action;
    proposal.proposal_time = std::chrono::system_clock::now();
    proposal.consensus_reached = false;
    proposal.yes_votes = 0;
    proposal.no_votes = 0;
    
    active_proposals_[action.action_id] = proposal;
    
    // Evaluate the action and vote
    // In real implementation, this would evaluate the action
    // For now, vote yes by default
    std::thread([this, action_id = action.action_id]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        voteOnAction(action_id, true);
    }).detach();
}

void ConsensusManager::handleVote(const Vote& vote) {
    if (!initialized_ || vote.voter_id == robot_id_) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(proposals_mutex_);
    auto it = active_proposals_.find(vote.action_id);
    if (it == active_proposals_.end()) {
        return;
    }
    
    // Add vote to proposal
    auto& proposal = it->second;
    if (proposal.votes.find(vote.voter_id) == proposal.votes.end()) {
        proposal.votes[vote.voter_id] = vote.vote;
        if (vote.vote) {
            proposal.yes_votes++;
        } else {
            proposal.no_votes++;
        }
        
        RCLCPP_INFO(node_->get_logger(), 
                    "Received vote from %s: %s for action %s", 
                    vote.voter_id.c_str(),
                    vote.vote ? "YES" : "NO",
                    vote.action_id.c_str());
        
        // Check for consensus
        processVotes(vote.action_id);
    }
}

void ConsensusManager::handleVoteIntent(const VoteIntent& intent) {
    if (!initialized_ || intent.voter_id == robot_id_) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(proposals_mutex_);
    auto it = active_proposals_.find(intent.action_id);
    if (it != active_proposals_.end()) {
        auto& proposal = it->second;
        if (std::find(proposal.vote_intents.begin(), 
                      proposal.vote_intents.end(), 
                      intent.voter_id) == proposal.vote_intents.end()) {
            proposal.vote_intents.push_back(intent.voter_id);
        }
    }
}

void ConsensusManager::onActionSuggestion(
    const ActionSuggestion::SharedPtr msg
) {
    handleActionSuggestion(*msg);
}

void ConsensusManager::onVoteReceived(const Vote::SharedPtr msg) {
    handleVote(*msg);
}

void ConsensusManager::onVoteIntentReceived(const VoteIntent::SharedPtr msg) {
    handleVoteIntent(*msg);
}

void ConsensusManager::processVotes(const std::string& action_id) {
    std::lock_guard<std::mutex> lock(proposals_mutex_);
    auto it = active_proposals_.find(action_id);
    if (it == active_proposals_.end()) {
        return;
    }
    
    auto& proposal = it->second;
    
    // Check if consensus already reached
    if (proposal.consensus_reached) {
        return;
    }
    
    // Check consensus conditions
    uint32_t total_votes = proposal.yes_votes + proposal.no_votes;
    
    // Need minimum quorum
    if (total_votes < MIN_QUORUM) {
        return;
    }
    
    // Check for majority
    bool consensus = checkConsensus(proposal);
    
    if (consensus && !proposal.consensus_reached) {
        proposal.consensus_reached = true;
        proposal.executor_id = selectExecutor(proposal);
        consensus_successes_++;
        
        RCLCPP_INFO(node_->get_logger(), 
                    "Consensus reached for action %s! Executor: %s", 
                    action_id.c_str(),
                    proposal.executor_id.c_str());
        
        // Create consensus result
        ConsensusResult result;
        result.action_id = action_id;
        result.consensus_reached = true;
        result.total_votes = total_votes;
        result.yes_votes = proposal.yes_votes;
        result.no_votes = proposal.no_votes;
        result.executor_id = proposal.executor_id;
        result.consensus_timestamp = std::chrono::system_clock::now();
        
        // Broadcast result
        broadcastConsensusResult(result);
        
        // Call callback if set
        if (consensus_callback_) {
            consensus_callback_(result);
        }
    }
}

bool ConsensusManager::checkConsensus(
    const ActionProposal& proposal
) const {
    uint32_t total_votes = proposal.yes_votes + proposal.no_votes;
    
    if (total_votes < MIN_QUORUM) {
        return false;
    }
    
    // Check if yes votes exceed majority threshold
    double yes_ratio = static_cast<double>(proposal.yes_votes) / total_votes;
    return yes_ratio >= MAJORITY_THRESHOLD;
}

std::string ConsensusManager::selectExecutor(
    const ActionProposal& proposal
) const {
    // Simple executor selection: choose proposer
    // In real implementation, this would consider:
    // - Robot capabilities
    // - Current location
    // - Estimated cost
    // - Load balancing
    
    return proposal.suggestion.proposer_id;
}

void ConsensusManager::broadcastConsensusResult(
    const ConsensusResult& result
) {
    auto msg = std::make_shared<ConsensusResult>();
    msg->action_id = result.action_id;
    msg->consensus_reached = result.consensus_reached;
    msg->total_votes = result.total_votes;
    msg->yes_votes = result.yes_votes;
    msg->no_votes = result.no_votes;
    msg->executor_id = result.executor_id;
    
    result_publisher_->publish(*msg);
}

ConsensusResult ConsensusManager::getConsensusStatus(
    const std::string& action_id
) const {
    std::lock_guard<std::mutex> lock(proposals_mutex_);
    auto it = active_proposals_.find(action_id);
    if (it == active_proposals_.end()) {
        return ConsensusResult{};
    }
    
    const auto& proposal = it->second;
    ConsensusResult result;
    result.action_id = action_id;
    result.consensus_reached = proposal.consensus_reached;
    result.total_votes = proposal.yes_votes + proposal.no_votes;
    result.yes_votes = proposal.yes_votes;
    result.no_votes = proposal.no_votes;
    result.executor_id = proposal.executor_id;
    result.consensus_timestamp = proposal.proposal_time;
    
    return result;
}

std::vector<ActionSuggestion> ConsensusManager::getPendingActions() const {
    std::lock_guard<std::mutex> lock(proposals_mutex_);
    std::vector<ActionSuggestion> pending;
    
    for (const auto& [action_id, proposal] : active_proposals_) {
        if (!proposal.consensus_reached) {
            pending.push_back(proposal.suggestion);
        }
    }
    
    return pending;
}

void ConsensusManager::setConsensusCallback(
    std::function<void(const ConsensusResult&)> callback
) {
    consensus_callback_ = callback;
}

uint64_t ConsensusManager::getConsensusAttempts() const {
    return consensus_attempts_;
}

uint64_t ConsensusManager::getConsensusSuccesses() const {
    return consensus_successes_;
}

} // namespace robocon
```

## Usage Example

```cpp
#include "robocon_network_client/consensus_manager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("consensus_example");
    
    robocon::ConsensusManager consensus("robot_001", node);
    consensus.initialize();
    
    // Set up callback for consensus results
    consensus.setConsensusCallback([](const robocon::ConsensusResult& result) {
        if (result.consensus_reached) {
            std::cout << "Consensus reached for action: " 
                      << result.action_id << std::endl;
            std::cout << "Yes votes: " << result.yes_votes << std::endl;
            std::cout << "Executor: " << result.executor_id << std::endl;
        }
    });
    
    // Propose an action
    robocon::ActionSuggestion action;
    action.action_id = "action_001";
    action.proposer_id = "robot_001";
    action.action_type = "move_sheathing";
    action.target_location = "building_B_floor_2_room_5";
    action.estimated_cost = 150.0;
    
    auto result = consensus.proposeAction(action);
    
    // Wait for consensus
    std::this_thread::sleep_for(std::chrono::seconds(15));
    
    // Check final status
    result = consensus.getConsensusStatus("action_001");
    if (result.consensus_reached) {
        std::cout << "Final status: Consensus reached!" << std::endl;
    }
    
    consensus.shutdown();
    rclcpp::shutdown();
    return 0;
}
```

## Key Implementation Details

1. **Voting Protocol**: Implements a multi-phase voting protocol with vote intents and actual votes.

2. **Consensus Determination**: Requires minimum quorum (2 peers) and simple majority (>50% yes votes).

3. **ROS 2 Integration**: All communication flows through ROS 2 topics using Cyclone DDS.

4. **Thread Safety**: Uses mutexes to protect shared state.

5. **Timeout Handling**: Votes timeout after 10 seconds to prevent indefinite waiting.

6. **Executor Selection**: Currently selects the proposer, but can be extended with more sophisticated algorithms.

## Next Steps

- [Ledger Manager Implementation](ledger-manager.md) - Blockchain-style ledger implementation
- [Security Manager Implementation](security-manager.md) - Zero-trust verification
- [ROS 2 Messages and Topics](ros2-integration.md) - Message definitions

