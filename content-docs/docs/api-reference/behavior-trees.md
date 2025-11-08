# Behavior Trees API

API for creating and managing behavior trees for robot decision making using BehaviorTree.CPP.

## Overview

ROBOCON SDK uses [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) (v4.6+) for creating hierarchical decision-making structures. Version 4.6 or later is required. Behavior Trees provide a powerful way to structure complex robot behaviors with:

- **Asynchronous Actions**: Non-blocking, reactive behaviors
- **Reactive Execution**: Multiple actions can run concurrently
- **XML-based Trees**: Trees defined in XML and loaded at runtime
- **ROS 2 Integration**: Seamless integration with ROS 2 nodes and topics
- **Visualization**: ROBOCON Interface graphical editor support

## Installation

**Version Requirement:** BehaviorTree.CPP v4.6 or later is required. The latest version is 4.8.2 (see [GitHub repository](https://github.com/BehaviorTree/BehaviorTree.CPP)).

```bash
# Install BehaviorTree.CPP via ROS 2 package
sudo apt install ros-jazzy-behaviortree-cpp-v3

# Or build from source
cd ~/robocon_ws/src
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd BehaviorTree.CPP
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

**Note:** The ROS 2 package is named `ros-jazzy-behaviortree-cpp-v3`, but it provides BehaviorTree.CPP v4.6 or later. The C++ includes are `behaviortree_cpp/` (not `behaviortree_cpp_v3/`). See the [official repository](https://github.com/BehaviorTree/BehaviorTree.CPP) for the latest version.

## Basic Usage (C++)

### Creating Custom Nodes

```cpp
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

class MyActionNode : public BT::StatefulActionNode
{
public:
  MyActionNode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("my_node");
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("input"), 
             BT::OutputPort<double>("output") };
  }

  BT::NodeStatus onStart() override
  {
    RCLCPP_INFO(node_->get_logger(), "Starting action...");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    rclcpp::spin_some(node_);
    
    // Check if action completed
    if (action_complete_)
    {
      return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    RCLCPP_INFO(node_->get_logger(), "Action halted");
  }

private:
  rclcpp::Node::SharedPtr node_;
  bool action_complete_ = false;
};
```

### Creating and Executing Trees

```cpp
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // Create factory
  BT::BehaviorTreeFactory factory;
  
  // Register custom nodes
  factory.registerNodeType<MyActionNode>("MyActionNode");
  
  // Load tree from XML
  auto tree = factory.createTreeFromFile("my_tree.xml");
  
  // Execute tree
  BT::NodeStatus status = tree.tickRoot();
  
  while (status == BT::NodeStatus::RUNNING)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    status = tree.tickRoot();
  }
  
  rclcpp::shutdown();
  return 0;
}
```

## Behavior Tree XML Format

Trees are defined in XML format:

```xml
<?xml version="1.0"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence name="MySequence">
            <MyActionNode name="Action1" input="value1" output="{result}"/>
            <MyActionNode name="Action2" input="{result}"/>
        </Sequence>
    </BehaviorTree>
</root>
```

## Control Nodes

BehaviorTree.CPP provides several control nodes:

### Sequence
Executes children in order, returns FAILURE if any child fails:
```xml
<Sequence>
    <Node1/>
    <Node2/>
</Sequence>
```

### Fallback (Selector)
Executes children until one succeeds:
```xml
<Fallback>
    <Node1/>
    <Node2/>
</Fallback>
```

### Parallel
Executes all children concurrently:
```xml
<Parallel success_count="1" failure_count="1">
    <Node1/>
    <Node2/>
</Parallel>
```

### Retry
Retries a node N times:
```xml
<Retry num_attempts="3">
    <Node1/>
</Retry>
```

### Repeat
Repeats a node N times:
```xml
<Repeat num_cycles="5">
    <Node1/>
</Repeat>
```

## ROS 2 Integration

BehaviorTree.CPP nodes can interact with ROS 2:

```cpp
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

class ROS2SubscriberNode : public BT::SyncActionNode
{
public:
  ROS2SubscriberNode(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("bt_subscriber");
    sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/topic", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        received_msg_ = msg->data;
      }
    );
  }

  BT::NodeStatus tick() override
  {
    rclcpp::spin_some(node_);
    if (!received_msg_.empty())
    {
      setOutput("message", received_msg_);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::string received_msg_;
};
```

## Dataflow Between Nodes

Nodes can share data through blackboard:

```xml
<Sequence>
    <!-- Node1 writes to blackboard -->
    <Node1 output="{shared_data}"/>
    
    <!-- Node2 reads from blackboard -->
    <Node2 input="{shared_data}"/>
</Sequence>
```

In C++:
```cpp
// Write to blackboard
setOutput("output_port", value);

// Read from blackboard
std::string input;
getInput("input_port", input);
```

## CMakeLists.txt Configuration

For BehaviorTree.CPP v4.6+, configure your CMakeLists.txt:

```cmake
find_package(behaviortree_cpp_v3 REQUIRED)
# Note: Package name is behaviortree_cpp_v3, but includes are behaviortree_cpp/

target_link_libraries(your_target
  behaviortree_cpp_v3::behaviortree_cpp_v3
)
```

Include in your source files:
```cpp
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
// etc.
```

## Visualization with ROBOCON Interface

ROBOCON Interface provides graphical editing and real-time visualization for behavior trees. ROBOCON Interface is based on Groot 1 and has been updated for use with BehaviorTree.CPP v4.6+.

**Availability:**
- Linux
- Windows
- Android

**Usage:**

```bash
# ROBOCON Interface is available for download from ROBOCON resources

# Enable ZMQ publisher in your code for visualization
BT::PublisherZMQ publisher_zmq(tree);

# Open ROBOCON Interface and connect to visualize tree execution
```

ROBOCON Interface provides:
- Real-time behavior tree visualization
- Graphical tree editing
- Node status monitoring
- Execution debugging tools

## Complete Example

See the [Red Flag Detection Example](../../ai-programs/examples/red-flag-detection.md) for a complete application demonstrating:
- Custom BT nodes
- ROS 2 integration
- Computer vision detection
- Navigation actions
- Motor control integration

## ROS 2 Package Reference

For ROS 2-specific BehaviorTree.CPP documentation:
- [ROS 2 Jazzy BehaviorTree.CPP Package](https://docs.ros.org/en/jazzy/p/behaviortree_cpp/) - ROS 2 Jazzy integration
- [BehaviorTree.CPP GitHub](https://github.com/BehaviorTree/BehaviorTree.CPP) - Source repository
- [Official Documentation](https://www.behaviortree.dev/docs/Intro) - Complete BehaviorTree.CPP documentation

**Note:** ROS 2 Jazzy uses BehaviorTree.CPP v4.6+ (package: `ros-jazzy-behaviortree-cpp-v3`). Version 4.6 or later is required. The C++ includes use `behaviortree_cpp/` (not `behaviortree_cpp_v3/`). The API uses `BT::NodeConfig` instead of `BT::NodeConfiguration`. For the latest version, see the [official repository](https://github.com/BehaviorTree/BehaviorTree.CPP) (currently 4.8.2).

## Multi-Robot Behavior Tree Synchronization

Behavior trees can be synchronized across multiple robots through the consensus-based multi-robot communication protocol. When multiple robots agree on a task through voting, action subtrees are automatically inserted into each robot's behavior tree:

```cpp
#include <robocon_multi_robot/voting_protocol.hpp>
#include <behaviortree_cpp/bt_factory.h>

class MultiRobotBTManager {
public:
    void onConsensusAchieved(const VotingResult& result) {
        // Insert agreed-upon subtree into local behavior tree
        auto subtree = factory_.createTreeFromText(
            result.action_subtree_xml
        );
        
        // Merge with main tree
        main_tree_.insertSubtree(
            result.suggestion_id,
            subtree
        );
    }
};
```

> **See Also**: [Multi-Robot Communication API](multi-robot-communication.md) - Complete multi-robot coordination documentation

## Behavior Tree Node Reference

For a complete reference of all available behavior tree nodes (low-level and high-level actions), see:

- [Behavior Tree Node Reference](../behavior-tree-node-reference.md) - Complete list of all ROBOCON OS behavior tree nodes with arguments and examples

## Next Steps

- [Behavior Tree Node Reference](../behavior-tree-node-reference.md) - Complete reference of all available nodes
- [Multi-Robot Communication API](multi-robot-communication.md) - Coordinate behavior trees across multiple robots
- [Red Flag Detection Example](../../ai-programs/examples/red-flag-detection.md) - Complete working example
- [Packaging AI Programs](../../ai-programs/packaging.md) - Package BT applications as .deb
- [BehaviorTree.CPP Documentation](https://www.behaviortree.dev/docs/Intro) - Advanced features

