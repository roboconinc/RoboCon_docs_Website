# Red Flag Detection and Digging Example

Complete example AI program using BehaviorTree.CPP for autonomous red flag detection, location saving, and digging operations.

## Overview

This example demonstrates how to build a ROBOCON AI application that:
- Detects red flag objects using computer vision
- Saves the detected location
- Navigates to the location
- Performs a digging operation

The application uses [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) (v4.6+) integrated with ROS 2 Jazzy. Version 4.6 or later is required.

## Prerequisites

### Install BehaviorTree.CPP

```bash
# Install dependencies
sudo apt update
sudo apt install -y libzmq3-dev libsqlite3-dev

# Option 1: Install via ROS 2 package (if available)
sudo apt install -y ros-jazzy-behaviortree-cpp-v3

# Option 2: Build from source
cd ~/robocon_ws/src
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd BehaviorTree.CPP
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

### Verify Installation

```bash
# Check library
pkg-config --modversion behaviortree_cpp
# Or check ROS 2 package
ros2 pkg list | grep behaviortree
```

## Project Structure

Create the following structure:

```
red_flag_detection_app/
├── CMakeLists.txt
├── package.xml
├── manifest.yaml
├── trees/
│   └── red_flag_detection_tree.xml
├── src/
│   ├── red_flag_detection_app_node.cpp
│   ├── bt_nodes/
│   │   ├── detect_red_flag_node.hpp
│   │   ├── detect_red_flag_node.cpp
│   │   ├── save_location_node.hpp
│   │   ├── save_location_node.cpp
│   │   ├── navigate_to_location_node.hpp
│   │   ├── navigate_to_location_node.cpp
│   │   ├── dig_at_location_node.hpp
│   │   └── dig_at_location_node.cpp
│   └── utils/
│       └── vision_utils.hpp
└── launch/
    └── red_flag_detection.launch.py
```

## Step 1: Create ROS 2 Package

```bash
cd ~/robocon_ws/src
ros2 pkg create --build-type ament_cmake red_flag_detection_app \
    --dependencies rclcpp rclpy behaviortree_cpp_v3 \
    nav2_msgs geometry_msgs sensor_msgs std_msgs
```

## Step 2: Package Configuration

### package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>red_flag_detection_app</name>
  <version>1.0.0</version>
  <description>Red flag detection and autonomous digging application using BehaviorTree.CPP</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>behaviortree_cpp_v3</depend>
  <depend>nav2_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(red_flag_detection_app)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)
include_directories(src)

# Behavior Tree executable
add_executable(red_flag_detection_node
  src/red_flag_detection_app_node.cpp
  src/bt_nodes/detect_red_flag_node.cpp
  src/bt_nodes/save_location_node.cpp
  src/bt_nodes/navigate_to_location_node.cpp
  src/bt_nodes/dig_at_location_node.cpp
)

ament_target_dependencies(red_flag_detection_node
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  nav2_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
)

# Install executable
install(TARGETS
  red_flag_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files (if needed for other packages)
install(DIRECTORY include/
  DESTINATION include/
)

# Install XML tree files
install(DIRECTORY trees/
  DESTINATION share/${PROJECT_NAME}/trees
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
```

## Step 3: Behavior Tree Nodes

### Detect Red Flag Node

**src/bt_nodes/detect_red_flag_node.hpp**

```cpp
#ifndef RED_FLAG_DETECTION_APP__DETECT_RED_FLAG_NODE_HPP_
#define RED_FLAG_DETECTION_APP__DETECT_RED_FLAG_NODE_HPP_

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <memory>

class DetectRedFlagNode : public BT::StatefulActionNode
{
public:
  DetectRedFlagNode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("detect_red_flag_node");
    
    // Subscribe to camera topic
    image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw",
      10,
      std::bind(&DetectRedFlagNode::imageCallback, this, std::placeholders::_1)
    );
    
    detection_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&DetectRedFlagNode::processImage, this)
    );
  }

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<geometry_msgs::msg::Point>("detected_location") };
  }

  BT::NodeStatus onStart() override
  {
    red_flag_detected_ = false;
    detected_location_.x = 0.0;
    detected_location_.y = 0.0;
    detected_location_.z = 0.0;
    
    RCLCPP_INFO(node_->get_logger(), "Starting red flag detection...");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    rclcpp::spin_some(node_);
    
    if (red_flag_detected_)
    {
      setOutput("detected_location", detected_location_);
      RCLCPP_INFO(node_->get_logger(), "Red flag detected at: (%.2f, %.2f, %.2f)",
                  detected_location_.x, detected_location_.y, detected_location_.z);
      return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    RCLCPP_INFO(node_->get_logger(), "Red flag detection halted");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    latest_image_ = msg;
  }

  void processImage()
  {
    if (!latest_image_)
      return;
    
    // Simple red color detection (replace with your CV algorithm)
    // In production, use OpenCV, YOLO, or similar
    detectRedFlagInImage(latest_image_);
  }

  void detectRedFlagInImage(const sensor_msgs::msg::Image::SharedPtr& img)
  {
    // Placeholder: Simple red detection
    // TODO: Implement actual computer vision detection
    // This is a simplified example - use OpenCV, YOLOv11, or similar
    
    // For demonstration: assume red flag detected at center of image
    // In real implementation, use object detection to find red flags
    red_flag_detected_ = true;
    
    // Convert pixel coordinates to world coordinates
    // This would use camera intrinsics and depth data
    detected_location_.x = 5.0;  // Example: 5m forward
    detected_location_.y = 2.0;   // Example: 2m to the right
    detected_location_.z = 0.0;   // Ground level
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr detection_timer_;
  
  sensor_msgs::msg::Image::SharedPtr latest_image_;
  bool red_flag_detected_;
  geometry_msgs::msg::Point detected_location_;
};

#endif  // RED_FLAG_DETECTION_APP__DETECT_RED_FLAG_NODE_HPP_
```

**src/bt_nodes/detect_red_flag_node.cpp**

```cpp
#include "detect_red_flag_node.hpp"
```

### Save Location Node

**src/bt_nodes/save_location_node.hpp**

```cpp
#ifndef RED_FLAG_DETECTION_APP__SAVE_LOCATION_NODE_HPP_
#define RED_FLAG_DETECTION_APP__SAVE_LOCATION_NODE_HPP_

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/point.hpp"
#include <fstream>
#include <string>

class SaveLocationNode : public BT::SyncActionNode
{
public:
  SaveLocationNode(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<geometry_msgs::msg::Point>("location") };
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::Point location;
    if (!getInput("location", location))
    {
      return BT::NodeStatus::FAILURE;
    }

    // Save location to file
    std::string home_dir = std::getenv("HOME");
    std::string filepath = home_dir + "/.robocon/saved_locations.txt";
    
    // Create directory if it doesn't exist
    system(("mkdir -p " + home_dir + "/.robocon").c_str());
    
    std::ofstream file(filepath, std::ios::app);
    if (file.is_open())
    {
      file << "Red Flag Location: x=" << location.x 
           << ", y=" << location.y 
           << ", z=" << location.z << std::endl;
      file.close();
      
      return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::FAILURE;
  }
};

#endif  // RED_FLAG_DETECTION_APP__SAVE_LOCATION_NODE_HPP_
```

**src/bt_nodes/save_location_node.cpp**

```cpp
#include "save_location_node.hpp"
```

### Navigate to Location Node

**src/bt_nodes/navigate_to_location_node.hpp**

```cpp
#ifndef RED_FLAG_DETECTION_APP__NAVIGATE_TO_LOCATION_NODE_HPP_
#define RED_FLAG_DETECTION_APP__NAVIGATE_TO_LOCATION_NODE_HPP_

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <memory>
#include <chrono>
#include "rclcpp_action/rclcpp_action.hpp"

class NavigateToLocationNode : public BT::StatefulActionNode
{
public:
  NavigateToLocationNode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("navigate_to_location_node");
    nav_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      node_, "navigate_to_pose");
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<geometry_msgs::msg::Point>("target_location") };
  }

  BT::NodeStatus onStart() override
  {
    geometry_msgs::msg::Point target;
    if (!getInput("target_location", target))
    {
      return BT::NodeStatus::FAILURE;
    }

    // Wait for action server
    if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(node_->get_logger(), "NavigateToPose action server not available");
      return BT::NodeStatus::FAILURE;
    }

    // Create goal
    auto goal = nav2_msgs::action::NavigateToPose::Goal();
    goal.pose.pose.position = target;
    goal.pose.pose.orientation.w = 1.0;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = node_->now();

    // Send goal
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = 
      [this](const auto& result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
          navigation_complete_ = true;
          navigation_success_ = true;
        }
        else
        {
          navigation_complete_ = true;
          navigation_success_ = false;
        }
      };

    nav_action_client_->async_send_goal(goal, send_goal_options);
    
    navigation_complete_ = false;
    navigation_success_ = false;
    
    RCLCPP_INFO(node_->get_logger(), "Navigating to location: (%.2f, %.2f, %.2f)",
                target.x, target.y, target.z);
    
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    rclcpp::spin_some(node_);
    
    if (navigation_complete_)
    {
      if (navigation_success_)
      {
        RCLCPP_INFO(node_->get_logger(), "Navigation completed successfully");
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Navigation failed");
        return BT::NodeStatus::FAILURE;
      }
    }
    
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    RCLCPP_INFO(node_->get_logger(), "Navigation halted");
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_action_client_;
  bool navigation_complete_;
  bool navigation_success_;
};

#endif  // RED_FLAG_DETECTION_APP__NAVIGATE_TO_LOCATION_NODE_HPP_
```

**src/bt_nodes/navigate_to_location_node.cpp**

```cpp
#include "navigate_to_location_node.hpp"
```

### Dig at Location Node

**src/bt_nodes/dig_at_location_node.hpp**

```cpp
#ifndef RED_FLAG_DETECTION_APP__DIG_AT_LOCATION_NODE_HPP_
#define RED_FLAG_DETECTION_APP__DIG_AT_LOCATION_NODE_HPP_

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>
#include <chrono>

class DigAtLocationNode : public BT::StatefulActionNode
{
public:
  DigAtLocationNode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("dig_at_location_node");
    
    // Publisher for digging commands (integrate with ROBOCON motor control)
    dig_command_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/robocon/dig_command", 10);
    
    dig_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/robocon/dig_status",
      10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "completed")
        {
          digging_complete_ = true;
          digging_success_ = true;
        }
        else if (msg->data == "failed")
        {
          digging_complete_ = true;
          digging_success_ = false;
        }
      }
    );
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override
  {
    digging_complete_ = false;
    digging_success_ = false;
    
    // Send digging command
    auto msg = std_msgs::msg::String();
    msg.data = "start_dig";
    dig_command_pub_->publish(msg);
    
    RCLCPP_INFO(node_->get_logger(), "Starting digging operation...");
    start_time_ = node_->now();
    
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    rclcpp::spin_some(node_);
    
    // Check timeout (30 seconds)
    auto elapsed = (node_->now() - start_time_).seconds();
    if (elapsed > 30.0)
    {
      RCLCPP_WARN(node_->get_logger(), "Digging operation timeout");
      return BT::NodeStatus::FAILURE;
    }
    
    if (digging_complete_)
    {
      if (digging_success_)
      {
        RCLCPP_INFO(node_->get_logger(), "Digging completed successfully");
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Digging operation failed");
        return BT::NodeStatus::FAILURE;
      }
    }
    
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    // Send stop command
    auto msg = std_msgs::msg::String();
    msg.data = "stop_dig";
    dig_command_pub_->publish(msg);
    
    RCLCPP_INFO(node_->get_logger(), "Digging operation halted");
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dig_command_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dig_status_sub_;
  bool digging_complete_;
  bool digging_success_;
  rclcpp::Time start_time_;
};

#endif  // RED_FLAG_DETECTION_APP__DIG_AT_LOCATION_NODE_HPP_
```

**src/bt_nodes/dig_at_location_node.cpp**

```cpp
#include "dig_at_location_node.hpp"
```

## Step 4: Behavior Tree XML

**trees/red_flag_detection_tree.xml**

```xml
<?xml version="1.0"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence name="RedFlagDetectionAndDigging">
            <!-- Detect red flag -->
            <DetectRedFlagNode name="DetectRedFlag" 
                              detected_location="{detected_location}"/>
            
            <!-- Save the location -->
            <SaveLocationNode name="SaveLocation" 
                            location="{detected_location}"/>
            
            <!-- Navigate to the location -->
            <NavigateToLocationNode name="NavigateToLocation"
                                   target_location="{detected_location}"/>
            
            <!-- Dig at the location -->
            <DigAtLocationNode name="DigAtLocation"/>
        </Sequence>
    </BehaviorTree>
</root>
```

## Step 5: Main Application Node

**src/red_flag_detection_app_node.cpp**

```cpp
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_zmq_publisher.h>
#include <rclcpp/rclcpp.hpp>
#include "red_flag_detection_app/detect_red_flag_node.hpp"
#include "red_flag_detection_app/save_location_node.hpp"
#include "red_flag_detection_app/navigate_to_location_node.hpp"
#include "red_flag_detection_app/dig_at_location_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = rclcpp::Node::make_shared("red_flag_detection_app");
  
  // Get tree file path
  std::string tree_file = "trees/red_flag_detection_tree.xml";
  if (argc > 1)
  {
    tree_file = argv[1];
  }
  
  RCLCPP_INFO(node->get_logger(), "Loading behavior tree from: %s", tree_file.c_str());
  
  // Create behavior tree factory
  BT::BehaviorTreeFactory factory;
  
  // Register custom nodes
  factory.registerNodeType<DetectRedFlagNode>("DetectRedFlagNode");
  factory.registerNodeType<SaveLocationNode>("SaveLocationNode");
  factory.registerNodeType<NavigateToLocationNode>("NavigateToLocationNode");
  factory.registerNodeType<DigAtLocationNode>("DigAtLocationNode");
  
  // Create tree
  auto tree = factory.createTreeFromFile(tree_file);
  
  // Optional: Enable ZMQ publisher for ROBOCON Interface visualization
  BT::PublisherZMQ publisher_zmq(tree);
  
  RCLCPP_INFO(node->get_logger(), "Behavior tree loaded. Starting execution...");
  
  // Execute tree
  BT::NodeStatus status = tree.tickRoot();
  
  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    status = tree.tickRoot();
  }
  
  RCLCPP_INFO(node->get_logger(), "Behavior tree execution completed with status: %d", status);
  
  rclcpp::shutdown();
  return 0;
}
```

## Step 6: Launch File

**launch/red_flag_detection.launch.py**

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='red_flag_detection_app',
            executable='red_flag_detection_node',
            name='red_flag_detection_app',
            parameters=[{
                'use_sim_time': False,
            }],
            arguments=['trees/red_flag_detection_tree.xml']
        )
    ])
```

## Step 7: Build and Run

```bash
# Navigate to workspace
cd ~/robocon_ws

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Build package
colcon build --packages-select red_flag_detection_app

# Source workspace
source install/setup.bash

# Run the application
ros2 launch red_flag_detection_app red_flag_detection.launch.py

# Or run directly
ros2 run red_flag_detection_app red_flag_detection_node
```

## Step 8: Visualize with ROBOCON Interface

ROBOCON Interface is a graphical editor for Behavior Trees, based on Groot 1 and updated for BehaviorTree.CPP v4.6+. Visualize your behavior tree:

**Availability:**
- Linux
- Windows
- Android

```bash
# ROBOCON Interface is available for download from ROBOCON resources

# The tree will be automatically published via ZMQ
# Open ROBOCON Interface and connect to see the tree execution in real-time
```

ROBOCON Interface provides:
- Real-time behavior tree visualization
- Graphical tree editing
- Node status monitoring
- Execution debugging tools

## Step 9: Build .deb Package

After testing your application, package it as a `.deb` for distribution:

```bash
cd ~/robocon_ws/src/red_flag_detection_app

# Using bloom (recommended)
bloom-generate rosdebian --os-name ubuntu --os-version noble --ros-distro jazzy

# Edit debian/control to ensure all dependencies are listed:
# Depends: ros-jazzy-rclcpp, ros-jazzy-rclpy, libbehaviortree-cpp-v3,
#          ros-jazzy-nav2-msgs, ros-jazzy-geometry-msgs,
#          ros-jazzy-sensor-msgs, python3-opencv

# Build the package
fakeroot debian/rules binary

# The .deb will be in the parent directory
# Example: ../robocon-red-flag-detection-app_1.0.0-1_amd64.deb
```

### Install and Test the .deb Package

```bash
# Install the package
sudo apt install ./robocon-red-flag-detection-app_1.0.0-1_amd64.deb

# Verify installation
dpkg -l | grep red-flag-detection

# Run the application
source /opt/ros/jazzy/setup.bash
ros2 launch red_flag_detection_app red_flag_detection.launch.py

# Uninstall
sudo apt remove robocon-red-flag-detection-app
```

## Complete File Structure Summary

After completing all steps, your package should have this structure:

```
red_flag_detection_app/
├── CMakeLists.txt
├── package.xml
├── manifest.yaml
├── trees/
│   └── red_flag_detection_tree.xml
├── src/
│   ├── red_flag_detection_app_node.cpp
│   └── bt_nodes/
│       ├── detect_red_flag_node.hpp
│       ├── detect_red_flag_node.cpp
│       ├── save_location_node.hpp
│       ├── save_location_node.cpp
│       ├── navigate_to_location_node.hpp
│       ├── navigate_to_location_node.cpp
│       ├── dig_at_location_node.hpp
│       └── dig_at_location_node.cpp
├── include/
│   └── red_flag_detection_app/  # Optional: for public headers
├── launch/
│   └── red_flag_detection.launch.py
└── debian/                      # Created by bloom
    ├── control
    ├── changelog
    ├── rules
    └── ...
```

## Troubleshooting

### BehaviorTree.CPP Not Found

```bash
# Ensure library is installed
pkg-config --libs behaviortree_cpp

# If building from source, set CMAKE_PREFIX_PATH
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/usr/local
```

### ROS 2 Topics Not Available

Ensure required ROS 2 nodes are running:
```bash
# Check if camera is publishing
ros2 topic list | grep camera

# Check if Nav2 is available
ros2 service list | grep navigate_to_pose
```

### Build Errors

```bash
# Clean and rebuild
cd ~/robocon_ws
rm -rf build install log
colcon build --packages-select red_flag_detection_app
```

## Next Steps

- [Packaging Guide](../packaging.md) - Detailed .deb packaging instructions
- [Deployment Guide](../deployment.md) - Deploy to ROBOCON robots
- [Behavior Trees API](../../api-reference/behavior-trees.md) - Complete BT.CPP API reference
- [BehaviorTree.CPP Documentation](https://www.behaviortree.dev/docs/Intro) - Advanced BT.CPP features

