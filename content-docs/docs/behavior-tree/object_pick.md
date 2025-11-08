# Object_Pick Action Node

`Object_Pick` is a high-level behavior tree action node that coordinates the complete pick operation for an object, including approach planning, grasp planning, execution, and verification.

## Overview

`Object_Pick` takes an object instance (typically found by `World_Find` or `Visual_Find`) and performs the complete pick operation. It interfaces with arm control and gripper systems to execute a safe and effective grasp.

## Arguments

- `Object_Name` (Text) - Object identifier (typically from blackboard variable)
  - Example: `{foundObjectId}` or `osb_001`
- `Articulation_Group` (Text, Optional) - Which articulated group of joints will be responsible for the picking
  - Example: `Right_Arm`, `Boom_System`, `Crane_Arm`
  - If left blank, automatically selects the best way to pick the object
  - Default: `""` (auto-select)

## XML Example

```xml
<!-- With automatic articulation group selection -->
<Object_Pick Object_Name="{foundObjectId}" Articulation_Group=""/>

<!-- With specific articulation group -->
<Object_Pick Object_Name="{osbStackId}" Articulation_Group="Boom_System"/>
```

## Operation Phases

`Object_Pick` coordinates the following phases:

1. **Approach Planning**: Computes a safe approach trajectory to the object
2. **Grasp Planning**: Determines optimal grasp pose and gripper configuration
3. **Execution**: Executes the approach and grasp motions
4. **Verification**: Verifies successful grasp and load engagement

## Automatic Articulation Group Selection

When `Articulation_Group` is left blank or empty, `Object_Pick` automatically selects the best articulated body for the operation based on:

- **Object Location**: Distance and reachability from each articulation group
- **Object Properties**: Size, weight, and geometry constraints
- **Current Configuration**: Current pose of each articulation group
- **Workspace Analysis**: Which groups can reach the object without collision

For example, a crane system might choose between:
- **Boom_System**: For objects within boom reach
- **Arm_System**: For objects requiring precise manipulation
- **Hybrid Approach**: Using both systems in coordination

## Integration with Reachability Checks

`Object_Pick` can be used with reachability checking nodes:

### Object_Pick_Check

`Object_Pick_Check` performs a fast check to determine if the articulated body can reach the object without needing to `Travel` (move the base). It outputs an incomplete plan containing:

- **Collision Check**: Whether the planned path avoids collisions
- **Rough Kinematic Check**: Whether the object is within the workspace
- **Reachability Result**: SUCCESS if reachable, FAILURE if not

**XML Example:**
```xml
<Object_Pick_Check Object_Name="{objectId}" Articulation_Group="Boom_System"/>
```

**Output**: Stores reachability information in the blackboard for use by `Object_Pick_Plan` or `Object_Pick`.

### Object_Pick_Plan

`Object_Pick_Plan` is a more costly operation that performs full planning for the pick operation. It also checks if the object cannot be safely or possibly picked:

- **Full Path Planning**: Complete trajectory from current pose to grasp
- **Collision Avoidance**: Detailed collision checking with environment
- **Safety Validation**: Verification that the pick operation is safe
- **Grasp Feasibility**: Confirms that a valid grasp exists

**XML Example:**
```xml
<Object_Pick_Plan Object_Name="{objectId}" Articulation_Group="Boom_System"/>
```

**Output**: Full pick plan stored in blackboard, or FAILURE if picking is not feasible.

## Usage Examples

### Example 1: Simple Pick with Auto-Selection

```xml
<Sequence>
    <World_Find ObjectClass="OSB" objectId="{osbId}"/>
    <Object_Pick Object_Name="{osbId}" Articulation_Group=""/>
</Sequence>
```

### Example 2: Pick with Specific Articulation Group

```xml
<Sequence>
    <Visual_Find ObjectClass="Red Cup" objectId="{cupId}"/>
    <Object_Pick Object_Name="{cupId}" Articulation_Group="Right_Arm"/>
</Sequence>
```

### Example 3: With Reachability Check

```xml
<Sequence>
    <World_Find ObjectClass="Stack_OSB" objectId="{stackId}"/>
    
    <Fallback>
        <!-- Fast check: can we reach without moving? -->
        <Object_Pick_Check Object_Name="{stackId}" Articulation_Group="Boom_System"/>
        
        <!-- If not reachable, travel first -->
        <Sequence>
            <Travel_Plan_ToOptimalLiftPoint 
                source="{stackId}" 
                target="{goalLocation}" 
                goal="{travelGoal}"/>
            <Travel_Follow/>
            <Object_Pick_Check Object_Name="{stackId}" Articulation_Group="Boom_System"/>
        </Sequence>
    </Fallback>
    
    <!-- Perform full planning and execution -->
    <Object_Pick_Plan Object_Name="{stackId}" Articulation_Group="Boom_System"/>
    <Object_Pick Object_Name="{stackId}" Articulation_Group="Boom_System"/>
</Sequence>
```

### Example 4: Complete Pick and Place Operation

```xml
<Sequence>
    <!-- Find object -->
    <World_Find ObjectClass="Stack_OSB" objectId="{osbId}"/>
    
    <!-- Check reachability -->
    <Object_Pick_Check Object_Name="{osbId}" Articulation_Group=""/>
    
    <!-- Plan and execute pick -->
    <Object_Pick_Plan Object_Name="{osbId}" Articulation_Group=""/>
    <Object_Pick Object_Name="{osbId}" Articulation_Group=""/>
    
    <!-- Transport object -->
    <Travel_Plan From="{Current Location}" To="Building Site"/>
    <Travel_Follow/>
    
    <!-- Place object -->
    <Object_Place Location="Foundation" Gripper="Suction"/>
</Sequence>
```

## C++ Implementation Example (BehaviorTree.CPP v4.6+)

```cpp
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/action/pick.hpp>

class ObjectPickNode : public BT::StatefulActionNode
{
public:
    ObjectPickNode(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("object_pick_node");
        pick_action_client_ = rclcpp_action::create_client<moveit_msgs::action::Pick>(
            node_, "pick_action");
        object_pose_client_ = node_->create_client<robot_interfaces::srv::GetObjectPose>(
            "/object_tracker/get_pose");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("Object_Name"),
            BT::InputPort<std::string>("Articulation_Group")
        };
    }

    BT::NodeStatus onStart() override
    {
        std::string object_name, articulation_group;
        
        if (!getInput("Object_Name", object_name))
        {
            RCLCPP_ERROR(node_->get_logger(), "Missing input: Object_Name");
            return BT::NodeStatus::FAILURE;
        }

        // Articulation_Group is optional
        getInput("Articulation_Group", articulation_group);
        
        // If not specified, auto-select best articulation group
        if (articulation_group.empty())
        {
            articulation_group = selectBestArticulationGroup(object_name);
            if (articulation_group.empty())
            {
                RCLCPP_ERROR(node_->get_logger(), 
                    "Could not determine suitable articulation group");
                return BT::NodeStatus::FAILURE;
            }
        }

        // Get object pose
        auto req = std::make_shared<robot_interfaces::srv::GetObjectPose::Request>();
        req->object_id = object_name;
        
        auto result = object_pose_client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(node_, result, std::chrono::seconds(2)) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), 
                "Failed to get pose for object: %s", object_name.c_str());
            return BT::NodeStatus::FAILURE;
        }

        geometry_msgs::msg::PoseStamped target_pose = result.get()->pose;

        // Create pick action goal
        auto goal = moveit_msgs::action::Pick::Goal();
        goal.target_name = object_name;
        goal.group_name = articulation_group;
        goal.target_pose = target_pose;
        goal.planning_options.plan_only = false; // Execute immediately

        // Send goal
        if (!pick_action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(node_->get_logger(), 
                "Pick action server not available");
            return BT::NodeStatus::FAILURE;
        }

        auto send_goal_options = 
            rclcpp_action::Client<moveit_msgs::action::Pick>::SendGoalOptions();
        send_goal_options.result_callback =
            [this](const auto& result)
            {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                {
                    pick_complete_ = true;
                    pick_success_ = true;
                }
                else
                {
                    pick_complete_ = true;
                    pick_success_ = false;
                }
            };

        pick_action_client_->async_send_goal(goal, send_goal_options);
        pick_complete_ = false;
        pick_success_ = false;

        RCLCPP_INFO(node_->get_logger(), 
            "Starting pick operation for %s using %s", 
            object_name.c_str(), articulation_group.c_str());

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (pick_complete_)
        {
            if (pick_success_)
            {
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                return BT::NodeStatus::FAILURE;
            }
        }
        return BT::NodeStatus::RUNNING;
    }

private:
    std::string selectBestArticulationGroup(const std::string& object_name)
    {
        // Get object pose
        auto req = std::make_shared<robot_interfaces::srv::GetObjectPose::Request>();
        req->object_id = object_name;
        
        auto result = object_pose_client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(node_, result) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return "";
        }

        geometry_msgs::msg::PoseStamped object_pose = result.get()->pose;

        // Evaluate each articulation group
        std::vector<std::string> groups = {"Boom_System", "Right_Arm", "Left_Arm"};
        std::string best_group = "";
        double best_score = -1.0;

        for (const auto& group : groups)
        {
            // Check reachability (simplified)
            double reachability_score = evaluateReachability(group, object_pose);
            if (reachability_score > best_score)
            {
                best_score = reachability_score;
                best_group = group;
            }
        }

        return best_group;
    }

    double evaluateReachability(const std::string& group, 
                               const geometry_msgs::msg::PoseStamped& target)
    {
        // Simplified: check if target is within workspace
        // In real implementation, use IK solver and collision checker
        return 1.0; // Placeholder
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<moveit_msgs::action::Pick>::SharedPtr pick_action_client_;
    rclcpp::Client<robot_interfaces::srv::GetObjectPose>::SharedPtr object_pose_client_;
    bool pick_complete_ = false;
    bool pick_success_ = false;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<ObjectPickNode>("Object_Pick");
}
```

## Integration with Arm Control

`Object_Pick` interfaces with:
- **Arm Control Systems**: For articulated arm manipulation
- **Gripper Systems**: For end effector control (suction, grippers, hooks)
- **Motion Planning**: For trajectory generation
- **Collision Checking**: For safe path planning

## Next Steps

- [World_Find](world_find.md) - Find objects in 3D scene before picking
- [Visual_Find](../api-reference/behavior-tree-high-level-nodes.md#visual_find) - Active camera-based object search
- [Object_Place](../api-reference/behavior-tree-high-level-nodes.md#object_place) - Place picked objects
- [Behavior Tree Node Reference](node-reference.md) - Complete node documentation
- [TSBT-VLA System Overview](../architecture/tsbt-vla-system/overview.md) - How nodes are generated

