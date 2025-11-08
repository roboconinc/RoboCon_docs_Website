# Behavior Tree High-Level Nodes

High-level nodes use inference and reasoning to develop execution plans. These nodes may:
- Generate sequences of low-level actions
- Create complex behavior tree subtrees dynamically
- Make context-aware decisions based on environment state
- Plan multi-step operations using the TSBT-VLA System

## Arm Control Nodes

### `Arm_Move_IK_Execute`

Executes a previously planned inverse kinematics movement.

**Description**: Executes a movement plan that was previously computed by an IK planning node. This separates planning from execution, allowing plans to be validated or modified before execution.

**Controls**:
- Arm joint controllers
- Trajectory execution
- Collision avoidance monitoring

**Arguments:** None

**XML Example:**
```xml
<Arm_Move_IK_Execute/>
```

**C++ Example (BehaviorTree.CPP v4.6+):**
```cpp
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class ArmMoveIKExecuteNode : public BT::StatefulActionNode
{
public:
    ArmMoveIKExecuteNode(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("arm_ik_execute_node");
        traj_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/arm/joint_trajectory", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus onStart() override
    {
        // Retrieve planned trajectory from blackboard
        auto* blackboard = config().blackboard.get();
        if (!blackboard)
        {
            RCLCPP_ERROR(node_->get_logger(), "Blackboard not available");
            return BT::NodeStatus::FAILURE;
        }

        trajectory_msgs::msg::JointTrajectory traj;
        if (!blackboard->get("planned_trajectory", traj))
        {
            RCLCPP_ERROR(node_->get_logger(), 
                "No planned trajectory found in blackboard");
            return BT::NodeStatus::FAILURE;
        }

        traj_pub_->publish(traj);
        RCLCPP_INFO(node_->get_logger(), 
            "Executing IK trajectory with %zu points", traj.points.size());

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        // Check trajectory execution status
        // In real implementation, subscribe to execution feedback
        if (/* trajectory complete */) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<ArmMoveIKExecuteNode>("Arm_Move_IK_Execute");
}
```

---

### `Arm_Move_IK_Plan_ByObjectId`

Plans an inverse kinematics movement to move an arm to a target object.

**Description**: Uses inverse kinematics to compute a trajectory from the current arm pose to a target object. The plan is stored for later execution by `Arm_Move_IK_Execute`. This node interfaces with the robot's IK solver and object tracking system.

**Controls**:
- IK solver
- Object pose queries
- Trajectory planning
- Collision checking

**Arguments:**
- `Articulation Group` (Text) - Arm articulation group identifier
  - Example: `Right_Arm`
- `From` (Text) - Starting position identifier (optional)
  - Example: (empty)
- `To` (Text) - Target object identifier
  - Example: `Screw Location`
- `Plan ID` (Integer) - Plan identifier
  - Example: `201`

**XML Example:**
```xml
<Arm_Move_IK_Plan_ByObjectId Articulation_Group="Right_Arm" To="Screw Location" Plan_ID="201"/>
```

**Example Prompt:** Move right arm to screw

**C++ Example (BehaviorTree.CPP v4.6+):**
```cpp
#include <geometry_msgs/msg/pose.hpp>

class ArmMoveIKPlanByObjectIdNode : public BT::SyncActionNode
{
public:
    ArmMoveIKPlanByObjectIdNode(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("arm_ik_plan_node");
        ik_client_ = node_->create_client<moveit_msgs::srv::GetPositionIK>(
            "/compute_ik");
        object_pose_client_ = node_->create_client<robot_interfaces::srv::GetObjectPose>(
            "/object_tracker/get_pose");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("Articulation_Group"),
            BT::InputPort<std::string>("To"),
            BT::InputPort<int>("Plan_ID")
        };
    }

    BT::NodeStatus tick() override
    {
        std::string arm_group, target_object;
        int plan_id;

        if (!getInput("Articulation_Group", arm_group) ||
            !getInput("To", target_object) ||
            !getInput("Plan_ID", plan_id))
        {
            return BT::NodeStatus::FAILURE;
        }

        // Get target object pose
        auto req = std::make_shared<robot_interfaces::srv::GetObjectPose::Request>();
        req->object_id = target_object;

        auto result = object_pose_client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(node_, result) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), 
                "Failed to get pose for object: %s", target_object.c_str());
            return BT::NodeStatus::FAILURE;
        }

        geometry_msgs::msg::Pose target_pose = result.get()->pose;

        // Compute IK solution
        auto ik_req = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
        ik_req->ik_request.group_name = arm_group;
        ik_req->ik_request.pose_stamped.pose = target_pose;

        auto ik_result = ik_client_->async_send_request(ik_req);
        if (rclcpp::spin_until_future_complete(node_, ik_result) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "IK planning failed");
            return BT::NodeStatus::FAILURE;
        }

        // Store trajectory in blackboard for execution
        auto* blackboard = config().blackboard.get();
        if (blackboard)
        {
            trajectory_msgs::msg::JointTrajectory traj;
            // Convert IK solution to trajectory...
            blackboard->set("planned_trajectory", traj);
            blackboard->set("plan_id", plan_id);
        }

        RCLCPP_INFO(node_->get_logger(), 
            "IK plan created for %s to %s (Plan ID: %d)", 
            arm_group.c_str(), target_object.c_str(), plan_id);

        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
    rclcpp::Client<robot_interfaces::srv::GetObjectPose>::SharedPtr object_pose_client_;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<ArmMoveIKPlanByObjectIdNode>("Arm_Move_IK_Plan_ByObjectId");
}
```

---

### `Arm_Sync`

Synchronizes multiple arms with specified timing.

**Description**: Coordinates multiple robotic arms to move in a synchronized manner with precise timing. This is useful for tasks requiring coordinated multi-arm manipulation.

**Controls**:
- Multiple arm controllers
- Synchronization timing
- Motion coordination

**Arguments:**
- `Arm Group` (Text) - Comma-separated list of arm groups
  - Example: `Arm A, Arm B`
- `Delay` (Text) - Synchronization delay
  - Example: `200ms`
- `Timeout` (Text) - Operation timeout
  - Example: `5min`
- `Mode` (Text) - Synchronization mode
  - Example: `Precision`

**XML Example:**
```xml
<Arm_Sync Arm_Group="Arm A, Arm B" Delay="200ms" Timeout="5min" Mode="Precision"/>
```

**Example Prompt:** Sync Arm A and B with 200ms delay

---

## Attachment and Tool Nodes

### `Attachment_Swap`

Swaps robot attachment/tool.

**Description**: Coordinates the exchange of end effector attachments or tools. This involves releasing the current tool, moving to a tool storage location, and engaging a new tool.

**Controls**:
- Tool release mechanism
- Tool storage system
- Tool engagement mechanism
- Tool identification

**Arguments:**
- `Current Tool` (Text) - Current tool identifier
  - Example: `Gripper`
- `New Tool` (Text) - Target tool identifier
  - Example: `Nail Gun`
- `Timeout` (Text) - Operation timeout
  - Example: `30s`
- `Priority` (Text) - Operation priority
  - Example: `High`

**XML Example:**
```xml
<Attachment_Swap Current_Tool="Gripper" New_Tool="Nail Gun" Timeout="30s" Priority="High"/>
```

**Example Prompt:** Swap from gripper to nail gun in 30s (high priority)

---

## Audio and Notification Nodes

### `Audio_Play`

Plays an audio file.

**Description**: Plays an audio file through the robot's audio system. Useful for user notifications, warning sounds, or operational feedback.

**Controls**:
- Audio playback system
- Volume control

**Arguments:**
- `Audio File` (Text) - Audio file identifier
  - Example: `Warning Beep`

**XML Example:**
```xml
<Audio_Play Audio_File="Warning Beep"/>
```

**Example Prompt:** Play warning beep sound

**C++ Example (BehaviorTree.CPP v4.6+):**
```cpp
#include <std_msgs/msg/string.hpp>

class AudioPlayNode : public BT::SyncActionNode
{
public:
    AudioPlayNode(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("audio_play_node");
        audio_pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/audio/play", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("Audio_File")};
    }

    BT::NodeStatus tick() override
    {
        std::string audio_file;
        if (!getInput("Audio_File", audio_file))
        {
            return BT::NodeStatus::FAILURE;
        }

        std_msgs::msg::String msg;
        msg.data = audio_file;
        audio_pub_->publish(msg);

        RCLCPP_INFO(node_->get_logger(), "Playing audio: %s", audio_file.c_str());

        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr audio_pub_;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<AudioPlayNode>("Audio_Play");
}
```

---

### `Notify`

Sends a notification message (typically for errors).

**Description**: Logs an error or warning notification to the system log and user interface.

**Controls**:
- Logging system
- User notification interface

**Arguments:**
- `errorMsg` (Text) - Error message text
  - Example: `Could not find red flag`

**XML Example:**
```xml
<Notify errorMsg="Could not find red flag"/>
```

---

### `User_Notify`

Sends a notification to the user.

**Description**: Sends a user-facing notification that appears in the user interface or control station.

**Controls**:
- User interface
- Notification system

**Arguments:**
- `Message` (Text) - Notification message
  - Example: `Task Complete`

**XML Example:**
```xml
<User_Notify Message="Task Complete"/>
```

**Example Prompt:** Notify user: Task complete

---

## Battery Management Nodes

### `Battery_Charge`

Moves robot to charging station and initiates charging.

**Description**: Plans a path to a charging station, navigates to it, and initiates the charging process. Monitors charging status until complete.

**Controls**:
- Navigation system
- Charging station interface
- Battery monitoring
- Charging protocol

**Arguments:**
- `Station` (Text) - Charging station identifier
  - Example: `Charging Dock 1`

**XML Example:**
```xml
<Battery_Charge Station="Charging Dock 1"/>
```

**Example Prompt:** Move to charging dock 1

---

### `Battery_Swap`

Performs battery swap operation.

**Description**: Coordinates the battery swap process, including navigation to a battery swap station, safe battery removal, and installation of a new battery.

**Controls**:
- Battery release mechanism
- Battery storage system
- Battery engagement system
- Safety interlocks

**Arguments:**
- `Battery Type` (Text) - Target battery type
  - Example: `High-Capacity`
- `Timeout` (Text) - Operation timeout
  - Example: `5min`
- `Mode` (Text) - Swap mode
  - Example: `Manual`
- `Safety` (Text) - Safety mode
  - Example: `Locked`

**XML Example:**
```xml
<Battery_Swap Battery_Type="High-Capacity" Timeout="5min" Mode="Manual" Safety="Locked"/>
```

**Example Prompt:** Swap to high-capacity battery (manual)

---

### `Battery_Test`

Performs battery testing operation.

**Description**: Runs diagnostic tests on the battery system to verify capacity, health, and performance characteristics.

**Controls**:
- Battery test equipment
- Load testing
- Performance monitoring

**Arguments:**
- `Test Type` (Text) - Type of test
  - Example: `Load`
- `Duration` (Text) - Test duration
  - Example: `5min`
- `Threshold` (Text) - Test threshold value
  - Example: `0.8`
- `Mode` (Text) - Test mode
  - Example: `Auto`

**XML Example:**
```xml
<Battery_Test Test_Type="Load" Duration="5min" Threshold="0.8" Mode="Auto"/>
```

**Example Prompt:** Test battery under load

---

## Boom Control (High-Level) Nodes

### `Boom_Extend` (High-Level)

High-level boom extension with additional safety and control parameters.

**Description**: Extends the boom with intelligent planning, safety checks, and automatic stabilizer deployment. This is a higher-level version of the low-level `Boom_Extend` node.

**Controls**:
- Boom extension system
- Stabilizer deployment
- Safety monitoring
- Speed control

**Arguments:**
- `Section` (Text) - Boom section identifier
  - Example: `Main Boom`
- `Length` (Text) - Target length
  - Example: `8 meters`
- `Speed` (Text) - Extension speed
  - Example: `Medium`
- `Stabilizers` (Text) - Stabilizer deployment mode
  - Example: `Deploy`

**XML Example:**
```xml
<Boom_Extend Section="Main Boom" Length="8 meters" Speed="Medium" Stabilizers="Deploy"/>
```

**Example Prompt:** Extend main boom to 8m (medium speed)

---

### `Boom_Fold`

Folds boom sections.

**Description**: Intelligently folds boom sections for transport or storage, with proper sequencing and safety checks.

**Controls**:
- Boom folding mechanism
- Section locks
- Safety interlocks

**Arguments:**
- `Section` (Text) - Boom section to fold
  - Example: `Upper Boom`
- `Speed` (Text) - Folding speed
  - Example: `Slow`
- `Lock` (Text) - Lock mode
  - Example: `Manual`
- `Timeout` (Text) - Operation timeout
  - Example: `2min`

**XML Example:**
```xml
<Boom_Fold Section="Upper Boom" Speed="Slow" Lock="Manual" Timeout="2min"/>
```

**Example Prompt:** Fold upper boom slowly (manual lock)

---

## Camera and Vision Nodes

### `Visual_Find`

Finds objects using computer vision.

**Description**: Uses computer vision and object detection to locate objects in the environment. The detected object ID is stored in the blackboard for use by subsequent nodes.

**Controls**:
- Camera system
- Object detection AI
- Object tracking
- Blackboard variables

**Arguments:**
- `ObjectClass` (Text) - Object class to find
  - Example: `Sheathing_Stack`
- `objectId` (Output) - Object identifier (output to blackboard)
  - Example: `4` (stored in blackboard)

**XML Example:**
```xml
<Visual_Find ObjectClass="Sheathing_Stack" objectId="{objectId}"/>
```

**Example Prompt:** Find the nearest wood screws

**C++ Example (BehaviorTree.CPP v4.6+):**
```cpp
#include <vision_msgs/msg/detection2_d_array.hpp>

class VisualFindNode : public BT::StatefulActionNode
{
public:
    VisualFindNode(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("visual_find_node");
        detection_sub_ = node_->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/object_detections", 10,
            std::bind(&VisualFindNode::detectionCallback, this, std::placeholders::_1));
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("ObjectClass"),
            BT::OutputPort<int>("objectId")
        };
    }

    BT::NodeStatus onStart() override
    {
        std::string object_class;
        if (!getInput("ObjectClass", object_class))
        {
            return BT::NodeStatus::FAILURE;
        }

        target_class_ = object_class;
        found_object_id_ = -1;

        RCLCPP_INFO(node_->get_logger(), 
            "Searching for object class: %s", object_class.c_str());

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (found_object_id_ >= 0)
        {
            setOutput("objectId", found_object_id_);
            RCLCPP_INFO(node_->get_logger(), 
                "Found object ID: %d", found_object_id_);
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
    {
        for (const auto& detection : msg->detections)
        {
            if (detection.results[0].id == target_class_)
            {
                found_object_id_ = detection.results[0].id;
                break;
            }
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
    std::string target_class_;
    int found_object_id_;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<VisualFindNode>("Visual_Find");
}
```

---

## Object Pick Reachability Nodes

### `Object_Pick_Check`

Performs a fast check to determine if the articulated body can reach the object without needing to `Travel` (move the base).

**Description**: Checks if an articulated body (boom, arm, etc.) can reach a target object given its current configuration. This is a lightweight check that performs collision checking and rough kinematic analysis. Outputs an incomplete plan containing information about how the check was performed.

**Controls**:
- Articulated body kinematic model
- Collision checking
- Workspace analysis
- Object pose queries

**Arguments:**
- `Object_Name` (Text) - Target object identifier
  - Example: `{targetObjectId}`
- `Articulation_Group` (Text, Optional) - Articulated group to check
  - Example: `Boom_System`, `Right_Arm`
  - If blank, uses default or auto-selected group

**Output:**
- Stores reachability information in blackboard
- Outputs incomplete plan with collision check results and rough kinematic check

**XML Example:**
```xml
<Object_Pick_Check Object_Name="{targetObjectId}" Articulation_Group="Boom_System"/>
```

**Returns:**
- **SUCCESS**: Object is reachable without base movement
- **FAILURE**: Object is not reachable, requires `Travel` or different articulation group

---

### `Object_Pick_Plan`

Performs full planning for the pick operation, including detailed collision checking and safety validation.

**Description**: This is a more costly operation that performs complete planning for picking an object. It checks if the object cannot be safely or possibly picked, and generates a full pick plan if feasible.

**Controls**:
- Full path planning system
- Detailed collision checking
- Safety validation
- Grasp feasibility analysis
- Trajectory generation

**Arguments:**
- `Object_Name` (Text) - Target object identifier
  - Example: `{targetObjectId}`
- `Articulation_Group` (Text, Optional) - Articulated group to use
  - Example: `Boom_System`, `Right_Arm`
  - If blank, uses default or auto-selected group

**Output:**
- Full pick plan stored in blackboard
- Trajectory information
- Grasp configuration

**Returns:**
- **SUCCESS**: Full pick plan generated and stored
- **FAILURE**: Object cannot be safely or possibly picked

**XML Example:**
```xml
<Object_Pick_Plan Object_Name="{targetObjectId}" Articulation_Group="Boom_System"/>
```

**Note**: This node is typically used after `Object_Pick_Check` confirms reachability, or in scenarios where full planning is required before execution.

---

## Navigation and Travel Nodes

### `Travel_Plan`

Plans a navigation path to a destination.

**Description**: Uses the navigation stack (Nav2) to plan an optimal path from the current location to a destination. The destination can be specified as a named location or semantic description.

**Controls**:
- Nav2 navigation stack
- Path planning
- Costmap updates
- Global and local planners

**Arguments:**
- `From` (Text) - Starting location (can use blackboard variable)
  - Example: `{Current Location}`
- `To` (Text) - Destination description
  - Example: `Building B, Floor 02, Bathroom`

**XML Example:**
```xml
<Travel_Plan From="{Current Location}" To="Building B, Floor 02, Bathroom"/>
```

**Example Prompt:** Travel to Building B second floor bathroom

**C++ Example (BehaviorTree.CPP v4.6+):**
```cpp
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <behaviortree_cpp/action_node.h>

class TravelPlanNode : public BT::StatefulActionNode
{
public:
    TravelPlanNode(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("travel_plan_node");
        nav_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            node_, "navigate_to_pose");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("From"),
            BT::InputPort<std::string>("To")
        };
    }

    BT::NodeStatus onStart() override
    {
        std::string from, to;
        if (!getInput("From", from) || !getInput("To", to))
        {
            return BT::NodeStatus::FAILURE;
        }

        // Query pose service for destination
        // This is simplified - in practice, you'd query a semantic location service
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.frame_id = "map";
        // Set goal_pose.position from location service...

        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose = goal_pose;

        if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(node_->get_logger(), "Navigation action server not available");
            return BT::NodeStatus::FAILURE;
        }

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            [this](const auto& result)
            {
                // Handle navigation result
            };

        nav_action_client_->async_send_goal(goal_msg, send_goal_options);

        RCLCPP_INFO(node_->get_logger(), 
            "Navigating from %s to %s", from.c_str(), to.c_str());

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        // Check navigation status
        // In practice, this would check the action status
        if (/* navigation complete */) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_action_client_;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<TravelPlanNode>("Travel_Plan");
}
```

---

### `Travel_Plan_ToOptimalLiftPoint`

Plans travel to optimal lift point between source and target.

**Description**: Computes an optimal position for the crane/robot that allows it to reach both a source and target location for lifting operations.

**Controls**:
- Path planning
- Workspace analysis
- Reachability computation

**Arguments:**
- `source` - Source object identifier (can use blackboard variable)
  - Example: `{AttachObjectId}`
- `target` - Target object identifier (can use blackboard variable)
  - Example: `{GoalPointObjectId}`
- `goal` (Output) - Output goal location (blackboard variable)
  - Example: `{travelGoal}`
- `mode` (Text) - Planning mode
  - Example: `within_range_of_both`

**XML Example:**
```xml
<Travel_Plan_ToOptimalLiftPoint source="{AttachObjectId}" target="{GoalPointObjectId}" goal="{travelGoal}" mode="within_range_of_both"/>
```

---

### `Travel_Follow`

Follows a path or leader.

**Description**: Follows a predefined path or tracks another robot/vehicle. Uses path tracking or leader-follower algorithms.

**Controls**:
- Path tracking
- Velocity control
- Leader detection

**Arguments:** None

**XML Example:**
```xml
<Travel_Follow/>
```

---

## Object Manipulation Nodes

### `Object_Pick`

Picks up an object.

**Description**: Coordinates the complete pick operation, including approach, grasp planning, execution, and verification. Interfaces with the arm control and gripper systems.

**Controls**:
- Arm control
- Gripper/end effector
- Object pose tracking
- Grasp planning

**Arguments:**
- `Object_Name` (Text) - Object identifier
  - Example: `{foundObjectId}` or `Red Cup`
- `Articulation_Group` (Text, Optional) - Which articulated group of joints will be responsible for the picking
  - Example: `Right_Arm`, `Boom_System`, `Crane_Arm`
  - If left blank, automatically selects the best way to pick the object
  - Default: `""` (auto-select)
- `Gripper` (Text, Optional) - Gripper/end effector type
  - Example: `Suction`

**XML Example:**
```xml
<!-- With automatic articulation group selection -->
<Object_Pick Object_Name="{foundObjectId}" Articulation_Group=""/>

<!-- With specific articulation group -->
<Object_Pick Object_Name="Red Cup" Articulation_Group="Right_Arm" Gripper="Suction"/>
```

**Example Prompt:** Pick up red cup with suction

**See Also:** [Object_Pick Detailed Documentation](../behavior-tree/object_pick.md) for complete details including reachability checks and automatic group selection.

---

### `Object_Place`

Places an object at a location.

**Description**: Coordinates placing an object at a specified location, including approach planning, placement execution, and release verification.

**Controls**:
- Arm control
- Gripper/end effector
- Place location tracking
- Placement verification

**Arguments:**
- `Location` (Text) - Target location identifier
  - Example: `Table 3`
- `Gripper` (Text) - Gripper/end effector type
  - Example: `Suction`

**XML Example:**
```xml
<Object_Place Location="Table 3" Gripper="Suction"/>
```

**Example Prompt:** Place object on Table 3 using suction

---

## Hook and Load Manipulation Nodes

### `Hook_Release`

Releases hook/load.

**Description**: Releases a load from the hook in a controlled manner. Supports various release modes including controlled drop, immediate release, or timed release.

**Controls**:
- Hook release mechanism
- Load monitoring
- Safety systems

**Arguments:**
- `Mode` (Text) - Release mode
  - Example: `Controlled Drop`
- `Height` (Text) - Release height
  - Example: `2m`
- `Load Type` (Text) - Type of load
  - Example: `Container`
- `Timeout` (Text) - Operation timeout
  - Example: `10s`

**XML Example:**
```xml
<Hook_Release Mode="Controlled Drop" Height="2m" Load_Type="Container" Timeout="10s"/>
```

**Example Prompt:** Release container from 2m (controlled)

---

## Emergency and Safety Nodes

### `Emergency_Stop`

Executes emergency stop procedure.

**Description**: Immediately stops all robot motion and activates safety systems. This is a high-priority safety node that overrides normal operation.

**Controls**:
- All motion systems
- Safety interlocks
- Emergency systems

**Arguments:**
- `Reason` (Text) - Emergency stop reason
  - Example: `Obstacle Detected`

**XML Example:**
```xml
<Emergency_Stop Reason="Obstacle Detected"/>
```

**Example Prompt:** Emergency stop due to obstacle

**C++ Example (BehaviorTree.CPP v4.6+):**
```cpp
#include <std_msgs/msg/string.hpp>

class EmergencyStopNode : public BT::SyncActionNode
{
public:
    EmergencyStopNode(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("emergency_stop_node");
        emergency_pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/emergency/stop", 10);
        // Also publish to direct hardware safety channel
        safety_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
            "/safety/emergency_stop", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("Reason")};
    }

    BT::NodeStatus tick() override
    {
        std::string reason;
        if (!getInput("Reason", reason))
        {
            reason = "Unknown";
        }

        // Immediate safety stop
        std_msgs::msg::Bool safety_msg;
        safety_msg.data = true;
        safety_pub_->publish(safety_msg);

        // Log emergency stop
        std_msgs::msg::String reason_msg;
        reason_msg.data = reason;
        emergency_pub_->publish(reason_msg);

        RCLCPP_ERROR(node_->get_logger(), 
            "EMERGENCY STOP: %s", reason.c_str());

        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr emergency_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_pub_;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<EmergencyStopNode>("Emergency_Stop");
}
```

---

## Next Steps

- [Low-Level Nodes](behavior-tree-low-level-nodes.md) - Direct hardware control nodes
- [Behavior Trees API](behavior-trees.md) - Complete BehaviorTree.CPP documentation
- [TSBT-VLA System Overview](../architecture/tsbt-vla-system/overview.md) - Dynamic behavior tree generation
- [Red Flag Detection Example](../ai-programs/examples/red-flag-detection.md) - Complete working example

