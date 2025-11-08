# Path Planning

Advanced path planning with Nav 2 in ROBOCON OS, including hybrid algorithmic and AI-based planning.

## Overview

Path planning in ROBOCON OS uses a hybrid approach combining:

- **Algorithmic Planning**: Traditional deterministic algorithms
- **AI-Based Planning**: Action inference model for intuitive paths
- **Animation Matching**: Reuse of recorded motion patterns
- **LLM-Assisted Planning**: Natural language to action decomposition

## Planner Server Architecture

### Hybrid Planning System

```
Goal Request
    │
    ├─► Algorithmic Planner (Fast, Deterministic)
    │   └─► Standard path planning algorithms
    │
    ├─► Action Inference Model (Intuitive, Adaptive)
    │   └─► AI-based path generation
    │
    └─► Animation Database Search (Optimized)
        └─► Pre-recorded motion patterns
```

### Action-Based Planning

**Action Type**: Planning  
**Input**: Goal pose, start pose, costmap, footprint  
**Output**: Path (sequence of poses)

## Planning Methods

### 1. Algorithmic Planning

Standard deterministic path planning algorithms.

#### Available Algorithms

**NavFn Planner**:
- Gradient-based planning
- Fast and reliable
- Good for simple environments

**Theta* Planner**:
- Any-angle path planning
- Smoother paths
- Better for large open spaces

**Smac Planner**:
- State Machine A* variant
- Efficient for dynamic environments
- Supports multiple motion models

**Configuration**:
```yaml
planner_server:
  planner_plugins: ["GridBased"]
  GridBased:
    plugin: "nav2_navfn_planner/NavfnPlanner"
    tolerance: 0.5
    use_astar: false
    allow_unknown: true
```

#### Path Planning 2D

**Plugin**: `nav2_planner::ComputePathToPose`

**Inputs**:
- **Costmap**: Global or local costmap representing obstacles
- **Footprint**: Robot footprint for collision checking
- **Start Pose**: Starting position
- **Goal Pose**: Target position

**Output**: Path (nav_msgs/Path)

**Usage**:
```python
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.planner_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose'
        )
    
    def plan_path(self, start, goal):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start
        goal_msg.goal = goal
        goal_msg.planner_id = "GridBased"
        
        self.planner_client.wait_for_server()
        future = self.planner_client.send_goal_async(goal_msg)
        return future
```

### 2. AI-Based Action Inference

Uses machine learning models to generate intuitive paths.

**Features**:
- Learns from past successful paths
- Adapts to environment patterns
- Handles complex scenarios
- Provides natural motion

**When to Use**:
- Complex environments
- Unconventional navigation needs
- Learning from operator patterns

### 3. Action Animation Database

Pre-recorded motion patterns for fast planning.

#### Local Vertex Animations

**Concept**: Recorded animations of robot/joint movements between poses

**Database Structure**:
```python
AnimationEntry:
    animation_id: str
    start_pose: Pose
    end_pose: Pose
    joint_trajectory: JointTrajectory
    duration: float
    metadata: dict
        - environment_type
        - conditions
        - success_rate
```

#### Animation Matching Process

```
1. Receive Goal
   ↓
2. Search Animation Database
   - Compare start/goal poses
   - Calculate similarity scores
   ↓
3. IF: High Similarity Found
   THEN: Reuse Animation
   ELSE: Use Full Planning
   ↓
4. Modify Animation (if needed)
   - Algorithmic adjustments
   - LLM-assisted modifications
   ↓
5. Output Path
```

#### Animation Modification

**Algorithmic Modification**:
- Scale trajectory timing
- Adjust joint angles proportionally
- Translate poses
- Rotate orientations

**LLM-Assisted Modification**:
- Understand context requirements
- Make intuitive adjustments
- Handle edge cases
- Optimize for specific tasks

**Example**:
```python
# Found animation: Move from A to B
# Goal: Move from A to C (similar to B but offset)
# 
# 1. Algorithm scales trajectory
# 2. LLM adjusts for offset
# 3. Result: Natural motion adapted to new goal
```

### 4. LLM-Assisted Action Decomposition

Breaking complex high-level actions into executable sequences.

#### Process Flow

```
High-Level Action (Text)
    ↓
LLM Processing
    ↓
Parse into Components:
  - Path segments
  - Coordinates
  - Transitions
  - Waypoints
    ↓
Generate Action Node Sequence
    ↓
Insert into Behavior Tree
    ↓
Execute Sequentially
```

#### Example Decomposition

**Input**: "Navigate through construction site to deliver materials to building 3, avoiding heavy equipment and following safety protocols"

**LLM Decomposition**:
```python
Actions = [
    {
        "type": "navigate",
        "goal": "site_entrance",
        "constraints": ["follow_path"]
    },
    {
        "type": "navigate",
        "goal": "construction_zone_1",
        "constraints": ["avoid_heavy_equipment", "safety_protocols"]
    },
    {
        "type": "navigate",
        "goal": "narrow_passage",
        "constraints": ["slow_speed", "obstacle_awareness"]
    },
    {
        "type": "navigate",
        "goal": "building_3_delivery",
        "constraints": ["precise_positioning"]
    }
]
```

**Execution**:
- Each action becomes a behavior tree node
- Navigator Server ticks through sequence
- Each path solved individually using appropriate method

## Path Planning Methods Comparison

### Animation Matching + Algorithm/AI Correction

**Speed**: ⚡⚡⚡ (Very Fast)  
**Accuracy**: ✅✅✅ (High - uses proven patterns)  
**Adaptability**: ✅✅ (Good - with corrections)

**Best For**:
- Common navigation patterns
- Repetitive tasks
- Fast execution needed

### Pure Algorithm Planning

**Speed**: ⚡⚡ (Fast)  
**Accuracy**: ✅✅✅ (High - deterministic)  
**Adaptability**: ✅ (Limited to algorithm capabilities)

**Best For**:
- Standard navigation
- Predictable environments
- Real-time requirements

### Full AI Planning

**Speed**: ⚡ (Slower)  
**Accuracy**: ✅✅✅✅ (Very High - adaptive)  
**Adaptability**: ✅✅✅✅ (Excellent)

**Best For**:
- Complex scenarios
- Novel situations
- Learning from patterns

## Trajectory Generation and Critiquing

### Trajectory Generation

After path planning, trajectories are generated:

```
Path
    ↓
┌─────────────────────┐
│ Algorithm Trajectory│
│ Generation          │
└─────────────────────┘
    ↓
┌─────────────────────┐
│ AI Trajectory       │
│ Generation (optional)│
└─────────────────────┘
    ↓
Both Trajectories
    ↓
Critic Evaluation
```

### Critic Evaluation

Trajectories are evaluated by critics to select best option:

**Critic Factors**:

1. **Speed Critic**
   - Execution time
   - Velocity profiles
   - Time to goal

2. **Accuracy Critic**
   - Path following precision
   - Goal alignment
   - Waypoint accuracy

3. **Safety Critic**
   - Collision margins
   - Obstacle distances
   - Emergency stop capability

4. **Smoothness Critic**
   - Jerk minimization
   - Acceleration limits
   - Comfort factor

5. **Energy Critic**
   - Power consumption
   - Efficiency
   - Battery impact

**Selection Criteria**:
```python
trajectory_score = (
    speed_weight * speed_score +
    accuracy_weight * accuracy_score +
    safety_weight * safety_score +
    smoothness_weight * smoothness_score +
    energy_weight * energy_score
)
```

**Trade-off Analysis**:
- Fast but less accurate: Algorithm trajectory
- Accurate but slower: AI trajectory
- Balanced: Hybrid approach

## Global Frame Planning

### Coordinate Frames

**Global Frame** (`map`):
- Fixed coordinate system
- Used for long-range planning
- Persistent across sessions

**Planning Process**:
```
Goal in Global Frame
    ↓
Planner Server (operates in global frame)
    ↓
Path in Global Frame
    ↓
Transform to Local Frame
    ↓
Controller Server (executes in local frame)
```

### Frame Transformation

```python
from tf2_ros import TransformListener
from geometry_msgs.msg import PoseStamped

class FrameTransformer(Node):
    def __init__(self):
        super().__init__('frame_transformer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def transform_to_global(self, pose_local, target_frame='map'):
        """Transform pose from local to global frame"""
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                pose_local.header.frame_id,
                rclpy.time.Time()
            )
            pose_global = do_transform_pose(pose_local, transform)
            return pose_global
        except Exception as e:
            self.get_logger().error(f"Transform failed: {e}")
            return None
```

## MoveIt 2 Integration

### Articulation Trajectory Planning

For articulated robots (arms, manipulators):

**Integration**:
- MoveIt 2 scene synchronization
- 3D collision checking
- Joint trajectory generation

**Process**:
```
Goal Pose (End Effector)
    ↓
MoveIt 2 Planning
    ↓
Joint Trajectory
    ↓
Controller Server (Articulation Mode)
    ↓
Motor Commands
```

## Configuration Examples

### Basic Configuration

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

### With Multiple Planners

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased", "ThetaStar", "SmacHybrid"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
    ThetaStar:
      plugin: "nav2_theta_star_planner/ThetaStarPlanner"
    SmacHybrid:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"
      resolution: 0.05
```

### Animation Database Configuration

```yaml
animation_database:
  ros__parameters:
    database_path: "/opt/robocon/animations"
    similarity_threshold: 0.85
    enable_llm_modification: true
    llm_model: "deepseek-chat"
```

## Code Examples

### Using Animation Database

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose
from animation_database_msgs.srv import SearchAnimation

class AnimationAwarePlanner(Node):
    def __init__(self):
        super().__init__('animation_aware_planner')
        self.animation_client = self.create_client(
            SearchAnimation, 'search_animation_database'
        )
    
    def plan_with_animation(self, start, goal):
        # First try animation database
        animation_request = SearchAnimation.Request()
        animation_request.start = start
        animation_request.goal = goal
        animation_request.similarity_threshold = 0.85
        
        future = self.animation_client.call_async(animation_request)
        
        # If found, use animation
        # Otherwise, use standard planner
        return future
```

### LLM-Assisted Planning

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from llm_planning_msgs.srv import DecomposeAction

class LLMPlanner(Node):
    def __init__(self):
        super().__init__('llm_planner')
        self.llm_client = self.create_client(
            DecomposeAction, 'decompose_navigation_action'
        )
    
    def decompose_action(self, action_text):
        request = DecomposeAction.Request()
        request.action_description = action_text
        
        future = self.llm_client.call_async(request)
        return future  # Returns sequence of action nodes
```

## Best Practices

1. **Choose Appropriate Method**
   - Use animation matching for common patterns
   - Use algorithms for standard navigation
   - Use AI for complex scenarios

2. **Optimize Database**
   - Regularly update animation database
   - Remove obsolete patterns
   - Add successful new patterns

3. **Balance Speed and Accuracy**
   - Configure critic weights appropriately
   - Adjust based on mission requirements
   - Consider energy constraints

4. **Monitor Performance**
   - Track planning success rates
   - Monitor planning times
   - Update parameters based on data

## Troubleshooting

### Planning Fails

1. Check costmap is updating
2. Verify start/goal are valid
3. Ensure footprint is correct
4. Check for planner errors

### Animation Database Issues

1. Verify database path is correct
2. Check similarity thresholds
3. Ensure animations are valid
4. Monitor database size

### LLM Planning Issues

1. Verify LLM service is running
2. Check action description clarity
3. Monitor decomposition quality
4. Validate generated sequences

## Next Steps

- [Nav 2 Integration](../architecture/nav2-integration.md) - Complete Nav 2 documentation
- [Navigation Stack](./navigation-stack.md) - Stack setup
- [Planners](./planners.md) - Individual planner details
- [Costmaps](./costmaps.md) - Costmap configuration
