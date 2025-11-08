# TSBT-VLA System Overview

**Title:** Text Scene Behavior Tree Auditable Visual Language Action System for Autonomous Equipment  
**Inventors:** Brent Lee, Brad Lee

## Overview

The **Text Scene Behavior Tree Auditable Visual Language Action System (TSBT-VLA)** is a novel, fully auditable, text-based architecture that enables robust, interpretable, and recursively updatable robotic planning pipelines for autonomous equipment.

Unlike humanoid robots optimized for lightweight inference, autonomous equipment requires **auditable, robust, and real-time accountable** decision-making due to the severe consequences of errors in industrial and construction environments.

### Key Advantages

- ✅ **Full Auditability**: Every decision is traceable through text-based logs
- ✅ **Interpretable Reasoning**: Human-readable scene descriptions and behavior trees
- ✅ **Recursive Planning**: Dynamic subtrees generated based on changing context
- ✅ **Symbolic Text-First Approach**: All transformations explicitly represented in text
- ✅ **Regulation Compliance**: Suitable for safety-critical, high-risk environments
- ✅ **High Compute Support**: Optimized for edge systems with 100+ TFLOPS (FP16)

## System Architecture

```mermaid
flowchart TD
    A[Sensor Integration Layer<br/>Depth Cameras, LiDAR, GPS, IMU,<br/>Odometry, Arms] -->|Sensor Topics<br/>Data Distribution Service| B[Text Conversion Layer]
    
    B --> C[Text World<br/>3D Scene to Text]
    B --> D[Sensor Text<br/>Temporal Logs]
    B --> E[High-Level Action<br/>User Input]
    
    C --> F[Language Model Inference<br/>Large Language Model<br/>Reasoning Engine]
    D --> F
    E --> F
    
    F --> G[Behavior Tree XML<br/>Subtree Generation]
    
    G --> H[Behavior Tree Execution<br/>Navigation Server<br/>Behavior Tree Library]
    
    H --> I[Hardware Control Layer<br/>Motors, Actuators, Sensors,<br/>Communication]
    
    style A fill:#e1f5ff
    style B fill:#fff4e1
    style F fill:#ffe1f5
    style H fill:#e1ffe1
    style I fill:#f5e1ff
```

## System Tracks

The TSBT-VLA System operates through four major processing tracks that work together to convert sensory input and user commands into executable behavior trees:

```mermaid
flowchart LR
    subgraph Track1[Track 1: 3D World to Text]
        T1A[Sensor Sources<br/>RGB Camera, LiDAR, Depth] --> T1B[Object Segmentation Module<br/>2D Polygons or Pixel Masks]
        T1B --> T1C[Object Extruder<br/>3D Mesh]
        T1C --> T1D[3D World Scene]
        T1D --> T1E[Text Converter]
        T1E --> T1F[Text World]
    end
    
    subgraph Track2[Track 2: Sensor to Text]
        T2A[Sensor Sources<br/>Energy, Temp, Battery] --> T2B[Sensor Topics]
        T2B --> T2C[Text Converter]
        T2C --> T2D[Sensor Text]
    end
    
    subgraph Track3[Track 3: User Input to Action]
        T3A[Speech/Chat Input] --> T3B[Voice-to-Text<br/>or Chat]
        T3B --> T3C[Sanitizer]
        T3C --> T3D[Behavior Tree Node]
        T3D --> T3E[Navigation Server]
    end
    
    T1F --> LLM[Large Language Model<br/>Reasoning Engine]
    T2D --> LLM
    T3E --> LLM
    
    LLM --> BT[Behavior Tree XML<br/>Subtree Generation]
    
    style Track1 fill:#e1f5ff
    style Track2 fill:#fff4e1
    style Track3 fill:#ffe1f5
    style LLM fill:#ffe1f5
    style BT fill:#e1ffe1
```

### Track 1: 3D World to Text

Converts 3D scene data from multiple sensors into a semantic text representation of the environment.

**Key Components:**
- RGB Camera → Object Segmentation Module → 2D Polygons or Pixel Masks
- LiDAR/Depth Camera → Point Clouds
- Object Extruder → 3D Mesh Reconstruction
- 3D World Scene Graph (Agents, Workpieces, Obstacles, Motion)
- Text Converter → Text World Output

**See**: [3D World to Text Track](3d-world-to-text.md) for detailed information.

---

### Track 2: Sensor to Text

Converts temporal sensor data into text logs for safety and feasibility reasoning.

**Key Components:**
- Sensor Sources (Energy, Temperature, Battery, etc.)
- Sensor Topics (ROS 2)
- Text Converter
- Sensor Text Output

**See**: [Sensor to Text Track](sensor-to-text.md) for detailed information.

---

### Track 3: User Input to Action

Processes natural language input from various interfaces into behavior tree nodes.

**Key Components:**
- Speech Input (Microphone → Voice-to-Text)
- Chat Input (Phone/Tablet/Laptop/Desktop)
- Text Sanitizer
- Behavior Tree Node Creation
- Navigation Server Integration

**See**: [User Input to Action Track](user-input-to-action.md) for detailed information.

---

### Track 4: Large Language Model Processing

The core reasoning engine that combines all text inputs to generate behavior tree XML.

**Inputs:**
- High-Level Action (from User Input)
- Text World (from 3D World to Text)
- Sensor Text (from Sensor to Text)

**Output:**
- Behavior Tree XML (subtree generation)

**Processing:**
- Recursive decomposition
- Dynamic subtree generation
- Context-aware planning

**See**: [Large Language Model Processing](llm-processing.md) for detailed information.

---

### Additional Components

#### Object Segmentation Module

The Object Segmentation Module processes RGB camera images to detect objects and outputs 2D polygons or pixel masks. The module provides a standardized interface for object detection that can be implemented using various computer vision models.

**Output**: 2D Polygons (bounding boxes) or Pixel Masks (segmentation masks)

**See**: [Object Segmentation Module](object-segmentation-module.md) for detailed implementation information.

## Execution Flow

```mermaid
flowchart TD
    Start([System Start]) --> Collect[1. Data Collection<br/>Sensors stream to ROS 2 topics]
    
    Collect --> TextConv[2. Text Conversion<br/>Three Parallel Tracks]
    
    TextConv --> TW[Text World<br/>3D Scene to Text]
    TextConv --> ST[Sensor Text<br/>Temporal Logs]
    TextConv --> HLA[High-Level Action<br/>User Input]
    
    TW --> LLM[3. LLM Processing<br/>Large Language Model]
    ST --> LLM
    HLA --> LLM
    
    LLM --> BTGen[4. Behavior Tree Generation<br/>XML Subtree Output]
    
    BTGen --> Recursive{5. Recursive<br/>Decomposition?}
    Recursive -->|High-Level Node| LLM
    Recursive -->|Low-Level Node| Execute
    
    Execute[6. Execution<br/>Navigation Server<br/>Behavior Tree Library]
    
    Execute --> Hardware[7. Hardware Output<br/>Motors, Actuators, LEDs,<br/>Fan, Speaker, Communication]
    
    Hardware --> End([Complete])
    
    style Collect fill:#e1f5ff
    style TextConv fill:#fff4e1
    style LLM fill:#ffe1f5
    style Execute fill:#e1ffe1
    style Hardware fill:#f5e1ff
```

## Output Hardware

The Behavior Tree Execution layer outputs commands to various hardware systems:

```mermaid
flowchart LR
    BT[Behavior Tree<br/>Execution] --> HW[Hardware Output]
    
    HW --> W1[Wheel Twist<br/>Base Movement]
    HW --> W2[Joint Twist<br/>Arm/Manipulator]
    HW --> W3[Actuator Movement<br/>Linear Actuators]
    HW --> W4[LED Color/Brightness<br/>Status Indicators]
    HW --> W5[Fan Speed<br/>Thermal Management]
    HW --> W6[User Text Prompt<br/>Display/Notification]
    HW --> W7[Speaker Output<br/>Audio Feedback]
    HW --> W8[WiFi/Bluetooth Signal<br/>Communication]
    
    style BT fill:#e1ffe1
    style HW fill:#fff4e1
```

## Recursive Decomposition

High-level actions are preserved in the output Behavior Tree and decomposed recursively:

```mermaid
graph TD
    Level1[Level 1: High-Level Command<br/>Example: Lift the sheathing<br/>to the red flag]
    
    Level1 --> Level2A[Level 2: NavigateToSheathing]
    Level1 --> Level2B[Level 2: GraspSheathing]
    Level1 --> Level2C[Level 2: NavigateToRedFlag]
    Level1 --> Level2D[Level 2: LiftToHeight]
    Level1 --> Level2E[Level 2: ReleaseSheathing]
    
    Level2A --> Level3A1[Level 3: ComputePathToPose]
    Level2A --> Level3A2[Level 3: FollowPath]
    
    Level2B --> Level3B1[Level 3: Outrigger_Rotate]
    Level2B --> Level3B2[Level 3: Boom_Slewing]
    Level2B --> Level3B3[Level 3: Pulley_Lift]
    
    Level2C --> Level3C1[Level 3: NavigateToPose]
    
    Level2D --> Level3D1[Level 3: Boom_Lift]
    Level2D --> Level3D2[Level 3: Pulley_Lift]
    
    Level2E --> Level3E1[Level 3: Hook_Release]
    
    Level3A1 -.->|Scene Change| Replan[LLM Replanning]
    Replan -.->|New Subtree| Level2A
    
    style Level1 fill:#ffe1f5
    style Level2A fill:#fff4e1
    style Level2B fill:#fff4e1
    style Level2C fill:#fff4e1
    style Level2D fill:#fff4e1
    style Level2E fill:#fff4e1
    style Level3A1 fill:#e1f5ff
    style Level3B1 fill:#e1f5ff
    style Replan fill:#ffe1f5
```

## Integration with ROBOCON OS

TSBT-VLA integrates seamlessly with ROBOCON OS components:

- **Large Language Model**: Edge-deployable reasoning engine via inference framework
- **Behavior Tree Library**: Behavior tree execution framework
- **Navigation Server**: Hosts and executes behavior trees
- **Sensor Integration**: Direct connection to ROBOCON sensor drivers
- **Hardware Control**: Output to ROBOCON motor controllers and actuators

**For implementation-specific details using YOLOv11, DeepSeek, ROS 2, and Nav 2, see**: [TSBT-VLA Implementation](../../tsbt-vla-system-implementation/overview.md)

## Safety and Auditing

Every inference step maintains complete logs for full traceability:

- **Input Prompt**: Full LLM prompt with all context
- **Scene State**: Snapshot of 3D world at inference time
- **Sensor Data**: Temporal logs used in reasoning
- **Generated Subtree**: Complete Behavior Tree XML output
- **Execution Log**: Node tick results and blackboard state
- **Hardware Actions**: All commands sent to actuators
- **Timestamp**: Precise timing for regulatory compliance

## Next Steps

- [Patent Documentation](patent.md) - Complete patent specification with diagrams
- [3D World to Text Track](3d-world-to-text.md) - Detailed 3D scene to text conversion
- [Sensor to Text Track](sensor-to-text.md) - Sensor data text conversion
- [User Input to Action Track](user-input-to-action.md) - Natural language to behavior tree
- [Large Language Model Processing](llm-processing.md) - LLM reasoning and subtree generation
- [Object Segmentation Module](object-segmentation-module.md) - Object segmentation implementation details
- [TSBT-VLA Implementation](../../tsbt-vla-system-implementation/overview.md) - Implementation using YOLOv11, DeepSeek, ROS 2, and Nav 2
- [Behavior Tree Node Reference](../../behavior-tree-node-reference.md) - Complete node reference

