# Track 4 Implementation: DeepSeek LLM

This document covers the implementation of Track 4 (Large Language Model Processing) using **DeepSeek-R1-Distill-Llama-8B-GGUF** via a GPT4All fork.

## Overview

The LLM processing track uses DeepSeek-R1-Distill-Llama-8B-GGUF as the reasoning engine that processes Text World, Sensor Text, and High-Level Actions to generate Behavior Tree XML subtrees.

## DeepSeek Model

### Model Details

**Model**: DeepSeek-R1-Distill-Llama-8B-GGUF

**Architecture**:
- **Base**: Llama3.1-8B-Instruct (8 billion parameters)
- **Format**: GGUF (GPT-Generated Unified Format) for efficient quantization
- **Context Window**: 128k tokens (131,072 tokens)
- **License**: llama3.1 license (derived from Llama3.1-8B-Base)

**Performance**:
- MATH-500: 89.1 pass@1
- CodeForces rating: 1205

**Source**: [Hugging Face - DeepSeek-R1-Distill-Llama-8B-GGUF](https://huggingface.co/GPT4All-Community/DeepSeek-R1-Distill-Llama-8B-GGUF)

### Deployment Framework

**Framework**: Forked version of [GPT4All](https://github.com/nomic-ai/gpt4all)

**Hardware**: AI Computer (ASRock B450M with Radeon RX 7600 XT)

**Acceleration**: GPU-accelerated inference via ROCm (AMD GPU support)

**Processing**: Edge-based (no cloud dependency)

## ROS 2 Node Implementation

### Node Structure

**Package**: `robocon_tsbt_vla`

**Node Name**: `llm_processing_node`

### Python Implementation

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robocon_tsbt_vla_msgs.srv import ProcessCommand
from gpt4all import GPT4All  # Forked version with ROCm support

class LLMProcessingNode(Node):
    def __init__(self):
        super().__init__('llm_processing_node')
        
        # Load DeepSeek model
        model_path = self.declare_parameter('model_path', '/path/to/deepseek-model.gguf').value
        self.model = GPT4All(
            model_name=model_path,
            device='gpu',  # Use GPU with ROCm
            n_threads=4
        )
        
        # Subscribe to text inputs
        self.text_world_sub = self.create_subscription(
            String,
            '/tsbt_vla/text_world',
            self.text_world_callback,
            10
        )
        
        self.sensor_text_sub = self.create_subscription(
            String,
            '/tsbt_vla/sensor_text',
            self.sensor_text_callback,
            10
        )
        
        self.high_level_action_sub = self.create_subscription(
            String,
            '/tsbt_vla/high_level_action',
            self.high_level_action_callback,
            10
        )
        
        # Publisher for generated behavior trees
        self.bt_xml_pub = self.create_publisher(
            String,
            '/tsbt_vla/behavior_tree_xml',
            10
        )
        
        # Service for command processing
        self.process_service = self.create_service(
            ProcessCommand,
            '/tsbt_vla/process_command',
            self.process_command_callback
        )
        
        # Current context
        self.current_text_world = ""
        self.current_sensor_text = ""
        self.current_high_level_action = ""
        
        self.get_logger().info('LLM Processing Node Started')
    
    def text_world_callback(self, msg):
        self.current_text_world = msg.data
    
    def sensor_text_callback(self, msg):
        self.current_sensor_text = msg.data
    
    def high_level_action_callback(self, msg):
        self.current_high_level_action = msg.data
    
    def process_command_callback(self, request, response):
        command = request.command
        
        # Construct prompt
        prompt = self.construct_prompt(
            high_level_action=command,
            text_world=self.current_text_world,
            sensor_text=self.current_sensor_text
        )
        
        # Generate behavior tree XML
        bt_xml = self.generate_behavior_tree(prompt)
        
        response.behavior_tree_xml = bt_xml
        response.success = True
        
        # Publish generated XML
        xml_msg = String()
        xml_msg.data = bt_xml
        self.bt_xml_pub.publish(xml_msg)
        
        return response
    
    def construct_prompt(self, high_level_action, text_world, sensor_text):
        prompt = f"""High-Level Action: {high_level_action}

Text World:
{text_world}

Sensor Text:
{sensor_text}

Available Low-Level Actions:
- Outrigger_Rotate(rotation)
- Outrigger_Extend(extension)
- Boom_Slewing(rotation)
- Boom_Lift(extension)
- Pulley_Lift(distance)
- Visual_Find(ObjectClass, objectId)
- Hook_Release()
- NavigateToPose(goal)
- ComputePathToPose(goal)
- FollowPath(path)

Generate a Behavior Tree XML subtree that accomplishes the high-level action.
"""
        return prompt
    
    def generate_behavior_tree(self, prompt):
        # Generate response using DeepSeek
        with self.model.chat_session():
            response = self.model.generate(
                prompt=prompt,
                max_tokens=2048,
                temperature=0.6,  # Recommended for behavior tree generation
                top_p=0.9,
                top_k=40
            )
        
        # Extract XML from response (may need post-processing)
        bt_xml = self.extract_xml(response)
        
        return bt_xml
    
    def extract_xml(self, response):
        # Extract XML subtree from LLM response
        # LLM may include markdown or explanations, extract just the XML
        import re
        
        # Look for XML content between <Sequence> or other BT tags
        xml_match = re.search(r'<Sequence[^>]*>.*?</Sequence>', response, re.DOTALL)
        if xml_match:
            return xml_match.group(0)
        
        # Fallback: return entire response if no XML found
        return response
```

### C++ Implementation (using GPT4All C++ bindings)

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <robocon_tsbt_vla_msgs/srv/process_command.hpp>
#include <gpt4all/gpt4all.h>  // C++ bindings

class LLMProcessingNode : public rclcpp::Node {
public:
    LLMProcessingNode() : Node("llm_processing_node") {
        // Load model
        std::string model_path = this->declare_parameter("model_path", "/path/to/deepseek-model.gguf").as_string();
        model_ = std::make_unique<GPT4All>(model_path);
        
        // Subscriptions
        text_world_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/tsbt_vla/text_world", 10,
            std::bind(&LLMProcessingNode::text_world_callback, this, std::placeholders::_1)
        );
        
        sensor_text_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/tsbt_vla/sensor_text", 10,
            std::bind(&LLMProcessingNode::sensor_text_callback, this, std::placeholders::_1)
        );
        
        // Publisher
        bt_xml_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/tsbt_vla/behavior_tree_xml", 10
        );
        
        // Service
        process_service_ = this->create_service<robocon_tsbt_vla_msgs::srv::ProcessCommand>(
            "/tsbt_vla/process_command",
            std::bind(&LLMProcessingNode::process_command_callback, this, std::placeholders::_1, std::placeholders::_2)
        );
    }

private:
    void text_world_callback(const std_msgs::msg::String::SharedPtr msg) {
        current_text_world_ = msg->data;
    }
    
    void sensor_text_callback(const std_msgs::msg::String::SharedPtr msg) {
        current_sensor_text_ = msg->data;
    }
    
    void process_command_callback(
        const std::shared_ptr<robocon_tsbt_vla_msgs::srv::ProcessCommand::Request> request,
        std::shared_ptr<robocon_tsbt_vla_msgs::srv::ProcessCommand::Response> response
    ) {
        // Construct prompt
        std::string prompt = construct_prompt(
            request->command,
            current_text_world_,
            current_sensor_text_
        );
        
        // Generate behavior tree
        std::string bt_xml = generate_behavior_tree(prompt);
        
        response->behavior_tree_xml = bt_xml;
        response->success = true;
        
        // Publish
        auto xml_msg = std_msgs::msg::String();
        xml_msg.data = bt_xml;
        bt_xml_pub_->publish(xml_msg);
    }
    
    std::string construct_prompt(const std::string& action, const std::string& world, const std::string& sensor) {
        // Construct prompt string
        return "High-Level Action: " + action + "\n\nText World:\n" + world + "\n\nSensor Text:\n" + sensor;
    }
    
    std::string generate_behavior_tree(const std::string& prompt) {
        // Use GPT4All C++ API
        return model_->generate(prompt, 2048, 0.6);
    }
    
    std::unique_ptr<GPT4All> model_;
    std::string current_text_world_;
    std::string current_sensor_text_;
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr text_world_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sensor_text_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bt_xml_pub_;
    rclcpp::Service<robocon_tsbt_vla_msgs::srv::ProcessCommand>::SharedPtr process_service_;
};
```

## ROS 2 Topics and Services

### Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/tsbt_vla/text_world` | `std_msgs/String` | Text World input |
| `/tsbt_vla/sensor_text` | `std_msgs/String` | Sensor Text input |
| `/tsbt_vla/high_level_action` | `std_msgs/String` | High-level action input |
| `/tsbt_vla/behavior_tree_xml` | `std_msgs/String` | Generated Behavior Tree XML |

### Services

| Service | Type | Request | Response | Description |
|---------|------|---------|----------|-------------|
| `/tsbt_vla/process_command` | `robocon_tsbt_vla_msgs/ProcessCommand` | `string command` | `string behavior_tree_xml, bool success` | Process command and generate BT |

## Configuration

```yaml
llm:
  model_path: "/path/to/DeepSeek-R1-Distill-Llama-8B-GGUF.gguf"
  device: "gpu"  # or "cpu"
  n_threads: 4
  
  inference:
    max_tokens: 2048
    temperature: 0.6  # Recommended for behavior tree generation
    top_p: 0.9
    top_k: 40
    
  rocm:
    enabled: true  # For AMD GPU acceleration
    device_id: 0
```

## Usage Recommendations

### Temperature Settings

- **0.5-0.7 (0.6 recommended)**: Prevents endless repetitions, ensures coherent behavior tree generation, balances creativity with determinism

### System Prompts

- **Avoid adding system prompts**: All instructions should be in the user prompt
- Text World and Sensor Text provide context
- Direct instructions work best

### Mathematical/Reasoning Tasks

- Include directive in prompt: "Please reason step by step, and put your final answer within \\boxed{}"
- Helps with complex planning tasks

### Evaluation

- Conduct multiple tests and average results
- LLM outputs can vary
- Average over multiple runs for benchmarking

## Example Behavior Tree Output

**Input Prompt**:
```
High-Level Action: Lift the sheathing to the red flag

Text World:
- Red flag is positioned at (2.5, 0.8, 2.0), color: red.
- Sheathing is at (1.2, 0.5, 0.0), below the red flag.

Sensor Text:
[ArmJointTorque] = [55.4Nm, 54.8Nm, 56.0Nm]
[SystemStatus] = normal
```

**Generated Behavior Tree XML**:
```xml
<Sequence name="LiftSheathingToRedFlag">
    <Outrigger_Rotate rotation="170"/>
    <Outrigger_Extend extension="0.350"/>
    <Visual_Find ObjectClass="sheathing" objectId="{targetSheathing}"/>
    <Boom_Slewing rotation="45"/>
    <Boom_Lift extension="3.0"/>
    <Pulley_Lift Distance="2.5"/>
    <Boom_Slewing rotation="90"/>
    <Pulley_Lift Distance="-1.5"/>
    <Hook_Release/>
</Sequence>
```

## Launch File

```xml
<launch>
    <node pkg="robocon_tsbt_vla" 
          exec="llm_processing_node" 
          name="llm_processing_node">
        <param name="model_path" value="/path/to/DeepSeek-R1-Distill-Llama-8B-GGUF.gguf"/>
        <param name="device" value="gpu"/>
        <param name="temperature" value="0.6"/>
    </node>
</launch>
```

## Next Steps

- [Behavior Tree Execution](behavior-tree-execution.md) - Nav 2 and BehaviorTree.CPP integration
- [Large Language Model Processing](../../tsbt-vla-system/llm-processing.md) - Overall track overview

