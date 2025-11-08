# Track 3 Implementation: ROS 2 User Input

This document covers the implementation of Track 3 (User Input to Action) using **ROS 2** services and topics.

## Overview

The User Input to Action track processes natural language input from speech and chat interfaces via ROS 2 services and topics, sanitizes the input, and creates behavior tree nodes that are inserted into the Navigation Server.

## ROS 2 Integration

### Node Structure

**Package**: `robocon_user_input`

**Node Name**: `user_input_node`

### Speech Input (Voice-to-Text)

#### ROS 2 Topics

**Input**:
- `/audio/microphone` (audio_msgs/AudioData) - Raw audio stream

**Output**:
- `/audio/transcription` (std_msgs/String) - Transcribed text

#### Implementation Example

```python
import rclpy
from rclpy.node import Node
from audio_msgs.msg import AudioData
from std_msgs.msg import String
import whisper  # or other speech-to-text library

class VoiceToTextNode(Node):
    def __init__(self):
        super().__init__('voice_to_text_node')
        
        # Subscribe to microphone
        self.mic_sub = self.create_subscription(
            AudioData,
            '/audio/microphone',
            self.audio_callback,
            10
        )
        
        # Publish transcription
        self.transcription_pub = self.create_publisher(
            String,
            '/audio/transcription',
            10
        )
        
        # Load Whisper model
        self.model = whisper.load_model("base")
    
    def audio_callback(self, msg):
        # Convert audio data to numpy array
        audio_data = np.frombuffer(msg.data, dtype=np.int16)
        
        # Transcribe
        result = self.model.transcribe(audio_data)
        text = result["text"]
        
        # Publish
        transcription_msg = String()
        transcription_msg.data = text
        self.transcription_pub.publish(transcription_msg)
```

### Chat Input (ROS 2 Service)

#### Service Definition

**Service**: `/tsbt_vla/chat_input`

**Service Type**: `std_srvs/SetString` or custom service

```python
# std_srvs/SetString.srv
string data
---
string message
```

#### Implementation Example

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetString
from std_msgs.msg import String

class ChatInputNode(Node):
    def __init__(self):
        super().__init__('chat_input_node')
        
        # Service server
        self.chat_service = self.create_service(
            SetString,
            '/tsbt_vla/chat_input',
            self.chat_input_callback
        )
        
        # Publisher for sanitized input
        self.sanitized_pub = self.create_publisher(
            String,
            '/tsbt_vla/sanitized_input',
            10
        )
    
    def chat_input_callback(self, request, response):
        text = request.data
        
        # Sanitize input
        sanitized = self.sanitize_text(text)
        
        # Publish sanitized input
        msg = String()
        msg.data = sanitized
        self.sanitized_pub.publish(msg)
        
        response.message = "Command received"
        return response
    
    def sanitize_text(self, text):
        # Text cleaning, normalization, etc.
        return text.lower().strip()
```

### Text Sanitizer Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robocon_tsbt_vla_msgs.msg import BehaviorTreeNode

class TextSanitizerNode(Node):
    def __init__(self):
        super().__init__('text_sanitizer_node')
        
        # Subscribe to text inputs
        self.text_sub = self.create_subscription(
            String,
            '/tsbt_vla/sanitized_input',
            self.text_callback,
            10
        )
        
        # Subscribe to audio transcription
        self.audio_sub = self.create_subscription(
            String,
            '/audio/transcription',
            self.text_callback,
            10
        )
        
        # Publish behavior tree nodes
        self.bt_node_pub = self.create_publisher(
            BehaviorTreeNode,
            '/navigation_server/behavior_tree/nodes',
            10
        )
    
    def text_callback(self, msg):
        text = msg.data
        
        # Parse intent
        intent = self.parse_intent(text)
        
        # Create behavior tree node
        bt_node = self.create_behavior_tree_node(intent, text)
        
        # Publish
        self.bt_node_pub.publish(bt_node)
    
    def parse_intent(self, text):
        # Simple intent parsing (can be enhanced with NLU)
        text_lower = text.lower()
        
        if "lift" in text_lower and "sheathing" in text_lower:
            return {
                'action': 'LiftSheathingToRedFlag',
                'objects': ['sheathing', 'red_flag']
            }
        elif "navigate" in text_lower:
            return {
                'action': 'NavigateToPose',
                'parameters': self.extract_position(text_lower)
            }
        # ... more intent parsing
        
        return {'action': 'UnknownAction'}
    
    def create_behavior_tree_node(self, intent, original_text):
        node = BehaviorTreeNode()
        node.action = intent['action']
        node.source = "chat"  # or "speech"
        node.timestamp = self.get_clock().now().to_msg()
        node.original_text = original_text
        
        # Add metadata
        if 'objects' in intent:
            node.object_references = intent['objects']
        if 'parameters' in intent:
            node.parameters = intent['parameters']
        
        return node
```

## ROS 2 Services and Topics

### Services

| Service | Type | Request | Response | Description |
|---------|------|---------|----------|-------------|
| `/tsbt_vla/chat_input` | `std_srvs/SetString` | `string data` | `string message` | Send chat text command |

### Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/audio/microphone` | `audio_msgs/AudioData` | Raw audio input |
| `/audio/transcription` | `std_msgs/String` | Transcribed speech text |
| `/tsbt_vla/sanitized_input` | `std_msgs/String` | Sanitized text input |
| `/navigation_server/behavior_tree/nodes` | `robocon_tsbt_vla_msgs/BehaviorTreeNode` | Behavior tree nodes to insert |

## Navigation Server Integration

### Behavior Tree Node Insertion

```python
import rclpy
from rclpy.node import Node
from robocon_tsbt_vla_msgs.msg import BehaviorTreeNode
from nav2_msgs.action import NavigateToPose

class NavigationServerNode(Node):
    def __init__(self):
        super().__init__('navigation_server_node')
        
        # Subscribe to behavior tree nodes
        self.bt_node_sub = self.create_subscription(
            BehaviorTreeNode,
            '/navigation_server/behavior_tree/nodes',
            self.bt_node_callback,
            10
        )
        
        # Behavior tree management
        self.behavior_tree = None  # BehaviorTree.CPP tree
        
    def bt_node_callback(self, msg):
        # Insert node into behavior tree
        if msg.action == 'LiftSheathingToRedFlag':
            # Insert high-level action node
            self.insert_high_level_node(msg)
        elif msg.action in ['NavigateToPose', 'Outrigger_Rotate']:
            # Insert low-level action node
            self.insert_low_level_node(msg)
    
    def insert_high_level_node(self, node_msg):
        # High-level nodes trigger LLM processing
        # LLM will generate subtree for this action
        pass
    
    def insert_low_level_node(self, node_msg):
        # Low-level nodes can be executed directly
        pass
```

## Launch File

```xml
<launch>
    <!-- Voice-to-Text Node -->
    <node pkg="robocon_user_input" 
          exec="voice_to_text_node" 
          name="voice_to_text_node"/>
    
    <!-- Chat Input Node -->
    <node pkg="robocon_user_input" 
          exec="chat_input_node" 
          name="chat_input_node"/>
    
    <!-- Text Sanitizer Node -->
    <node pkg="robocon_user_input" 
          exec="text_sanitizer_node" 
          name="text_sanitizer_node"/>
</launch>
```

## Usage Examples

### Chat Input via ROS 2 Service

```bash
ros2 service call /tsbt_vla/chat_input \
    std_srvs/SetString \
    "{data: 'Lift the sheathing to the red flag'}"
```

### Monitor Transcriptions

```bash
ros2 topic echo /audio/transcription
```

### Monitor Behavior Tree Nodes

```bash
ros2 topic echo /navigation_server/behavior_tree/nodes
```

## Configuration

```yaml
user_input:
  speech:
    enabled: true
    voice_to_text_model: "whisper"
    language: "en-US"
    microphone_topic: "/audio/microphone"
    
  chat:
    enabled: true
    service_name: "/tsbt_vla/chat_input"
    interfaces:
      - "phone_app"
      - "tablet"
      - "laptop"
      - "desktop"
      
  sanitizer:
    enable_preprocessing: true
    add_metadata: true
    validate_syntax: true
    object_resolution: true
```

## Next Steps

- [Track 4: DeepSeek LLM](track4-deepseek-llm.md) - LLM processing
- [Behavior Tree Execution](behavior-tree-execution.md) - Navigation Server
- [User Input to Action Track](../../tsbt-vla-system/user-input-to-action.md) - Overall track overview

