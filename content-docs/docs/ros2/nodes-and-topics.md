# ROS 2 Nodes and Topics

Detailed guide to developing ROS 2 nodes and working with topics.

## Creating Nodes

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
```

## Working with Topics

Publishers and subscribers for asynchronous communication.

## Next Steps

- [Publishers and Subscribers](publishers-and-subscribers.md) - Detailed topic usage
- [Services and Actions](services-and-actions.md) - Synchronous communication

