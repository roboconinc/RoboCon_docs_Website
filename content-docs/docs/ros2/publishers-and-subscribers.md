# Publishers and Subscribers

Guide to ROS 2 pub/sub communication patterns.

## Creating Publishers

```python
from geometry_msgs.msg import Twist

self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
```

## Creating Subscribers

```python
self.subscription = self.create_subscription(
    Twist,
    '/cmd_vel',
    self.callback,
    10
)
```

