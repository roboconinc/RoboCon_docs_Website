# Basic Motion Routines

Pre-built motion routines for common robot movements, inspired by Unitree's motion patterns.

## Overview

Basic Motion Routines provide high-level motion primitives that simplify common tasks:

- Walking patterns
- Turning movements
- Standing and balancing
- Gesture movements

## BasicMotionRoutine Class

```python
from robocon_sdk.motor_control import BasicMotionRoutine

motion = BasicMotionRoutine()
```

## Walking Routines

### Forward Walk

```python
# Walk forward for specified number of steps
motion.walk_forward(
    steps=10,
    step_height=0.05,  # meters
    step_length=0.2,   # meters
    speed=0.5          # m/s
)
```

### Backward Walk

```python
motion.walk_backward(
    steps=5,
    step_height=0.05,
    step_length=0.15,
    speed=0.3
)
```

### Sideways Walk

```python
# Walk to the left
motion.walk_left(
    steps=8,
    step_height=0.05,
    step_width=0.15
)

# Walk to the right
motion.walk_right(
    steps=8,
    step_height=0.05,
    step_width=0.15
)
```

## Turning Routines

### Turn Left/Right

```python
# Turn left by specified angle (degrees)
motion.turn_left(angle=90)

# Turn right by specified angle
motion.turn_right(angle=45)
```

### Turn in Place

```python
motion.turn_in_place(
    angle=180,  # degrees
    speed=0.3   # rad/s
)
```

## Standing Routines

### Stand Up

```python
# Stand from lying position
motion.stand_up()
```

### Sit Down

```python
# Sit down from standing
motion.sit_down()
```

### Balance

```python
# Maintain balance with active balancing
motion.balance(enabled=True)
```

## Gesture Routines

### Wave

```python
# Wave with specified arm
motion.wave(arm='right', times=3)
```

### Nod

```python
# Nod head
motion.nod_head(times=2)
```

## Custom Routines

### Define Custom Sequence

```python
# Create custom motion sequence
sequence = motion.create_sequence([
    {'action': 'walk_forward', 'steps': 5},
    {'action': 'turn_left', 'angle': 90},
    {'action': 'walk_forward', 'steps': 3},
    {'action': 'stand', 'duration': 2.0}
])

# Execute sequence
motion.execute_sequence(sequence)
```

## Integration with ROS 2

Motion routines can be used in ROS 2 nodes:

```python
import rclpy
from rclpy.node import Node
from robocon_sdk.motor_control import BasicMotionRoutine

class MotionNode(Node):
    def __init__(self):
        super().__init__('motion_node')
        self.motion = BasicMotionRoutine()
        
    def execute_walk_pattern(self):
        self.motion.walk_forward(steps=10)
        self.motion.turn_right(angle=90)
        self.motion.walk_forward(steps=5)
```

## Unitree Compatibility

These routines follow similar patterns to Unitree's motion routines:

- Similar function names and parameters
- Compatible motion patterns
- Easy porting of Unitree code

Example Unitree-style code that works with ROBOCON:

```python
# Unitree-style code
from unitree import A1

# Similar ROBOCON code
from robocon_sdk.motor_control import BasicMotionRoutine

motion = BasicMotionRoutine()
motion.walk_forward(steps=10)  # Same pattern
```

## Next Steps

- [Low-Level Control](low-level-control.md) - Direct motor control
- [Unitree Compatibility](unitree-compatibility.md) - Detailed porting guide
- [Examples](examples.md) - Complete examples

