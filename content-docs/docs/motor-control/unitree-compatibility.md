# Unitree Compatibility

Guide to porting applications between Unitree and ROBOCON platforms.

## Overview

The ROBOCON SDK motor control API is designed to be highly compatible with Unitree's API, enabling easy migration of applications between the two platforms.

## API Similarities

### Function Names

Most function names match Unitree's API:

```python
# Unitree style
from unitree import A1
robot = A1()
robot.set_motor_position(0, 1.57)

# ROBOCON style (same pattern)
from robocon_sdk.motor_control import MotorController
controller = MotorController()
controller.set_motor_position(0, 1.57)
```

### Command Structure

Command parameters are similar:

```python
# Both support similar parameters
# Unitree
robot.set_motor_position(joint_id=0, position=1.57, kp=50.0, kd=5.0)

# ROBOCON
controller.set_motor_position(joint_id=0, position=1.57, kp=50.0, kd=5.0)
```

## Porting Guide

### Step 1: Replace Imports

```python
# Before (Unitree)
from unitree import A1

# After (ROBOCON)
from robocon_sdk.motor_control import MotorController
```

### Step 2: Replace Initialization

```python
# Before
robot = A1()
robot.start()

# After
controller = MotorController()
```

### Step 3: Update Function Calls

Most function calls remain the same:

```python
# Works on both platforms
controller.set_motor_position(0, 1.57)
controller.set_motor_velocity(1, 0.5)
controller.set_motor_torque(2, 10.0)
controller.execute()
```

### Step 4: Handle Differences

Minor differences to account for:

```python
# Unitree: robot.read_motor_state(0)
# ROBOCON: controller.get_motor_state(0)

state = controller.get_motor_state(0)
```

## Motion Routines

### Basic Motion Compatibility

```python
# Unitree motion routines
from unitree import BasicMotion

# ROBOCON equivalent
from robocon_sdk.motor_control import BasicMotionRoutine

motion = BasicMotionRoutine()

# Same function calls
motion.walk_forward(steps=10)
motion.turn_left(angle=90)
```

## Complete Porting Example

### Original Unitree Code

```python
from unitree import A1

robot = A1()
robot.start()

# Move forward
robot.walk_forward(steps=10)

# Turn
robot.turn_left(angle=90)

# Move forward again
robot.walk_forward(steps=5)
```

### Ported ROBOCON Code

```python
from robocon_sdk.motor_control import MotorController, BasicMotionRoutine

controller = MotorController()
motion = BasicMotionRoutine()

# Move forward
motion.walk_forward(steps=10)

# Turn
motion.turn_left(angle=90)

# Move forward again
motion.walk_forward(steps=5)
```

## ROS 2 Integration

Both platforms integrate with ROS 2:

```python
# Works with both Unitree and ROBOCON
import rclpy
from geometry_msgs.msg import Twist

# ROS 2 integration is compatible
```

## Differences and Limitations

### Known Differences

1. **Joint ID Mapping**: Joint IDs may differ between platforms
2. **Parameter Ranges**: Some parameter ranges may vary
3. **Advanced Features**: Some Unitree-specific features may not be available

### Handling Differences

```python
# Use configuration to map joint IDs
config = {
    'joint_mapping': {
        'unitree_joint_0': 'robocon_joint_0',
        'unitree_joint_1': 'robocon_joint_1',
    }
}

controller = MotorController(config=config)
```

## Best Practices

1. **Test Incrementally**: Port and test one module at a time
2. **Check Parameters**: Verify parameter ranges and units
3. **Validate Behavior**: Compare behavior between platforms
4. **Update Documentation**: Update any platform-specific notes

## Next Steps

- [Basic Motion Routines](basic-motion-routines.md) - Motion patterns
- [Low-Level Control](low-level-control.md) - Advanced control
- [Examples](examples.md) - Complete porting examples

