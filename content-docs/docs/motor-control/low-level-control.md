# Low-Level Control

Direct control over individual motors and drivers for advanced applications.

## Overview

Low-level control provides fine-grained control over:

- Individual joint control
- Driver configuration
- Real-time motor commands
- Direct hardware access

## MotorController Class

```python
from robocon_sdk.motor_control import MotorController

controller = MotorController()
```

## Individual Joint Control

### Position Control

```python
# Set position with full control parameters
controller.set_motor_position(
    joint_id=0,
    position=1.57,      # rad
    kp=50.0,            # Position gain
    kd=5.0,             # Velocity gain
    ki=0.0,             # Integral gain
    max_velocity=5.0,   # rad/s limit
    max_torque=20.0     # Nm limit
)
```

### Velocity Control

```python
controller.set_motor_velocity(
    joint_id=1,
    velocity=0.5,       # rad/s
    kd=5.0,            # Damping
    max_torque=15.0    # Nm limit
)
```

### Torque Control

```python
controller.set_motor_torque(
    joint_id=2,
    torque=10.0,       # Nm
    max_velocity=10.0   # rad/s safety limit
)
```

## Driver Configuration

### Set Driver Parameters

```python
# Configure motor driver
controller.configure_driver(
    joint_id=0,
    config={
        'current_limit': 15.0,      # Amps
        'velocity_limit': 10.0,      # rad/s
        'temperature_limit': 80.0,  # Celsius
        'enable_brake': True
    }
)
```

### Enable/Disable Motors

```python
# Enable motor
controller.enable_motor(joint_id=0)

# Disable motor
controller.disable_motor(joint_id=0)

# Enable all motors
controller.enable_all_motors()
```

## Real-Time Control

### High-Frequency Updates

```python
import time

# Real-time control loop (100 Hz)
rate = 100  # Hz
dt = 1.0 / rate

while running:
    # Read sensor data
    state = controller.get_motor_state(0)
    
    # Calculate control output
    error = target_position - state.position
    torque = kp * error + kd * (error - prev_error) / dt
    
    # Apply control
    controller.set_motor_torque(0, torque)
    controller.execute()
    
    prev_error = error
    time.sleep(dt)
```

## Synchronized Control

### Sync Multiple Joints

```python
# Control multiple joints synchronously
positions = {
    0: 0.0,
    1: 1.57,
    2: 0.0
}

# Set all at once (atomic operation)
controller.set_motor_positions_sync(positions)

# Execute synchronously
controller.execute_sync()
```

## Safety and Limits

### Dynamic Limits

```python
# Adjust limits dynamically
controller.set_position_limits(0, -2.0, 2.0)
controller.set_velocity_limits(0, 5.0)
controller.set_torque_limits(0, 15.0)
```

### Safety Checks

```python
# Enable safety checks
controller.enable_safety_checks(True)

# Custom safety callback
def safety_callback(joint_id, violation):
    print(f"Safety violation on joint {joint_id}: {violation}")
    controller.emergency_stop()

controller.set_safety_callback(safety_callback)
```

## Direct Hardware Access

### Raw Command Interface

```python
# Direct hardware commands (advanced)
raw_command = {
    'joint_id': 0,
    'mode': 'position',
    'position': 1.57,
    'velocity': 0.0,
    'kp': 50.0,
    'kd': 5.0,
    'torque': 0.0
}

controller.send_raw_command(raw_command)
```

## Next Steps

- [Unitree Compatibility](unitree-compatibility.md) - Porting Unitree code
- [Examples](examples.md) - Complete examples
- [API Reference](../api-reference/motor-control.md) - Full API docs

