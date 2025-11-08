# Motor Control Examples

Complete examples demonstrating motor control capabilities.

## Example 1: Simple Position Control

```python
from robocon_sdk.motor_control import MotorController
import time

controller = MotorController()

# Move joint to 90 degrees
controller.set_motor_position(0, 1.57)
controller.execute()
time.sleep(2.0)

# Return to zero
controller.set_motor_position(0, 0.0)
controller.execute()
```

## Example 2: Trajectory Following

```python
import numpy as np

positions = np.linspace(0, 3.14, 100)

for pos in positions:
    controller.set_motor_position(0, pos)
    controller.execute()
    time.sleep(0.01)
```

## Example 3: Multi-Joint Control

```python
positions = {0: 0.0, 1: 1.57, 2: 3.14}
controller.set_motor_positions(positions)
controller.execute()
```

