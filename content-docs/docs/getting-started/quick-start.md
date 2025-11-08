# Quick Start

Get up and running with the ROBOCON SDK in 5 minutes.

## Prerequisites

- ROBOCON SDK installed (see [Installation Guide](installation.md))
- Access to a ROBOCON robot or simulator

## Your First Program

Let's create a simple program that moves a robot forward.

### Step 1: Create a Workspace

```bash
mkdir -p ~/robocon_quickstart/src
cd ~/robocon_quickstart/src
```

### Step 2: Create Your Package

```bash
ros2 pkg create --build-type ament_python robocon_hello_world
cd robocon_hello_world
```

### Step 3: Write Your First Node

Create `robocon_hello_world/robocon_hello_world/move_forward.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robocon_sdk.motor_control import MotorController
from geometry_msgs.msg import Twist

class MoveForwardNode(Node):
    def __init__(self):
        super().__init__('move_forward_node')
        
        # Initialize motor controller
        self.motor_controller = MotorController()
        
        # Create publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create timer to publish commands
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Move Forward Node started')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5  # Move forward at 0.5 m/s
        msg.angular.z = 0.0  # No rotation
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MoveForwardNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 4: Make it Executable

```bash
chmod +x robocon_hello_world/robocon_hello_world/move_forward.py
```

### Step 5: Update Package Files

Edit `setup.py`:

```python
from setuptools import setup

setup(
    name='robocon_hello_world',
    version='0.0.1',
    packages=['robocon_hello_world'],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'move_forward = robocon_hello_world.move_forward:main',
        ],
    },
)
```

### Step 6: Build and Run

```bash
cd ~/robocon_quickstart
colcon build
source install/setup.bash

# Run your node
ros2 run robocon_hello_world move_forward
```

## Using Motor Control API

For direct motor control (similar to Unitree), use the Motor Control API:

```python
from robocon_sdk.motor_control import MotorController

# Initialize controller
controller = MotorController()

# Set motor position (radians)
controller.set_motor_position(joint_id=0, position=1.57)  # 90 degrees

# Set motor velocity (rad/s)
controller.set_motor_velocity(joint_id=0, velocity=0.5)

# Set motor torque (Nm)
controller.set_motor_torque(joint_id=0, torque=10.0)

# Execute motion
controller.execute()
```

## Basic Motion Routine

Create a simple walking pattern:

```python
import rclpy
from rclpy.node import Node
from robocon_sdk.motor_control import BasicMotionRoutine

class WalkRoutine(Node):
    def __init__(self):
        super().__init__('walk_routine')
        self.motion = BasicMotionRoutine()
        
    def run(self):
        # Forward walk
        self.motion.walk_forward(steps=10, step_height=0.05)
        
        # Turn left
        self.motion.turn_left(angle=90)
        
        # Forward walk again
        self.motion.walk_forward(steps=10, step_height=0.05)

def main():
    rclpy.init()
    node = WalkRoutine()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Next Steps

- [Your First Application](your-first-application.md) - Build a complete application
- [Motor Control Basics](../motor-control/basic-motion-routines.md) - Learn motion routines
- [ROS 2 Integration](../ros2/nodes-and-topics.md) - Deep dive into ROS 2

## Examples Repository

For more complete examples, check out:

```bash
git clone https://github.com/roboconinc/robocon-sdk-examples.git
cd robocon-sdk-examples
```

