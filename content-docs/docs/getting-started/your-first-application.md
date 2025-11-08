# Your First Application

Build a complete ROBOCON application from scratch.

## Overview

In this tutorial, you'll build a simple autonomous navigation application that:

1. Uses motor control to move the robot
2. Integrates with ROS 2 for communication
3. Uses Nav 2 for path planning
4. Implements a simple behavior using the SDK

## Project Structure

```
my_robocon_app/
├── src/
│   └── my_robocon_app/
│       ├── __init__.py
│       ├── navigation_node.py
│       ├── motor_control_node.py
│       └── main.py
├── setup.py
├── setup.cfg
└── package.xml
```

## Step 1: Create the Package

```bash
mkdir -p ~/my_robocon_ws/src
cd ~/my_robocon_ws/src
ros2 pkg create --build-type ament_python my_robocon_app
cd my_robocon_app
```

## Step 2: Package Configuration

### `package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robocon_app</name>
  <version>0.0.1</version>
  <description>My first ROBOCON application</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclpy</depend>
  <depend>robocon_sdk</depend>
  <depend>nav2_msgs</depend>
  <depend>geometry_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### `setup.py`

```python
from setuptools import setup, find_packages

package_name = 'my_robocon_app'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='My first ROBOCON application',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'navigation_node = my_robocon_app.navigation_node:main',
            'motor_control_node = my_robocon_app.motor_control_node:main',
            'main = my_robocon_app.main:main',
        ],
    },
)
```

## Step 3: Motor Control Node

Create `my_robocon_app/motor_control_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robocon_sdk.motor_control import MotorController

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        
        self.motor_controller = MotorController()
        
        # Subscribe to velocity commands
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            10
        )
        
        self.get_logger().info('Motor Control Node started')

    def velocity_callback(self, msg):
        """Convert Twist message to motor commands"""
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Convert to motor velocities (simplified)
        left_velocity = linear_x - angular_z
        right_velocity = linear_x + angular_z
        
        # Set motor velocities
        self.motor_controller.set_motor_velocity(0, left_velocity)
        self.motor_controller.set_motor_velocity(1, right_velocity)
        self.motor_controller.execute()
        
        self.get_logger().debug(
            f'Velocity: linear={linear_x}, angular={angular_z}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    
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

## Step 4: Navigation Node

Create `my_robocon_app/navigation_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        # Create Nav2 action client
        self.nav_action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        self.get_logger().info('Navigation Node started')
        
    def navigate_to(self, x, y, theta=0.0):
        """Navigate to a goal pose"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.z = float(theta)
        
        self.get_logger().info(f'Navigating to ({x}, {y})')
        
        self.nav_action_client.wait_for_server()
        send_goal_future = self.nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        
    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Distance remaining: {feedback.distance_remaining:.2f}m'
        )

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    
    # Example: Navigate to a position
    node.navigate_to(2.0, 3.0, 0.0)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 5: Main Application

Create `my_robocon_app/main.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_robocon_app.motor_control_node import MotorControlNode
from my_robocon_app.navigation_node import NavigationNode

class RoboconApp(Node):
    def __init__(self):
        super().__init__('robocon_app')
        
        # Initialize nodes
        self.motor_node = MotorControlNode()
        self.nav_node = NavigationNode()
        
        self.get_logger().info('ROBOCON Application started')
        
    def run(self):
        """Main application loop"""
        # Start motor control
        # Motor control runs in its own thread via rclpy.spin
        
        # Execute navigation task
        self.nav_node.navigate_to(5.0, 5.0, 1.57)
        
        self.get_logger().info('Application completed')

def main(args=None):
    rclpy.init(args=args)
    
    app = RoboconApp()
    
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        app.motor_node.destroy_node()
        app.nav_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 6: Build and Run

```bash
cd ~/my_robocon_ws
colcon build --packages-select my_robocon_app
source install/setup.bash

# In separate terminals:
# Terminal 1: Start motor control
ros2 run my_robocon_app motor_control_node

# Terminal 2: Run navigation
ros2 run my_robocon_app navigation_node

# Terminal 3: Run main application
ros2 run my_robocon_app main
```

## Next Steps

- [API Reference](../api-reference/motor-control.md) - Full API documentation
- [ROS 2 Integration](../ros2/nodes-and-topics.md) - Advanced ROS 2 patterns
- [Nav 2 Integration](../nav2/path-planning.md) - Advanced navigation

