## Creating an action
- Set up a workspace and create a package named action_tutorials_interfaces:
```
mkdir -p ros2_ws/src #you can reuse existing workspace with this naming convention
cd ros2_ws/src
ros2 pkg create action_tutorials_interfaces
```
![image](https://user-images.githubusercontent.com/95737530/196892969-b634b2cf-7094-4ff1-a727-0c9be4ef26e2.png)
## Tasks
### 1 Denfining an action

Say we want to define a new action “Fibonacci” for computing the Fibonacci sequence.

Create an action directory in our ROS 2 package action_tutorials_interfaces:

```
cd action_tutorials_interfaces
mkdir action
```
Within the action directory, create a file called Fibonacci.action with the following contents:
```
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```
The goal request is the order of the Fibonacci sequence we want to compute,
the result is the final sequence, and the feedback is the partial_sequence computed so far.
### 2 Building an action
This is accomplished by adding the following lines to our CMakeLists.txt before the 
ament_package() line, in the action_tutorials_interfaces:
![image](https://user-images.githubusercontent.com/95737530/196898650-06159041-5628-4a89-8aea-b47d23d1a4be.png)
```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)
```
We should also add the required dependencies to our package.xml:

```
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<depend>action_msgs</depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```
Note, we need to depend on action_msgs since action definitions include additional metadata (e.g. goal IDs).

We should now be able to build the package containing the Fibonacci action definition:
```
# Change to the root of the workspace
cd ~/ros2_ws
# Build
colcon build
```
![image](https://user-images.githubusercontent.com/95737530/196901556-6f221171-6175-4047-98d3-2edd017e124a.png)
We’re done!

By convention, action types will be prefixed by their package name and the word action. 
So when we want to refer to our new action, it will have the full name action_tutorials_interfaces/action/Fibonacci.
```
# Source our workspace
# On Windows: call install/setup.bat
. install/setup.bash
# Check that our action definition exists
ros2 interface show action_tutorials_interfaces/action/Fibonacci
```
## Writing an action server and client (Python)

### Step  1 Writing an action server
Open a new file in  home directory, let’s call it fibonacci_action_server.py, and add the following code:
```
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = Fibonacci.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()
 ```
Line 8 defines a class FibonacciActionServer that is a subclass of Node. The class is initialized 
by calling the Node constructor, naming our node fibonacci_action_server
```
        super().__init__('fibonacci_action_server')
```
In the constructor we also instantiate a new action server:
```
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
```
An action server requires four arguments:

    A ROS 2 node to add the action client to: self.

    The type of the action: Fibonacci (imported in line 5).

    The action name: 'fibonacci'.

    A callback function for executing accepted goals: self.execute_callback. This callback must return a result message for the action type.

An action server requires four arguments:
- A ROS 2 node to add the action client to: self.
- The type of the action: Fibonacci (imported in line 5).
- The action name: 'fibonacci'.
- A callback function for executing accepted goals: self.execute_callback. This callback must return a result message for the action type.
We also define an execute_callback method in our class:

```
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = Fibonacci.Result()
        return result
```
