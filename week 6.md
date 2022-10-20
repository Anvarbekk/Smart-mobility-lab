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
![image](https://user-images.githubusercontent.com/95737530/196901556-6f221171-6175-4047-98d3-2edd017e124a.png)



