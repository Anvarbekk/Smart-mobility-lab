
- 1 Create a package
```
ros2 pkg create --build-type ament_python py_pubsub
```
- 2 Write the publisher node
```
wget https://raw.githubusercontent.com/ros2/examples/rolling/rclpy/topics/min
imal_publisher/examples_rclpy_minimal_publisher/publisher_member_functio
n.py
```
### This a python file 
```
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
### 2.1 Testing the code 
The first lines of code after the comments import rclpy so its Node class can be used.
```
import rclpy
from rclpy.node import Node
```
The next statement imports the built-in string message 
type that the node uses to structure the data that it passes on the topic.
```
from std_msgs.msg import String
```
Next, the MinimalPublisher class is created, which inherits from (or is a subclass of) Node.
```
class MinimalPublisher(Node):
```
Next, a timer is created with a callback to execute every 0.5 seconds. 
self.i is a counter used in the callback.
```
def __init__(self):
    super().__init__('minimal_publisher')
    self.publisher_ = self.create_publisher(String, 'topic', 10)
    timer_period = 0.5  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.i = 0
```
timer_callback creates a message with the counter value appended, 
and publishes it to the console with get_logger().info.
```
def timer_callback(self):
    msg = String()
    msg.data = 'Hello World: %d' % self.i
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg.data)
    self.i += 1
```
Lastly, the main function is defined.
```
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown(
```
First the rclpy library is initialized, then the node is created, and then 
it “spins” the node so its callbacks are called.

  ### 2.2 Add dependencies
  
  As mentioned in the previous tutorial, make sure to fill in the 
  <description>, <maintainer> and <license> tags:
```
  <description>Examples of minimal publisher/subscriber using rclpy</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```
  After the lines above, add the following dependencies corresponding 
  to node’s import statements:
```
  <exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```
  This declares the package needs rclpy and std_msgs when its code is executed.
  
Make sure to save the file.
  ### 2.3 Add an entry point
  
  Open the setup.py file. Again, match the maintainer, maintainer_email, 
  description and license fields to your package.xml:
```
  maintainer='YourName',
maintainer_email='you@email.com',
description='Examples of minimal publisher/subscriber using rclpy',
license='Apache License 2.0',
```
```
  entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
},
```
  Don’t forget to save.
  
  ### 2.4 Check setup.cfg

The contents of the setup.cfg file should be correctly populated automatically, like so:
```
[develop]
script-dir=$base/lib/py_pubsub
[install]
install-scripts=$base/lib/py_pubsub
```
This is simply telling setuptools to put your executables in lib, 
  because ros2 run will look for them there.
  
## 3 Write the subscriber node
  
 Return to ros2_ws/src/py_pubsub/py_pubsub to create the next node. 
  Enter the following code in terminal
```
  wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py

```
  ![image](https://user-images.githubusercontent.com/95737530/196692059-1fce0000-1fcb-44de-8f98-0219b5876d36.png)
Now the directory should have these files:
```
  __init__.py  publisher_member_function.py  subscriber_member_function.py
```
  ### 3.1 Examine the code
    
Open the subscriber_member_function.py with  text editor.
```
  import rclpy
  from rclpy.node import Node

from std_msgs.msg import String
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


    def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
The subscriber node’s code is nearly identical to the publisher’s. 
  The constructor creates a subscriber with the same arguments as the 
  publisher. Recall from the topics tutorial that the topic name and message 
  type used by the publisher and subscriber must match to allow them to communicate.
    
```
  self.subscription = self.create_subscription(
    String,
    'topic',
    self.listener_callback,
    10)
```
  he callback definition simply prints an info message to 
  the console, along with the data it received. 
  Recall that the publisher defines msg.data = 'Hello World: %d' % self.i
    
```
  def listener_callback(self, msg):
    self.get_logger().info('I heard: "%s"' % msg.data)
```
  The main definition is almost exactly the same, replacing 
  the creation and spinning of 
  the publisher with the subscriber.
    
```
    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)
```
Since this node has the same dependencies as the publisher, there’s 
  nothing new to add to package.xml. The setup.cfg file can also remain untouched.
  
### 3.2 Add an entry point

Reopen setup.py and add the entry point for the subscriber node below the 
  publisher’s entry point. The entry_points field should now look like this:
```
 entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
        ],
},
    
```
    
 Make sure to save the file, and then pub/sub system should be ready for use.
    
  ### 4 Build and run
    
It’s good practice to run rosdep in the root of your workspace (ros2_ws) 
    to check for missing dependencies before building
  
```
    rosdep install -i --from-path src --rosdistro foxy -y
```
 Still in the root of  workspace, ros2_ws, build  new package
```
    colcon build --packages-select py_pubsub
```
    
![image](https://user-images.githubusercontent.com/95737530/196710262-1d8e7cbb-c3d8-4f32-93a3-f05eac6e0890.png)

Open a new terminal, navigate to ros2_ws, and source the setup files
    
```
    . install/setup.bash
```
Now run the talker node:
```
    ros2 run py_pubsub talker
```
Open another terminal, source the setup files 
    from inside ros2_ws again, and then start the listener node:
```
    ros2 run py_pubsub listener
```
with combination Ctrl+C in each terminal to stop the nodes from looping.

    
