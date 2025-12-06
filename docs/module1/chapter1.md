---
sidebar_position: 1
---

# Chapter 1: Introduction to ROS 2 and Core Concepts

Welcome to the exciting world of modern robotics! This chapter lays the essential groundwork for your entire journey, introducing you to the Robot Operating System (ROS 2). Far more than a traditional OS, ROS 2 is a powerful, flexible framework of tools and libraries designed to help you build complex robotic applications. We'll explore its fundamental concepts through the analogy of a biological nervous system, making it intuitive to understand how different parts of a robot can communicate and work together seamlessly.

## What is ROS 2? The "Robotic Nervous System" Analogy

At its heart, a robot is a distributed system. It has sensors (eyes, ears), actuators (muscles, motors), and a "brain" that processes information and makes decisions. How do all these disparate parts communicate reliably? This is where ROS 2 comes in.

Think of ROS 2 as the robot's nervous system:
-   **Nerves (Topics)**: These are the communication channels, or buses, that carry data throughout the robot. For example, a `/camera/image` topic might carry a stream of video frames, while a `/cmd_vel` topic might carry velocity commands.
-   **Nerve Endings (Nodes)**: These are the individual processes that perform a specific function. A camera driver node might capture images and publish them. A motor controller node might subscribe to velocity commands and actuate the wheels. A navigation node might process sensor data and decide where to go next.
-   **Signals (Messages)**: These are the actual data packets that travel along the topics. Each topic has a defined message type, like a `sensor_msgs/Image` or a `geometry_msgs/Twist`.

This publish-subscribe architecture allows for incredible flexibility. You can add a new sensor just by creating a new node that publishes to a topic, without having to modify any existing code. You can have multiple nodes listening to the same sensor data for different purposes. This modularity is the superpower of ROS 2.

## Understanding the ROS 2 Graph

The collection of all your nodes and the topics that connect them is known as the **ROS 2 Graph**. It's a visual representation of the data flow within your robotic system.

Let's break down the key components:
-   **Nodes**: A node is an executable that uses ROS 2 to communicate with other nodes. Each node in a ROS system is responsible for a single, module purpose (e.g., one node for controlling wheel motors, one node for controlling a laser range-finder).
-   **Topics**: A topic is a named bus over which nodes exchange messages. Topics are one-way communication paths: a node that sends data to a topic is a **publisher**, and a node that receives data from a topic is a **subscriber**.
-   **Messages**: When a publisher sends data to a topic, it sends a **message**. A message is a simple data structure, like an integer, a float, or a string. ROS 2 also has a rich library of standard message types for common robotic data, such as `sensor_msgs/LaserScan` or `nav_msgs/Odometry`.

## Creating a Python-based ROS 2 Workspace and Package

Let's get our hands dirty. A **workspace** is a directory containing your ROS 2 packages. A **package** is an organized collection of your nodes, launch files, and configuration.

1.  **Create a workspace:**
    ```bash
    mkdir -p ros2_ws/src
    cd ros2_ws
    ```

2.  **Create a Python package:**
    ```bash
    cd src
    ros2 pkg create --build-type ament_python my_first_package
    ```
    This command creates a new directory `my_first_package` with all the necessary files for a Python-based ROS 2 package.

## Writing a Publisher Node

Now, let's create a node that publishes a simple "hello world" message.

Inside your `my_first_package/my_first_package` directory, create a new file named `my_publisher.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    my_publisher = MyPublisher()
    rclpy.spin(my_publisher)
    my_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

You'll also need to update `setup.py` to make this script an executable entry point.

## Writing a Subscriber Node

Next, create a node that subscribes to the `chatter` topic.

In the same directory, create `my_subscriber.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MySubscriber(Node):
    def __init__(self):
        super().__init__('my_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    my_subscriber = MySubscriber()
    rclpy.spin(my_subscriber)
    my_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Again, update `setup.py` to add this script as an entry point.

## Building and Running

1.  **Build your package:**
    Navigate to the root of your workspace (`ros2_ws`) and run:
    ```bash
    colcon build
    ```

2.  **Source the setup file:**
    ```bash
    . install/setup.bash
    ```

3.  **Run your nodes:**
    In one terminal, run the publisher:
    ```bash
    ros2 run my_first_package my_publisher
    ```
    In another terminal, run the subscriber:
    ```bash
    ros2 run my_first_package my_subscriber
    ```
    You should see the publisher sending messages and the subscriber receiving them!

## Inspecting the System

You can use ROS 2's command-line tools to inspect your running system:
-   `ros2 topic list`: Lists all active topics.
-   `ros2 node list`: Lists all active nodes.
-   `ros2 topic echo /chatter`: Shows the messages being published on the `/chatter` topic.

This chapter has provided your first taste of ROS 2's power and simplicity. In the next chapter, we'll explore more advanced communication patterns that will allow you to build even more sophisticated robotic applications.