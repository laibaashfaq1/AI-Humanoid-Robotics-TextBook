---
sidebar_position: 2
---

# Chapter 2: Services, Actions, and Launch Files

In the previous chapter, you learned about the fundamental publish-subscribe communication pattern in ROS 2. While incredibly powerful for streaming data, sometimes you need a more direct, request-response interaction, or a way to manage long-running, goal-oriented tasks. This chapter introduces ROS 2 **Services** and **Actions**, alongside **Launch Files** for orchestrating complex robotic systems.

## Services: Synchronous Request-Response

Imagine you need a robot to perform a single, specific task, like opening a gripper, or calculating the inverse kinematics for a target pose. You send a request, and you expect a response indicating success or failure, possibly with some data. This is a perfect use case for ROS 2 Services.

**Services** are built on a client-server model:
-   A **service server** waits for requests from clients. When it receives one, it performs the requested computation and sends back a response.
-   A **service client** sends a request to a service server and waits for the response.

### Implementing a ROS 2 Service Server and Client (Python)

Let's create a simple service that adds two integers.

First, define the service interface. ROS 2 services require a `.srv` file to define the request and response message types. Create a file `AddTwoInts.srv` inside a `srv` directory within your ROS 2 package (e.g., `my_first_package/srv/AddTwoInts.srv`):

```
int64 a
int64 b
---
int64 sum
```

Now, update your `CMakeLists.txt` and `package.xml` to include this new service definition. For Python, this typically involves adding dependencies on `rosidl_default_generators` and `std_msgs` (if using standard types) and ensuring `rosidl_default_runtime` is exported.

**Service Server (`add_two_ints_server.py`)**:

```python
import rclpy
from rclpy.node import Node
from my_first_package.srv import AddTwoInts  # Import your custom service type

class AddTwoIntsService(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('AddTwoInts Service Server ready.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}. Sending response: sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Service Client (`add_two_ints_client.py`)**:

```python
import rclpy
from rclpy.node import Node
from my_first_package.srv import AddTwoInts  # Import your custom service type
import sys

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 3:
        node = rclpy.create_node('add_two_ints_client_error')
        node.get_logger().info('Usage: ros2 run my_first_package add_two_ints_client <int> <int>')
        rclpy.shutdown()
        sys.exit(1)

    client = AddTwoIntsClient()
    response = client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    if response:
        client.get_logger().info(f'Result of add_two_ints: {response.sum}')
    else:
        client.get_logger().error('Service call failed.')
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

After rebuilding your package (`colcon build`) and sourcing the setup files, you can run:
1.  `ros2 run my_first_package add_two_ints_server` (in one terminal)
2.  `ros2 run my_first_package add_two_ints_client 5 7` (in another terminal)

You should see the client sending the request and the server responding with `sum=12`.

## Actions: Asynchronous, Goal-Oriented Tasks

For tasks that take a longer time to complete (e.g., navigating to a destination, performing a complex manipulation sequence), you need more than a simple request-response. You need feedback on progress, the ability to cancel the task, and a final result. This is where ROS 2 Actions come in.

**Actions** are also client-server based, but provide:
-   A **goal** (what you want to achieve).
-   **Feedback** (progress updates during execution).
-   A **result** (final outcome of the task).

### Implementing a ROS 2 Action Server and Client (Python)

Similar to services, actions require a `.action` file definition (e.g., `my_first_package/action/Fibonacci.action`):

```
int64 order
---
int64[] sequence
---
int64[] partial_sequence
```

**Action Server (`fibonacci_action_server.py`)**:

```python
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from my_first_package.action import Fibonacci # Import your custom action type

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
        self.get_logger().info('Fibonacci Action Server ready.')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        # Append the Fibonacci sequence
        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1) # Simulate long-running task

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        self.get_logger().info('Goal succeeded: {0}'.format(result.sequence))
        return result

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Action Client (`fibonacci_action_client.py`)**:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from my_first_package.action import Fibonacci # Import your custom action type

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
        self.get_logger().info('Fibonacci Action Client ready.')

    def send_goal(self, order):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info('Sending goal request...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info('Received feedback: {0}'.format(feedback_msg.feedback.partial_sequence))

def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()
    action_client.send_goal(10) # Request Fibonacci sequence of order 10
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

After rebuilding and sourcing, run:
1.  `ros2 run my_first_package fibonacci_action_server`
2.  `ros2 run my_first_package fibonacci_action_client`

You will see feedback messages printed as the sequence is generated, and finally the full result.

## Managing Complexity: Launch Files

As your robotic system grows, you'll have many nodes, services, and parameters to manage. Manually starting each one in a separate terminal is tedious and error-prone. **Launch Files** are XML or Python files that allow you to define and run a set of nodes, configure parameters, and even orchestrate complex startup routines.

**Example Python Launch File (`my_launch_file.launch.py`)**:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_first_package',
            executable='my_publisher',
            name='publisher_node',
            output='screen'
        ),
        Node(
            package='my_first_package',
            executable='my_subscriber',
            name='subscriber_node',
            output='screen'
        ),
        Node(
            package='my_first_package',
            executable='add_two_ints_server',
            name='add_server_node',
            output='screen'
        )
        # You can add more nodes here, including action servers/clients
    ])
```

To run this launch file:
```bash
ros2 launch my_first_package my_launch_file.launch.py
```

This will start all the defined nodes in a single command. Launch files are incredibly versatile and can be used for:
-   Passing parameters to nodes.
-   Remapping topics and services.
-   Conditional launching of nodes.
-   Including other launch files for modularity.

By mastering Services, Actions, and Launch Files, you gain powerful tools to build more robust, interactive, and manageable ROS 2 applications. In the next module, we will explore how to bring our robots to life in simulation.