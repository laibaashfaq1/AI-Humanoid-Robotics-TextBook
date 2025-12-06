---
sidebar_position: 2
---

# Chapter 4: Simulating Robots with Gazebo

In Chapter 3, you learned how to describe your robot's physical properties using URDF. Now, it's time to bring that description to life in a dynamic, physics-based simulator. This chapter will guide you through using **Gazebo Harmonic**, the successor to classic Gazebo, to simulate your robot and interact with it via ROS 2.

## Introduction to Gazebo Harmonic

Gazebo Harmonic is a powerful 3D robotics simulator that provides a robust physics engine, high-quality graphics, and a convenient interface for creating and managing complex simulation environments. It's designed for use with ROS 2 and offers excellent integration capabilities, allowing you to develop and test your robot control software in a safe and repeatable virtual world.

### Key Features of Gazebo Harmonic:
-   **Physics Engine**: Supports various physics engines (e.g., DART, ODE, Bullet) for realistic robot dynamics.
-   **Sensors**: Emulates a wide range of sensors (cameras, LiDAR, IMU) with customizable noise models.
-   **Environments**: Allows creation of complex 3D worlds with lighting, textures, and interactive objects.
-   **Plugins**: Extensible architecture through plugins for custom robot behaviors and ROS 2 integration.

## Spawning a URDF Model in a Gazebo World

To get started, you'll need a simple world file and a launch file to bring up Gazebo and spawn your URDF model.

1.  **Create a simple world file (`empty_world.sdf`)**:
    Create a directory `code-examples/gazebo/worlds` and save this:
    ```xml
    <?xml version="1.0" ?>
    <sdf version="1.6">
      <world name="default">
        <include>
          <uri>model://sun</uri>
        </include>
        <include>
          <uri>model://ground_plane</uri>
        </include>
      </world>
    </sdf>
    ```

2.  **Create a Gazebo launch file (`robot_arm_gazebo.launch.py`)**:
    Create a directory `code-examples/gazebo/launch` and save this (assuming your URDF is `code-examples/common/robot-arm.urdf`):
    ```python
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import Node

    def generate_launch_description():
        # Get the path to the common code examples
        common_path = os.path.join(
            get_package_share_directory('my_gazebo_package'), # This package needs to exist for get_package_share_directory
            '../../../../common' # Relative path from share/my_gazebo_package to common/
        )
        urdf_file_path = os.path.join(common_path, 'robot-arm.urdf')

        # Path to Gazebo launch files
        gazebo_ros_launch_dir = get_package_share_directory('gazebo_ros')

        # Start Gazebo simulation
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_launch_dir, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': os.path.join(common_path, 'empty_world.sdf')}.items()
        )

        # Robot State Publisher node
        # This node takes the URDF description and publishes the robot's state to ROS 2 topics
        with open(urdf_file_path, 'r') as infp:
            robot_desc = infp.read()

        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        )

        # Spawn the URDF model in Gazebo
        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'robot_arm'],
            output='screen'
        )

        return LaunchDescription([
            gazebo_launch,
            robot_state_publisher_node,
            spawn_entity
        ])
    ```
    **Note**: You'll need to create a ROS 2 package (e.g., `my_gazebo_package`) to properly use `get_package_share_directory` to locate your URDF and world files. For simplicity, the example above assumes a relative path.

To launch your robot in Gazebo:
```bash
ros2 launch my_gazebo_package robot_arm_gazebo.launch.py
```
(Replace `my_gazebo_package` with the actual name of the package containing your launch file and URDF/world if you followed the package creation approach.)

## Using Gazebo Plugins for ROS 2 Integration

Gazebo's power is greatly enhanced by its plugins, especially those designed for ROS 2 integration. These plugins allow Gazebo to publish sensor data and subscribe to control commands via ROS 2 topics.

### Example: ROS 2 Control with Diff Drive Controller

For mobile robots, a common controller is the `diff_drive_controller`. While our robot arm isn't a differential drive, the principle applies: a Gazebo plugin exposes an interface (e.g., a ROS 2 topic for wheel velocities) that your ROS 2 nodes can interact with.

For a robot arm, you'd typically use **`ros2_control`** and its Gazebo plugin. This setup defines hardware interfaces (e.g., for position or velocity control of joints) in your URDF, and the Gazebo plugin simulates these interfaces, allowing you to command your robot arm using standard ROS 2 control interfaces.

To achieve this, your URDF needs `ros2_control` tags, and you'll need to load a controller manager in Gazebo. This involves:
1.  **Adding `<ros2_control>` tags to your URDF**: Define your joints and their hardware interfaces (e.g., `hardware_interface/PositionJointInterface`).
2.  **Creating controller configurations**: YAML files that specify which controllers (`joint_trajectory_controller`, `joint_state_broadcaster`) to use for your robot.
3.  **Launching the controller manager**: Use a ROS 2 launch file to load the `ros2_control` plugin into Gazebo and spawn your desired controllers.

Once set up, you can publish to topics like `/joint_trajectory_controller/joint_trajectory` to command your robot arm's movement in Gazebo.

## Subscribing to Simulated Sensor Data

Gazebo plugins can also simulate sensor data and publish it to ROS 2 topics. For example, a simulated camera would publish `sensor_msgs/Image` messages to a `/camera/image_raw` topic.

To subscribe to this data in your ROS 2 nodes:
1.  Ensure your Gazebo world includes a simulated camera sensor on your robot.
2.  Launch Gazebo and the necessary `camera_controller` plugin.
3.  Write a ROS 2 subscriber node (similar to Chapter 1) that listens to the camera topic and processes the image data.

This ability to generate realistic sensor data in simulation is invaluable for developing perception algorithms without needing physical hardware.

## Controlling Simulated Joints with ROS 2 Messages

Once you have `ros2_control` and controllers set up in Gazebo, you can control your robot arm.

**Example: Controlling a Joint Position via `joint_trajectory_controller`**:

You would typically use a `JointTrajectory` message to send a target position for your arm's joints.

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(1.0, self.publish_trajectory) # Publish every second

    def publish_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = ['shoulder_joint', 'elbow_joint', 'wrist_joint'] # Your joint names

        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.5, 0.0] # Target positions for each joint
        point.time_from_start = Duration(sec=1, nanosec=0)
        msg.points.append(point)

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing joint trajectory')

def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

By connecting your ROS 2 control nodes to Gazebo through these interfaces, you can fully test your robot's software logic in a high-fidelity virtual environment. In the next chapter, we'll explore another powerful simulation platform: Unity.