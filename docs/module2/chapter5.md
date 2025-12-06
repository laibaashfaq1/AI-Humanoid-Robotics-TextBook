---
sidebar_position: 3
---

# Chapter 5: Advanced Simulation with Unity

While Gazebo excels in robotics-specific simulation, **Unity** offers unparalleled visual fidelity, a powerful game engine, and a flexible environment for developing complex, interactive simulations. With the Unity Robotics Hub, it also provides robust integration with ROS 2, allowing you to use Unity as a high-fidelity sensor and actuator simulation platform. This chapter will guide you through setting up Unity for robotics and establishing a seamless communication bridge with your ROS 2 workspace.

## Setting Up Unity for Robotics

To leverage Unity for robotics, you'll need a specific setup:

1.  **Install Unity Hub and Unity 2022 LTS**: Download and install Unity Hub, then install a **Unity 2022 LTS (Long Term Support)** version. LTS versions are recommended for stability in robotics projects.
2.  **Create a New Unity Project**: Open Unity Hub, create a new 3D project.
3.  **Install Unity Robotics Hub Packages**:
    *   **ROS-TCP-Connector**: This package facilitates communication between Unity and ROS 2 over TCP.
    *   **ROS-TCP-Endpoint**: The counterpart to the Connector, running in your ROS 2 environment.
    *   **URDF Importer**: Allows you to import your URDF robot models directly into Unity.
    *   **Other Robotics Packages**: Depending on your needs, you might install packages like `Unity.Robotics.Visualizations`, `Unity.Robotics.Pointcloud`, etc.

    You can install these packages via the Unity Package Manager (`Window > Package Manager`) by adding them from a Git URL (typically from the [Unity Robotics Hub GitHub repository](https://github.com/Unity-Technologies/Unity-Robotics-Hub)).

## Importing and Configuring a URDF for Unity

The `URDF Importer` package simplifies bringing your robot models into Unity:

1.  **Import URDF**: Go to `Robotics > URDF Importer > Import Robot from URDF`. Select your `robot-arm.urdf` file (e.g., from `code-examples/common/robot-arm.urdf`).
2.  **Configure Import Settings**: The importer provides options to generate colliders, rigidbodies, and control joints. Ensure that "Generate Robot Description" and "Generate Articulation Body" (for physics-based control) are selected.
3.  **Spawn Robot**: Once imported, your robot prefab will be in your project assets. Drag it into your scene.

Unity will create a hierarchy of GameObjects representing your robot's links and joints. The ArticulationBody component on each joint will allow for realistic physics simulation and control.

## The ROS-TCP-Connector: Linking Unity and ROS 2

The `ROS-TCP-Connector` package in Unity and the `ROS-TCP-Endpoint` ROS 2 package are the keys to real-time communication.

### 1. Setting up the ROS-TCP-Endpoint (ROS 2 side)

In your ROS 2 workspace, install and run the `ros_tcp_endpoint` package:

```bash
# Install the package (replace <ros2_distro> with your distro, e.g., humble)
sudo apt install ros-<ros2_distro>-ros-tcp-endpoint

# Or if building from source:
cd ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
# Go back to ros2_ws and colcon build

# Run the endpoint
ros2 run ros_tcp_endpoint default_server_endpoint
```
This will start a TCP server in your ROS 2 environment, listening for connections from Unity.

### 2. Setting up the ROS-TCP-Connector (Unity side)

In your Unity project:
1.  **Add a `ROSConnection` Component**: Create an empty GameObject in your scene and add the `ROSConnection` script component to it.
2.  **Configure `ROSConnection`**: Set the `Ros IP Address` to your ROS 2 machine's IP address (or `localhost` if running on the same machine). The `Ros Port` should match the port used by `default_server_endpoint` (default 10000).

Once configured, when you play your Unity scene, the `ROSConnection` component will attempt to connect to the `ROS-TCP-Endpoint`.

## Receiving ROS 2 Messages to Control Robot Joints in Unity

With the connection established, you can now write Unity C# scripts to subscribe to ROS 2 topics and control your robot.

### Example: Subscribing to `JointState` and applying to ArticulationBody

You would create a C# script (e.g., `RosJointController.cs`) that attaches to your robot's root GameObject in Unity. This script would subscribe to a ROS 2 topic (e.g., `/joint_states`) and update the target positions of the `ArticulationBody` components on your robot's joints.

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor; // Or your custom JointCommand message type

public class RosJointController : MonoBehaviour
{
    ROSConnection ros;
    public string jointStateTopic = "/joint_states"; // Or a specific command topic

    // Map your Unity joints to ROS joint names
    public ArticulationBody[] robotJoints; // Assign in Inspector

    void Start()
    {
        ros = ROSConnection.Get
();
        ros.Subscribe<JointStateMsg>(jointStateTopic, ReceiveJointState);
    }

    void ReceiveJointState(JointStateMsg jointState)
    {
        // Example: Apply position commands from ROS to Unity ArticulationBodies
        // This is a simplified example; a real implementation would handle joint mapping carefully
        for (int i = 0; i < jointState.position.Length; i++)
        {
            if (i < robotJoints.Length)
            {
                var drive = robotJoints[i].xDrive;
                drive.target = (float)jointState.position[i] * Mathf.Rad2Deg; // ROS angles are radians, Unity needs degrees for ArticulationBody.target
                robotJoints[i].xDrive = drive;
            }
        }
    }
}
```
In your ROS 2 workspace, you could then publish `JointState` messages (or custom joint command messages) to the `/joint_states` topic, and your Unity robot would respond in real-time.

## Publishing Sensor Data from Unity Back to ROS 2

Unity can also act as a rich sensor data generator, publishing information from its virtual environment back to ROS 2.

### Example: Publishing Camera Image

You could attach a `ROSImagePublisher` script (custom or from Unity Robotics Hub) to a camera in your Unity scene. This script would capture the camera feed and publish it as a `sensor_msgs/Image` message to a ROS 2 topic (e.g., `/unity_camera/image_raw`).

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Sensor; // For ImageMsg

public class UnityCameraPublisher : MonoBehaviour
{
    public string topicName = "/unity_camera/image_raw";
    public Camera renderCamera; // Assign your scene camera here
    public int resolutionWidth = 640;
    public int resolutionHeight = 480;

    private Texture2D texture;
    private ROSConnection ros;
    private ImagePublisher<ImageMsg> imagePublisher;

    void Start()
    {
        ros = ROSConnection.Get
();
        imagePublisher = new ImagePublisher<ImageMsg>(ros, topicName, renderCamera, resolutionWidth, resolutionHeight);
        texture = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RGB24, false);
    }

    void Update()
    {
        // Example: Publish image every frame or at a fixed rate
        imagePublisher.UpdateCameraInfo(renderCamera);
        imagePublisher.Publish(renderCamera);
    }
}
```

This capability allows you to generate synthetic datasets for training perception models or to visualize the robot's "view" from within your ROS 2 tools.

Unity, with its powerful graphics and physics, combined with the ROS-TCP-Connector, provides a compelling platform for advanced robotics simulation. This hybrid approach offers the best of both worlds: the robust control and communication of ROS 2, and the rich, interactive environments of Unity.