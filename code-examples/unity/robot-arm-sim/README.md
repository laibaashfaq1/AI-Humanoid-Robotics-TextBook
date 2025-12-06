# Unity Robot Arm Simulation Project

This directory is a placeholder for the Unity project that simulates the robot arm. To set up this project, please follow the instructions in **Chapter 5: Advanced Simulation with Unity**.

## Manual Setup Steps:

1.  **Create a new Unity Project**:
    *   Open Unity Hub.
    *   Create a new 3D project named `robot-arm-sim` inside the `code-examples/unity/` directory.

2.  **Install Unity Robotics Hub Packages**:
    *   In your new Unity project, go to `Window > Package Manager`.
    *   Click the `+` icon and select "Add package from git URL...".
    *   Add the following packages from the Unity Robotics Hub repository:
        *   `com.unity.robotics.ros-tcp-connector`
        *   `com.unity.robotics.urdf-importer`
        *   (Optional) `com.unity.robotics.visualizations`

3.  **Import the Robot URDF**:
    *   In the Unity Editor, go to `Robotics > URDF Importer > Import Robot from URDF`.
    *   Select the `robot-arm.urdf` file located in the `code-examples/common/` directory.
    *   Use the default import settings to generate the robot prefab.

4.  **Set up the Scene**:
    *   Drag the generated robot arm prefab into your scene.
    *   Create an empty GameObject and add the `ROSConnection` script component to it.
    *   Configure the `ROSConnection` component to connect to your ROS 2 environment (as described in Chapter 5).

The C# scripts for controlling the robot and publishing sensor data (as outlined in Chapter 5) should be created within the `Assets/Scripts` folder of this Unity project.
