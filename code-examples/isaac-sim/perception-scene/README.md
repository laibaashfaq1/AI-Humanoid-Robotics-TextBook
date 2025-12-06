# Isaac Sim Perception Scene

This directory is intended to hold the assets and Python scripts for the Isaac Sim perception scene described in **Chapter 6: Introduction to Isaac Sim for Perception**.

## Manual Setup Steps:

1.  **Launch Isaac Sim**: Start your Isaac Sim application.

2.  **Create a New Scene**: Start with an empty stage or a pre-built environment (e.g., a simple room).

3.  **Import the Robot**:
    *   Use the URDF Importer (`File > Import > Robot (URDF)`) to import the `robot-arm.urdf` file from the `code-examples/common/` directory.
    *   Save the imported robot as a USD asset within this project directory.

4.  **Add Target Objects**:
    *   Create simple geometric shapes like cubes and spheres (`Create > Shape > ...`).
    *   Position these objects in the scene where the robot can "see" them.

5.  **Add a Camera to the Robot**:
    *   Attach a camera to the robot's end-effector link (e.g., the `wrist_link`).
    *   Ensure the camera is pointing forward.

6.  **Create a Python Script for Logic**:
    *   Create a Python script (e.g., `perception_logic.py`) in this directory.
    *   This script will contain the logic for setting up the scene programmatically, running domain randomization, and publishing sensor data via the ROS 2 bridge, as described in Chapter 6.

The specific Python code for controlling the scene and generating data should be developed based on the examples and concepts outlined in the book chapter.
