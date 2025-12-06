# Code Example Interface Contracts

**Branch**: `001-humanoid-robotics-book` | **Date**: 2025-12-04
**Input**: Implementation plan from `specs/001-humanoid-robotics-book/plan.md`
**Purpose**: Defines the expected structure, execution, and interaction patterns for all runnable code snippets in the book. This serves as a "contract" for how code examples integrate with the book content and development environment.

## 1. Directory Structure Contract

All runnable code snippets will reside in the `code-examples/` directory at the repository root, organized by technology and module.

```text
code-examples/
├── ros2/
│   ├── module1_chapterX_exampleY/
│   │   ├── src/                 # Source files (Python, C++)
│   │   ├── package.xml          # ROS 2 package definition
│   │   └── CMakeLists.txt       # ROS 2 build system
│   └── ...
├── gazebo/
│   ├── module2_chapterX_exampleY/
│   │   ├── model/               # SDF/URDF model files
│   │   ├── world/               # World definition files
│   │   └── launch/              # Launch files
│   └── ...
├── unity/
│   ├── module2_chapterX_exampleY/
│   │   ├── Assets/              # Unity project assets
│   │   ├── ProjectSettings/
│   │   └── ...
│   └── ...
├── isaac-sim/
│   ├── module3_chapterX_exampleY/
│   │   ├── scripts/             # Python scripts for Omniverse/Isaac Sim
│   │   └── assets/              # USD assets
│   └── ...
└── capstone/
    ├── final_project/
    │   ├── ros2_pkgs/
    │   ├── isaac_scripts/
    │   └── ...
    └── ...
```

## 2. Execution Contract

*   **Standard Entry Point**: Each example (or set of related examples within a directory) MUST provide a clear, documented entry point for execution. This could be:
    *   A `launch` file for ROS 2.
    *   A shell script (`run.sh` or `start.bat`) for complex setups.
    *   Clear instructions for Unity project loading and execution.
    *   A Python script directly runnable for Isaac Sim or standalone Python code.
*   **Dependencies**: All examples MUST declare their specific dependencies (Python packages, ROS 2 packages, apt packages) in a `requirements.txt` (Python) or `package.xml` (ROS 2) or equivalent.
*   **Environment**: Examples are expected to run within the defined `Target Platform` (Ubuntu 22.04 LTS with specified hardware, or WSL2 equivalent).
*   **Reproducibility**: Each example MUST be runnable independently and produce consistent, verifiable output as described in the accompanying book chapter.

## 3. Input/Output Contract

*   **Configuration**: Examples SHOULD use configuration files (e.g., YAML for ROS 2 parameters, JSON for generic settings) rather than hardcoding values for flexibility.
*   **Logging/Output**: Examples SHOULD output clear, informative messages to the console for demonstration purposes. Errors or unexpected behavior SHOULD be handled gracefully and reported.
*   **Interaction**: If an example requires user interaction (e.g., keyboard input, joystick control), it MUST be clearly documented in the chapter.

## 4. Licensing Contract

All code examples provided in the `code-examples/` directory and embedded in the book content MUST be licensed under the Apache 2.0 License. A `LICENSE` file for each top-level example directory is recommended if the license applies specifically to that example.

## 5. Documentation Contract (within code)

*   **In-code Comments**: Code SHOULD be well-commented to explain non-obvious logic, data structures, and algorithms.
*   **README**: Each top-level example directory (`ros2/module1_chapterX_exampleY/`) SHOULD contain a `README.md` file detailing:
    *   Purpose of the example.
    *   How to set up and run the example.
    *   Expected output.
    *   Specific dependencies.
    *   Troubleshooting tips.
