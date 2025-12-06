# Quickstart Guide: Physical AI & Humanoid Robotics Book

**Branch**: `001-humanoid-robotics-book` | **Date**: 2025-12-04
**Input**: Implementation plan from `specs/001-humanoid-robotics-book/plan.md`
**Purpose**: Provides a concise guide for setting up the necessary hardware and software environment to follow the book's examples.

## 1. Hardware Requirements

To fully utilize the code examples and simulations in this book, a system meeting the following minimum specifications is recommended:

*   **Operating System**: Ubuntu 22.04 LTS (Desktop)
*   **Processor**: Intel i7 or AMD Ryzen 7 (or equivalent)
*   **RAM**: 32 GB or more
*   **Graphics Card**: Dedicated NVIDIA GPU with at least 8 GB VRAM (e.g., NVIDIA RTX 3060 or equivalent). NVIDIA CUDA support is essential.

**Alternative**: Windows 10/11 with Windows Subsystem for Linux 2 (WSL2) running Ubuntu 22.04. Performance may vary.

## 2. Software Setup (Ubuntu 22.04 LTS)

Follow these steps to set up your core development environment:

### 2.1. NVIDIA Driver and CUDA Installation

Ensure you have the latest stable NVIDIA GPU drivers and the corresponding CUDA Toolkit installed. Refer to the official NVIDIA documentation for precise instructions based on your GPU model and Ubuntu version.

### 2.2. Git Installation

```bash
sudo apt update
sudo apt install git
```

### 2.3. Python 3 and pip

Python 3.10 is typically pre-installed on Ubuntu 22.04. Ensure `pip` is also available:

```bash
sudo apt install python3-pip
```

### 2.4. Visual Studio Code (Recommended IDE)

Install Visual Studio Code for an optimized development experience:

```bash
sudo apt install software-properties-common apt-transport-https wget
wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
sudo apt update
sudo apt install code
```

### 2.5. Docker and NVIDIA Container Toolkit

For certain advanced examples (especially with NVIDIA Isaac Sim), Docker and the NVIDIA Container Toolkit may be required. Follow official Docker and NVIDIA documentation for installation.

## 3. Technology-Specific Setup (Within Modules)

Detailed installation and configuration instructions for specific robotics software will be provided at the beginning of each relevant module:

*   **ROS 2 Humble Hawksbill**: Module 1 will cover installation and basic setup.
*   **Gazebo Harmonic**: Module 2 will cover installation and integration with ROS 2.
*   **Unity 2022 LTS**: Module 2 will cover installation and setup for robotics simulation.
*   **NVIDIA Isaac Sim**: Module 3 will cover installation and integration with ROS 2 and Python environments.
*   **OpenAI API**: Module 4 will provide guidance on setting up API access.

## 4. Obtaining Code Examples

All code examples for the book can be found in the `code-examples/` directory of the book's GitHub repository. Each example directory will contain its own `README.md` with specific instructions.

```bash
git clone https://github.com/your_github_username/book-hackathon.git
cd book-hackathon/code-examples
```

## 5. Troubleshooting

*   **"Permission denied" errors**: Ensure you have correct permissions for directories or use `sudo` where appropriate.
*   **Missing dependencies**: Refer to the `README.md` within each code example for specific requirements.
*   **NVIDIA GPU issues**: Double-check NVIDIA driver and CUDA installations.
```
