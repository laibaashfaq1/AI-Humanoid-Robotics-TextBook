# Research Findings: Physical AI & Humanoid Robotics Book

**Branch**: `001-humanoid-robotics-book` | **Date**: 2025-12-04
**Input**: Implementation plan from `specs/001-humanoid-robotics-book/plan.md`

## Overview

This document consolidates research findings for critical technical clarifications identified during the planning phase. These decisions aim to establish a stable and consistent development environment for the book's code examples and tutorials.

## Clarification Research

### 1. ROS 2 Distribution

**Question**: What specific ROS 2 distribution should be used for the book's code examples?

**Decision**: ROS 2 Humble Hawksbill (LTS)

**Rationale**: Humble Hawksbill is a Long Term Support (LTS) release, providing extended maintenance and stability, which is crucial for an educational resource. This ensures that code examples remain functional and relevant for a longer period, minimizing the need for frequent updates due to API changes in non-LTS releases. It also has a mature ecosystem and wide community support.

**Alternatives Considered**:
*   **ROS 2 Iron Irwini**: Newer, but not an LTS release, meaning shorter support cycles and potential for breaking changes before book publication.
*   **ROS 2 Foxy Fitzroy**: An older LTS release, but Humble offers more recent features and better compatibility with newer hardware/software.

### 2. Gazebo Distribution

**Question**: What specific Gazebo distribution should be used for the book's simulation environments?

**Decision**: Gazebo Harmonic (also known as Ignition Gazebo)

**Rationale**: Gazebo Harmonic is the latest stable release of Ignition Gazebo, which is the successor to classic Gazebo. It offers significant improvements in modularity, performance, and features, aligning with modern robotics development. Its integration with ROS 2 Humble is robust.

**Alternatives Considered**:
*   **Gazebo Fortress/Garden**: Older Ignition releases; Harmonic provides the latest features and bug fixes.
*   **Classic Gazebo**: While widely used, Ignition Gazebo represents the future of Gazebo development and better integrates with modern ROS 2 releases.

### 3. Unity LTS Version

**Question**: What specific Unity LTS (Long Term Support) version should be used for the book's Unity-based simulations?

**Decision**: Unity 2022 LTS (e.g., 2022.3.x LTS)

**Rationale**: Using a Unity LTS release provides long-term stability and support, critical for avoiding compatibility issues with robotics packages (like Unity Robotics Hub) and ensuring the longevity of the book's examples. Unity 2022 LTS offers a balance of new features and stability.

**Alternatives Considered**:
*   **Newer non-LTS Unity versions**: May introduce instability or breaking changes that could quickly invalidate book content.
*   **Older Unity LTS versions**: Might lack features or optimizations beneficial for modern robotics simulations.

## Conclusion

The chosen versions for ROS 2, Gazebo, and Unity establish a coherent and stable development and simulation environment. These decisions are foundational for the book's reproducibility and its long-term educational value.
