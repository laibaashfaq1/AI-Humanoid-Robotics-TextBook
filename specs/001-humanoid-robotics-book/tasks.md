# Tasks: Physical AI & Humanoid Robotics Book

**Input**: Design documents from `specs/001-humanoid-robotics-book/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

## Format: `[ID] [P?] [Story] Description`
- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Phase 1: Setup (Shared Infrastructure)
**Purpose**: Project initialization and basic structure.
- [X] T001 [P] Initialize a Docusaurus project in the `book/` directory.
- [X] T002 [P] Configure `docusaurus.config.js` with the book title, theme, and navigation structure.
- [X] T003 [P] Create the directory structure for code examples in `code-examples/` (ros2, gazebo, unity, isaac-sim, capstone).
- [X] T004 Create placeholder markdown files for each chapter in `book/docs/` based on the book outline.

---

## Phase 2: Foundational (Blocking Prerequisites)
**Purpose**: Core infrastructure and content that MUST be complete before any user story can be fully implemented.
- [X] T005 [P] Write Chapter 1: "Introduction to ROS 2 and Core Concepts" in `book/docs/module1/chapter1.md`.
- [X] T006 [P] Develop the "hello world" ROS 2 publisher/subscriber example in `code-examples/ros2/hello-world/`.
- [X] T007 Integrate and test the "hello world" example in Chapter 1.
- [X] T008 [P] Write Chapter 3: "Introduction to Simulation and URDF" in `book/docs/module2/chapter3.md`.
- [X] T009 [P] Create the URDF for a simple robot arm in `code-examples/common/robot-arm.urdf`.
- [X] T010 Integrate and display the URDF in Chapter 3.

---

## Phase 3: User Story 1 - Module 1: ROS 2 (Priority: P1)
**Goal**: The learner can create a simple ROS 2 publisher and subscriber.
**Independent Test**: The learner can run the provided ROS 2 publisher and subscriber nodes and see them communicate.
- [X] T011 Write Chapter 2: "Services, Actions, and Launch Files" in `book/docs/module1/chapter2.md`.
- [X] T012 [P] [US1] Develop a ROS 2 service example (server/client) in `code-examples/ros2/service-example/`.
- [X] T013 [P] [US1] Develop a ROS 2 action example (server/client) in `code-examples/ros2/action-example/`.
- [X] T014 [US1] Create a launch file for the service and action examples in `code-examples/ros2/launch/`.
- [X] T015 [US1] Integrate and test the service, action, and launch file examples in Chapter 2.

---

## Phase 4: User Story 2 - Module 2: Gazebo & Unity (Priority: P2)
**Goal**: The learner can create a digital twin of a robot in Gazebo and Unity.
**Independent Test**: The learner can launch the simulated robot in Gazebo and Unity and control it via ROS 2.
- [X] T016 Write Chapter 4: "Simulating Robots with Gazebo" in `book/docs/module2/chapter4.md`.
- [X] T017 [P] [US2] Create a Gazebo world and launch file for the robot arm in `code-examples/gazebo/robot-arm-sim/`.
- [X] T018 [P] [US2] Implement a Gazebo plugin for ROS 2 joint control in `code-examples/gazebo/robot-arm-sim/`.
- [X] T019 [US2] Integrate and test the Gazebo simulation in Chapter 4.
- [X] T020 Write Chapter 5: "Advanced Simulation with Unity" in `book/docs/module2/chapter5.md`.
- [X] T021 [P] [US2] Set up a Unity project with the Robotics Hub package in `code-examples/unity/robot-arm-sim/`.
- [X] T022 [P] [US2] Implement the ROS-TCP-Connector to control the robot in Unity in `code-examples/unity/robot-arm-sim/`.
- [X] T023 [US2] Integrate and test the Unity simulation in Chapter 5.

---

## Phase 5: User Story 3 - Module 3: NVIDIA Isaac (Priority: P3)
**Goal**: The learner can use NVIDIA Isaac to implement AI perception and manipulation.
**Independent Test**: The learner can run the Isaac Sim application to detect an object and plan a pick-and-place motion.
- [X] T024 Write Chapter 6: "Introduction to Isaac Sim for Perception" in `book/docs/module3/chapter6.md`.
- [X] T025 [P] [US3] Create an Isaac Sim scene with a robot and target objects in `code-examples/isaac-sim/perception-scene/`.
- [X] T026 [P] [US3] Implement a script for domain randomization and synthetic data generation in `code-examples/isaac-sim/perception-scene/`.
- [X] T027 [US3] Integrate and test the perception examples in Chapter 6.
- [X] T028 Write Chapter 7: "Robot Manipulation with Isaac Gym" in `book/docs/module3/chapter7.md`.
- [X] T029 [P] [US3] Implement a motion planning example using Isaac MoveIt in `code-examples/isaac-sim/manipulation-example/`.
- [X] T030 [US3] Integrate and test the manipulation example in Chapter 7.

---

## Phase 6: User Story 4 - Module 4: VLA & Capstone (Priority: P4)
**Goal**: The learner can integrate a VLA model with their robot and complete a capstone project.
**Independent Test**: The learner can give a natural language command to the robot, and the robot will execute the command in the simulation.
- [X] T031 Write Chapter 8: "Understanding Vision-Language-Action Models" in `book/docs/module4/chapter8.md`.
- [X] T032 Write Chapter 9: "Integrating a VLA with ROS 2" in `book/docs/module4/chapter9.md`.
- [X] T033 [P] [US4] Develop a ROS 2 node to query a VLA API in `code-examples/ros2/vla-integration/`.
- [X] T034 [US4] Integrate the VLA node with the motion planning and perception systems.
- [X] T035 Write Chapter 10: "Capstone Project: Autonomous Humanoid Task" in `book/docs/module4/chapter10.md`.
- [X] T036 [US4] Develop the full capstone project, integrating all components, in `code-examples/capstone/final-project/`.
- [X] T037 [US4] Write a comprehensive `README.md` for the capstone project.

---

## Phase 7: Polish & Cross-Cutting Concerns
**Purpose**: Final improvements and validation.
- [X] T038 [P] Review all chapters for clarity, consistency, and adherence to the Flesch score target.
- [X] T039 [P] Validate all code examples for reproducibility and adherence to the coding standards.
- [X] T040 [P] Check all citations and ensure at least 30% are from authoritative sources.
- [X] T041 Build the full Docusaurus site and test all navigation, links, and embedded components.
- [X] T042 Deploy the final Docusaurus site to GitHub Pages.

---

## Dependencies & Execution Order
- **Setup (Phase 1)** and **Foundational (Phase 2)** must be completed before any user story phases.
- **User Story 1 (Phase 3)** is a prerequisite for all subsequent user stories.
- **User Story 2 (Phase 4)** depends on Phase 3.
- **User Story 3 (Phase 5)** depends on Phase 4.
- **User Story 4 (Phase 6)** depends on Phase 5.
- **Polish (Phase 7)** can only begin after all user story phases are complete.

## Implementation Strategy
- **MVP First**: Complete Phase 1, 2, and 3 to have a foundational book with the first module complete and testable.
- **Incremental Delivery**: Each subsequent phase (4, 5, 6) can be delivered as a complete, testable module.
