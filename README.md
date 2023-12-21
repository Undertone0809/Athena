<h1 align="center">
    Athena
</h1>

<p align="center">
    A Mulitmodal Robot Agent based on ROS2.
</p>

## Brief
Athena is a multi-modal robot agent framework based on [ROS2](https://github.com/ros2) and [promptulate](https://github.com/Undertone0809/promptulate).
It is designed to provide a unified interface for robot agents to interact with the environment and users. It is also designed to be modular and extensible, so that it can be easily extended to support new modalities and new tasks.

This project is startup by [Promptulate AI](https://github.com/Undertone0809/promptulate), which is a LLM application and Agent development community. We are interested in the application of LLM in the field of robotics, and we are also interested in the development of robot agents. We hope to promote the development of LLM and robot agents.

About basic proposal of this project, see more here: [Generating Action Sequences for Robot Task Plans using LLM](https://github.com/kubeedge/community/tree/master/sig-robotics/proposal/Generating%20Action%20Sequences%20for%20Robot%20Task%20Plans%20using%20LLM).

## Features
As a multi-modal robot agent framework, Athena has the following features:
- Driven by Agent, using promptulate to generate action sequences for robot task plans.
- Support for multiple modalities, including text, speech, vision, and action.
- Interaction by task driven, including task planning, task execution, and task monitoring.
- Modular and extensible, including the ability to add new modalities and new tasks, you can extend the specified sensor or controller easily.

## Framework
Athena consists of three main modules: **RobotAgent, RobotController, and RobotObserver**.

- RobotAgent: This core module handles the behavior control and task scheduling of the Robot. It provides functions for user input to initiate task scheduling, as well as functionalities of its sub-modules such as task planning, behavior control, environment perception, behavior reasoning, event looping, etc.
- RobotController: This module provides behavior controllers for the Robot. It contains an array of Operators, where each Operator offers control for a specific type of behavior.
- RobotObserver: This module provides environment perception for the Robot. It contains an array of Sensors, where each Sensor provides information about the environment and its own operational status for the current state of the Robot.

## Quick Start

Here is a quick start guide to get you started with Athena in ROS2.

### Environment

Ensure that you have installed the following dependencies:

> ⚠ The following environment is recommended, other environments are not guaranteed to work properly.

- Ubuntu 22.04
- ROS2 humble
- Any LLM model, gpt-4-turbo is recommended
- gazebo

**Clone the community/sig-robotics respository**

```shell
git clone https://github.com/Undertone0809/Athena.git
```

**Install relational packages and build the project**

```shell
cd ./Athena
rosdep install --from-paths src -y
colcon build
```

**Simulate in gazebo**

```shell
source install/setup.bash
ros2 launch athena gazebo.launch.py
ros2 run athena user_client
```

## Roadmap

- [ ] Extract the core framework from the project.
- [ ] Create pypi package to use easily
- [ ] Provide more notebook example.
- [ ] Support more modalities, sensors, and controllers.
- [ ] Build API style client to use easily.

# Contributions

I am currently exploring more comprehensive abstraction patterns to improve compatibility with the framework and the extended use of external tools. If you have any suggestions, I welcome discussions and exchanges. I'm excited to see more people getting involved and optimizing it.
