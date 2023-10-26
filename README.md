# Generating Action Sequences for Robot Task Plans using LLM

## Environment

- Ubuntu 22.04
- ROS2 humble
- gpt-3.5-turbo
- gazebo

## Setup

**Clone the community/sig-robotics respository**

```shell
git clone https://github.com/kubeedge/community
```

```shell
rosdep install --from-paths src -y
colcon build
```

**Run in RVIZ**

```shell
source install/setup.bash
ros2 launch bot_description display_rviz2.launch.py
```

**Gazebo simulation**

```shell
source install/setup.bash
ros2 launch bot_description gazebo.launch.py
```
