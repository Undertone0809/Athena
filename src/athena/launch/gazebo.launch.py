import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "athena"
    urdf_name = "bot_gazebo.urdf"
    robot_name_of_model = "bot"

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, f"urdf/{urdf_name}")
    gazebo_world_path = os.path.join(pkg_share, "world/neighborhood.world")

    # startup gazebo simulation
    start_gazebo_cmd = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            gazebo_world_path,
        ],
        output="screen",
    )
    ld.add_action(start_gazebo_cmd)

    # create the robot
    spawn_entity_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", robot_name_of_model, "-file", urdf_model_path],
        output="screen",
    )
    ld.add_action(spawn_entity_cmd)

    # Start Robot State publisher
    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=[urdf_model_path],
    )
    ld.add_action(start_robot_state_publisher_cmd)

    return ld
