#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from typing import List

from promptulate.utils.color_print import print_text

from bot_description.core import (
    RobotAgent,
    RobotController,
    RobotObserver,
)
from bot_description.schema import Operator


def get_operators(node: Node):
    """Get all operators of the robot"""
    operators: List[Operator] = []

    def go_front_callback(distance: float):
        # broadcast to send message
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.2

        if not getattr(node, "publisher", None):
            node.publisher = node.create_publisher(Twist, "cmd_vel", 10)
        node.publisher.publish(cmd_vel)

    operators.append(
        Operator(
            name="go_front", description="robot go front", callback=go_front_callback
        )
    )

    def go_back_callback(distance: float):
        # broadcast to send message
        cmd_vel = Twist()
        cmd_vel.linear.x = -0.2

        if not getattr(node, "publisher", None):
            node.publisher = node.create_publisher(Twist, "cmd_vel", 10)
        node.publisher.publish(cmd_vel)

    operators.append(
        Operator(name="go_back", description="robot go back", callback=go_back_callback)
    )

    def turn_left_callback(angle: float):
        # broadcast to send message
        cmd_vel = Twist()
        cmd_vel.linear.angular.z = 0.5

        if not getattr(node, "publisher", None):
            node.publisher = node.create_publisher(Twist, "cmd_vel", 10)
        node.publisher.publish(cmd_vel)

    operators.append(
        Operator(
            name="turn_left",
            description="Turn left in place, No displacement",
            callback=turn_left_callback,
        )
    )

    def turn_right_callback(angle: float):
        # broadcast to send message
        cmd_vel = Twist()
        cmd_vel.linear.angular.z = -0.5

        if not getattr(node, "publisher", None):
            node.publisher = node.create_publisher(Twist, "cmd_vel", 10)
        node.publisher.publish(cmd_vel)

    operators.append(
        Operator(
            name="turn_right",
            description="Turn right in place, No displacement",
            callback=turn_right_callback,
        )
    )

    def stop_callback():
        # broadcast to send message
        cmd_vel = Twist()

        if not getattr(node, "publisher", None):
            node.publisher = node.create_publisher(Twist, "cmd_vel", 10)
        node.publisher.publish(cmd_vel)

    operators.append(
        Operator(name="stop", description="stop the robot", callback=stop_callback)
    )

    return operators


class UserClientNode(Node):
    def __init__(self):
        super().__init__("user_client")
        print_text("user_client node startup", "green")

        controller = RobotController(get_operators(self))
        observer = RobotObserver([])
        self.robot_agent = RobotAgent(controller, observer)

    def run(self):
        """Startup RobotAgent"""
        user_input: str = input("Please input your demand: ")
        self.robot_agent.run(user_input)


# class TeleopNode(Node):
#     def __init__(self):
#         super().__init__("teleop_node")
#         self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
#         self.cmd_vel_ = Twist()
#         self.get_logger().info("user_client node startup")

#     def run(self):
#         while True:
#             user_input: str = input("Please input your demand: ")
#             response: Response = self.robot_agent.run(
#                 prompt=user_input, output_schema=Response
#             )
#             self.get_logger().info(f"RobotAgent instruction: {response}")
#             cmd = response.command

#             if cmd == "w":
#                 self.cmd_vel_.linear.x = 0.2  # 设置线速度为0.2 m/s
#             elif cmd == "s":
#                 self.cmd_vel_.linear.x = -0.2  # 设置线速度为-0.2 m/s，即后退
#             elif cmd == "a":
#                 self.cmd_vel_.angular.z = 0.5  # 设置角速度为0.5 rad/s，即左转
#             elif cmd == "d":
#                 self.cmd_vel_.angular.z = -0.5  # 设置角速度为-0.5 rad/s，即右转
#             else:
#                 self.cmd_vel_.linear.x = 0.0  # 停止线速度
#                 self.cmd_vel_.angular.z = 0.0  # 停止角速度

#             self.publisher_.publish(self.cmd_vel_)


def main(args=None):
    rclpy.init(args=args)
    node = UserClientNode()
    node.run()
    rclpy.shutdown()
