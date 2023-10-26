#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from promptulate.llms import ChatOpenAI


class TeleopNode(Node):
    def __init__(self):
        super().__init__("user_client")
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.cmd_vel_ = Twist()
        self.get_logger().info("user_client node startup")

    def run(self):
        while True:
            user_input: str = input("Please input your demand: ")
            response: Response = self.robot_agent.run(
                prompt=user_input, output_schema=Response
            )
            self.get_logger().info(f"RobotAgent instruction: {response}")
            cmd = response.command

            if cmd == "w":
                self.cmd_vel_.linear.x = 0.2  # 设置线速度为0.2 m/s
            elif cmd == "s":
                self.cmd_vel_.linear.x = -0.2  # 设置线速度为-0.2 m/s，即后退
            elif cmd == "a":
                self.cmd_vel_.angular.z = 0.5  # 设置角速度为0.5 rad/s，即左转
            elif cmd == "d":
                self.cmd_vel_.angular.z = -0.5  # 设置角速度为-0.5 rad/s，即右转
            else:
                self.cmd_vel_.linear.x = 0.0  # 停止线速度
                self.cmd_vel_.angular.z = 0.0  # 停止角速度

            self.publisher_.publish(self.cmd_vel_)


def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()
    teleop_node.run()
    rclpy.shutdown()
