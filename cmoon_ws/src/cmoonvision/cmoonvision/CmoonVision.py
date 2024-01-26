#!/usr/bin/env python3
# coding: UTF-8 
# author: Cmoon
# date: 2024/1/26$ 下午2:59$
import os
import sys

sys.path.append(os.path.dirname(__file__))
import rclpy
from rclpy.node import Node


class MyNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("Hello ROS2 cmoon12")


def main():
    rclpy.init()
    node = MyNode("node_name")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
