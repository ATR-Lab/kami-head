#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.get_logger().info("TTS node initialized")


def main(args=None):
    rclpy.init(args=args)
    tts_node = TTSNode()
    rclpy.spin(tts_node)
    tts_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()