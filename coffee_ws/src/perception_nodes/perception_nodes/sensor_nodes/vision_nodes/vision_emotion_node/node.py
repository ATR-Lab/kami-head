#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class EmotionNode(Node):
    def __init__(self):
        super().__init__('emotion_node')
        self.get_logger().info("Emotion node initialized")

        # Create a publisher for the emotion topic
        self.emotion_publisher = self.create_publisher(Emotion, 'system/perception/sensor/emotion', 10)

        # Create a timer to publish the emotion message
        self.timer = self.create_timer(1.0, self.publish_emotion)

    def publish_emotion(self, emotion: int):
        msg = Emotion()
        msg.emotion = emotion
        self.emotion_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    emotion_node = EmotionNode()
    rclpy.spin(emotion_node)
    emotion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()