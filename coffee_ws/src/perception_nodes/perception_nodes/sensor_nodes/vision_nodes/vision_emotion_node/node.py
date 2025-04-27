# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node

# class VisionEmotionNode(Node):
#     def __init__(self):
#         super().__init__('vision_emotion_node')
#         self.get_logger().info("Vision emotion node initialized")

#         # Create a publisher for the emotion topic
#         self.emotion_publisher = self.create_publisher(Emotion, 'system/perception/sensor/emotion', 10)

#         # Create a timer to publish the emotion message
#         self.timer = self.create_timer(1.0, self.publish_emotion)

#     def publish_emotion(self, emotion: int):
#         msg = Emotion()
#         msg.emotion = emotion
#         self.emotion_publisher.publish(msg)


# def main(args=None):
#     rclpy.init(args=args)
#     vision_emotion_node = VisionEmotionNode()
#     rclpy.spin(vision_emotion_node)
#     vision_emotion_node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()