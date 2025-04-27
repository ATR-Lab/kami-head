# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node

# class VisionFacePositionNode(Node):
#     def __init__(self):
#         super().__init__('vision_face_position_node')
#         self.get_logger().info("Vision face position node initialized")

#         # Create a publisher for the face position topic
#         self.face_position_publisher = self.create_publisher(FacePosition, 'system/perception/sensor/face_position', 10)

#         # Create a timer to publish the face position message
#         self.timer = self.create_timer(1.0, self.publish_face_position)

#     def publish_face_position(self, face_position: int):
#         msg = FacePosition()
#         msg.face_position = face_position
#         self.face_position_publisher.publish(msg)


# def main(args=None):
#     rclpy.init(args=args)
#     vision_face_position_node = VisionFacePositionNode()
#     rclpy.spin(vision_face_position_node)
#     vision_face_position_node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()