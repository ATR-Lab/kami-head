#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node

# class LLMSensorNode(Node):
#     def __init__(self):
#         super().__init__('llm_sensor_node')
#         self.get_logger().info("LLM sensor node initialized")

#         # Subscribe to voice_intent_node topic
#         self.voice_intent_subscriber = self.create_subscription(
#             'system/perception/sensor/voice/intent/classifier',
#             self.voice_intent_callback,
#             10
#         )

#     def voice_intent_callback(self, msg: IntentClassification):
#         self.get_logger().info(f"Received voice intent: {msg.intent}")

# def main(args=None):
#     rclpy.init(args=args)
#     llm_sensor_node = LLMSensorNode()
#     rclpy.spin(llm_sensor_node)
#     llm_sensor_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()