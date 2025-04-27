#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from coffee_expressions_msgs.msg import AffectiveState
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time
from typing import Dict, Optional, Set
from shared_configs.shared_configs import (
    EMOTION_TOPIC,
    FACE_POSITION_TOPIC,
    VOICE_INTENT_TOPIC,
    EVENT,
    STATE_MANAGER_AFFECTIVE_STATE_TOPIC,
    DIAGNOSTICS
)


class StateManagerNode(Node):
    # Valid expressions that the robot can display
    VALID_EXPRESSIONS: Set[str] = {
        "Neutral", "Happy", "Sad", "Angry", "Surprised",
        "Loving", "Curious", "Excited", "Rejoice"
    }

    # Mapping of voice intents to expressions
    INTENT_TO_EXPRESSION: Dict[str, str] = {
        "Agree": "Happy",
        "Disagree": "Angry",
        "Joke": "Loving",
        "Question": "Curious",
        "None": "Neutral"
    }

    # Mapping of system events to expressions with their override durations
    EVENT_TO_EXPRESSION: Dict[str, tuple] = {
        "PaymentSuccess": ("Rejoice", 3.0),
        "PaymentFailed": ("Sad", 3.0),
        "CoffeeReady": ("Excited", 3.0),
        "CoffeeError": ("Angry", 3.0)
    }

    def __init__(self):
        super().__init__('state_manager_node')

        # Load parameters with default values
        self.declare_parameter('idle_timeout', 5.0)
        self.declare_parameter('publish_rate', 0.01)  # 10Hz
        self.declare_parameter('default_expression', 'Neutral')

        # Internal state
        self._base_expression = self.get_parameter('default_expression').value
        self._last_voice_intent = "None"
        self._last_face_position = Point(x=0.0, y=0.0, z=1.0)
        self._last_face_position_v2 = String()
        self._override_expression: Optional[str] = None
        self._override_reason: Optional[str] = None
        self._override_expire_time: Optional[float] = None
        self._last_active_time = time.time()
        self._idle_timeout = self.get_parameter('idle_timeout').value

        # QoS profile for reliable message delivery
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Subscribers
        self.create_subscription(
            String, EMOTION_TOPIC, self.vision_callback, qos)
        self.create_subscription(
            String, VOICE_INTENT_TOPIC, self.voice_callback, qos)
        self.create_subscription(
            String, FACE_POSITION_TOPIC, self.face_position_callback_v2, qos)

        self.create_subscription(
            String, EVENT, self.event_callback, qos)

        # Publishers
        self.state_pub = self.create_publisher(
            AffectiveState, STATE_MANAGER_AFFECTIVE_STATE_TOPIC, qos)
        self.diagnostics_pub = self.create_publisher(
            String, DIAGNOSTICS, qos)

        # Main loop timer
        period = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(period, self.publish_state)

        self.get_logger().info("State Manager Node initialized")
        self._publish_diagnostics("Node initialized with idle_timeout="
                                f"{self._idle_timeout}s, rate={1/period}Hz")

    def _validate_expression(self, expression: str) -> str:
        """Validate and return a safe expression."""
        if expression not in self.VALID_EXPRESSIONS:
            self.get_logger().warn(
                f"Invalid expression '{expression}', falling back to Neutral")
            return "Neutral"
        return expression

    def _publish_diagnostics(self, message: str):
        """Publish diagnostic information."""
        msg = String()
        msg.data = f"[{time.strftime('%H:%M:%S')}] {message}"
        self.diagnostics_pub.publish(msg)

    def vision_callback(self, msg: String):
        """Handle incoming emotion detection."""
        expression = self._validate_expression(msg.data)
        if expression != self._base_expression:
            self._base_expression = expression
            self._publish_diagnostics(f"Vision update: {expression}")
        self._last_active_time = time.time()

    def voice_callback(self, msg: String):
        """Handle incoming voice intents."""
        intent = msg.data
        if intent in self.INTENT_TO_EXPRESSION:
            self._base_expression = self.INTENT_TO_EXPRESSION[intent]
            self._publish_diagnostics(
                f"Voice intent: {intent} → {self._base_expression}")
        self._last_active_time = time.time()

    def face_position_callback(self, msg: Point):
        """Handle incoming face position updates."""
        self._last_face_position = msg
        self._last_active_time = time.time()
    
    def face_position_callback_v2(self, msg: String):
        """Handle incoming face position updates."""
        self._last_face_position_v2.data = msg.data
        self._last_active_time = time.time()

    def event_callback(self, msg: String):
        """Handle incoming system events."""
        event = msg.data
        if event in self.EVENT_TO_EXPRESSION:
            expression, duration = self.EVENT_TO_EXPRESSION[event]
            self._override_expression = expression
            self._override_reason = event
            self._override_expire_time = time.time() + duration
            self._last_active_time = time.time()
            self._publish_diagnostics(
                f"Event override: {event} → {expression} for {duration}s")

    def publish_state(self):
        """Publish the current affective state."""
        current_time = time.time()

        # Determine if an override is still active
        if (self._override_expression and 
            current_time < self._override_expire_time):
            expression = self._override_expression
            trigger_source = self._override_reason
        else:
            # Clear override if expired
            if self._override_expression is not None:
                self._publish_diagnostics(
                    f"Override expired: {self._override_expression}")
                self._override_expression = None
                self._override_reason = None
                self._override_expire_time = None
            expression = self._base_expression
            trigger_source = "perception"

        # Determine if robot is idle
        is_idle = (current_time - self._last_active_time > self._idle_timeout)
        if is_idle:
            expression = "Neutral"
            trigger_source = "idle"

        # Publish affective state
        msg = AffectiveState()
        msg.expression = expression
        msg.trigger_source = trigger_source
        msg.gaze_target = self._last_face_position
        msg.gaze_target_v2 = self._last_face_position_v2.data
        msg.is_idle = is_idle

        self.state_pub.publish(msg)
        self.get_logger().debug(
            f"State: {expression} ({trigger_source}), "
            f"idle={is_idle}, gaze=({msg.gaze_target.x:.2f}, "
            f"{msg.gaze_target.y:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = StateManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
