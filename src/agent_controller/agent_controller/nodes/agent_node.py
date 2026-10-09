from rclpy.node import Node


class AgentNode():
    def __init__(self):
        super().__init__('main_agent_node')
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("AGENTIC NODE HAS BEEN STARTED")

    def timer_callback(self):
        self.get_logger().info("Agent Node is transmitting data...")