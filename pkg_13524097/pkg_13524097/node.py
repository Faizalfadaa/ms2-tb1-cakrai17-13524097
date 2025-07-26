!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class NodeTemplate(Node):
    def __init__(self):
        super().__init__('node_template')
       
        # ==============================================
        # PARAMETER DECLARATION
        # ==============================================
        self.declare_parameter('param_name', 'default_value')
        self.declare_parameter('publish_rate', 1.0)
       
        # ==============================================
        # PARAMETER RETRIEVAL
        # ==============================================
        self.param_value = self.get_parameter('param_name').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
       
        # ==============================================
        # PUBLISHERS
        # ==============================================
        self.publisher = self.create_publisher(
            Twist,           # Message type
            'topic_name',    # Topic name
            10               # Queue size
        )
       
        # ==============================================
        # SUBSCRIBERS
        # ==============================================
        self.subscriber = self.create_subscription(
            String,              # Message type
            'input_topic',       # Topic name
            self.subscriber_callback,  # Callback function
            10                   # Queue size
        )
       
        # ==============================================
        # TIMER
        # ==============================================
        self.timer = self.create_timer(
            1.0 / publish_rate,  # Timer period (seconds)
            self.timer_callback  # Callback function
        )
       
        self.get_logger().info('Node Template started!')
   
    # ==================================================
    # SUBSCRIBER CALLBACK
    # ==================================================
    def subscriber_callback(self, msg):
        """Callback for subscriber"""
        # TODO: Implement subscriber logic
        pass
   
    # ==================================================
    # TIMER CALLBACK
    # ==================================================
    def timer_callback(self):
        """Callback for timer"""
        # TODO: Implement timer logic
        pass

def main(args=None):
    rclpy.init(args=args)
    node = NodeTemplate()
   
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
