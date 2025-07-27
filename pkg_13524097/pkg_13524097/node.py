!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class NodeTemplate(Node):
    def __init__(self):
        super().__init__('new_node')
       
        # ==============================================
        # PARAMETER DECLARATION
        # ==============================================
        # self.declare_parameter('param_name', 'default_value')

        # Declare parameters for topic names 
        self.declare_parameter('cmd_type_topic', 'cmd_type') # Publisher
        self.declare_parameter('cmd_vel_topic', 'cmd_vel') # Publisher
        self.declare_parameter('autonomous_vel_topic', 'autonomous_vel') # Subscriber
        self.declare_parameter('publish_rate', 5) # 5 message / second (5 Hz)

        # ==============================================
        # PARAMETER RETRIEVAL
        # ==============================================
        # self.param_value = self.get_parameter('param_name').get_parameter_value().string_value

        # Get parameter values
        cmd_type_topic = self.get_parameter('cmd_type_topic').get_parameter_value().string_value 
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        autonomous_vel_topic = self.get_parameter('autonomous_vel_topic').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value 
        
        # ==============================================
        # PUBLISHERS
        # ==============================================

        # Create Publishers
        # 1. Publisher cmd_type_topic
        self.cmd_type_pub = self.create_publisher(
            String,           # Message type
            cmd_type_topic,    # Topic name
            10               # Queue size
        )

        # 2. Publisher cmd_vel_topic
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10
        )
       
        # ==============================================
        # SUBSCRIBERS
        # ==============================================

        # Create Subscribers
        # 1. Subscriber autonomous_vel_topic
        self.autonomous_vel_sub = self.create_subscription(
            Twist,              # Message type
            autonomous_vel_topic,       # Topic name
            self.autonomous_vel_callback,  # Callback function
            10                   # Queue size
        )
       
        # ==============================================
        # TIMER
        # ==============================================

        # Create Timer
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
