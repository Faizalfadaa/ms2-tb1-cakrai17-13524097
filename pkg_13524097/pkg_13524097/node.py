#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import threading
from queue import PriorityQueue


class NodeTemplate(Node):
    def __init__(self):
        super().__init__('node')
       
        # ==============================================
        # PARAMETER DECLARATION
        # ==============================================

        # Declare parameters for topic names 
        self.declare_parameter('cmd_type_topic', 'cmd_type')             # Publisher
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')               # Publisher
        self.declare_parameter('autonomous_vel_topic', 'autonomous_vel') # Subscriber
        self.declare_parameter('publish_rate', 5.0)                      # 5 message / second (5 Hz)

        # Spesifikasi Bonus 2
        self.declare_parameter('joy_vel_topic', 'joy_vel')
        self.declare_parameter('keyboard_vel_topic', 'keyboard_vel')

        # ==============================================
        # PARAMETER RETRIEVAL
        # ==============================================

        # Get parameter values
        cmd_type_topic = self.get_parameter('cmd_type_topic').get_parameter_value().string_value 
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        autonomous_vel_topic = self.get_parameter('autonomous_vel_topic').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value 
        # self.get_logger().info(f'Publish rate: {publish_rate}')

        # Spesifikasi Bonus 2
        joy_vel_topic = self.get_parameter('joy_vel_topic').get_parameter_value().string_value
        keyboard_vel_topic = self.get_parameter('keyboard_vel_topic').get_parameter_value().string_value
        self.lock = threading.Lock()
        self.pq = PriorityQueue() 

        # ==============================================
        # PUBLISHERS
        # ==============================================

        # Create Publishers
        # 1. Publisher cmd_type_topic
        self.cmd_type_pub = self.create_publisher(
            String,            # Message type
            cmd_type_topic,    # Topic name
            10                 # Queue size
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
            Twist,                         # Message type
            autonomous_vel_topic,          # Topic name
            self.autonomous_vel_callback,  # Callback function
            10                             # Queue size
        )

        # Spesifikasi Bonus 2
        # 2. Subscriber joy_vel_topic
        self.joy_vel_sub = self.create_subscription(
            Twist,
            joy_vel_topic,
            self.joy_vel_callback,
            10

        )
        # 3. Subscriber keyboard_vel_topic
        self.keyboard_vel_sub = self.create_subscription(
            Twist,
            keyboard_vel_topic,
            self.keyboard_vel_callback,
            10
        )
       
        # ==============================================
        # TIMER
        # ==============================================

        # Create Timer
        self.timer = self.create_timer(
            1.0 / publish_rate,  # Timer period (seconds)
            self.timer_callback  # Callback function
        )

        # Initialize the current autonomous_vel
        self.current_autonomous_vel = Twist()
   
    # ==================================================
    # SUBSCRIBER CALLBACK
    # ==================================================

    # Create Subscriber Callback Function
    def autonomous_vel_callback(self, msg):
        """Callback for autonomous_vel messages"""
        with self.lock:
            self.current_cmd_vel = msg
            self.current_cmd_type = "autonomous"
    
    # Spesifikasi Bonus 2
    def joy_vel_callback(self, msg):
        """Callback for joy_vel messages"""
        with self.lock:
            self.current_cmd_vel = msg
            self.current_cmd_type = "joy"
    
    def keyboard_vel_callback(self, msg):
        """Callback for keyboard_vel messages"""
        with self.lock:
            self.current_cmd_vel = msg
            self.current_cmd_type = "keyboard"
   
    # ==================================================
    # TIMER CALLBACK
    # ==================================================

    # Create Timer Callabck Function
    def timer_callback(self):
        """Timer callback for publish"""
        msg_type = ["keyboard", "joy", "autonomous"]

        if self.current_cmd_type == msg_type[0]:
            self.pq.put((1, msg_type[0]))
        elif self.current_cmd_type == msg_type[1]:
            self.pq.put((2, msg_type[1]))
        elif self.current_cmd_type == msg_type[2]:
            self.pq.put((3, msg_type[2]))

        with self.lock:
            self.current_cmd_type = self.pq.get()[1]
            msg = String()
            msg.data = self.current_cmd_type
            self.cmd_type_pub.publish(msg)
            self.cmd_vel_pub.publish(self.current_cmd_vel)

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