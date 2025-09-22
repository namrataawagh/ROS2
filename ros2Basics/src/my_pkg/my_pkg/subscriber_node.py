#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class SubscriberNode(Node):
   def __init__(self):
       super().__init__("subscriber_node")
       self.subscriber = self.create_subscription(Int64,"number",self.callback_number,10)
       self.counter = 0
       self.get_logger().info("number counter has been started")

   def callback_number(self, msg: Int64):
    self.counter = msg.data
    self.get_logger().info("Counter: " + str(self.counter))




def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()
