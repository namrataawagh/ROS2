#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class PublisherNode(Node):
   def __init__(self):
       super().__init__("publisher_node")
       self.publisher = self.create_publisher(Int64,"number",10)
       self.number = 2
       self.timer = self.create_timer(1.0,self.publish_number)
       self.get_logger().info("number publisher has been published")

   def publish_number(self):
       msg = Int64()
       msg.data = self.number
       self.publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()
