#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServerNode(Node):
   def __init__(self):
       super().__init__("server_node")
       self.server = self.create_service(AddTwoInts,"add_int",self.add_int_callback)
       self.get_logger().info("service is ready: 'add two integers'")
       
   def add_int_callback(self,request,response):
       response.sum = request.a + request.b
       self.get_logger().info(f"request a ={request.a} + b = {request.b} = sum{response.sum}")
       return response
       



def main(args=None):
    rclpy.init(args=args)
    node = ServerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()
