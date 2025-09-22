#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ClientNode(Node):
   def __init__(self):
       super().__init__("client_node")
       self.client = self.create_client(AddTwoInts,"add_int")
       while not self.client.wait_for_service(timeout_sec= 1.0):
            self.get_logger().warn("waiting for the service server...")

       self.send_request()

   def send_request(self):
       request = AddTwoInts.Request()
       request.a = 10
       request.b = 5 
       future = self.client.call_async(request)
       future.add_done_callback(self.handle_response) 

   def handle_response(self, future):
    response = future.result()
    self.get_logger().info(f"result: {response.sum}")

       
           

       


def main(args=None):
    rclpy.init(args=args)
    node = ClientNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()
