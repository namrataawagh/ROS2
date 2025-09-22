#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen


class SetpenClientNode(Node):
    def __init__(self):
        super().__init__("turtle_setpen_client")
        self.client = self.create_client(SetPen,"/turtle1/set_pen")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for service....")

        self.set_pen()

    def set_pen(self):
        request = SetPen.Request()
        request.r = 255
        request.g = 0
        request.b = 0
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self,future):
        self.get_logger().info("Successfully changed pen color")

def main(args=None):
    rclpy.init(args=args)
    node = SetpenClientNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

