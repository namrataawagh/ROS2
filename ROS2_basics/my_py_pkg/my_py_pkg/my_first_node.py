#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):                # inherit ftom the node class that we imported from rclpy

    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("Hello World")
        self.counter_ = 0
        self.create_timer(1.0,self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello! " + str(self.counter_))
        self.counter_ +=1

def main(args=None):
    rclpy.init(args=args)   # initialize 
    node = MyNode("py_test")
    rclpy.spin(node)  # spin keeps the node running and not exit until you press ctrl+c
    rclpy.shutdown()  # Last line in every ros node

if __name__ == "__main__":
    main()