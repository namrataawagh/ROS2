#!/usr/bin/env python3  # Use Python 3 to execute this script

import rclpy  # Import ROS 2 Python library
from rclpy.node import Node  # Import the base class for creating ROS 2 nodes

class MyCustomNode(Node):  # Define a custom node class
    def __init__(self):
        super().__init__("my_node")  # Set the node name
        self.get_logger().info("Hello from MyCustomNode!")  # Log a startup message

def main(args=None):  # Entry point for the node
    rclpy.init(args=args)  # Initialize ROS 2 Python client library
    node = MyCustomNode()  # Create node instance
    rclpy.spin(node)  # Keep the node alive to handle callbacks
    node.destroy_node()  # Cleanup: destroy the node
    rclpy.shutdown()  # Shutdown ROS 2

if __name__ == '__main__':
    main()
