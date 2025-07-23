#!/usr/bin/env python3  # Use Python 3 to execute this script

import rclpy  # Import ROS 2 Python library
from rclpy.node import Node  # Import the base class for creating ROS 2 nodes

class MyCustomNode(Node):  # Define a custom node class inheriting from Node
    def __init__(self):
        super().__init__("my_node")  # Call base class constructor and set node name
        self.get_logger().info("Hello from MyCustomNode!.")  # Log a message

def main(args=None):  # Define the main function to start the node
    rclpy.init(args=args)  # Initialize ROS 2 Python client library
    node = MyCustomNode()  # Create an instance of your node
    rclpy.spin(node)  # Keep node running, waiting for callbacks
    node.destroy_node()  # Cleanly destroy the node before shutting down
    rclpy.shutdown()  # Shut down the ROS 2 client library

if __name__ == '__main__':  # Check if this script is run directly
    main()  # Call the main function
