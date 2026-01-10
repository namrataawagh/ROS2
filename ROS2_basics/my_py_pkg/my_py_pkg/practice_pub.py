#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class PracticePublisher(Node):
    def __init__(self):
        super().__init__("practice_pub")
        self.publisher = self.create_publisher(String,"practice_topic",10)
        self.counter_ = 0
        self.timer = self.create_timer(1.0,self.publish_news)
        self.get_logger().info("This publisher has started ")


    def publish_news(self):
        msg = String()
        msg.data = " Hello " + str(self.counter_)
        self.publisher.publish(msg)
        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args)
    node = PracticePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()



