#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class PracticeSubscriber(Node):
    def __init__(self):
        super().__init__("practice_sub")
        self.subscriber = self.create_subscription(String,'practice_topic',
                                                   self.callback_practice_pub,10)

    def callback_practice_pub(self,msg:String):
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = PracticeSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

