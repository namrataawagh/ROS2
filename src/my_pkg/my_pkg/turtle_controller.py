#!/usr/bin/env python3


import rclpy  
from rclpy.node import Node  
from geometry_msgs.msg import Twist

class TurtleController(Node):  
    def __init__(self):
        super().__init__("turtle_controller")  
        self.turtle = self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.timer = self.create_timer(1.0,self.cmd_vel_call)

    def cmd_vel_call(self):
        cmd = Twist()
        cmd.linear.x = 4.0
        cmd.angular.z = 2.0
        self.turtle.publish(cmd)

def main(args=None):  
    rclpy.init(args=args)  
    node = TurtleController()  
    rclpy.spin(node)  
    node.destroy_node() 
    rclpy.shutdown()  

if __name__ == '__main__':
    main()
