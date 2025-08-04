#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import math

class FKPublisher(Node):
    def __init__(self):
        super().__init__('fk_publisher')
        self.sub = self.create_subscription(JointState, '/joint_states', self.callback_function, 1)
        self.pub = self.create_publisher(Point, '/end_effector_position', 1)
        self.l1 = 2.0
        self.l2 = 1.5

    def callback_function(self, msg):
        theta1 = msg.position[0] + 3.14159 / 2  
        theta2 = msg.position[1]

        x = self.l1 * math.cos(theta1) + self.l2 * math.cos(theta1 + theta2)
        y = self.l1 * math.sin(theta1) + self.l2 * math.sin(theta1 + theta2)

        point = Point()
        point.x = x
        point.y = y
        point.z = 0.0 

        self.pub.publish(point)

def main(args=None):
    rclpy.init(args=args)
    node = FKPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

