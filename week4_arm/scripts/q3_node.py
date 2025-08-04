#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
import math
import threading

class RandomNode(Node):
    def __init__(self):
        super().__init__('random_ass_node')
        self.sub = self.create_subscription(Point, '/end_effector_position', self.callback_function, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/joint_angles_goal', 10)

        self.l1 = 2.0
        self.l2 = 1.5
        self.current_point = None

        # Start input thread
        self.input_thread = threading.Thread(target=self.get_user_input)
        self.input_thread.daemon = True
        self.input_thread.start()

    def callback_function(self, msg):
        self.current_point = (msg.x, msg.y)
        
    def get_user_input(self):
        while rclpy.ok():
            if self.current_point is None:
                continue 

            user_direction = input("Enter the direction in which you want to move (x/y): ")

            user_distance = float(input("Enter the particular distance in that direction (max 0.5): "))
            
            if user_distance > 0.5:
                self.get_logger().warn("Distance too large. Max is 0.5.")
                continue

            x, y = self.current_point
            if user_direction == 'x':
                x += user_distance
            else:
                y += user_distance

            self.get_logger().info(f"New target position: ({x:.2f}, {y:.2f})")

            result = self.ik_function(x, y)
            if result is None:
                self.get_logger().warn("Target is unreachable.")
                continue

            theta1, theta2 = result
            msg = Float64MultiArray()
            msg.data = [theta1, theta2]
            self.pub.publish(msg)
            self.get_logger().info(f"Published θ1={theta1:.3f}, θ2={theta2:.3f}")

    def ik_function(self, x, y):
        D = (x**2 + y**2 - self.l1**2 - self.l2**2)/(2 * self.l1 * self.l2)
        
        if abs(D) > 1.0:
            return None
        theta2 = math.atan2(math.sqrt(1 - D**2), D)
        theta1 = math.atan2(y, x) - math.atan2(
            self.l2 * math.sin(theta2),
            self.l1 + self.l2 * math.cos(theta2)
        )
        return theta1, theta2

def main(args=None):
    rclpy.init(args=args)
    node = RandomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


