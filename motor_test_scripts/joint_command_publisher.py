#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__("mypublisher")
        self.mypublisher = self.create_publisher(
            JointState, "/joint_commands", 10)
        self.temperature_timer_ = self.create_timer(
            2.0, self.publish_joint_commands)

    def publish_joint_commands(self):
        msg = JointState()
        msg.effort = [0.]*14
        # msg.effort[3] = math.pi/4 # radians
        msg.effort[3] = -3.15
        print(msg.effort)
        self.mypublisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointCommandPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()