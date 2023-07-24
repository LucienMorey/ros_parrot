#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__("mypublisher")
        self.first_joint_msg = None

        self.mypublisher = self.create_publisher(
            JointState, "/joint_commands", 10)
        
        my_qos_profile = QoSProfile(depth=10)
        my_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.jointsubscriber = self.create_subscription(JointState, "/joint_states",self.joint_callback,qos_profile = my_qos_profile)
        self.publisher_timer_ = self.create_timer(
            2.0, self.publish_joint_commands)
        
    def joint_callback(self,msg: JointState):
        if self.first_joint_msg is None: 
            self.first_joint_msg = msg
            print('Set first_joint_msg to: {}'.format(self.first_joint_msg.position))


    def publish_joint_commands(self):
        if self.first_joint_msg is None:
            print('Havent received subscriber message yet, skipping.')
            return
            
        msg = JointState()
        BODY_INDEX_OFFSET = 6

        # construct joint command
        msg.effort = [0.]*14
        msg.effort[0] = 0. # t-motor
        msg.effort[1] = 0. # t-motor 
        msg.effort[2] = self.first_joint_msg.position[2+BODY_INDEX_OFFSET]
        msg.effort[3] = self.first_joint_msg.position[3+BODY_INDEX_OFFSET]
        msg.effort[4] = 0. # t-motor
        msg.effort[5] = 0. # t-motor
        msg.effort[6] = self.first_joint_msg.position[6+BODY_INDEX_OFFSET]
        msg.effort[7] = 0*self.first_joint_msg.position[7+BODY_INDEX_OFFSET]
        msg.effort[8] = 0. # t-motor
        msg.effort[9] = 0. # t-motor
        msg.effort[10] = 0. # t-motor
        msg.effort[11] = self.first_joint_msg.position[11+BODY_INDEX_OFFSET]
        msg.effort[12] = self.first_joint_msg.position[12+BODY_INDEX_OFFSET]
        msg.effort[13] = 0. # odrive in torque mode

        print(msg.effort)
        self.mypublisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointCommandPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()