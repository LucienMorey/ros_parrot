#!/usr/bin/env python3

import operator
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
# from rclpy.qos_overriding_options import QoSOverridingOptions
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger
from std_msgs.msg import (
    Float32MultiArray,
    MultiArrayLayout,
    MultiArrayDimension,
)
from sensor_msgs.msg import JointState
from parody_control.parody import Parody
from parody_msgs.srv import MotorTrigger
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy

import transforms3d
import numpy as np

import time

CONTROL_FREQ_HZ = 150

class ParodyRosWrapper(Node):
    BODY_JOINT_OFFSET: int = 6

    def __init__(self) -> None:
        # Init Node
        super().__init__(node_name="parody_controller")

        # Create Robot
        self.robot = Parody(CONTROL_FREQ_HZ)

        # Create Zero Service
        self.zero_joint_service = self.create_service(
            MotorTrigger, "zero_joint", self.zero_joint_service_callback
        )

        self.find_motor_index_service = self.create_service(
            MotorTrigger, "find_motor_index", self.index_search_motor_service_callback
        )

        self.arm_motors_service = self.create_service(
            Trigger, "arm_motors", self.arm_motors_service_callback
        )
        self.disarm_motors_service = self.create_service(
            Trigger, "disarm_motors", self.disarm_motors_service_callback
        )

        # TODO create publisher for reported bus voltages
        voltage_dimension = MultiArrayDimension()
        voltage_dimension.size = 14
        voltage_dimension.stride = 14
        voltage_dimension.label = "length"

        voltage_layout = MultiArrayLayout()
        voltage_layout.data_offset = 0
        voltage_layout.dim.append(voltage_dimension)

        self.bus_voltages_msg = Float32MultiArray()
        self.bus_voltages_msg.layout = voltage_layout
        self.bus_voltages_msg.data = [-1.0] * voltage_dimension.size

        self.bus_voltage_pub = self.create_publisher(
            Float32MultiArray, "bus_voltages", 1
        )
        self.bus_voltage_pub_timer = self.create_timer(1/CONTROL_FREQ_HZ, self.publish_bus_voltages)

        my_qos_profile = QoSProfile(depth=10)
        my_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        
        # Create Joint State Pub
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", qos_profile=my_qos_profile)
        self.joint_state_pub_timer = self.create_timer(1/CONTROL_FREQ_HZ, self.publish_joint_states)

        self.raw_encoder_positions_pub = self.create_publisher(JointState, "raw_encoder_positions", qos_profile=my_qos_profile)
        self.raw_encoder_positions_pub_timer = self.create_timer(0.1, self.publish_raw_encoder_positions)

        # Create Joint Command Sub
        self.joint_command_sub = self.create_subscription(
            JointState, "joint_commands", self.joint_command_callback, qos_profile=my_qos_profile
        )

        # Create vicon sub
        self.vicon_sub = self.create_subscription(
            TransformStamped, "vicon/robot/robot", self.vicon_callback, 10
        )

        self.joint_states_msg = JointState()
        self.raw_encoder_positions_msg = JointState()
        # TODO set header for joint states if required
        self.joint_states_msg.name.append("bodyroll")
        self.joint_states_msg.name.append("bodypitch")
        self.joint_states_msg.name.append("bodyyaw")
        self.joint_states_msg.name.append("bodyx")
        self.joint_states_msg.name.append("bodyy")
        self.joint_states_msg.name.append("bodyz")
        self.joint_states_msg.name.append("leftShoulder")
        self.joint_states_msg.name.append("leftElbow")
        self.joint_states_msg.name.append("leftWrist1")
        self.joint_states_msg.name.append("leftWrist2")

        self.joint_states_msg.name.append("rightShoulder")
        self.joint_states_msg.name.append("rightElbow")
        self.joint_states_msg.name.append("rightWrist1")
        self.joint_states_msg.name.append("rightWrist2")

        self.joint_states_msg.name.append("neckShoulder1")
        self.joint_states_msg.name.append("neckShoulder2")
        self.joint_states_msg.name.append("neckElbow")
        self.joint_states_msg.name.append("neckWrist1")
        self.joint_states_msg.name.append("neckWrist2")

        self.joint_states_msg.name.append("tail")
        self.joint_states_msg.position = [0.0] * len(self.joint_states_msg.name)
        self.joint_states_msg.velocity = [0.0] * len(self.joint_states_msg.name)
        self.joint_states_msg.effort = [0.0] * len(self.joint_states_msg.name)

        self.raw_encoder_positions_msg.name.append("leftWrist1")
        self.raw_encoder_positions_msg.name.append("leftWrist2")
        self.raw_encoder_positions_msg.name.append("RightWrist1")
        self.raw_encoder_positions_msg.name.append("RightWrist2")
        self.raw_encoder_positions_msg.name.append("NeckWrist1")
        self.raw_encoder_positions_msg.name.append("NeckWrist2")
        self.raw_encoder_positions_msg.name.append("tail")
        self.raw_encoder_positions_msg.position = [0.0] * len(self.raw_encoder_positions_msg.name)


        # initialize body transform
        self.body_transform = None
        self.body_transform_prev = None
        self.body_vel = None

        # disarm motors on startup
        disarmed = self.robot.disarm_motors()
        if disarmed:
            print("Motors disarmed.")

    def arm_motors_service_callback(
        self, req: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        armed = self.robot.arm_motors()
        if armed:
            print("Motors armed.")

        resp.success = armed
        return resp

    def disarm_motors_service_callback(
        self, srv: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        disarmed = self.robot.disarm_motors()
        if disarmed:
            print("Motors disarmed.")

        resp.success = disarmed
        return resp

    def zero_joint_service_callback(
        self, srv: MotorTrigger.Request, resp: MotorTrigger.Response
    ) -> MotorTrigger.Response:
        success = self.robot.zero_motor(srv.joint_number)

        # TODO decide what feedback is helpful here
        resp.success = success
        if not resp.success:
            resp.message = "failed to zero motor. check print out from node"
        else:
            resp.message = "zeroed motor"
        return resp

    def index_search_motor_service_callback(
        self, srv: MotorTrigger.Request, resp: MotorTrigger.Response
    ) -> MotorTrigger.Response:
        success = self.robot.find_motor_index_pulse(srv.joint_number)

        # # TODO decide what feedback is helpful here
        resp.success = success
        if not resp.success:
            resp.message = "failed to get index pulse. check print out from node"
        else:
            resp.message = "found index pulse"
            if self.robot.check_all_indexes_found():
                resp.message += ".all index pulses have now been found"
                print("all index pulses have now been found")
        return resp

    def publish_joint_states(self) -> None:
        # get joint states
        joint_states = self.robot.get_states()

        # sometimes member variables doesn't exist yet? maybe async timer callback fires before __init__() finishes?
        try:
        #     populate body transform
        # if (
            # True
            # self.body_transform and self.body_vel
        # ):  # there'll be one timestep where vel will be empty, but whatever
            for index in range(self.BODY_JOINT_OFFSET):
                self.joint_states_msg.position[index] = self.body_transform[index]
                self.joint_states_msg.velocity[index] = self.body_vel[index]
        except:
            # print('Couldnt populate body pos/vel in /joint_states publication message!')
            pass

        # convert to msg type
        for index, joint_state in enumerate(joint_states):
            index += self.BODY_JOINT_OFFSET
            self.joint_states_msg.position[index] = joint_state[0]
            self.joint_states_msg.velocity[index] = joint_state[1]
            self.joint_states_msg.effort[index] = joint_state[2]

        # publish data
        self.joint_states_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_pub.publish(self.joint_states_msg)

    def publish_raw_encoder_positions(self) -> None:
        self.raw_encoder_positions_msg.position[0] = self.robot.leftWrist1.get_raw_position()
        self.raw_encoder_positions_msg.position[1] = self.robot.leftWrist2.get_raw_position()
        self.raw_encoder_positions_msg.position[2] = self.robot.rightWrist1.get_raw_position()
        self.raw_encoder_positions_msg.position[3] = self.robot.rightWrist2.get_raw_position()
        self.raw_encoder_positions_msg.position[4] = self.robot.neckWrist1.get_raw_position()
        self.raw_encoder_positions_msg.position[5] = self.robot.neckWrist2.get_raw_position()
        self.raw_encoder_positions_msg.position[6] = self.robot.tail.get_raw_position()

        self.raw_encoder_positions_pub.publish(self.raw_encoder_positions_msg)

    def joint_command_callback(self, joint_command: JointState) -> None:
        # set torques
        if self.robot.check_all_indexes_found():
            self.robot.set_commands(joint_command.effort)
        else:
            print(
                "Not all odrives have found their index. Ignoring command"
            )

    def shutdown(self) -> None:
        # set hold current position or jsut disable motors
        self.robot.disarm_motors()

    def publish_bus_voltages(self) -> None:
        voltages = self.robot.get_reported_bus_voltages()

        for index, voltage in enumerate(voltages):
            self.bus_voltages_msg.data[index] = voltage

        self.bus_voltage_pub.publish(self.bus_voltages_msg)

    def vicon_callback(self, msg: TransformStamped) -> None:
        # expects vicon frame to be x-up, y-left, z-out-of-plane, in mm
        # our coordinate system is x-up, y-right, z-into-plane, in m
        

        # populate body transform rotation
        q_vicon = [
            msg.transform.rotation.w,
            msg.transform.rotation.x,
            msg.transform.rotation.y,
            msg.transform.rotation.z,
        ]
        # print(q_vicon)
        R_vicon = transforms3d.quaternions.quat2mat(q_vicon)
        Rrel = np.array([[1, 0, 0],[0, -1, 0],[0, 0, -1]]) # vicon and our frame differ by 180 degree rotation about x-axis
        R = Rrel@R_vicon # body orientation in our frame
        roll, pitch, yaw = transforms3d.euler.mat2euler(R)

        # Note: y,z are flipped from vicon coordinate system, which is z-up
        self.body_transform = np.array([
            roll,
            pitch,
            yaw,
            msg.transform.translation.x*0.001, # position reported by vicon in mm
            -msg.transform.translation.y*0.001, # y-axis of vicon frame points to the left, our y-axis points to the right
            -msg.transform.translation.z*0.001,
        ])

        # update velocities
        if self.body_transform_prev is not None:
            # # "self.body_transform - self.body_transform_prev"
            # self.body_vel = list(
            #     map(operator.sub, self.body_transform, self.body_transform_prev)
            # )*CONTROL_FREQ_HZ
            self.body_vel = (self.body_transform - self.body_transform_prev)*CONTROL_FREQ_HZ
            # print(self.body_vel)

        else:
            # print('Couldnt find body_transform_prev')
            pass

        # store previous timestep's body transform
        self.body_transform_prev = self.body_transform


def main(args=None):
    rclpy.init(args=args)

    parody_ros_wrapper = ParodyRosWrapper()

    try:
        rclpy.spin(parody_ros_wrapper)
    except SystemExit:
        pass

    parody_ros_wrapper.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
