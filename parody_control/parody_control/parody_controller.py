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

import time


class ParodyRosWrapper(Node):
    BODY_JOINT_OFFSET: int = 6

    def __init__(self) -> None:
        # Init Node
        super().__init__(node_name="parody_controller")

        # Create Robot
        self.robot = Parody()

        # Create Zero Service
        self.zero_all_service = self.create_service(
            Trigger, "zero_all_joints", self.zero_all_service_callback
        )

        self.zero_joint_service = self.create_service(
            MotorTrigger, "zero_joint", self.zero_joint_service_callback
        )

        self.find_all_index_service = self.create_service(
            Trigger, "find_all_index", self.index_search_all_service_callback
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
        self.bus_voltage_pub_timer = self.create_timer(0.01, self.publish_bus_voltages)

        # Clyde fucking around trying to figure out QoS
        #qos_override =  QoSProfile(ReliabilityPolicy.BEST_EFFORT) #<- what to put in here????
        my_qos_profile = QoSProfile(depth=10)
        my_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        
        # qos_override = _rclpy.rmw_qos_profile_t.predefined('qos_profile_sensor_data').to_dict()

        # Create Joint State Pub
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", qos_profile=my_qos_profile)
        self.joint_state_pub_timer = self.create_timer(0.01, self.publish_joint_states)

        # initialise watchdog
        self.watchdog_count = 0
        self.watchdog_timer = self.create_timer(0.01, self.watchdog_callback)

        # Create Joint Command Sub
        self.joint_command_sub = self.create_subscription(
            JointState, "joint_commands", self.joint_command_callback, qos_profile=my_qos_profile
        )

        # Create vicon sub
        self.vicon_sub = self.create_subscription(
            TransformStamped, "vicon_sub", self.vicon_callback, 10
        )

        self.joint_states_msg = JointState()
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

        # initialize body transform
        self.body_transform = None
        self.body_transform_prev = None
        self.body_vel = None

        self.all_zeroed = False
        self.all_indexes_found = False

        # disarmmotors on startup
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
        # resp.message = 'Motors armed'
        return resp

    def disarm_motors_service_callback(
        self, srv: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        disarmed = self.robot.disarm_motors()
        if disarmed:
            print("Motors disarmed.")

        resp.success = disarmed
        # resp.message = 'Motors disarmed'
        return resp

    def zero_all_service_callback(
        self, srv: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        self.all_zeroed = (
            self.robot.zero_all()
        )  # TODO: doesnt print to terminal that it's all_zeroed
        if self.all_zeroed:
            print("All motors zeroed")

        # TODO decide what feedback is helpful here
        resp.success = self.all_zeroed
        if not resp.success:
            resp.message = "failed to zero all motors. check print out from node"
        else:
            resp.message = "zeroed all motors"
        return resp

    def zero_joint_service_callback(
        self, srv: MotorTrigger.Request, resp: MotorTrigger.Response
    ) -> MotorTrigger.Response:
        success = self.robot.zero_motor(srv.joint_number)
        self.all_zeroed = self.robot.check_all_zeroed()

        # TODO decide what feedback is helpful here
        resp.success = success
        if not resp.success:
            resp.message = "failed to zero motor. check print out from node"
        else:
            resp.message = "zeroed motor"
            if self.all_zeroed:
                resp.message += ". all motors now zeroed"
                print("all motors now zeroed")
        return resp

    def index_search_motor_service_callback(
        self, srv: MotorTrigger.Request, resp: MotorTrigger.Response
    ) -> MotorTrigger.Response:
        success = self.robot.find_motor_index_pulse(srv.joint_number)
        self.all_indexes_found = self.robot.check_all_indexes_found()

        # # TODO decide what feedback is helpful here
        resp.success = success
        if not resp.success:
            resp.message = "failed to get index pulse. check print out from node"
        else:
            resp.message = "found index pulse"
            if self.all_indexes_found:
                resp.message += ".all index pulses have now been found"
                print("all index pulses have now been found")
        return resp

    def index_search_all_service_callback(
        self, srv: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        self.all_indexes_found = self.robot.find_all_index_pulses()

        # TODO decide what feedback is helpful here
        resp.success = self.all_indexes_found
        if not resp.success:
            resp.message = "failed to get all index pulses. check print out from node"
        else:
            resp.message = "found all index pulses"
        return resp

    def publish_joint_states(self) -> None:
        # get joint states
        joint_states = self.robot.get_states()

        # sometimes member variables doesn't exist yet? maybe async timer callback fires before __init__() finishes?
        # try:
            # populate body transform
        # if (
        #     True
        #     # self.body_transform and self.body_vel
        # ):  # there'll be one timestep where vel will be empty, but whatever
        #     for index in range(self.BODY_JOINT_OFFSET):
        #         self.joint_states_msg.position[index] = self.body_transform[index]
        #         self.joint_states_msg.velocity[index] = self.body_vel[index]

        # convert to msg type
        for index, joint_state in enumerate(joint_states):
            index += self.BODY_JOINT_OFFSET
            self.joint_states_msg.position[index] = joint_state[0]
            self.joint_states_msg.velocity[index] = joint_state[1]
            self.joint_states_msg.effort[index] = joint_state[2]

        # publish data
        self.joint_states_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_pub.publish(self.joint_states_msg)
        # except:
        #     # oh well
        #     self.get_logger().info("HELLLOOOOOOO")

        #     pass

    def joint_command_callback(self, joint_command: JointState) -> None:
        # reset watchdog
        self.watchdog_count = 0

        # set torques
        # if self.all_indexes_found and self.all_zeroed:
        self.robot.set_torques(joint_command.effort)
            # self.robot.set_torques_no_limit_check_5dof_only(joint_command.effort)
        # else:
        #     print(
        #         "Not all odrives have found their index and been zeroed. Ignoring command"
        #     )

        # # skip check for 5DOF test
        # self.robot.set_torques(joint_command.effort)
        # print('set torques.')

    def watchdog_callback(self) -> None:
        # TODO figure out how to start this properly
        # self.watchdog_count += 1

        if self.watchdog_count >= 100:
            self.shutdown()
            self.get_logger().error("Simulink to ROS Watchdog not fed")
            raise SystemExit

    def shutdown(self) -> None:
        # set hold current position or jsut disable motors
        self.robot.disarm_motors()

    def publish_bus_voltages(self) -> None:
        voltages = self.robot.get_reported_bus_voltages()

        for index, voltage in enumerate(voltages):
            self.bus_voltages_msg.data[index] = voltage

        self.bus_voltage_pub.publish(self.bus_voltages_msg)

    def vicon_callback(self, msg: TransformStamped) -> None:
        self.body_transform_prev = self.body_transform

        # populate body transform rotation
        q = [
            msg.transform.rotation.x,
            msg.transform.rotation.y,
            msg.transform.rotation.z,
            msg.transform.rotation.w,
        ]
        roll, pitch, yaw = transforms3d.euler.quat2euler(q)

        # Note: y,z are flipped from vicon coordinate system, which is z-up
        self.body_transform = [
            roll,
            pitch,
            yaw,
            msg.transform.translation.x,
            msg.transform.translation.y,
            msg.transform.translation.z,
        ]

        # update velocities
        if self.body_transform_prev:
            # "self.body_transform - self.body_transform_prev"
            self.body_vel = list(
                map(operator.sub, self.body_transform, self.body_transform_prev)
            )


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
