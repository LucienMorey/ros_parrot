from motorcan.TMotorManager import TMotorManager, TMotorControlState
from motorcan.OdriveManager import OdriveAxisHandle
from parody_control.types import JointLimits
from typing import Union
from odrive.enums import AxisState, ControlMode, InputMode
from math import pi

import time
import logging


class Parody:

    LEFT_WRIST_1_ZERO_OFFSET:float = -2.5145542253452406
    LEFT_WRIST_2_ZERO_OFFSET:float = -4.94363658415845
    RIGHT_WRIST_1_ZERO_OFFSET:float = -0.747408170447468
    RIGHT_WRIST_2_ZERO_OFFSET:float = 2.690218806754443
    NECK_WRIST_1_ZERO_OFFSET:float = 0.1154320542883945
    NECK_WRIST_2_ZERO_OFFSET:float = -0.7995635172355797
    TAIL_ZERO_OFFSET:float = 4.550937502459726
    def __init__(self):

        self.motors: list[Union[TMotorManager, OdriveAxisHandle]] = []
        self.joint_limits: list[JointLimits] = []

        # Create left limb motor handles
        self.leftShoulder = TMotorManager("AK80-9", 8)
        self.leftShoulder.invert_direction()
        self.leftShoulder.invert_encoder()
        self.leftElbow = TMotorManager("AK80-9", 2)
        self.leftElbow.invert_direction()
        self.leftElbow.invert_encoder()
        self.leftWrist1 = OdriveAxisHandle(62)
        self.leftWrist1.invert_motor()
        self.leftWrist1.invert_encoder()
        self.leftWrist1.set_zero_offset(self.LEFT_WRIST_1_ZERO_OFFSET)
        self.leftWrist2 = OdriveAxisHandle(55)
        self.leftWrist2.invert_motor()
        self.leftWrist2.invert_encoder()
        self.leftWrist2.set_zero_offset(self.LEFT_WRIST_2_ZERO_OFFSET)
        self.left_arm_motors: list[Union[TMotorManager, OdriveAxisHandle]] = [
            self.leftShoulder,
            self.leftElbow,
            self.leftWrist1,
            self.leftWrist2,
        ]

        self.left_arm_joint_limits: list[JointLimits] = [
            JointLimits(pi / 2, -pi / 2),
            JointLimits(17*pi / 18, 0),
            JointLimits(pi / 2, -pi / 2),
            JointLimits(pi/2, -pi),
        ]

        # create right limb motor handles
        self.rightShoulder = TMotorManager("AK80-9", 3) 
        self.rightElbow = TMotorManager("AK80-9", 4)
        self.rightWrist1 = OdriveAxisHandle(61)
        self.rightWrist1.invert_motor()
        self.rightWrist1.invert_encoder()  # no motor inversion required
        self.rightWrist1.set_zero_offset(self.RIGHT_WRIST_1_ZERO_OFFSET)
        self.rightWrist2 = OdriveAxisHandle(
            60
        )  # no motor or encoder inversion required
        self.rightWrist2.set_zero_offset(self.RIGHT_WRIST_2_ZERO_OFFSET)
        self.right_arm_motors: list[Union[TMotorManager, OdriveAxisHandle]] = [
            self.rightShoulder,
            self.rightElbow,
            self.rightWrist1,
            self.rightWrist2,
        ]

        self.right_arm_joint_limits: list[JointLimits] = [
            JointLimits(pi / 2, -pi / 2),
            JointLimits(17*pi/18,0),
            JointLimits(pi / 2, -pi / 2),
            JointLimits(pi/2, -pi ),
        ]

        # create neck limb motor handles
        self.neckShoulder1 = TMotorManager("AK80-9", 5)

        self.neckShoulder2 = TMotorManager("AK80-9", 6)
        self.neckShoulder2.invert_direction()
        self.neckShoulder2.invert_encoder()

        self.neckElbow = TMotorManager("AK80-9", 7)
        self.neckElbow.invert_direction()
        self.neckElbow.invert_encoder()

        self.neckWrist1 = OdriveAxisHandle(58)
        self.neckWrist1.invert_motor()
        self.neckWrist1.invert_encoder()
        self.neckWrist1.set_zero_offset(self.NECK_WRIST_1_ZERO_OFFSET)

        self.neckWrist2 = OdriveAxisHandle(59)
        self.neckWrist2.set_zero_offset(self.NECK_WRIST_2_ZERO_OFFSET)
        # self.neckWrist2.invert_motor()
        self.neckWrist2.invert_encoder()
        self.neck_motors: list[Union[TMotorManager, OdriveAxisHandle]] = [
            self.neckShoulder1,
            self.neckShoulder2,
            self.neckElbow,
            self.neckWrist1,
            self.neckWrist2,
        ]

        NECK_ELBOW_GEAR_RATIO = 13/20
        self.neck_joint_limits: list[JointLimits] = [
            JointLimits(pi / 3, -pi / 3),
            JointLimits(3*pi / 4, 0),
            JointLimits(17*pi / 18 / NECK_ELBOW_GEAR_RATIO, 0),
            JointLimits(pi / 2, -pi / 2),
            JointLimits(pi / 2, -pi / 2 )
        ]

        TAIL_MOTOR_GEAR_RATIO = 0.9/10
        self.tail = OdriveAxisHandle(57)
        self.tail.set_zero_offset(self.TAIL_ZERO_OFFSET)

        # TODO Perform direction and encoder inversion here as required
        self.tail_limit = JointLimits(pi / 4 / TAIL_MOTOR_GEAR_RATIO, pi / 4 / TAIL_MOTOR_GEAR_RATIO)
        # append motors to motor list
        self.motors.extend(self.left_arm_motors)
        self.motors.extend(self.right_arm_motors)
        self.motors.extend(self.neck_motors)
        self.motors.append(self.tail)

        # set joint limits
        self.joint_limits.extend(self.left_arm_joint_limits)
        self.joint_limits.extend(self.right_arm_joint_limits)
        self.joint_limits.extend(self.neck_joint_limits)
        self.joint_limits.append(self.tail_limit)

    # Should turn all motors on
    def arm_motors(self) -> bool:
        success = True

        for motor in self.motors:
            
            motor_success = True
            # if motor is a tmotor it needs to be turned on before it can be used
            if isinstance(motor, TMotorManager):
                motor_success = motor.power_on()
                motor_success &= motor.set_control_mode(TMotorControlState.CURRENT)
                success &= motor_success
            # odrives do not have this issue and can just sit tight for now
            elif isinstance(motor, OdriveAxisHandle):
                motor.set_torque(0)
                motor_success = motor.set_controller_mode(
                    ControlMode.TORQUE_CONTROL, InputMode.PASSTHROUGH
                )
                motor_success &= motor.set_axis_state(AxisState.CLOSED_LOOP_CONTROL)
                success &= motor_success
            # TODO decide how to hanlde error if motor is not correct type if required
            else:
                raise TypeError(
                    "Motor handle type is neither TMotorManager nor OdriveAxisHandle!"
                )

            # TODO put this in the above if statements to report motor CAN id
            if not motor_success:
                # raise RuntimeError("a motor failed to arm!")
                print("Motor with CAN ID",motor.get_id(),'failed to arm! oh well.')

        return success

    def disarm_motors(self) -> bool:
        success = True

        for motor in self.motors:
            motor_success = True

            # if motor is a tmotor it can be turned off and set to idle
            if isinstance(motor, TMotorManager):
                if motor._control_state == TMotorControlState.IDLE:
                    continue

                # not already idle, probably in current control.
                # set torque to zero
                try:
                    motor.set_output_torque_newton_meters(0)
                except:
                    logging.warn(
                        "Could not set T-motor (ID {}) torque to zero during disarm. May not have been in current control mode.".format(
                            motor.ID
                        )
                    )
                motor_success &= motor.set_control_mode(TMotorControlState.IDLE)
                motor_success = motor.power_off()
                success &= motor_success
            # if motor is an odrive it can just be set to idle
            elif isinstance(motor, OdriveAxisHandle):
                try:
                    motor.set_torque(0)
                except:
                    logging.warn(
                        "Could not set ODrive (ID {}) torque to zero during disarm. May not have been in current control mode.".format(
                            motor.get_id()
                        )
                    )
                motor_success &= motor.stop_motor()
                success &= motor_success
            # TODO decide how to handle errors for bad typing if deemed to be requried
            else:
                pass

            # decide how to handle erros here if its actually required
            if not motor_success:
                pass
        return success

    def zero_motor(self, motor_num: int) -> bool:
        
        if not isinstance(self.motors[motor_num], TMotorManager):
            print("motor is not a tmotor and should not be zeroed dynamically")
            return False
        
        motor_success = self.motors[motor_num].zero()
        if not motor_success:
            print("failed to zero Tmotor ", self.motors[motor_num].get_id())

        return motor_success

    def find_motor_index_pulse(self, motor_num: int) -> bool:
        if isinstance(self.motors[motor_num], OdriveAxisHandle):
            motor_success = self.motors[motor_num].find_index_pulse()
            if not motor_success:
                print(
                    "failed to find index for motor ", self.motors[motor_num].get_id()
                )
            else: 
                print("found index pulse for motor ", self.motors[motor_num].get_id())
            return motor_success
        elif isinstance(self.motors[motor_num], TMotorManager):
            print("Tmotors do not need to find an index pulse")
            return True
        return False

    # Should pass desired torque commands. If joint limits are violated then the motor
    # will disregard the command on that joint
    def set_torques(self, torques: "list[float]") -> bool:
        # get current joint states
        success = True
        motor_states = self.get_states()

        # probably do some error checking here to make sure that length of all iterators is the same
        for motor, motor_state, desired_torque, joint_limit in zip(
            self.motors, motor_states, torques, self.joint_limits
        ):
            # compare command against joint limits and current state
            # limit if past lower limit and trying to go lower
            if motor_state[0] <= joint_limit.lower_limit and desired_torque < 0:
                desired_torque = 0.0
                if isinstance(motor, OdriveAxisHandle):
                    print(
                        "motor {} attempt to violate lower joint limit. zeroing command".format(
                            motor.axis_id
                        )
                    )
                if isinstance(motor, TMotorManager):
                    print(
                        "motor {} attempt to violate lower joint limit. zeroing command".format(
                            motor.ID
                        )
                    )

            # limit is past upper limit and trying to go higher
            elif motor_state[0] >= joint_limit.upper_limit and desired_torque > 0:
                desired_torque = 0.0
                # print("attempt to violate upper joint limit. zeroing command")
                if isinstance(motor, OdriveAxisHandle):
                    print(
                        "motor {} attempt to violate upper joint limit. zeroing command".format(
                            motor.axis_id
                        )
                    )
                if isinstance(motor, TMotorManager):
                    print(
                        "motor {} attempt to violate upper joint limit. zeroing command".format(
                            motor.ID
                        )
                    )
            # TODO decide if the above constitute a motor error or if its jsut failing to send a message

            motor_success = True

            if isinstance(motor, OdriveAxisHandle):
                motor_success = motor.set_torque(desired_torque)
                success &= motor_success
            elif isinstance(motor, TMotorManager):
                motor_success = motor.set_output_torque_newton_meters(desired_torque)
                success &= motor_success
            else:
                pass

            # TODO decide if anything should be done at this level and how to handle
            if not motor_success:
                pass

        return success

    # def get joint states for all motors (position, velocity , current?)
    def get_states(self) -> "list[list[float]]":
        motor_states: list[list[float]] = []
        for motor in self.motors:
            motor_state: list[float] = [0.0, 0.0, 0.0]
            if isinstance(motor, OdriveAxisHandle):
                motor_state[0] = motor.get_position()
                motor_state[1] = motor.get_velocity()
                motor_state[2] = motor.get_torque()
                # TODO populate current for odrive motors
                motor_states.append(motor_state)
            elif isinstance(motor, TMotorManager):
                motor_state[0] = motor.get_output_angle_radians()
                motor_state[1] = motor.get_output_velocity_radians_per_second()
                motor_state[2] = motor.get_output_torque_newton_meters()
                motor_states.append(motor_state)
            # TODO decide how to handle errors in this case if required at all
            else:
                pass
        return motor_states

    def clear_motor_errors(self) -> bool:
        success = True

        for motor in self.motors:
            if isinstance(motor, OdriveAxisHandle):
                motor_success = motor.clear_errors()
                success &= motor_success

                # TODO decide how to handle failure to clear
                if not motor_success:
                    pass
            elif isinstance(motor, TMotorManager):
                pass
            else:
                pass

        return success

    def check_all_indexes_found(self) -> bool:
        all_found = True

        for motor in self.motors:
            if isinstance(motor, OdriveAxisHandle) and not motor.get_index_found():
                all_found = False
                print(f"Motor with CAN ID {motor.get_id()} has not yet found its index")

        return all_found

    def get_reported_bus_voltages(self) -> "list[float]":
        bus_voltages: list[float] = []

        for motor in self.motors:
            if isinstance(motor, OdriveAxisHandle):
                V = motor.get_bus_voltage()
                bus_voltages.append(V)
                if V <= 19.8 and V >= 0.1:
                    logging.warning('LOW BATTERY (V<19.8V)! Odrive axis ID {}: {} V!'.format(motor.axis_id,V))
            elif isinstance(motor, TMotorManager):
                bus_voltages.append(-1.0)
            else:
                pass

        return bus_voltages
