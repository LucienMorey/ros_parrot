from motorcan.TMotorManager import TMotorManager, TMotorControlState
from motorcan.OdriveManager import OdriveAxisHandle, GimbalMotorType
from parody_control.types import JointLimits
from typing import Union
from odrive.enums import AxisState, ControlMode, InputMode
from math import pi

import time
import logging


class Parody:

    LEFT_WRIST_1_ZERO_OFFSET:float = -2.2867818595405196
    LEFT_WRIST_2_ZERO_OFFSET:float = 2.2476653494494356
    RIGHT_WRIST_1_ZERO_OFFSET:float = -0.8609227487510054
    RIGHT_WRIST_2_ZERO_OFFSET:float = 2.6948209363716145
    NECK_WRIST_1_ZERO_OFFSET:float = 1.2164468584200705
    NECK_WRIST_2_ZERO_OFFSET:float = 3.862947119093016
    TAIL_ZERO_OFFSET:float = 1.3717623195667346
    def __init__(self,control_freq_hz: float):

        self.motors: list[Union[TMotorManager, OdriveAxisHandle]] = []
        self.joint_limits: list[JointLimits] = []
        self.control_freq_hz = control_freq_hz

        # Create left limb motor handles
        self.leftShoulder = TMotorManager("AK80-9", 8)
        self.leftShoulder.invert_direction()
        self.leftShoulder.invert_encoder()
        self.leftElbow = TMotorManager("AK80-9", 2)
        self.leftElbow.invert_direction()
        self.leftElbow.invert_encoder()
        self.leftWrist1 = OdriveAxisHandle(62,self.control_freq_hz,ControlMode.POSITION_CONTROL)
        self.leftWrist1.invert_motor()
        self.leftWrist1.invert_encoder()
        self.leftWrist1.set_zero_offset(self.LEFT_WRIST_1_ZERO_OFFSET)
        self.leftWrist2 = OdriveAxisHandle(55,self.control_freq_hz,ControlMode.POSITION_CONTROL)
        # self.leftWrist2.invert_motor()
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
        self.rightWrist1 = OdriveAxisHandle(61,self.control_freq_hz, ControlMode.POSITION_CONTROL, motor_model = GimbalMotorType.GL40_KV70)
        self.rightWrist1.invert_motor()
        self.rightWrist1.invert_encoder()  # no motor inversion required
        self.rightWrist1.set_zero_offset(self.RIGHT_WRIST_1_ZERO_OFFSET)
        self.rightWrist2 = OdriveAxisHandle(60,self.control_freq_hz, ControlMode.POSITION_CONTROL)  # no motor or encoder inversion required
        #self.rightWrist2.invert_motor()
        #self.rightWrist2.invert_encoder()
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

        self.neckWrist1 = OdriveAxisHandle(56,self.control_freq_hz, ControlMode.POSITION_CONTROL)
        self.neckWrist1.invert_motor()
        self.neckWrist1.invert_encoder()
        self.neckWrist1.set_zero_offset(self.NECK_WRIST_1_ZERO_OFFSET)

        self.neckWrist2 = OdriveAxisHandle(59,self.control_freq_hz, ControlMode.POSITION_CONTROL)
        self.neckWrist2.set_zero_offset(self.NECK_WRIST_2_ZERO_OFFSET)
        self.neckWrist2.invert_motor()
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

        TAIL_MOTOR_GEAR_RATIO = 3/10
        self.tail = OdriveAxisHandle(57,self.control_freq_hz,motor_model = GimbalMotorType.GL80_KV30)
        self.tail.set_zero_offset(self.TAIL_ZERO_OFFSET)
        self.tail.invert_encoder()

        # TODO Perform direction and encoder inversion here as required
        self.tail_limit = JointLimits(pi / 4 / TAIL_MOTOR_GEAR_RATIO, -pi / 4 / TAIL_MOTOR_GEAR_RATIO)
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
                if motor.control_mode is ControlMode.TORQUE_CONTROL: 
                    motor.set_torque(0)
                    motor_success = motor.set_controller_mode(
                        ControlMode.TORQUE_CONTROL, InputMode.PASSTHROUGH
                    )
                
                if motor.control_mode is ControlMode.POSITION_CONTROL: 
                    motor.set_position(motor.get_position())
                    motor_success = motor.set_controller_mode(
                        ControlMode.POSITION_CONTROL, InputMode.PASSTHROUGH
                    )
                motor_success &= motor.set_axis_state(AxisState.CLOSED_LOOP_CONTROL)
                if not motor_success and motor.control_mode is ControlMode.POSITION_CONTROL: print("failed to put position mode odrive with CAN ID {} into CLOSED_LOOP_CONTROL mode".format(motor.axis_id))
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
                    if motor.get_control_mode is ControlMode.TORQUE_CONTROL: motor.set_torque(0)
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
                print('Failed to disarm!')
                # pass
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

    # Sets commands (either torque or position) depending on control mode. Only Odrives can
    # be in position mode.
    # If joint limits are violated then the motor will disregard the command on that joint
    def set_commands(self, command: "list[float]") -> bool:
        # get current joint states
        success = True
        motor_states = self.get_states()

        # probably do some error checking here to make sure that length of all iterators is the same
        for motor, motor_state, desired_command, joint_limit in zip(
            self.motors, motor_states, command, self.joint_limits
        ):          
            # compare command against joint limits and current state
            # limit if past lower limit and trying to go lower
            if motor_state[0] <= joint_limit.lower_limit and desired_command < 0:
                if isinstance(motor,OdriveAxisHandle):
                    if motor.get_control_mode() is ControlMode.TORQUE_CONTROL:
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
            elif motor_state[0] >= joint_limit.upper_limit and desired_command > 0:
                if isinstance(motor,OdriveAxisHandle):
                    if motor.get_control_mode() is ControlMode.TORQUE_CONTROL:
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
                if motor.get_control_mode() == ControlMode.POSITION_CONTROL:
                    motor_success = motor.set_position(desired_command) # command is a POSITION, rad
                else:
                    motor_success = motor.set_torque(desired_command) # command is a TORQUE, Nm
                success &= motor_success
            elif isinstance(motor, TMotorManager):
                motor_success = motor.set_output_torque_newton_meters(desired_command) # command is a TORQUE, Nm
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
