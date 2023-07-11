#!/usr/bin/env python3

from math import sin
import time

from motorcan.TMotorManager import TMotorManager
from motorcan.TMotorStructures import TMotorControlState

import math


if __name__ == "__main__":

    motor_1_handle = TMotorManager(motor_type="AK80-9", motor_ID=2)
    # motor_1_handle.invert_direction()
    # motor_1_handle.invert_encoder()

    success = motor_1_handle.power_on()
    if success: 
        print('power on success')
    else:
        print('couldnt power on')
    motor_1_handle.set_control_mode(TMotorControlState.IDLE)


    # success &= motor_1_handle.zero()
    # if success: 
    #     print('motor zero success')
    # else:
    #     print('couldnt zero motor/encoder')

    # time.sleep(2)

    success = motor_1_handle.set_control_mode(TMotorControlState.CURRENT)
    if success:
        print('control mode set to current control')
    else:
        print('failed to set control mode')
        exit(1)
    # motor_1_handle.set_output_torque_newton_meters(0.1)

    # time.sleep(1)
    
    start_time = time.time()
    while time.time() - start_time < 5:
        print(motor_1_handle.get_output_angle_radians(), " ", motor_1_handle.get_output_velocity_radians_per_second(), " ", motor_1_handle.get_current_qaxis_amps())
        # motor_1_handle.set_output_torque_newton_meters(0)
        motor_1_handle.set_motor_torque_newton_meters(2)
        # motor_1_handle.set_output_torque_newton_meters(1.0 + math.sin(time.time()))
        time.sleep(0.01)
    #print(motor_1_handle.device_info_string())

    time.sleep(0.1)
    motor_1_handle.set_control_mode(TMotorControlState.IDLE)

    motor_1_handle.power_off()
