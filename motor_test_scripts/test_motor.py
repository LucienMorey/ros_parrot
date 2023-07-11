from motorcan.OdriveManager import OdriveAxisHandle
from odrive.enums import AxisState, ControlMode, InputMode
import time
import math

if __name__ == "__main__":
    motor = OdriveAxisHandle(55) # M0 left
    # motor = OdriveAxisHandle(62) # M1 left

    # motor = OdriveAxisHandle(61) # right wrist 1
    # motor = OdriveAxisHandle(60) # right wrist 2

    # motor = OdriveAxisHandle(56) # M0 neck
    # motor = OdriveAxisHandle(59) # M1 neck 

    # motor = OdriveAxisHandle(57) # tail motor

    motor.clear_errors()
    # motor.invert_motor()

    index_found = motor.find_index_pulse()

    if not motor.index_found:
       print('Failed to find index pulse. Press ctrl+c to exit.')
       exit(1)

    # motor.calibrate()

    time.sleep(5)

    motor.set_controller_mode(ControlMode.TORQUE_CONTROL, InputMode.PASSTHROUGH)

    motor.set_axis_state(AxisState.CLOSED_LOOP_CONTROL)

    last_time = time.time()
    set_torque = 0.0
    torque_lim = 0.2
    while time.time() - last_time < 5:
        if set_torque <= torque_lim:
            set_torque = set_torque + 0.001
        motor.set_torque(set_torque)
        # motor.set_torque(0.1*math.sin(2*3.14*time.time()/2))

        print(
            motor.get_position(),
            motor.get_velocity(),
            motor.get_torque(),
            motor.get_current(),
            motor.get_bus_voltage(),
            motor.get_axis_error(),
            motor.get_encoder_error()
        )
        time.sleep(0.01)

    motor.set_torque(0.0)

    motor.stop_motor()
