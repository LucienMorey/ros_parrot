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

    time.sleep(2)
    motor.set_controller_mode(ControlMode.POSITION_CONTROL, InputMode.PASSTHROUGH)
    print(motor.get_control_mode().name)

    
    initial_pos = motor.get_position()
    motor.set_position(initial_pos)

    motor.set_axis_state(AxisState.CLOSED_LOOP_CONTROL)

    last_time = time.time()    

    while time.time() - last_time < 6:
        motor.set_position(initial_pos + math.pi/4*math.sin(2*3.14*time.time()/2))

        print(
            motor.get_position(),
            motor.get_velocity(),
            motor.get_torque(),
            motor.get_axis_error(),
            motor.get_encoder_error()
        )
        time.sleep(0.01)

    stopped_safe = motor.stop_motor()
    if stopped_safe: print('Set axis state to IDLE successfully.')
