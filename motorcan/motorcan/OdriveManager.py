from cmath import pi
from motorcan.CAN_Manager import CAN_Manager
from odrive.enums import ControlMode, InputMode, AxisState, AxisError, EncoderError, ControllerError
from motorcan.OdriveStructures import (
    OdriveMotorState,
    OdriveMotorFeedback,
    OdriveEncoderFeedback,
)
from can.message import Message
from can.listener import Listener
import cantools
import candatabase
import os
from time import sleep
from motorcan.helper import RepeatTimer
import enum

class GimbalMotorType(enum.Enum):
    GL40_KV210                       = 0
    GL40_KV70                        = 1
    GL80_KV30                        = 2

db = cantools.db.load_file(
    os.path.dirname(candatabase.__file__) + "/odrive-cansimple.dbc"
)


class OdriveAxisHandle:
    GL40_KV70_TORQUE_CONST: float = 0.14 #[Nm/A]
    GL40_KV70_PHASE_INDUCTANCE: float = 1.8e-3  # H
    GL40_KV70_PHASE_RESISTANCE: float = 4.5 # Ohm
    GL40_KV70_POLE_PAIRS: int = 14
    GL40_KV70_CONTINUOUS_CURRENT_RATING: float = 1.65 # A
    
    GL40_KV210_TORQUE_CONST: float = 0.046 #[Nm/A]
    GL40_KV210_PHASE_INDUCTANCE: float = 0.18e-3  # H
    GL40_KV210_PHASE_RESISTANCE: float = 0.49 # Ohm
    GL40_KV210_POLE_PAIRS: int = 14
    GL40_KV210_CONTINUOUS_CURRENT_RATING: float = 7 # A

    GL80_KV30_TORQUE_CONST: float = 0.17 # Nm/A
    GL80_KV30_PHASE_INDUCTANCE: float = 1.1e-3 # H
    GL80_KV30_PHASE_RESISTANCE: float = 1.8 # Ohm
    GL80_KV30_POLE_PAIRS: int = 21
    GL80_KV30_CONTINUOUS_CURRENT_RATING: float = 2.8 # A



    def __init__(self, axisID: int, control_freq_hz = 100.0, control_mode = ControlMode.TORQUE_CONTROL, motor_model = GimbalMotorType.GL40_KV210) -> None:
        self.axis_id = axisID
        self.control_mode = control_mode
        if self.control_mode is ControlMode.POSITION_CONTROL: print('ODrive with CAN ID {} started in position mode.'.format(self.axis_id))

        self.motor_model = motor_model
        if self.motor_model == GimbalMotorType.GL40_KV70:
            self._torque_constant = self.GL40_KV70_TORQUE_CONST
            self._current_rating = self.GL40_KV70_CONTINUOUS_CURRENT_RATING
        elif self.motor_model == GimbalMotorType.GL40_KV210:
            self._torque_constant = self.GL40_KV210_TORQUE_CONST
            self._current_rating = self.GL40_KV210_CONTINUOUS_CURRENT_RATING
        elif self.motor_model == GimbalMotorType.GL80_KV30:
            self._torque_constant = self.GL80_KV30_TORQUE_CONST
            self._current_rating = self.GL80_KV30_CONTINUOUS_CURRENT_RATING
        else:
            raise TypeError('Odrive with CAN ID {}: Could not recognize motor_model = {} from GimbalMotorType enums.'.format(self.axis_id,motor_model))

        self._can_manager = CAN_Manager()
        self._can_manager.add_motor_listener(OdriveMotorListener(self))
        self._control_freq_hz = control_freq_hz
        

        self.current_state = OdriveMotorState(
            AxisState.UNDEFINED, AxisError.INVALID_STATE
        )

        self.encoder_error = EncoderError.NONE

        self.feedback = OdriveMotorFeedback(0.0, 0.0, 0.0)
        self.bus_voltage = 0.0
        self.reported_position = 0.0

        self.index_found = False
        self.zeroed = False
        self.zero_offset = 0.0

        self.direction = 1.0
        self.encoder_direction = 1.0

        self.get_telemetry_timer = RepeatTimer(1/control_freq_hz, self._send_telemetry_requests)
        self.get_telemetry_timer.start()

    def get_id(self) -> int:
        return self.axis_id
    
    def get_control_mode(self) -> ControlMode:
        return self.control_mode

    def get_position(self) -> float:
        return self.feedback.position
    
    def get_raw_position(self)->float:
        return self.reported_position

    def get_velocity(self) -> float:
        return self.feedback.velocity

    def get_torque(self) -> float:
        torque = self.feedback.current * self._torque_constant
        if self.control_mode is ControlMode.POSITION_CONTROL and abs(torque) >= self._current_rating:
            print('ODrive @ CAN ID {} exceeded current rating! Actual: {}Nm, current: {}Nm'.format(self.axis_id,torque,self._current_rating*self._torque_constant))
        return torque
        

    def get_current(self) -> float:
        return self.feedback.current

    def get_bus_voltage(self) -> float:
        return self.bus_voltage

    def clear_errors(self) -> bool:
        return self._can_manager.send(self._pack_clear_errors_message())

    def get_axis_error(self) -> AxisError:
        return self.current_state.axis_error

    def get_encoder_error(self) -> EncoderError:
        return self.encoder_error
    
    def get_controller_error(self) -> ControllerError:
        return self.controller_error

    def get_index_found(self) -> bool:
        return self.index_found
    
    def set_zero_offset(self, offset: float) -> None:
        self.zero_offset = offset

    def set_torque(self, torque: float) -> bool:
        if self.control_mode is not ControlMode.TORQUE_CONTROL:
            print('ODrive with CAN ID {} tried to set_torque() but is not in torque mode! Ignoring.'.format(self.axis_id))
            return False
        # Pack input torque msg
        msg = self._pack_torque_msg(torque * self.direction)
        if not self._can_manager.send(msg):
            return False
        else:
            return True

        # # Send control mode message to respond to can commands
        # if not self.set_controller_mode(
        #     ControlMode.TORQUE_CONTROL, InputMode.PASSTHROUGH
        # ):
        #     return False

        # # set axis mode to begin motion
        # return self.set_axis_state(AxisState.CLOSED_LOOP_CONTROL)

    def set_position(self, position_rad: float) -> bool:
        if self.control_mode is not ControlMode.POSITION_CONTROL:
            print('ODrive with CAN ID {} tried to set_position() but is not in position mode! Ignoring.'.format(self.axis_id))
            return False
        send_pos = self.encoder_direction*(position_rad + self.zero_offset)/2/pi
        # print('Commanding position {} [revolutions]'.format(send_pos))
        msg = self._pack_position_msg(send_pos) # Odrive takes positions in [rev], not [rad]
        if not self._can_manager.send(msg):
            return False
        else:
            return True

    def set_controller_mode(
        self, control_mode: ControlMode, input_mode: InputMode
    ) -> bool:
        # Pack controller mode message
        msg = self._pack_controller_mode_msg(control_mode, input_mode)
        self.control_mode = control_mode

        return self._can_manager.send(msg)

    def find_index_pulse(self) -> bool:
        # try to send can message to set state
        self.clear_errors()
        success = self.set_axis_state(AxisState.ENCODER_INDEX_SEARCH)

        # sleep for next hearbeat message to make sure it has transitioned states
        # TODO stop this from being magic number
        sleep(0.015)

        # wait until the axis state returns to idle unless the message failed to send
        # TODO rate limit this so its not just busy waiting
        while (
            self.current_state.axis_state != AxisState.IDLE
            and self.current_state.axis_error == AxisError.NONE
            and success
        ):
            pass

        #  if no errors have occured then assume the operation was successful and update index found state
        if self.current_state.axis_error == AxisError.NONE and success:
            self.index_found = True
            return True

        return False

    def stop_motor(self) -> bool:
        self.get_telemetry_timer.cancel()
        return self.set_axis_state(AxisState.IDLE)

    def set_axis_state(self, axis_state: AxisState) -> bool:
        msg = self._pack_axis_state_msg(axis_state)
        return self._can_manager.send(msg)

    def _update_pos_vel_async(self, new_feedback: OdriveEncoderFeedback) -> None:
        self.reported_position = new_feedback.position * self.encoder_direction
        self.feedback.velocity = new_feedback.velocity * self.encoder_direction
        self.feedback.position = self.reported_position - self.zero_offset

    def update_current_async(self, iq: float) -> None:
        self.feedback.current = iq * self.direction

    def update_bus_voltage_async(self, bus_voltage: float) -> None:
        self.bus_voltage = bus_voltage

    def _send_telemetry_requests(self) -> bool:
        success = True
        success &= self._send_get_iq_message()
        success &= self._send_get_bus_voltage_message()
        success &= self._send_get_encoder_error_message()

        return success

    def _send_get_iq_message(self) -> bool:
        return self._can_manager.send(self._pack_get_iq_message())

    def _send_get_bus_voltage_message(self) -> bool:
        return self._can_manager.send(self._pack_get_bus_voltage_message())

    def _send_get_encoder_error_message(self) -> bool:
        return self._can_manager.send(self._pack_get_encoder_error_message())
    
    def _send_get_controller_error_message(self) -> bool:
        return self._can_manager.send(self._pack_get_controller_error_message())

    def _update_state_async(self, new_state: OdriveMotorState) -> None:
        self.current_state = new_state

    def update_encoder_error_async(self, error_code: int) -> None:
        self.encoder_error = EncoderError(error_code)

    def update_controller_error_async(self, error_code: int) -> None:
        self.controller_error = ControllerError

    def calibrate(self) -> bool:
        # try to send a command to calibrate the motor
        msg = self._pack_calibrate_msg()
        success = self._can_manager.send(msg)

        # sleep for next hearbeat message to make sure it has transitioned states
        # TODO stop this from being magic number
        sleep(0.015)

        # wait for current axis to return to idle state if command was sent
        # TODO rate limit this so its not just busy waiting
        while (
            self.current_state.axis_state != AxisState.IDLE
            and self.current_state.axis_error == AxisError.NONE
            and success
        ):
            pass

        # return if calibration was successful
        return self.current_state.axis_error == AxisError.NONE and success

    def _pack_calibrate_msg(self) -> Message:
        msg = db.get_message_by_name("Set_Axis_State")
        data = msg.encode(
            {"Axis_Requested_State": AxisState.FULL_CALIBRATION_SEQUENCE.value}
        )
        msg = Message(
            arbitration_id=msg.frame_id | self.axis_id << 5,
            is_extended_id=(self.axis_id > 0x3F),
            data=data,
        )

        return msg

    def _pack_axis_state_msg(self, axis_state: AxisState) -> Message:
        msg = db.get_message_by_name("Set_Axis_State")
        data = msg.encode({"Axis_Requested_State": axis_state.value})
        msg = Message(
            arbitration_id=msg.frame_id | self.axis_id << 5,
            is_extended_id=(self.axis_id > 0x3F),
            data=data,
        )

        return msg

    def _pack_torque_msg(self, torque: float) -> Message:
        msg = db.get_message_by_name("Set_Input_Torque")
        data = msg.encode({"Input_Torque": torque})
        msg = Message(
            arbitration_id=msg.frame_id | self.axis_id << 5,
            is_extended_id=(self.axis_id > 0x3F),
            data=data,
        )

        return msg
    
    def _pack_position_msg(self, position: float) -> Message:
        msg = db.get_message_by_name("Set_Input_Pos")
        data = msg.encode(
            {"Input_Pos": position, "Vel_FF": 0, "Torque_FF": 0}
        )
        msg = Message(
            arbitration_id=msg.frame_id | self.axis_id << 5,
            is_extended_id=(self.axis_id > 0x3F),
            data=data,
        )

        return msg

    def _pack_controller_mode_msg(
        self, control_mode: ControlMode, input_mode: InputMode
    ) -> Message:
        msg = db.get_message_by_name("Set_Controller_Mode")
        data = msg.encode(
            {"Input_Mode": input_mode.value, "Control_Mode": control_mode.value}
        )
        msg = Message(
            arbitration_id=msg.frame_id | self.axis_id << 5,
            is_extended_id=(self.axis_id > 0x3F),
            data=data,
        )

        return msg

    def _pack_get_iq_message(self) -> Message:
        msg = db.get_message_by_name("Get_Iq")
        msg = Message(
            arbitration_id=msg.frame_id | self.axis_id << 5,
            is_extended_id=(self.axis_id > 0x3F),
            is_remote_frame=True,
        )
        return msg

    def _pack_get_bus_voltage_message(self) -> Message:
        msg = db.get_message_by_name("Get_Vbus_Voltage")
        msg = Message(
            arbitration_id=msg.frame_id | self.axis_id << 5,
            is_extended_id=(self.axis_id > 0x3F),
            is_remote_frame=True,
        )
        return msg

    def _pack_get_encoder_error_message(self) -> Message:
        msg = db.get_message_by_name("Get_Encoder_Error")
        msg = Message(
            arbitration_id=msg.frame_id | self.axis_id << 5,
            is_extended_id=(self.axis_id > 0x3F),
            is_remote_frame=True,
        )
        return msg

    def _pack_get_controller_error_message(self) -> Message:
        msg = db.get_message_by_name("Get_Controller_Error")
        msg = Message(
            arbitration_id=msg.frame_id | self.axis_id << 5,
            is_extended_id=(self.axis_id > 0x3F),
            is_remote_frame=True,
        )
        return msg

    def _pack_clear_errors_message(self) -> Message:
        msg = db.get_message_by_name("Clear_Errors")
        msg = Message(
            arbitration_id=msg.frame_id | self.axis_id << 5,
            is_extended_id=(self.axis_id > 0x3F),
            is_remote_frame=True,
        )
        return msg

    def invert_motor(self) -> bool:
        self.direction = -self.direction
        return True
    
    def invert_encoder(self) -> bool:
        self.encoder_direction = - self.encoder_direction
        return True


# python-can listener object, with handler to be called upon reception of a message on the CAN bus
class OdriveMotorListener(Listener):
    """Python-can listener object, with handler to be called upon reception of a message on the CAN bus"""

    def __init__(self, motor: OdriveAxisHandle):
        """
        Sets stores can manager and motor object references
        """
        self.motor = motor
        self.axisID = motor.get_id()

    def on_message_received(self, msg: Message):
        """
        Updates this listener's motor with the info contained in msg, if that message was for this motor.

        args:
            msg: A python-can CAN message
        """
        # if motor type is gimble
        if msg.arbitration_id == ((self.axisID << 5)| db.get_message_by_name("Get_Encoder_Estimates").frame_id):
            # decode readings
            current_vel = (
                db.decode_message("Get_Encoder_Estimates", msg.data)["Vel_Estimate"]
                * 2
                * pi
            )
            current_pos = (
                db.decode_message("Get_Encoder_Estimates", msg.data)["Pos_Estimate"]
                * 2
                * pi
            )

            self.motor._update_pos_vel_async(
                OdriveEncoderFeedback(current_pos, current_vel)
            )

        elif msg.arbitration_id == ((self.axisID << 5) | db.get_message_by_name("Heartbeat").frame_id):
            axis_state = db.decode_message("Heartbeat", msg.data)["Axis_State"]
            axis_error = db.decode_message("Heartbeat", msg.data)["Axis_Error"]

            self.motor._update_state_async(
                OdriveMotorState(AxisState(axis_state), AxisError(axis_error))
            )
        elif (
            msg.arbitration_id
            == ((self.axisID << 5) | db.get_message_by_name("Get_Iq").frame_id)
            and msg.is_remote_frame == False
        ):
            axis_current = db.decode_message("Get_Iq", msg.data)["Iq_Measured"]
            self.motor.update_current_async(axis_current)
        elif (
            msg.arbitration_id
            == (
                (self.axisID << 5) | db.get_message_by_name("Get_Vbus_Voltage").frame_id
            )
            and msg.is_remote_frame == False
        ):
            axis_current = db.decode_message("Get_Vbus_Voltage", msg.data)[
                "Vbus_Voltage"
            ]
            self.motor.update_bus_voltage_async(axis_current)
        elif (
            msg.arbitration_id
            == (
                (self.axisID << 5)
                | db.get_message_by_name("Get_Encoder_Error").frame_id
            )
            and msg.is_remote_frame == False
        ):
            encoder_error_code = db.decode_message("Get_Encoder_Error", msg.data)[
                "Encoder_Error"
            ]
            self.motor.update_encoder_error_async(encoder_error_code)
        # elif (
        #     msg.arbitration_id == ((self.axisID << 5)| db.get_message_by_name("Get_Controller_Error").frame_id) and msg.is_remote_frame == False ):
        #     controller_error_code = db.decode_message("Get_Controller_Error", msg.data)["Controller_Error"]
        #     self.motor.update_controller_error_async(controller_error_code)
