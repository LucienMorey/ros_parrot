import can
import cantools
from dataclasses import dataclass


@dataclass
class MotorState:
    position: float
    velocity: float


db = cantools.db.load_file("odrive-cansimple.dbc")


class OdriveAxisHandle:
    AXIS_FULL_CALIBRATION_SEQUENCE = 0x03

    axisID: int
    is_extended: bool

    def __init__(self, axisID: int, bus: str) -> None:
        self.axisID = axisID
        if axisID > 0x3F:
            self.is_extended = True
        else:
            self.is_extended = False

        self.bus = can.Bus(bus, bustype="socketcan")

    def set_vel(self, velocity: float, torque_ff: float) -> bool:
        # Convert torque to appropriate range

        # Get db msg
        msg = db.get_message_by_name("Set_Input_Vel")
        # Encode value into msg
        data = msg.encode({"Input_Vel": velocity, "Input_Torque_FF": torque_ff})
        # Create CAN message
        msg = can.Message(
            arbitration_id=msg.frame_id | self.axisID << 5,
            is_extended_id=self.is_extended,
            data=data,
        )
        try:
            # send msg on bus
            self.bus.send(msg)
            return True
        except can.CanError:
            # error on bus
            return False

    def set_torque(self, torque: float) -> bool:
        # Convert torque to appropriate range

        # Get db msg
        msg = db.get_message_by_name("Set_Input_Torque")
        # Encode value into msg
        data = msg.encode({"Input_Torque": torque})
        # Create CAN message
        msg = can.Message(
            arbitration_id=msg.frame_id | self.axisID << 5,
            is_extended_id=self.is_extended,
            data=data,
        )
        try:
            # send msg on bus
            self.bus.send(msg)
            return True
        except can.CanError:
            # error on bus
            return False

    def get_state(self) -> MotorState | None:
        # Read msg until correct one found
        try:
            while True:
                msg = self.bus.recv()
                # Check for if msg is correct device id and correct msg number according to spec
                if msg.arbitration_id == (
                    (self.axisID << 5)
                    | db.get_message_by_name("Get_Encoder_Estimates").frame_id
                ):
                    # decode readings
                    current_vel = db.decode_message("Get_Encoder_Estimates", msg.data)[
                        "Vel_Estimate"
                    ]
                    current_pos = db.decode_message("Get_Encoder_Estimates", msg.data)[
                        "Pos_Estimate"
                    ]

                    # return with state info
                    return MotorState(current_pos, current_vel)
        except can.CanError:
            # catch failure and return none
            return None

    def calibrate(self) -> bool:
        msg = db.get_message_by_name("Set_Axis_State")
        data = msg.encode({"Axis_Requested_State": self.AXIS_FULL_CALIBRATION_SEQUENCE})
        msg = can.Message(
            arbitration_id=msg.frame_id | self.axisID << 5,
            is_extended_id=self.is_extended,
            data=data,
        )
        try:
            self.bus.send(msg)

            # Read messages infinitely and wait for the right ID to show up
            while True:
                msg = self.bus.recv()
                if msg.arbitration_id == (
                    (self.axisID << 5) | db.get_message_by_name("Heartbeat").frame_id
                ):
                    current_state = db.decode_message("Heartbeat", msg.data)[
                        "Axis_State"
                    ]
                    if current_state == 0x1:
                        # print("\nAxis has returned to Idle state.")
                        break
            return True
        except can.CanError:
            return False
