from dataclasses import dataclass
from enum import Enum
from collections import namedtuple


# possible states for the controller
class TMotorControlState(Enum):
    """
    An Enum to keep track of different control states
    """

    IDLE = 0
    IMPEDANCE = 1
    CURRENT = 2
    FULL_STATE = 3
    SPEED = 4


# Parameter dictionary for each specific motor that can be controlled with this library
# Thresholds are in the datasheet for the motor on cubemars.com

MIT_Params = {
    "ERROR_CODES": {
        0: "No Error",
        1: "Over temperature fault",
        2: "Over current fault",
        3: "Over voltage fault",
        4: "Under voltage fault",
        5: "Encoder fault",
        6: "Phase current unbalance fault (The hardware may be damaged)",
    },
    "AK80-9": {
        "P_min": -12.5,
        "P_max": 12.5,
        "V_min": -50.0,
        "V_max": 50.0,
        "T_min": -18.0,
        "T_max": 18.0,
        "Kp_min": 0.0,
        "Kp_max": 500.0,
        "Kd_min": 0.0,
        "Kd_max": 5.0,
        "Kt_TMotor": 0.091,  # from TMotor website (actually 1/Kvll)
        "Current_Factor": 0.59,  # to correct the qaxis current
        "Kt_actual": 0.115,  # Need to use the right constant -- 0.115 by our calcs, 0.091 by theirs. At output leads to 1.31 by them and 1.42 by us.
        "GEAR_RATIO": 9.0,  # hence the 9 in the name
        "Use_derived_torque_constants": True,  # true if you have a better model
        "a_hat": [0.0, 1.15605006e00, 4.17389589e-04, 2.68556072e-01, 4.90424140e-02]
        #'a_hat' : [0.0,  8.23741648e-01, 4.57963164e-04,     2.96032614e-01, 9.31279510e-02]# [7.35415941e-02, 6.26896231e-01, 2.65240487e-04,     2.96032614e-01,  7.08736309e-02]# [-5.86860385e-02,6.50840079e-01,3.47461078e-04,8.58635580e-01,2.93809281e-01]
    },
    "AK10-9": {
        "P_min": -12.5,
        "P_max": 12.5,
        "V_min": -50.0,
        "V_max": 50.0,
        "T_min": -65.0,
        "T_max": 65.0,
        "Kp_min": 0.0,
        "Kp_max": 500.0,
        "Kd_min": 0.0,
        "Kd_max": 5.0,
        "Kt_TMotor": 0.16,  # from TMotor website (actually 1/Kvll)
        "Current_Factor": 0.59,  # UNTESTED CONSTANT!
        "Kt_actual": 0.206,  # UNTESTED CONSTANT!
        "GEAR_RATIO": 9.0,
        "Use_derived_torque_constants": False,  # true if you have a better model
    },
    "AK60-6": {
        "P_min": -12.5,
        "P_max": 12.5,
        "V_min": -50.0,
        "V_max": 50.0,
        "T_min": -15.0,
        "T_max": 15.0,
        "Kp_min": 0.0,
        "Kp_max": 500.0,
        "Kd_min": 0.0,
        "Kd_max": 5.0,
        "Kt_TMotor": 0.068,  # from TMotor website (actually 1/Kvll)
        "Current_Factor": 0.59,  # # UNTESTED CONSTANT!
        "Kt_actual": 0.087,  # UNTESTED CONSTANT!
        "GEAR_RATIO": 6.0,
        "Use_derived_torque_constants": False,  # true if you have a better model
    },
    "AK70-10": {
        "P_min": -12.5,
        "P_max": 12.5,
        "V_min": -50.0,
        "V_max": 50.0,
        "T_min": -25.0,
        "T_max": 25.0,
        "Kp_min": 0.0,
        "Kp_max": 500.0,
        "Kd_min": 0.0,
        "Kd_max": 5.0,
        "Kt_TMotor": 0.095,  # from TMotor website (actually 1/Kvll)
        "Current_Factor": 0.59,  # # UNTESTED CONSTANT!
        "Kt_actual": 0.122,  # UNTESTED CONSTANT!
        "GEAR_RATIO": 10.0,
        "Use_derived_torque_constants": False,  # true if you have a better model
    },
    "AK80-6": {
        "P_min": -12.5,
        "P_max": 12.5,
        "V_min": -76.0,
        "V_max": 76.0,
        "T_min": -12.0,
        "T_max": 12.0,
        "Kp_min": 0.0,
        "Kp_max": 500.0,
        "Kd_min": 0.0,
        "Kd_max": 5.0,
        "Kt_TMotor": 0.091,  # from TMotor website (actually 1/Kvll)
        "Current_Factor": 0.59,  # # UNTESTED CONSTANT!
        "Kt_actual": 0.017,  # UNTESTED CONSTANT!
        "GEAR_RATIO": 6.0,
        "Use_derived_torque_constants": False,  # true if you have a better model
    },
    "AK80-64": {
        "P_min": -12.5,
        "P_max": 12.5,
        "V_min": -8.0,
        "V_max": 8.0,
        "T_min": -144.0,
        "T_max": 144.0,
        "Kp_min": 0.0,
        "Kp_max": 500.0,
        "Kd_min": 0.0,
        "Kd_max": 5.0,
        "Kt_TMotor": 0.119,  # from TMotor website (actually 1/Kvll)
        "Current_Factor": 0.59,  # # UNTESTED CONSTANT!
        "Kt_actual": 0.153,  # UNTESTED CONSTANT!
        "GEAR_RATIO": 80.0,
        "Use_derived_torque_constants": False,  # true if you have a better model
    },
}
"""
A Dictionary containing the parameters of each type of motor, as well as the error
code definitions for the AK-series TMotor actuators.
You could use an optional torque model that accounts for friction losses if one is available.
So far, such a model is only available for the AK80-9.

This model comes from a linear regression with the following constants:
    - a_hat[0] = bias
    - a_hat[1] = standard torque constant multiplier
    - a_hat[2] = nonlinear torque constant multiplier
    - a_hat[3] = coloumb friction
    - a_hat[4] = gearbox friction

The model has the form:
τ = a_hat[0] + gr*(a_hat[1]*kt - a_hat[2]*abs(i))*i - (v/(ϵ + np.abs(v)) )*(a_hat[3] + a_hat[4]*np.abs(i))

with the following values:
    - τ = approximated torque
    - gr = gear ratio
    - kt = nominal torque constant
    - i = current
    - v = velocity
    - ϵ = signum velocity threshold
"""


@dataclass
class TMotorState:
    position: float
    velocity: float
    current: float
    temperature: float
    error: int
    acceleration: float


# Data structure to store MIT_command that will be sent upon update
class MIT_command:
    """Data structure to store MIT_command that will be sent upon update"""

    def __init__(self, position, velocity, kp, kd, current):
        """
        Sets the motor state to the input.

        Args:
            position: Position in rad
            velocity: Velocity in rad/s
            kp: Position gain
            kd: Velocity gain
            current: Current in amps
        """
        self.position = position
        self.velocity = velocity
        self.kp = kp
        self.kd = kd
        self.current = current


# motor state from the controller, uneditable named tuple
MIT_motor_state = namedtuple(
    "motor_state", "position velocity current temperature error"
)
"""
Motor state from the controller, uneditable named tuple
"""
