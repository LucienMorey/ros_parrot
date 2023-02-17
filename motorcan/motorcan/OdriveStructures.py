from dataclasses import dataclass

from odrive.enums import AxisError, AxisState


@dataclass
class OdriveMotorState:
    axis_state: AxisState
    axis_error: AxisError


@dataclass
class OdriveEncoderFeedback:
    position: float
    velocity: float


@dataclass
class OdriveMotorFeedback:
    position: float
    velocity: float
    current: float
