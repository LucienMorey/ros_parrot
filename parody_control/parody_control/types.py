from dataclasses import dataclass


@dataclass
class JointLimits:
    upper_limit: float
    lower_limit: float
