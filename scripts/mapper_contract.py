from dataclasses import dataclass
from enum import Enum

@dataclass
class HandObservation:
    visible: bool       # detection variable
    confidence: float   # detection/tracking confidence
    wrist_x: float      # normalized 0-1, horizontal
    wrist_y: float      # normalized 0-1, vertical
    rotation: float     # radians
    pinch: float        # raw distance between thumb to index

@dataclass
class HandsState:
    timestamp: float
    left: HandObservation
    right: HandObservation

class ArmMode(Enum):
    TRACKING = "tracking"           # live and tracking
    HOLDING_LAST = "holding"        # holding last target
    RETURNING_HOME = "returning"    # returning to home
    IDLE = "idle"                   # no target yet

@dataclass
class ArmTarget:
    x: float            # meters
    y: float            # meters
    theta: float        # radians
    gripper: float      # 0-1, open-closed
    mode: ArmMode       # state signal for consumer

@dataclass
class TeleopCommand:
    timestamp: float
    left: ArmTarget
    right: ArmTarget

