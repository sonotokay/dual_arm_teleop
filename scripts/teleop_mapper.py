from mapper_contract import HandObservation, HandsState, ArmMode, ArmTarget, TeleopCommand

PINCH_OPEN_DISTANCE = 0.2
PINCH_CLOSED_DISTANCE = 0.05

XMIN, XMAX = -0.25, 0.25
YMIN, YMAX  = 0.25, 0.60

def normalize_pinch(raw: float) -> float:
    span = PINCH_OPEN_DISTANCE - PINCH_CLOSED_DISTANCE
    value = (PINCH_OPEN_DISTANCE - raw) / span
    return max(0.0, min(1.0, value))

def map_range(value, in_min, in_max, out_min, out_max):
    
    pass

def image_to_workspace():
    pass


