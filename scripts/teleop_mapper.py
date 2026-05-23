from mapper_contract import HandsState, ArmMode, ArmTarget, TeleopCommand

PINCH_OPEN_DISTANCE = 0.2
PINCH_CLOSED_DISTANCE = 0.05

XMIN, XMAX = -0.25, 0.25
YMIN, YMAX  = 0.25, 0.60

def normalize_pinch(raw: float) -> float:
    span = PINCH_OPEN_DISTANCE - PINCH_CLOSED_DISTANCE
    value = (PINCH_OPEN_DISTANCE - raw) / span
    return max(0.0, min(1.0, value))

def map_range(value, in_min, in_max, out_min, out_max):
    current_range =(value - in_min) / (in_max - in_min)
    target_range = out_min + current_range * (out_max - out_min)
    return target_range

def image_to_workspace(wrist_x, wrist_y):
    wrist_x = map_range(wrist_x, 0, 1, XMIN, XMAX)
    wrist_y = map_range(wrist_y, 0, 1, YMIN, YMAX)
    return wrist_x, wrist_y

def hands_to_command(hands: HandsState) -> TeleopCommand:
    if hands.left.visible:
        lx, ly = image_to_workspace(hands.left.wrist_x, hands.left.wrist_y)
        left = ArmTarget(x=lx, y=ly, theta=hands.left.rotation,
                        gripper=normalize_pinch(hands.left.pinch),
                        mode=ArmMode.TRACKING)
    else:
        left = ArmTarget(
            x=0.0,
            y=0.0,
            theta=0.0,
            gripper=0.0,
            mode=ArmMode.IDLE,
        )
    if hands.right.visible:
        lx, ly = image_to_workspace(hands.left.wrist_x, hands.left.wrist_y)
        right= ArmTarget(x=lx, y=ly, theta=hands.left.rotation,
                        gripper=normalize_pinch(hands.left.pinch),
                        mode=ArmMode.TRACKING)
    else:
        right = ArmTarget(
            x=0.0,
            y=0.0,
            theta=0.0,
            gripper=0.0,
            mode=ArmMode.IDLE,
        )
    return TeleopCommand(hands.timestamp, left, right)

