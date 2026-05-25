from mapper_contract import HandsState, ArmMode, ArmTarget, TeleopCommand

PINCH_OPEN_DISTANCE = 0.2
PINCH_CLOSED_DISTANCE = 0.05

XMIN, XMAX = -0.5, 0.5
YMIN, YMAX  = 0.15, 0.70

def normalize_pinch(raw: float) -> float:
    span = PINCH_OPEN_DISTANCE - PINCH_CLOSED_DISTANCE
    value = (PINCH_OPEN_DISTANCE - raw) / span
    return max(0.0, min(1.0, value))

def map_range(value, in_min, in_max, out_min, out_max):
    current_range =(value - in_min) / (in_max - in_min)
    target_range = out_min + current_range * (out_max - out_min)
    return target_range

def image_to_workspace(wrist_x, wrist_y):
    # Image y=0 is the top of the frame; invert so hand-up gives a larger workspace y.
    ws_x = map_range(wrist_x, 0, 1, XMIN, XMAX)
    ws_y = map_range(wrist_y, 0, 1, YMAX, YMIN)
    return ws_x, ws_y

def hands_to_command(hands: HandsState) -> TeleopCommand:
    def to_target(obs):
        if not obs.visible:
            return ArmTarget(x=0.0, y=0.0, theta=0.0, gripper=0.0, mode=ArmMode.IDLE)
        x, y = image_to_workspace(obs.wrist_x, obs.wrist_y)
        return ArmTarget(
            x=x, y=y, theta=obs.rotation,
            gripper=normalize_pinch(obs.pinch),
            mode=ArmMode.TRACKING,
        )
    return TeleopCommand(hands.timestamp, to_target(hands.left), to_target(hands.right))

