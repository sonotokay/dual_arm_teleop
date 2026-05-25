import pytest
from teleop_mapper import normalize_pinch, hands_to_command
from mapper_contract import HandObservation, HandsState, ArmMode

def test_normalize_pinch():
    assert normalize_pinch(0.2) == 0.0
    assert normalize_pinch(0.05) == 1.0 
    assert normalize_pinch(0.1) == pytest.approx(0.667, abs=0.001)
    assert normalize_pinch(0.4) == 0.0
    assert normalize_pinch(-0.1) == 1.0

def test_hands_to_command():
    # Different left/right observations so the test catches any L/R cross-talk.
    left_hand = HandObservation(True, 0.9, 0.3, 0.4, 0.5, 0.05)
    right_hand = HandObservation(True, 0.9, 0.7, 0.2, -0.5, 0.20)
    hands = HandsState(0.0, left_hand, right_hand)
    cmd = hands_to_command(hands)

    assert cmd.left.mode == ArmMode.TRACKING
    assert cmd.right.mode == ArmMode.TRACKING

    # Left: wrist_x=0.3 -> XMIN..XMAX (-0.5..0.5) maps to -0.20;
    # wrist_y=0.4 with y inverted over YMAX..YMIN (0.70..0.15) -> 0.48.
    assert cmd.left.x == pytest.approx(-0.20, abs=0.001)
    assert cmd.left.y == pytest.approx(0.48, abs=0.001)
    assert cmd.left.theta == pytest.approx(0.5, abs=0.001)
    assert cmd.left.gripper == pytest.approx(1.0, abs=0.001)  # pinch=0.05 -> fully closed

    # Right: wrist_x=0.7 -> +0.20; wrist_y=0.2 with y inverted -> 0.59.
    assert cmd.right.x == pytest.approx(0.20, abs=0.001)
    assert cmd.right.y == pytest.approx(0.59, abs=0.001)
    assert cmd.right.theta == pytest.approx(-0.5, abs=0.001)
    assert cmd.right.gripper == pytest.approx(0.0, abs=0.001)  # pinch=0.20 -> fully open

def test_hands_to_command_invisible():
    blind = HandObservation(False, 0.0, 0.0, 0.0, 0.0, 0.0)
    hands = HandsState(0.0, blind, blind)
    cmd = hands_to_command(hands)
    assert cmd.left.mode == ArmMode.IDLE
    assert cmd.right.mode == ArmMode.IDLE

def test_hands_to_command_mixed_visibility():
    """Only one hand visible -> only that arm tracks."""
    left = HandObservation(True, 0.9, 0.5, 0.5, 0.0, 0.1)
    blind = HandObservation(False, 0.0, 0.0, 0.0, 0.0, 0.0)
    cmd = hands_to_command(HandsState(0.0, left, blind))
    assert cmd.left.mode == ArmMode.TRACKING
    assert cmd.right.mode == ArmMode.IDLE

