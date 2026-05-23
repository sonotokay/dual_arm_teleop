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
    left_hand = HandObservation(True, 0.2, 0.3, 0.4, 0.5, 0.6)
    right_hand = HandObservation(True, 0.2, 0.3, 0.4, 0.5, 0.6)
    hands = HandsState(0.0, left_hand, right_hand)
    cmd = hands_to_command(hands)

    assert cmd.left.mode == ArmMode.TRACKING
    assert cmd.right.mode == ArmMode.TRACKING
    assert cmd.left.x == pytest.approx(-0.10, abs=0.001)   
    assert cmd.left.y == pytest.approx(0.39, abs=0.001)    
    assert cmd.left.theta == pytest.approx(0.5, abs=0.001) 
    assert cmd.left.gripper == pytest.approx(0.0, abs=0.001)  

def test_hands_to_command_invisible():
    blind = HandObservation(False, 0.0, 0.0, 0.0, 0.0, 0.0)
    hands = HandsState(0.0, blind, blind)
    cmd = hands_to_command(hands)
    assert cmd.left.mode == ArmMode.IDLE
    assert cmd.right.mode == ArmMode.IDLE

