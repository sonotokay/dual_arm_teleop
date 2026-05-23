import pytest
from teleop_mapper import normalize_pinch

def test_normalize_pinch():
    assert normalize_pinch(0.2) == 0.0                      # open
    assert normalize_pinch(0.05) == 1.0                     # close
    assert normalize_pinch(0.1) == pytest.approx(0.667, 0.001)     # float drift handling
    assert normalize_pinch(0.4) == 0.0                      # should clamp
    assert normalize_pinch(-0.1) == 1.0                     # should clamp

