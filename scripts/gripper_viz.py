"""
Bridges /left_gripper_cmd and /right_gripper_cmd (Float64, [0=open, 1=closed])
to sensor_msgs/JointState so RViz animates the Robotiq 2F-85 grippers.

Gripper joints live in the URDF but NOT in ros2_control's hardware interface,
so joint_state_broadcaster ignores them. robot_state_publisher merges whatever
it receives on /joint_states, so this node's output goes straight to the display.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# Robotiq 2F-85 (simplified): knuckle travels 0 (open) → 0.804 rad (closed).
# Left and right joints use mirrored axes so the same positive value closes both.
_KNUCKLE_MAX = 0.804  # rad

# (joint_suffix, multiplier_of_knuckle_pos) — prefix is prepended at runtime.
_JOINT_MIMIC = [
    ("robotiq_85_left_knuckle_joint",      1.0),
    ("robotiq_85_right_knuckle_joint",     1.0),
    ("robotiq_85_left_finger_tip_joint",   1.0),
    ("robotiq_85_right_finger_tip_joint",  1.0),
]


class GripperVizNode(Node):
    def __init__(self):
        super().__init__("gripper_viz_node")

        self._cmd = {"left": 0.0, "right": 0.0}

        for side in ("left", "right"):
            self.create_subscription(
                Float64, f"/{side}_gripper_cmd",
                lambda msg, s=side: self._on_cmd(s, msg),
                10,
            )

        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(0.02, self._publish)  # 50 Hz

    def _on_cmd(self, side: str, msg: Float64):
        self._cmd[side] = max(0.0, min(1.0, float(msg.data)))

    def _publish(self):
        now = self.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = now

        for side in ("left", "right"):
            knuckle_pos = self._cmd[side] * _KNUCKLE_MAX
            prefix = f"{side}_"
            for suffix, mult in _JOINT_MIMIC:
                js.name.append(prefix + suffix)
                js.position.append(knuckle_pos * mult)

        self._pub.publish(js)


def main():
    rclpy.init()
    node = GripperVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
