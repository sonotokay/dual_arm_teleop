"""
Teleoperation demonstration recorder.

Records timestamped episodes of joint positions, gripper state, and end-effector
poses suitable for imitation learning (behaviour cloning, ACT, etc.).

Usage
-----
    python3 scripts/data_recorder.py                  # saves to ./episodes/
    python3 scripts/data_recorder.py --out /tmp/demos --episode-time 30

Each episode is saved as a NumPy archive (episodes/episode_NNNN.npz):

    timestamps       (T,)      Unix time in seconds
    joint_names      (12,)     string array — same order every episode
    joint_positions  (T, 12)   arm joint angles in radians
    gripper          (T, 2)    normalised [0=open, 1=closed], columns: [left, right]
    ee_pose_left     (T, 7)    [x, y, z, qx, qy, qz, qw] in world frame
    ee_pose_right    (T, 7)

Press  Enter  to start a new episode (or end the current one).
Press  Ctrl-C  to quit.
"""

import argparse
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import tf2_ros
from tf2_ros import TransformException
from rclpy.time import Time

ARM_JOINTS = [
    "left_shoulder_pan_joint",
    "left_shoulder_lift_joint",
    "left_elbow_joint",
    "left_wrist_1_joint",
    "left_wrist_2_joint",
    "left_wrist_3_joint",
    "right_shoulder_pan_joint",
    "right_shoulder_lift_joint",
    "right_elbow_joint",
    "right_wrist_1_joint",
    "right_wrist_2_joint",
    "right_wrist_3_joint",
]


def _tf_to_pose(tf) -> list[float]:
    t = tf.transform.translation
    r = tf.transform.rotation
    return [t.x, t.y, t.z, r.x, r.y, r.z, r.w]


class RecorderNode(Node):
    def __init__(self, out_dir: str, episode_time: float | None):
        super().__init__("data_recorder")

        self._out_dir = out_dir
        self._episode_time = episode_time
        os.makedirs(out_dir, exist_ok=True)

        self._joint_index: dict[str, int] = {}  # filled on first /joint_states message
        self._latest_joints: list[float] = [0.0] * 12
        self._latest_gripper = [0.0, 0.0]  # [left, right]

        self._recording = False
        self._episode_start: float = 0.0
        self._buf_time: list[float] = []
        self._buf_joints: list[list[float]] = []
        self._buf_gripper: list[list[float]] = []
        self._buf_ee_left: list[list[float]] = []
        self._buf_ee_right: list[list[float]] = []

        self._episode_count = self._count_existing()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        for side in ("left", "right"):
            idx = 0 if side == "left" else 1
            self.create_subscription(
                Float64, f"/{side}_gripper_cmd",
                lambda msg, i=idx: self._on_gripper(i, msg),
                10,
            )

        self.create_timer(0.02, self._tick)  # 50 Hz sample rate

    def _count_existing(self) -> int:
        existing = [
            f for f in os.listdir(self._out_dir) if f.startswith("episode_") and f.endswith(".npz")
        ]
        return len(existing)

    def _on_joints(self, msg: JointState):
        if not self._joint_index:
            self._joint_index = {n: i for i, n in enumerate(msg.name)}
        for arm_joint in ARM_JOINTS:
            idx = self._joint_index.get(arm_joint)
            if idx is not None and idx < len(msg.position):
                self._latest_joints[ARM_JOINTS.index(arm_joint)] = msg.position[idx]

    def _on_gripper(self, side_idx: int, msg: Float64):
        self._latest_gripper[side_idx] = float(msg.data)

    def _lookup_ee(self, side: str) -> list[float] | None:
        try:
            tf = self._tf_buffer.lookup_transform("world", f"{side}_tool0", Time())
            return _tf_to_pose(tf)
        except TransformException:
            return None

    def _tick(self):
        if not self._recording:
            return

        if self._episode_time and (time.time() - self._episode_start) >= self._episode_time:
            self.end_episode()
            return

        ee_left = self._lookup_ee("left")
        ee_right = self._lookup_ee("right")
        if ee_left is None or ee_right is None:
            return

        self._buf_time.append(time.time())
        self._buf_joints.append(list(self._latest_joints))
        self._buf_gripper.append(list(self._latest_gripper))
        self._buf_ee_left.append(ee_left)
        self._buf_ee_right.append(ee_right)

    def start_episode(self):
        self._buf_time.clear()
        self._buf_joints.clear()
        self._buf_gripper.clear()
        self._buf_ee_left.clear()
        self._buf_ee_right.clear()
        self._episode_start = time.time()
        self._recording = True
        print(f"\n[recorder] episode {self._episode_count + 1} started — press Enter to stop")

    def end_episode(self):
        self._recording = False
        n = len(self._buf_time)
        if n < 5:
            print("[recorder] too short, discarded")
            return
        fname = os.path.join(self._out_dir, f"episode_{self._episode_count + 1:04d}.npz")
        np.savez(
            fname,
            timestamps=np.array(self._buf_time, dtype=np.float64),
            joint_names=np.array(ARM_JOINTS),
            joint_positions=np.array(self._buf_joints, dtype=np.float32),
            gripper=np.array(self._buf_gripper, dtype=np.float32),
            ee_pose_left=np.array(self._buf_ee_left, dtype=np.float32),
            ee_pose_right=np.array(self._buf_ee_right, dtype=np.float32),
        )
        duration = self._buf_time[-1] - self._buf_time[0]
        self._episode_count += 1
        print(f"[recorder] saved {fname}  ({n} steps, {duration:.1f}s)")


def _keyboard_loop(node: RecorderNode):
    print("[recorder] press Enter to start/stop episodes, Ctrl-C to quit")
    while True:
        try:
            input()
        except EOFError:
            break
        if node._recording:
            node.end_episode()
        else:
            node.start_episode()


def main():
    parser = argparse.ArgumentParser(description="Record teleoperation episodes")
    parser.add_argument("--out", default="episodes", help="Output directory")
    parser.add_argument("--episode-time", type=float, default=None,
                        help="Auto-stop episode after this many seconds")
    args = parser.parse_args()

    rclpy.init()
    node = RecorderNode(out_dir=args.out, episode_time=args.episode_time)

    kb_thread = threading.Thread(target=_keyboard_loop, args=(node,), daemon=True)
    kb_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node._recording:
            node.end_episode()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
