import json
import math
import socket

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TwistStamped, Quaternion
from std_msgs.msg import Float64

import tf2_ros
from tf2_ros import TransformException

from mapper_contract import HandObservation, HandsState, ArmMode
from one_euro import OneEuro
from teleop_mapper import hands_to_command


BRIDGE_HOST = "172.22.128.1"
BRIDGE_PORT = 5555

REACH_FORWARD = 0.40   # m, fixed base_link x; motion is in the y-z plane

# P-control gains and twist clips.
KP_LIN = 4.0
KP_ANG = 4.0
MAX_LIN = 0.50   # m/s
MAX_ANG = 1.00   # rad/s

FILTER_MIN_CUTOFF = 1.0
FILTER_BETA = 0.05


def quat_mul(a: Quaternion, b: Quaternion) -> Quaternion:
    out = Quaternion()
    out.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y
    out.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x
    out.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    out.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    return out


def quat_inv(q: Quaternion) -> Quaternion:
    out = Quaternion()
    out.x = -q.x
    out.y = -q.y
    out.z = -q.z
    out.w = q.w
    return out


def yaw_quat(theta: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(theta / 2.0)
    q.w = math.cos(theta / 2.0)
    return q


def quat_to_rotvec(q: Quaternion) -> tuple[float, float, float]:
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    if qw < 0.0:  # short-path
        qx, qy, qz, qw = -qx, -qy, -qz, -qw
    nrm = math.sqrt(qx * qx + qy * qy + qz * qz)
    if nrm < 1e-9:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(nrm, qw)
    s = angle / nrm
    return (qx * s, qy * s, qz * s)


def clip_mag(v: tuple[float, float, float], max_mag: float) -> tuple[float, float, float]:
    m = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if m <= max_mag or m == 0.0:
        return v
    s = max_mag / m
    return (v[0] * s, v[1] * s, v[2] * s)


class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_node")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.get_logger().info(f"connecting to bridge {BRIDGE_HOST}:{BRIDGE_PORT} ...")
        self.sock.connect((BRIDGE_HOST, BRIDGE_PORT))
        self.sock.setblocking(False)
        self.get_logger().info("connected to bridge")
        self.recv_buffer = ""

        self.twist_pub = {
            side: self.create_publisher(
                TwistStamped, f"/{side}_servo_node/delta_twist_cmds", 10
            )
            for side in ("left", "right")
        }

        self.gripper_pub = {
            side: self.create_publisher(Float64, f"/{side}_gripper_cmd", 10)
            for side in ("left", "right")
        }

        self.filters = {
            side: {
                ch: OneEuro(FILTER_MIN_CUTOFF, FILTER_BETA)
                for ch in ("wrist_x", "wrist_y", "rotation", "pinch")
            }
            for side in ("left", "right")
        }

        home_q = Quaternion(x=-0.70710678, y=0.70710678, z=0.0, w=0.0)
        self.home_orient = {"left": home_q, "right": home_q}

        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz

    def read_lines(self):
        try:
            chunk = self.sock.recv(4096).decode("utf-8")
            if chunk == "":
                self.get_logger().warn("bridge closed connection")
            self.recv_buffer += chunk
        except BlockingIOError:
            pass
        except OSError as e:
            self.get_logger().warn(f"socket error: {e}")
        lines = []
        while "\n" in self.recv_buffer:
            line, self.recv_buffer = self.recv_buffer.split("\n", 1)
            if line.strip():
                lines.append(line)
        return lines

    def parse_hands(self, raw: dict) -> HandsState:
        def obs(d):
            return HandObservation(
                visible=d["visible"],
                confidence=d["confidence"],
                wrist_x=d["wrist"][0],
                wrist_y=d["wrist"][1],
                rotation=d["rotation"],
                pinch=d["pinch"],
            )
        return HandsState(
            timestamp=raw["t"],
            left=obs(raw["left"]),
            right=obs(raw["right"]),
        )

    def filter_hand(self, side: str, obs: HandObservation, t: float) -> HandObservation:
        f = self.filters[side]
        if not obs.visible:
            for ch in f.values():
                ch.reset()
            return obs
        return HandObservation(
            visible=True,
            confidence=obs.confidence,
            wrist_x=f["wrist_x"](obs.wrist_x, t),
            wrist_y=f["wrist_y"](obs.wrist_y, t),
            rotation=f["rotation"](obs.rotation, t),
            pinch=f["pinch"](obs.pinch, t),
        )

    def publish_twist_from_target(self, side: str, target):
        try:
            tf = self.tf_buffer.lookup_transform(
                f"{side}_base_link", f"{side}_tool0", Time()
            )
        except TransformException:
            return

        cur_pos = tf.transform.translation
        cur_orient = tf.transform.rotation

        ex = REACH_FORWARD - cur_pos.x
        ey = (-target.x) - cur_pos.y
        ez = target.y - cur_pos.z
        lin = clip_mag((KP_LIN * ex, KP_LIN * ey, KP_LIN * ez), MAX_LIN)

        q_target = quat_mul(self.home_orient[side], yaw_quat(target.theta))
        q_err = quat_mul(q_target, quat_inv(cur_orient))
        _, _, rz = quat_to_rotvec(q_err)
        ang_z = max(-MAX_ANG, min(MAX_ANG, KP_ANG * rz))
        ang = (0.0, 0.0, ang_z)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"{side}_base_link"
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = lin
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = ang
        self.twist_pub[side].publish(msg)

    def publish_gripper(self, side: str, value: float):
        msg = Float64()
        msg.data = float(value)
        self.gripper_pub[side].publish(msg)

    def tick(self):
        lines = self.read_lines()
        if not lines:
            return
        try:
            raw = json.loads(lines[-1])
        except json.JSONDecodeError:
            return

        hands = self.parse_hands(raw)
        filtered = HandsState(
            timestamp=hands.timestamp,
            left=self.filter_hand("left", hands.left, hands.timestamp),
            right=self.filter_hand("right", hands.right, hands.timestamp),
        )
        cmd = hands_to_command(filtered)

        for side, target in (("left", cmd.left), ("right", cmd.right)):
            if target.mode == ArmMode.TRACKING:
                self.publish_twist_from_target(side, target)
                self.publish_gripper(side, target.gripper)


def main():
    rclpy.init()
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
