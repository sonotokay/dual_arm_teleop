import json
import socket

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

from mapper_contract import HandObservation, HandsState


BRIDGE_HOST = "172.22.128.1"
BRIDGE_PORT = 5555

# joystick-style velocity: hand offset from image center -> end-effector velocity
DEADZONE = 0.08          # ignore small offsets near center (noise / resting hand)
LINEAR_GAIN = 0.15       # m/s at full offset; tune if too fast/slow


class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_node")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.get_logger().info(f"connecting to bridge {BRIDGE_HOST}:{BRIDGE_PORT} ...")
        self.sock.connect((BRIDGE_HOST, BRIDGE_PORT))
        self.sock.setblocking(False)
        self.get_logger().info("connected to bridge")
        self.recv_buffer = ""

        self.pub = {
            "left": self.create_publisher(
                TwistStamped, "/left_servo_node/delta_twist_cmds", 10
            ),
            "right": self.create_publisher(
                TwistStamped, "/right_servo_node/delta_twist_cmds", 10
            ),
        }
        self.frame = {"left": "left_base_link", "right": "right_base_link"}

        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz

    def read_lines(self):
        """Drain available bytes, return complete JSON lines."""
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

    @staticmethod
    def _axis(offset: float) -> float:
        """Center-offset (-0.5..0.5) -> velocity, with deadzone."""
        if abs(offset) < DEADZONE:
            return 0.0
        return offset * 2.0 * LINEAR_GAIN   # *2 so full edge ~= LINEAR_GAIN

    def publish_twist(self, side: str, hand: HandObservation):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame[side]

        if hand.visible:
            # image: x 0(left)->1(right), y 0(top)->1(bottom)
            dx = hand.wrist_x - 0.5
            dy = hand.wrist_y - 0.5
            # map hand right/left -> base-frame y; hand up/down -> base-frame z
            msg.twist.linear.y = -self._axis(dx)   # sign so hand-right moves arm one way; flip if reversed
            msg.twist.linear.z = -self._axis(dy)   # hand up (smaller y) -> +z
        # if not visible: all zeros -> arm holds (Servo gets zero twist)

        self.pub[side].publish(msg)

    def tick(self):
        lines = self.read_lines()
        if not lines:
            return
        try:
            raw = json.loads(lines[-1])   # freshest message, drop backlog
        except json.JSONDecodeError:
            return
        hands = self.parse_hands(raw)
        self.publish_twist("left", hands.left)
        self.publish_twist("right", hands.right)


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