import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class TwistTest(Node):
    def __init__(self, side: str):
        super().__init__(f"{side}_twist_test")
        self.frame = f"{side}_base_link"
        self.pub = self.create_publisher(
            TwistStamped, f"/{side}_servo_node/delta_twist_cmds", 10
        )
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz

    def tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame

        amp = 0.08
        msg.twist.linear.x = amp * math.sin(2 * math.pi * 0.20 * t)
        msg.twist.linear.y = amp * math.sin(2 * math.pi * 0.30 * t)
        msg.twist.linear.z = amp * math.sin(2 * math.pi * 0.15 * t)

        self.pub.publish(msg)


def main():
    side = sys.argv[1] if len(sys.argv) > 1 else "left"
    if side not in ("left", "right"):
        print(f"usage: {sys.argv[0]} [left|right]", file=sys.stderr)
        sys.exit(1)
    rclpy.init()
    rclpy.spin(TwistTest(side))


if __name__ == "__main__":
    main()
