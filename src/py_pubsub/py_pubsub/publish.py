from __future__ import annotations
from dataclasses import dataclass
import sys

import rclpy
from std_msgs.msg import String

from py_pubsub.nice_node import NiceNode


@dataclass(eq=False)
class Publisher(NiceNode):
    """Simple publisher."""

    topic: str = "topic"
    """topic to publish to"""
    pub_msg: str = "Hello World"
    """msg to publish"""

    def __post_init__(self):
        super(Publisher, self).__post_init__()

        self._publisher = self.create_publisher(String, self.topic, 10)
        self._timer = self.create_timer(1.0 / self.max_fps, self._timer_cb)
        self._i = 0

    def _timer_cb(self):
        msg = String()
        msg.data = f"[{self._i}] {self.pub_msg}"
        self._publisher.publish(msg)
        self._log.info(f"Publish: {msg.data}")
        self._i += 1


def main(args=None):
    if __name__ == "__main__" and args is None:
        args = sys.argv

    rclpy.init(args=args)

    publisher = Publisher(
        node_name="minimal_publisher_test",
        max_fps=5,
        topic="hello_topic",
    )

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
