from __future__ import annotations
from dataclasses import dataclass, field, replace
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from py_pubsub.job import Job, JobCfg


# this is the name that displays in debugging tools
NODE_NAME = "minimal_publisher_test"

@dataclass
class PublisherCfg(JobCfg):
    topic: str = "topic"
    """Topic to publish to."""
    pub_msg: str = "Hello World"
    """Msg to publish."""


@dataclass
class Publisher(Job):
    """Simple publisher."""

    ini_cfg: PublisherCfg = field(default_factory=PublisherCfg)

    def attach_params(self, node, params):
        super(Publisher, self).attach_params(node, params)

        self.topic = node.declare_parameter('topic', params.topic)
        self.pub_msg = node.declare_parameter('pub_msg', params.pub_msg)

    def attach_behaviour(self, node, params: PublisherCfg):
        super(Publisher, self).attach_behaviour(node, params)

        self._publisher = node.create_publisher(String, params.topic, 10)
        self._timer = node.create_timer(1.0 / params.max_rate, self._timer_cb)
        self._i = 0
        self.log.info(f'Publishing to "{params.topic}" at {params.max_rate}Hz.')

    def _timer_cb(self):
        msg = String()
        msg.data = f"[{self._i}] {self.pub_msg}"
        self._publisher.publish(msg)
        self.log.info(f"Publish: {msg.data}")
        self._i += 1

    def on_params_change(self, node, params):
        for name, param in params.items():
            # this isn't done in job.py to give chance to reject config
            # TODO: we need a current config dict mirroring parameters
            # AKA maybe dont directly setattr anywhere.
            setattr(self, name, param)

        self.log.info(f'Config changed: {params}.')
        if 'max_rate' in params or 'topic' in params:
            self.log.info(f'Config change requires restart. Restarting...')
            node.destroy_publisher(self._publisher)
            node.destroy_timer(self._timer)
            self.attach_behaviour(node, replace(self.ini_cfg, **params))
        return True


def main(args=None):
    if __name__ == "__main__" and args is None:
        args = sys.argv

    rclpy.init(args=args)

    node = Node(NODE_NAME)

    cfg = PublisherCfg(max_rate=5, topic="hello_topic", pub_msg="hey")
    Publisher(node, cfg)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
