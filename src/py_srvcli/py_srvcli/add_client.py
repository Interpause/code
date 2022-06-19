from __future__ import annotations
import sys

from tutorial_interfaces.srv import AddThreeInts
import rclpy
from rclpy.node import Node

class AddClient(Node):

    def __init__(self):
        super().__init__('add_client')
        self._cli = self.create_client(AddThreeInts, 'add_three_ints')
        self._req = AddThreeInts.Request()

    def send_request(self, a: int, b: int, c: int):
        self._req.a = a
        self._req.b = b
        self._req.c = c

        while not self._cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.future = self._cli.call_async(self._req)


def main():
    rclpy.init()

    a,b,c = sys.argv[1:4]
    client = AddClient()
    client.send_request(int(a), int(b), int(c))

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                res = client.future.result()
            except Exception as e:
                client.get_logger().info(f'Service call failed {e}')
            else:
                client.get_logger().info(f'Result: {a} + {b} + {c} = {res.sum}')

            break

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()