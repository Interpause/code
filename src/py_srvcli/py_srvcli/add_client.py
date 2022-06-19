from __future__ import annotations
import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class AddClient(Node):

    def __init__(self):
        super().__init__('add_client')
        self._cli = self.create_client(AddTwoInts, 'add_two_ints')
        self._req = AddTwoInts.Request()

    def send_request(self, a: int, b: int):
        self._req.a = a
        self._req.b = b

        while not self._cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.future = self._cli.call_async(self._req)


def main():
    rclpy.init()

    a,b = sys.argv[1:3]
    client = AddClient()
    client.send_request(int(a), int(b))

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                res = client.future.result()
            except Exception as e:
                client.get_logger().info(f'Service call failed {e}')
            else:
                client.get_logger().info(f'Result: {a} + {b} = {res.sum}')

            break

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()