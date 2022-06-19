from __future__ import annotations
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node

class AddService(Node):

    def __init__(self):
        super().__init__('minimal_add_service')
        self._srv = self.create_service(AddTwoInts, 'add_two_ints', self._cb)

    def _cb(self, req: AddTwoInts.Request, res: AddTwoInts.Response):
        res.sum = req.a + req.b
        self.get_logger().info(f'Request: {req.a} + {req.b} = {res.sum}')
        return res


def main():
    rclpy.init()

    srv = AddService()

    rclpy.spin(srv)
    srv.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()