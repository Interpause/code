from __future__ import annotations
from tutorial_interfaces.srv import AddThreeInts

import rclpy
from rclpy.node import Node

class AddService(Node):

    def __init__(self):
        super().__init__('minimal_add_service')
        self._srv = self.create_service(AddThreeInts, 'add_three_ints', self._cb)

    def _cb(self, req: AddThreeInts.Request, res: AddThreeInts.Response):
        res.sum = req.a + req.b + req.c
        self.get_logger().info(f'Request: {req.a} + {req.b} + {req.c} = {res.sum}')
        return res


def main():
    rclpy.init()

    srv = AddService()

    rclpy.spin(srv)
    srv.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()