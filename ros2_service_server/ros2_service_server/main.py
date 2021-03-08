# service server to add two integers and return the result
# MathsSrv from ros2_interface
# 
import rclpy
from rclpy.node import Node 

from ros2_interface.srv import MathsSrv

class AddNumbers(Node):

    def __init__(self):
        super().__init__('add_numbers_server')
        self.add_srv = self.create_service(MathsSrv,'add_two_numbers', self.math_operation_cb)

    def math_operation_cb(self, request, response):
        response.result = request.a + request.b
        self.get_logger().info('Client request ADD "%d" and "%d"'%(request.a, request.b))
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        add_numbers_ = AddNumbers()
        rclpy.spin(add_numbers_)

    except KeyboardInterrupt:
        add_numbers_.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()