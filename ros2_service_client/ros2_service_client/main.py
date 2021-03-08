import rclpy
from rclpy.node import Node 

import sys
from ros2_interface.srv import MathsSrv

class AsncClient(Node):

    def __init__(self):
        super().__init__('async_client')
        self.client = self.create_client(MathsSrv, 'add_two_numbers')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting...')
        self.req = MathsSrv.Request()
    
    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.future = self.client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    try:
        add_client = AsncClient()
        add_client.send_request()

        while rclpy.ok():
            rclpy.spin_once(add_client)
            if add_client.future.done():
                try:
                    response = add_client.future.result()
                except Exception as e:
                    add_client.get_logger().info('Service call failed %r'% (e,))
                else:
                    add_client.get_logger().info('Server responds "%d" + "%d" = "%d"'%
                    (add_client.req.a, add_client.req.b, response.result))
                break

    except KeyboardInterrupt:
        add_client.destroy_node()
        rclpy.shutdown()
    
if __name__=='__main__':
    main()