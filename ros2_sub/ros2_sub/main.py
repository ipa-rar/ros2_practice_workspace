# Subscribe 'MathsMsg'
# from Topic 'maths_topic'
import rclpy
from rclpy.node import Node 
from ros2_interface.msg import MathsMsg

class NumberSubscriber(Node):
    def __init__(self):
        super().__init__('number_subscriber')
        self.sub = self.create_subscription(
            MathsMsg,
            'maths_topic',
            self.subscriber_cb,
            10
        )
    
    def subscriber_cb(self, msg):
        self.get_logger().info('"%d" is converted to"%f'%(msg.result, msg.result**2))

def main(args=None):
    rclpy.init(args=args)
    try:
        number_sub = NumberSubscriber()
        rclpy.spin(number_sub)
    except KeyboardInterrupt:
        number_sub.destroy_node()
        rclpy.shutdown()


if __name__ =='__main__':
    main()