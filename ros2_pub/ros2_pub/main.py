# Publish 'MathsMsg'
# to Topic 'maths_topic'
# 0.2 sec interval publishing
# QoS is 10 by default
import rclpy
from rclpy.node import Node 
from ros2_interface.msg import MathsMsg

class NumberPublisher(Node):
    def __init__(self):
        super().__init__('number_publisher')
        self.pub = self.create_publisher(
            MathsMsg, 
            'maths_topic', 
            10)
        time_period = 0.2
        self.timer = self.create_timer(time_period, self.publisher_cb)
        self.i = 0

    def publisher_cb(self):
        msg = MathsMsg()
        msg.result = self.i
        self.pub.publish(msg)
        self.get_logger().info('publishing: "%d"'% msg.result)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    try:
        number_publisher = NumberPublisher()
        rclpy.spin(number_publisher)
    except KeyboardInterrupt:
        number_publisher.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()