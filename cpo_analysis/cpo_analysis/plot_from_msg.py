import rclpy
from rclpy.node import Node

from cpo_interfaces.msg import TDCP


class TdcpSubscriber(Node):

  def __init__(self):
    super().__init__('tdcp_subscriber')
    self.subscription = self.create_subscription(
      TDCP,
      'tdcp',
      self.tdcp_callback,
      10)
    self.subscription  # prevent unused variable warning

  def tdcp_callback(self, msg):
    self.get_logger().info('I heard something')


def main(args=None):
  rclpy.init(args=args)

  tdcp_subscriber = TdcpSubscriber()

  rclpy.spin(tdcp_subscriber)

  tdcp_subscriber.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
