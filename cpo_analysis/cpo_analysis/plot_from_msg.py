import rclpy
from rclpy.node import Node

import matplotlib.pyplot as plt
import numpy as np

from cpo_interfaces.msg import TDCP

plt.ion()  # make plotting interactive

# set up overhead plot
fig, ax = plt.subplots()
plot = ax.scatter([], [])
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)


# todo: make pretty


class TdcpSubscriber(Node):

  def __init__(self):
    super().__init__('tdcp_subscriber')
    self.tdcp_sub = self.create_subscription(
      TDCP,
      'tdcp',
      self.tdcp_callback,
      10)
    self.tdcp_sub  # prevent unused variable warning
    # todo: add PoseWithCovariance subscriber

  def tdcp_callback(self, msg):
    self.msg_count += 1
    self.get_logger().info(
      'Found ENU position {0:.2f}, {1:.2f}, {2:.2f} [m] {3}'.format(msg.enu_pos.x, msg.enu_pos.y, msg.enu_pos.z,
                                                                    self.msg_count))
    point = (msg.enu_pos.x, msg.enu_pos.y)

    # plot East-North code solutions as we go
    array = plot.get_offsets()
    array = np.append(array, [point], axis=0)
    plot.set_offsets(array)
    ax.set_xlim(array[:, 0].min() - 1, array[:, 0].max() + 1)
    ax.set_ylim(array[:, 1].min() - 1, array[:, 1].max() + 1)
    fig.canvas.draw()

  msg_count = 0


def main(args=None):
  rclpy.init(args=args)

  tdcp_subscriber = TdcpSubscriber()

  rclpy.spin(tdcp_subscriber)

  tdcp_subscriber.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
