import rclpy
from rclpy.node import Node

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
import seaborn as sns

from cpo_interfaces.msg import TDCP
from geometry_msgs.msg import PoseWithCovariance

sns.set_style("whitegrid")
plt.ion()  # make plotting interactive

# set up overhead plot
fig, ax = plt.subplots(figsize=[8, 4])
plot = ax.scatter([], [], c='C5', label='Receiver code solutions', s=1.5)
plot2 = ax.scatter([], [], c='C1', label='Vehicle position estimates', s=1.5)
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_title('Live Overhead View (not aligned)')
ax.set_xlabel('Easting (m)')
ax.set_ylabel('Northing (m)')
ax.legend(loc="upper right")
ax.set_aspect('equal')


class TdcpSubscriber(Node):
    """
    Plots both the regular GPS (code) solutions and the Carrier Phase Odometry estimates in real-time.
    *** Note: there will likely be some fixed offset between the two (both in position and time).
    """

    def __init__(self):
        super().__init__('tdcp_subscriber')
        self.tdcp_sub = self.create_subscription(
            TDCP,
            'tdcp',
            self.tdcp_callback,
            10)
        self.tdcp_sub
        self.enu_est_sub = self.create_subscription(
            PoseWithCovariance,
            'cpo_enu',
            self.enu_est_callback,
            10)
        self.enu_est_sub  # prevent unused variable warning

    def tdcp_callback(self, msg):
        """Subscribes to TDCP msgs from frontend and plots code solutions."""
        self.tdcp_msg_count += 1
        print('Code solution     {0:.2f}, {1:.2f}, {2:.2f} [m] {3}'.format(msg.enu_pos.x, msg.enu_pos.y, msg.enu_pos.z,
                                                                           self.tdcp_msg_count))
        point = (msg.enu_pos.x, msg.enu_pos.y)

        # plot East-North code solutions as we go
        array = plot.get_offsets()
        array = np.append(array, [point], axis=0)
        plot.set_offsets(array)
        fig.canvas.draw()

    def enu_est_callback(self, msg):
        """Subscribes to PoseWithCovariance msgs from backend and plots 2D position."""
        self.enu_msg_count += 1

        r_ab_inb = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        C_ba = R.from_quat(
            [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        r_ba_ina = -1 * C_ba.as_matrix().transpose() @ r_ab_inb.transpose()

        print('Est. ENU position {0:.2f}, {1:.2f}, {2:.2f} [m] {3}'.format(r_ba_ina[0], r_ba_ina[1],
                                                                           r_ba_ina[2], self.enu_msg_count))
        point = (r_ba_ina[0], r_ba_ina[1])

        # plot East-North estimates as we go
        array = plot2.get_offsets()
        array = np.append(array, [point], axis=0)
        plot2.set_offsets(array)
        ax.set_xlim(array[:, 0].min() - 10, array[:, 0].max() + 10)
        ax.set_ylim(array[:, 1].min() - 10, array[:, 1].max() + 10)
        fig.canvas.draw()

    tdcp_msg_count = 0
    enu_msg_count = 0


def main(args=None):
    rclpy.init(args=args)

    tdcp_subscriber = TdcpSubscriber()

    rclpy.spin(tdcp_subscriber)

    tdcp_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
