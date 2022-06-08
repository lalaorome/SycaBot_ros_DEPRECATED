import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import PoseStamped
from message_filters import TimeSynchronizer, Subscriber


class OptiTrackLinker(Node):

    def __init__(self, jbType = 'waveshare'):
        super().__init__('SycaBot_W' + str(n_SycaBots), namespace= 'central')
        self.np_pose = None
        self.pose = None
        self.declare_parameter('n_SycaBots', 1)

        if jbType == 'waveshare':
            self.subscription = self.create_subscription(
                PoseStamped,
                '/vrpn_client_node/SycaBot_W' + str(n_SycaBots) +'/pose',
                self.listener_callback,
                10)

        self.publisher_ = self.create_publisher(PoseStamped, 'pose', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.position_giver)


    def position_giver(self):
        if self.pose is not None :
            self.publisher_.publish(self.pose)

    def listener_callback(self, p):
        self.np_pose = np.array([p.pose.position.x, p.pose.position.y])
        self.pose = p
        # self.get_logger().info('%d x position : %f \n y position : %f \n'
        # % (self.id, self.np_pose[0], self.np_pose[1]))
        


def main(args=None):
    rclpy.init(args=args)

    OptiTrack_subscriber = OptiTrackSubscriber()

    rclpy.spin(OptiTrack_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    OptiTrack_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
