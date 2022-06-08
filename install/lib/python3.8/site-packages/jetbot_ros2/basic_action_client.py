import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from jetbot_actions.action import BasicControl


class BasicActionClient(Node):

    def __init__(self):
        super().__init__('basic_action_client')
        self._action_client = ActionClient(self, BasicControl, 'BasicControl')

    def send_goal(self, order):
        goal_msg = BasicControl.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = BasicActionClient()

    future = action_client.send_goal('advance')

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()