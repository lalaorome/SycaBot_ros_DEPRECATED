import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import math as m

from jetbot_actions.action import BasicControl


class BasicActionServer(Node):

    def __init__(self):
        super().__init__('basic_action_server')
        self._action_server = ActionServer(
            self,
            BasicControl,
            'BasicControl',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        if goal_handle.request.order == 'advance' :
            self.get_logger().info('move')
        else :
            self.get_logger().info('didnt move')
        
        goal_handle.succeed()
        result = BasicControl.Result()
        result.achieved = True
        return result


def main(args=None):
    rclpy.init(args=args)

    basic_action_server = BasicActionServer()

    rclpy.spin(basic_action_server)


if __name__ == '__main__':
    main()