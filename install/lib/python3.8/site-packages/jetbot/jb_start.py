import rclpy
from rclpy.node import Node

import numpy as np
from numpy.linalg import norm
from scipy.optimize import linear_sum_assignment, minimize
import math as m
from jetbot.utils import quat2eul

from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool
from interfaces.srv import Task


from message_filters import ApproximateTimeSynchronizer, Subscriber

# SYCABOT_ID = 1
verbose = True

class jetbot_client(Node):
    def __init__(self, SYCABOT_ID):
        super().__init__('client', namespace = 'Sycabot_W' + str(SYCABOT_ID))
        self.state = [0,0,0]
        self.exec_state = False
        self.id = SYCABOT_ID

        # Subscribe to pose topic
        self.pose_sub = self.create_subscription(PoseStamped, '/SycaBot_W' + str(SYCABOT_ID) + '/pose', self.get_pose_cb, 10)
        # Subscribe to execution state topic
        self.exec_state_sub = self.create_subscription(Bool, '/central_pc/exec_state', self.get_exec_state_cb, 10)

        # Define task service
        self.task_cli = self.create_client(Task, '/central_pc/task_srv')
        while not self.task_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...\n')
        
    
    def get_exec_state_cb(self, state):
        '''
        Get execution state. 

        arguments :
            state (Bool) = state of the execution
                True : execution is ongoing
                False : nothing should happen
        ------------------------------------------------
        return :
        '''
        self.exec_state = state.data
        return
        
    
    def task_request(self):
        self.task_req = Task.Request()
        self.task_req.id = self.id
        self.future = self.task_cli.call_async(self.task_req) 

    def get_pose_cb(self, p):
        '''
        Get jetbot positions.

        arguments :
            p (PoseStamped) = position of the jetbots
        ------------------------------------------------
        return :
        '''
        quat = [p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w]
        theta = quat2eul(quat)
        if verbose :
            self.state = np.array([p.pose.position.x, p.pose.position.y, theta])
            self.get_logger().info(
                        'State of robot is : (x=%f, y=%f, theta=%f)\n' %(self.state[0], self.state[1], self.state[2]))
        return

def main(args=None):
    rclpy.init(args=args)
    SYCABOT_ID = int(input('ID of the Sycabots ?'))
    jb_client = jetbot_client(SYCABOT_ID)
    start = False

    # Initialisation : Ask for its initial task
    jb_client.task_request()
    while rclpy.ok():
        rclpy.spin_once(jb_client)
        if jb_client.future.done():
            try:
                response = jb_client.future.result()
            except Exception as e:
                if verbose :
                    jb_client.get_logger().info(
                        'Service call failed %r' % (e,))
            else:
                if verbose :
                    jb_client.get_logger().info(
                        'Pose of task is (x=%f, y=%f, z=%f)\n' %(response.task.x, response.task.y, response.task.z))
            break
    
    
    while(1):
        rclpy.spin_once(jb_client)
        start = jb_client.exec_state
        # Step 1 : Wait for start signal
        if not start : jb_client.get_logger().info('Execution has not started\n')
        
        else : # Do control
            jb_client.get_logger().info('Execution is starting\n')

        
    
    rclpy.spin(jb_client)
    jb_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
