import rclpy
from rclpy.node import Node

import numpy as np
from numpy.linalg import norm
import math as m
import time 
from jetbot.jb_utils import quat2eul
from jetbot.jb_control import align_with_goal

from geometry_msgs.msg import PoseStamped, Point, Twist
from std_msgs.msg import Bool
from interfaces.srv import Task


from message_filters import ApproximateTimeSynchronizer, Subscriber

verbose = False

class jetbot_client(Node):
    def __init__(self):
        super().__init__('tester')
        self.state = np.array([0,0,0])
        self.exec_state = False
        self.declare_parameter('id', 1)
        self.id = self.get_parameter('id').value
        self.goal = np.array([1,1])

        # # Subscribe to pose topic
        # self.pose_sub = self.create_subscription(PoseStamped, '/vrpn_client_node/SycaBot_W'+ str(self.id) +'/pose', self.get_pose_cb, 10)
        # # Subscribe to execution state topic
        # self.exec_state_sub = self.create_subscription(Bool, '/central_pc/exec_state', self.get_exec_state_cb, 10)
        # Create motor publisher
        self.vel_cmd_pub = self.create_publisher(Twist, '/SycaBot_W' + str(self.id) + '/cmd_vel', 10)

        # # Define task service
        # self.task_cli = self.create_client(Task, '/central_pc/task_srv')
        # while not self.task_cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...\n')
   
    
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
    jb_client = jetbot_client()
    start = False

    i = 0
    while(1):
        time.sleep(0.5)
        
        
        if i <= 10 :
            if verbose : jb_client.get_logger().info('Publishing\n')
            vel = Twist()
            vel.linear.x = 0.2
        else :
            vel.linear.x = 0.0
        #jb_client.get_logger().info('id : %d\n' %(jb_client.id))
        jb_client.vel_cmd_pub.publish(vel)
        i+=1
        # cmd_vel = Twist()
        # cmd_vel.angular.z = align_with_goal(jb_client.goal, jb_client.state, cmd_vel)
        # jb_client.vel_cmd_pub.publish(cmd_vel)
        rclpy.spin_once(jb_client, timeout_sec=1)
    
    jb_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
