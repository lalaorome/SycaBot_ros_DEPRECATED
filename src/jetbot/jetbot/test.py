from logging import raiseExceptions
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

import numpy as np
from numpy.linalg import norm
import math as m
import time 
import matplotlib.pyplot as plt
from jetbot.jb_utils import quat2eul
from jetbot.jb_control import calculate_angle, LQRcontroller

from geometry_msgs.msg import PoseStamped, Point, Twist
from std_msgs.msg import Bool
from interfaces.srv import Task

from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data


from message_filters import ApproximateTimeSynchronizer, Subscriber

SYCABOT_ID = 1
verbose = False
plot = True

class jetbot_client(Node):
    def __init__(self):
        super().__init__('client')
        self.state = np.array([0,0,0]) # x,y,theta: [-pi,pi]
        self.exec_state = False
        self.declare_parameter('id', 1)
        self.id = self.get_parameter('id').value
        self.goal = None
        self.time_init = time.time()
        qos = qos_profile_sensor_data

        # # Subscribe to pose topic
        # self.pose_sub = self.create_subscription(PoseStamped, '/vrpn_client_node/SycaBot_W'+ str(self.id) +'/pose', self.get_pose_cb, qos)

        # # Subscribe to execution state topic
        # self.exec_state_sub = self.create_subscription(Bool, '/exec_state', self.get_exec_state_cb, qos)

        # Create motor publisher
        self.vel_cmd_pub = self.create_publisher(Twist, '/SycaBot_W' + str(self.id) + '/cmd_vel', qos)


def main(args=None):
    finish = False
    rclpy.init(args=args)             
    t_init = time.time()
    jb_client = jetbot_client()
    while(not finish):
        t = time.time()
        time.sleep(0.1)
        cmd_vel = Twist()
        if t - t_init > 3. :
            cmd_vel.angular.z = 0.
            finish = True
        else : 
            cmd_vel.angular.z = 1.5

        jb_client.vel_cmd_pub.publish(cmd_vel)
        time.sleep(0.1)
        # rclpy.spin_once(jb_client)
    cmd_vel.angular.z = 0.
    jb_client.vel_cmd_pub.publish(cmd_vel)
    


if __name__ == '__main__':
    main()
    