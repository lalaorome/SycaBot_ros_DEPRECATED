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
from interfaces.msg import BeaconMsg

from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data



from message_filters import ApproximateTimeSynchronizer, Subscriber

SYCABOT_ID = 1
verbose = True
plot = True

class jetbot(Node):
    def __init__(self):
        super().__init__('test')
        self.declare_parameter('id', 1)
        self.id = self.get_parameter('id').value

        self.beacon = self.create_publisher(BeaconMsg, 'beacon', 10)
        self.timer = self.create_timer(1., self.pub_beacon_cb)

    def pub_beacon_cb(self):
        data = BeaconMsg()
        data.id = self.id
        self.beacon.publish(data)
        

def main(args=None):

    rclpy.init(args=args)
    jb = jetbot()
    rclpy.spin(jb)
    return
    


if __name__ == '__main__':
    main()
    