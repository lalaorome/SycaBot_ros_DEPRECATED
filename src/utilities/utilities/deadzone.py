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
from scipy.interpolate import interp1d
from sklearn import linear_model, svm

from utilities.toolbox import *
from utilities.identification import identifier

from geometry_msgs.msg import PoseStamped, Twist
from interfaces.msg import Motor

from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data


from message_filters import ApproximateTimeSynchronizer, Subscriber

verbose = False
plot = True

def main(args=None):
    rclpy.init(args=args)
    idfier = identifier()
    

    # Initialisation : Wait for pose
    while idfier.rob_state[0] == 999. :
            time.sleep(0.1)
            idfier.get_logger().info('No pose yet, waiting again...\n')
            rclpy.spin_once(idfier)

    deadzones = idfier.deadzone()
    indent = '  '
    f = open("src/utilities/config/params_identification.yaml", "w")
    f.write("/**:\n"+ indent +"ros__parameters:")
    f.write("\n"+indent + indent + "deadzones: " + str(deadzones))
    f.close()


if __name__ == '__main__':
    main()