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

from geometry_msgs.msg import PoseStamped, Twist
from interfaces.msg import Motor

from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data


from message_filters import ApproximateTimeSynchronizer, Subscriber

verbose = False
plot = True

class identifier(Node):
    def __init__(self):
        super().__init__('client')
        
        self.declare_parameter('id', 1)
        self.declare_parameter('deadzones', [0.1, 0.1, -0.1, -0.1]) #(Right, Left, -Right, -Left)
        self.declare_parameter('f_linear_coefs', [0.,0.])
        self.declare_parameter('f_angular_coefs', [0.,0.])

        self.id = self.get_parameter('id').value
        self.deadzones = self.get_parameter('deadzones').value
        self.f_linear_coefs = self.get_parameter('f_linear_coefs').value
        self.f_angular_coefs = self.get_parameter('f_angular_coefs').value

        self.rob_state = np.array([999.,0.,0.]) # x,y,theta: [-pi,pi]
        self.time_init = time.time()
        self.time = 0.
        qos = qos_profile_sensor_data
        self.RIGHT_WHEEL = 0
        self.LEFT_WHEEL = 1

        # Subscribe to pose topic
        self.pose_sub = self.create_subscription(PoseStamped, '/mocap_node/SycaBot_W'+ str(self.id) +'/pose', self.get_pose_cb, 1)

        # Create motor publisher
        self.vel_cmd_pub = self.create_publisher(Motor, '/SycaBot_W' + str(self.id) + '/cmd_vel', 10)
    

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
        self.rob_state = np.array([p.pose.position.x, p.pose.position.y, theta])
        self.time = float(p.header.stamp.sec) + float(p.header.stamp.nanosec)*10e-10

        if verbose :
            self.get_logger().info(
                        'rob_state of robot is : (x=%f, y=%f, theta=%f)\n' %(self.rob_state[0], self.rob_state[1], self.rob_state[2]))
        return

    def deadzone(self, n_runs=1):
        cmd_vel = Motor()

        for j in range(4): # Right and left Wheel deadzone identification
            for _ in range(n_runs):
                deadzone = []
                rclpy.spin_once(self)
                states = [self.rob_state]
                k = 0
                moved = False
                while not moved :
                    rclpy.spin_once(self)
                    states.append(self.rob_state)

                    if abs(states[-1][2] - states[-2][2]) > 0.2 : # If robot has moved
                        if j%2 == self.RIGHT_WHEEL : deadzone.append(cmd_vel.right)
                        else : deadzone.append(cmd_vel.left)
                        cmd_vel.left, cmd_vel.right = 0.,0.
                        self.vel_cmd_pub.publish(cmd_vel) # Stop the robot
                        moved = True
                        time.sleep(1.)
                    else :
                        if j == self.RIGHT_WHEEL : cmd_vel.right = 0.01 + k*0.01
                        elif j == self.LEFT_WHEEL : cmd_vel.left = 0.01 + k*0.01
                        elif j%2 == self.RIGHT_WHEEL : cmd_vel.right = -0.01 - k*0.01
                        else : cmd_vel.left = -0.01 - k*0.01
                        k+=1
                    self.vel_cmd_pub.publish(cmd_vel)
                    time.sleep(1.)

            self.deadzones[j] = np.max(deadzone)

        cmd_vel.left, cmd_vel.right = 0.,0.
        self.vel_cmd_pub.publish(cmd_vel) # Stop the robot
        return self.deadzones



def main(args=None):
    rclpy.init(args=args)
    idfier = identifier()

    n_inputs = 100
    Ts = 0.01
    interpolate = True
    # Geenrate inputs for the identification
    print(idfier.deadzones)
    u1,u2 = generate_inputs(Ts, idfier.deadzones, n=n_inputs, input='ramp', Vr_max=0.2, Vl_max=0.2)
           
    # Initialisation : Wait for pose
    while idfier.rob_state[0] == 999. :
            time.sleep(Ts)
            idfier.get_logger().info('No pose yet, waiting ...\n')
            rclpy.spin_once(idfier)
    idfier.get_logger().info('Got first pose !\n') 
    time.sleep(0.5)
    cmd_vel = Motor()
    times = []
    states_arr = []
    tspin = []
    tpub = []
    twait = []
    wheels_vel_cmd_arr = []
    time_stamps = []
    Ts_arr = []
    t_init = time.time()
    for i in range(n_inputs):
        tic = time.time()

        # Spin the node to get the pose
        rclpy.spin_once(idfier)
        states_arr.append(idfier.rob_state)
        if i == 0 : 
            theta = [states_arr[0][2]]
            x_predict = [states_arr[0][0]]
            y_predict = [states_arr[0][1]]
        time_stamps.append(idfier.time)
        ts = time.time()
        tspin.append(ts-tic)
        # To plot 
        times.append(time.time() - t_init)

        # Send velocity command to the robot
        cmd_vel.right, cmd_vel.left = u1[i],u2[i]
        cmd_vel.right, cmd_vel.left = 0.,0.
        idfier.vel_cmd_pub.publish(cmd_vel)
        wheels_vel_cmd_arr.append([cmd_vel.right, cmd_vel.left])

        # to plot publish time
        tp = time.time()
        tpub.append(tp - ts)
        
        # Wait
        time.sleep(Ts)
        tw = time.time()
        twait.append(tw- tp)

        # Real sampling time
        Ts_arr.append(time.time()-tic)

        # Transform lists to numpy arrays
        states = np.array(states_arr)
        wheels_vel_cmd = np.array(wheels_vel_cmd_arr)
        
        # Get velocity from measured inputs
        Ts_arr = [time_stamps[i+1]-time_stamps[i] for i in range(len(time_stamps)-1)]
        if i != 0 :
            vel_m = p2vel(states,Ts=Ts_arr,n=i+1)
            f_linear = linear_model.LinearRegression(fit_intercept=False)
            f_angular = linear_model.LinearRegression(fit_intercept=False)
            f_linear.fit(wheels_vel_cmd[:-1],vel_m[:,0])
            f_angular.fit(wheels_vel_cmd[:-1],vel_m[:,1])

            v_predict = f_linear.coef_[0]*wheels_vel_cmd[:,0]+ f_linear.coef_[1]*wheels_vel_cmd[:,1]
            w_predict = f_angular.coef_[0]*wheels_vel_cmd[:,0]+ f_angular.coef_[1]*wheels_vel_cmd[:,1]
            print(v_predict, w_predict)
            theta.append(theta[i] + w_predict[i]*Ts)
            x_predict.append(x_predict[i] + v_predict[i]*Ts*m.cos(theta[i]))
            y_predict.append(y_predict[i] + v_predict[i]*Ts*m.sin(theta[i]))
        
    # Stop the robot
    cmd_vel.left, cmd_vel.right = 0.,0.
    idfier.vel_cmd_pub.publish(cmd_vel)

    

    # if interpolate :
    #     f_linear = linear_model.LinearRegression(fit_intercept=False)
    #     f_angular = linear_model.LinearRegression(fit_intercept=False)
    #     f_linear.fit(wheels_vel_cmd[:-1],vel_m[:,0])
    #     f_angular.fit(wheels_vel_cmd[:-1],vel_m[:,1])

    #     v_predict = f_linear.coef_[0]*wheels_vel_cmd[:,0]+ f_linear.coef_[1]*wheels_vel_cmd[:,1]
    #     w_predict = f_angular.coef_[0]*wheels_vel_cmd[:,0]+ f_angular.coef_[1]*wheels_vel_cmd[:,1]

    #     indent = '  '
    #     with open('src/utilities/config/params_identification.yaml', 'r') as file:
    #         # read a list of lines into data
    #         data = file.readlines()
    #     try :
    #         data[3] = indent + indent + "f_linear_coefs: " + "[" + str(f_linear.coef_[0]) + "," + str(f_linear.coef_[1]) + "]" + "\n" 
    #     except :
    #         print('exception 1 raised\n')
    #         data.append("\n" + indent + indent + "f_linear_coefs: " + "[" + str(f_linear.coef_[0]) + "," + str(f_linear.coef_[1]) + "]" + "\n")
    #     try :
    #         data[4] = indent + indent + "f_angular_coefs: " + "[" + str(f_angular.coef_[0]) + "," + str(f_angular.coef_[1]) + "]" + "\n"
    #     except :
    #         print('exception 2 raised\n')
    #         data.append(indent + indent + "f_angular_coefs: " + "[" + str(f_angular.coef_[0]) + "," + str(f_angular.coef_[1]) + "]" + "\n")
    #     finally :
    #         with open('src/utilities/config/params_identification.yaml', 'w') as file:
    #             file.writelines( data )
                       
    # else :
    #     v_predict = idfier.f_linear_coefs[0]*wheels_vel_cmd[:,0]+ idfier.f_linear_coefs[1]*wheels_vel_cmd[:,1]
    #     w_predict = idfier.f_angular_coefs[0]*wheels_vel_cmd[:,0]+ idfier.f_angular_coefs[1]*wheels_vel_cmd[:,1]
    #     theta = [states[0,2]]
    #     x_predict = [states[0,0]]
    #     y_predict = [states[0,1]]
    #     for i in range(0,len(w_predict)):
    #         theta.append(theta[i] + w_predict[i]*Ts)
    #         x_predict.append(x_predict[i] + v_predict[i]*Ts*m.cos(theta[i]))
    #         y_predict.append(y_predict[i] + v_predict[i]*Ts*m.sin(theta[i]))

    if plot :

        # Plot informations (Loop time, ....)
        if interpolate :
            fig,ax = plt.subplots(1,3,figsize=(16,8))
            ax[0].plot(times,states[:,0])
            ax[0].set_ylabel('x [m]')
            ax[0].set_xlabel('time [s]')

            ax[1].plot(times,states[:,1])
            ax[1].set_ylabel('y [m]')
            ax[1].set_xlabel('time [s]')

            ax[2].plot(times, states[:,2])
            ax[2].plot(times, [m.pi for i in range(len(times))], label='pi')
            ax[2].plot(times, [-m.pi for i in range(len(times))], label='-pi')
            ax[2].set_ylabel('angle [rad]')
            ax[2].set_xlabel('time [s]')
            ax[2].legend()
        else : 
            fig,ax = plt.subplots(1,3,figsize=(16,8))
            ax[0].plot(times,states[:,0], '-', times, x_predict[1:], '--')
            ax[0].set_ylabel('x [m]')
            ax[0].set_xlabel('time [s]')
            ax[0].legend(['measured', 'predicted'])

            ax[1].plot(times,states[:,1], '-', times, y_predict[1:], '--')
            ax[1].set_ylabel('y [m]')
            ax[1].set_xlabel('time [s]')
            ax[1].legend(['measured', 'predicted'])

            ax[2].plot(times, states[:,2], '-', times, theta[1:], '--')
            ax[2].plot(times, [m.pi for i in range(len(times))], label='pi')
            ax[2].plot(times, [-m.pi for i in range(len(times))], label='-pi')
            ax[2].set_ylabel('angle [rad]')
            ax[2].set_xlabel('time [s]')
            ax[2].legend(['measured', 'predicted'])

        fig.suptitle('Sycabot_W'+str(idfier.id))

        # Plot communication and loop informations
        fig,ax = plt.subplots(1,2,figsize=(16,8))
        ax[0].bar('Loop Time', np.mean(tspin), width=0.35, yerr=np.std(tspin), label='mean spin time [s]')
        ax[0].bar('Loop Time', np.mean(tpub), width=0.35, yerr=np.std(tspin), bottom=np.mean(tspin), label='mean publish time [s]')
        ax[0].bar('Loop Time', np.mean(twait), width=0.35, yerr=np.std(tspin), bottom=np.mean(tspin) + np.mean(tpub), label='mean wait time [s]')
        ax[0].legend()

        ax[1].plot(times[:-1], Ts_arr)
        ax[1].set_ylabel('point by point frequency')
        ax[1].set_xlabel('time [s]')
        

        # Plot velocities
        fig,ax = plt.subplots(1,2,figsize=(16,8))
        ax[0].plot(times, wheels_vel_cmd[:,0])
        ax[0].set_ylabel('input Vr')
        ax[0].set_xlabel('time [s]')

        ax[1].plot(times, wheels_vel_cmd[:,1])
        ax[1].set_ylabel('input Vl')
        ax[1].set_xlabel('time [s]')

        # Plot measured velocities
        fig,ax = plt.subplots(1,2,figsize=(16,8))

        ax[0].plot(times[:-1], vel_m[:,0], '-', times, v_predict, '--')
        ax[0].set_ylabel('linear velocity')
        ax[0].set_xlabel('time [s]')
        ax[0].legend(['measured', 'interpolate'])

        ax[1].plot(times[:-1], vel_m[:,1], '-', times, w_predict, '--')
        ax[1].set_ylabel('angular velocity')
        ax[1].set_xlabel('time [s]')
        ax[1].legend(['measured', 'predicted'])

        plt.legend()
        plt.show()

    





if __name__ == '__main__':
    main()