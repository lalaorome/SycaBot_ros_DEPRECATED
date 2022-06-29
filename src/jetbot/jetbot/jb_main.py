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
from jetbot.jb_control import calculate_angle, PIDController, LQRcontroller

from geometry_msgs.msg import PoseStamped, Point, Twist
from std_msgs.msg import Bool
from interfaces.srv import Task


from message_filters import ApproximateTimeSynchronizer, Subscriber

verbose = False

class jetbot_client(Node):
    def __init__(self):
        super().__init__('client')
        self.state = np.array([0,0,0]) # x,y,theta: [-pi,pi]
        self.exec_state = False
        self.declare_parameter('id', 1)
        self.id = self.get_parameter('id').value
        self.goal = np.array([-1.,2.])
        self.time_init = time.time()

        # Subscribe to pose topic
        self.pose_sub = self.create_subscription(PoseStamped, '/vrpn_client_node/SycaBot_W'+ str(self.id) +'/pose', self.get_pose_cb, 10)
        # Subscribe to execution state topic
        self.exec_state_sub = self.create_subscription(Bool, '/exec_state', self.get_exec_state_cb, 10)
        # Create motor publisher
        self.vel_cmd_pub = self.create_publisher(Twist, '/SycaBot_W' + str(self.id) + '/cmd_vel', 10)

        # Define task service
        # self.task_cli = self.create_client(Task, '/task_srv')
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
        self.state = np.array([p.pose.position.x, p.pose.position.y, theta])

        if verbose :
            self.get_logger().info(
                        'State of robot is : (x=%f, y=%f, theta=%f)\n' %(self.state[0], self.state[1], self.state[2]))
        return

def main(args=None):
    try :
        rclpy.init(args=args)
        jb_client = jetbot_client()
        start = True

        # # Initialisation : Ask for its initial task
        # jb_client.task_request()
        # while rclpy.ok():
        #     rclpy.spin_once(jb_client)
        #     if jb_client.future.done():
        #         try:
        #             response = jb_client.future.result()
        #         except Exception as e:
        #             if verbose :
        #                 jb_client.get_logger().info(
        #                     'Service call failed %r' % (e,))
        #         else:
        #             if not verbose :
        #                 jb_client.get_logger().info(
        #                     'Pose of task is (x=%f, y=%f, z=%f)\n' %(response.task.x, response.task.y, response.task.z))
        #                 jb_client.goal = np.array([response.task.x, response.task.y])
        #                 alpha = calculate_angle(jb_client.goal, jb_client.state)
        #         break

        # Set LQR
        alpha = calculate_angle(jb_client.goal, jb_client.state)
        R=60.
        Q=np.array([[100,0],[0,1]])
        LQRw = LQRcontroller(R=R, Q=Q)
        LQRw.setPoint(alpha)

        


        stabilized = 0
        turned = False
        finish=False
        time_serie = [0]
        angle = [jb_client.state[2]]
        w = [0]
        alpha_sp = [alpha]
        err=[0]
        derr=[0]
        ierr=[0]
        pose=[]

        vel = Twist()
        iter = 0
        t_init = time.time()
        previous_angle = 0
        while(not finish):
            rclpy.spin_once(jb_client)
            time.sleep(0.01)
            start = jb_client.exec_state
            
            # Step 1 : Wait for start signal
            if not start : jb_client.get_logger().info('Execution has not started\n')
            
            else : # Do control
                # vel.linear.x = 0.12
                # vel.angular.z = 1.5
                ## Do PID Control
                #vel.angular.z, sum_error, error_diff, current_value, error = PIDw.update(jb_client.state[2])
                # offset = 0.85
                # if vel.angular.z <= offset :
                #     if vel.angular.z >= 0. : vel.angular.z = offset
                #     elif vel.angular.z >= -offset : vel.angular.z = -offset
                # err.append(error)
                # derr.append(error_diff)
                # serr.append(sum_error)

                ## Do LQR control
                vel.angular.z, err, ierr,current_value = LQRw.update(jb_client.state[2])
                
                angle.append(jb_client.state[2])
                pose.append(jb_client.state[0:2])
                w.append(vel.angular.z)
                time_serie.append(time.time()-t_init)

                
                if abs(jb_client.state[2] - alpha) < 0.2 and abs(jb_client.state[2]-previous_angle) < 0.00001 :
                    stabilized += 1
                    if stabilized == 10 : turned = True

                if turned or time_serie[-1] > 5:
                    vel.linear.x=0.15
                    alpha = calculate_angle(jb_client.goal, jb_client.state)
                    LQRw.setPoint(alpha, reset=False)
                    # iter += 1
                    # vel.linear.x=0.
                    # vel.angular.z=0.
                    # if iter == 5 : finish = True
                


                if norm(jb_client.state[0:2]-jb_client.goal) < 0.05 or time_serie[-1] > 20: 
                    vel.linear.x=0.
                    vel.angular.z=0.
                    finish = True
                alpha_sp.append(alpha)
                previous_angle = jb_client.state[2]
                jb_client.vel_cmd_pub.publish(vel)
        
        vel.linear.x=0.
        vel.angular.z=0.
        jb_client.vel_cmd_pub.publish(vel)
        fig,ax = plt.subplots(1,4,figsize=(16,8))
        pose = np.array(pose)
        ax[0].scatter(pose[:,0], pose[:,1], s=5)
        ax[0].scatter(jb_client.goal[0], jb_client.goal[1], s=20)
        ax[0].set_ylabel('x [m]')
        ax[0].set_xlabel('y [m]')
        ax[0].set_xlim((-2,2))
        ax[0].set_ylim((-4,4))

        ax[1].plot(time_serie,w)
        ax[1].set_ylabel('angular vel. [% max speed]')
        ax[1].set_xlabel('time [s]')

        ax[2].plot(time_serie, angle, label='angle value')
        ax[2].plot(time_serie, [m.pi for i in range(len(time_serie))], label='pi')
        ax[2].plot(time_serie, [-m.pi for i in range(len(time_serie))], label='-pi')
        ax[2].plot(time_serie, alpha_sp, label= 'alpha')
        ax[2].set_ylabel('angle [rad]')
        ax[2].set_xlabel('time [s]')
        ax[2].legend()

        ierr= np.array(ierr)/np.max(ierr)
        ax[3].plot(time_serie,err, label='error')
        # ax[3].plot(time_serie,derr, label='error diff')
        ax[3].plot(time_serie,ierr, label='error sum')
        ax[3].set_ylabel('error')
        ax[3].set_xlabel('time [s]')
        ax[3].legend()
        plt.legend()
        plt.show()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    except Exception as e:
        print('Something went wrong.\n' + str(e))
    finally :
        jb_client.destroy_node()
        rclpy.shutdown()
        return
    


if __name__ == '__main__':
    main()
    