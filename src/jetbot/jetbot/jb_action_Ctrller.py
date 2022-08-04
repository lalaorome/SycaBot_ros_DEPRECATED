import numpy as np
from jetbot.jb_utils import quat2eul
import time
import math as m
from Adafruit_MotorHAT import Adafruit_MotorHAT

from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import PoseStamped
from interfaces.action import Control
from interfaces.msg import Motor, Viz


class CtrllerActionServer(Node):
    """
    Abstract controller base node for supporting different Controller.
    """

    RIGHT_WHEEL = 0
    LEFT_WHEEL = 1
    X=0
    Y=1
    THETA=2
    Ts = 0.1
    LINEAR = 0
    ANGULAR = 1

    def __init__(self, name):
        super().__init__(f'{name}_controller')

        self.declare_parameter('id', 1)
        self.declare_parameter('deadzones', [0., 0., -0., -0.])
        self.declare_parameter('f_coefs', [35.85760339557938, 36.162967632872])
        self.declare_parameter('wheel_separation', 0.1016)  # 4 inches
        self.declare_parameter('wheel_diameter', 0.060325)  # 2 3/8 inches
        self.declare_parameter('max_linear_velocity', 0.2)
        self.declare_parameter('max_angular_velocity', 4.)

        self.id = self.get_parameter('id').value
        self.deadzones = self.get_parameter('deadzones').value
        self.f_coefs = self.get_parameter('f_coefs').value
        self.R = self.get_parameter('wheel_diameter').value/2
        self.L = self.get_parameter('wheel_separation').value
        self.max_lin_vel = self.get_parameter('max_linear_velocity').value
        self.max_ang_vel = self.get_parameter('max_angular_velocity').value

        # open Adafruit MotorHAT driver
        self.driver = Adafruit_MotorHAT(i2c_bus=1)
        
        # get motor objects from driver
        self.motors = {
            self.MOTOR_LEFT : self.driver.getMotor(self.MOTOR_LEFT),
            self.MOTOR_RIGHT : self.driver.getMotor(self.MOTOR_RIGHT)
        }
        
        self.pwm_channels = {
            self.MOTOR_LEFT : (1, 0),
            self.MOTOR_RIGHT : (2, 3)
        }

        cb_group = ReentrantCallbackGroup()
        qos = qos_profile_sensor_data

        self._action_server = ActionServer(
            self,
            Control,
            f'/SycaBot_W{self.id}/{name}_start_control',
            self.control_cb, callback_group=cb_group)

        self.rob_state = np.array([False,False,False]) # x,y,theta: [-pi,pi]
        self.velocity = np.array([0.,0.])
        self.previous_state  = np.array([0.,0.,0.])
        
        # Subscribe to pose topic
        self.pose_sub = self.create_subscription(PoseStamped, f'/mocap_node/SycaBot_W{self.id}/pose', self.get_pose_cb, 1, callback_group=cb_group)

        # Create motor publisher
        self.vel_cmd_pub = self.create_publisher(Motor, f'/SycaBot_W{self.id}/cmd_vel', qos, callback_group=cb_group)

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
        self.previous_state = self.rob_state
        self.rob_state = np.array([p.pose.position.x, p.pose.position.y, theta])
        self.time = float(p.header.stamp.sec) + float(p.header.stamp.nanosec)*10e-10
        self.velocity = self.get_velocity()
        return
    
    def control_cb(self, goal_handle):
        '''
        Does the control of the robot. 
        Each controller should implement its own definition.
        It should indicate whether the control was a success or not.
        '''
        raise NotImplementedError('CtrllerActionServer subclasses should implement control_cb()')

    def sendVelCmd_deprecated(self,Vr,Vl):
        vel_cmd = Motor()
        vel_cmd.right = Vr
        vel_cmd.left = Vl
        self.vel_cmd_pub.publish(vel_cmd)
    
    def stop_deprecated(self):
        vel_cmd = Motor()
        vel_cmd.right = 0.
        vel_cmd.left = 0.
        self.vel_cmd_pub.publish(vel_cmd)

    def velocities2wheelinput(self, v, w):
        Vr = (v + self.L*w/2)/(self.R*self.f_coefs[0])
        Vl = (v - self.L*w/2)/(self.R*self.f_coefs[1])
        return Vr,Vl

    def wheelinput2velocities(self, Vr,Vl):
        v = self.R*(self.f_coefs[0]*Vr + self.f_coefs[1]*Vl)/2
        w = self.R*(self.f_coefs[0]*Vr - self.f_coefs[1]*Vl)/self.L
        return v,w

    def wait4pose(self):
        # Initialisation : Wait for pose
        while not np.all(self.rob_state) :
                time.sleep(0.1)
                self.get_logger().info('No pose yet, waiting again...\n')

        return
    
    def get_velocity(self):
        v = np.linalg.norm(self.rob_state[0:2]-self.previous_state[0:2])/self.Ts
        w = m.atan2(m.sin(self.rob_state[2]-self.previous_state[2]),m.cos(self.rob_state[2]-self.previous_state[2]))/self.Ts
        return np.array([v,w])
    
    def destroy_node(self):
        self.get_logger().info(f"shutting down, stopping robot...")
        self.stop()
        
    def parameters_callback(self, params):
        for param in params:
            if param.name == 'left_trim':
                self.left_trim = param.value
            elif param.name == 'right_trim':
                self.right_trim = param.value
            elif param.name == 'max_pwm':
                self.max_pwm = param.value
            elif param.name == 'wheel_separation':
                self.wheel_separation = param.value
            else:
                raise ValueError(f'unknown parameter {param.name}')
                
        return SetParametersResult(successful=True)

    def stop(self):
        self.set_speed(0,0)
    
    def set_speed(self, right, left):
        """
        Sets the motor speeds between [-1.0, 1.0]
        """
        self._set_pwm(self.MOTOR_LEFT, -left, self.left_trim)
        self._set_pwm(self.MOTOR_RIGHT, -right, self.right_trim)
      
    def _set_pwm(self, motor, value, trim):
        # apply trim and convert [-1,1] to PWM value
        pwm = int(min(max((abs(value) + trim) * self.max_pwm, 0), self.max_pwm))
        self.motors[motor].setSpeed(pwm)

        # set the motor direction
        ina, inb = self.pwm_channels[motor]
        
        if value > 0:
            self.motors[motor].run(Adafruit_MotorHAT.FORWARD)
            self.driver._pwm.setPWM(ina, 0, pwm * 16)
            self.driver._pwm.setPWM(inb, 0, 0)
        elif value < 0:
            self.motors[motor].run(Adafruit_MotorHAT.BACKWARD)
            self.driver._pwm.setPWM(ina, 0, 0)
            self.driver._pwm.setPWM(inb, 0, pwm * 16)
        else:
            self.motors[motor].run(Adafruit_MotorHAT.RELEASE)
            self.driver._pwm.setPWM(ina, 0, 0)
            self.driver._pwm.setPWM(inb, 0, 0)


if __name__ == '__main__':
    raise NotImplementedError("CtrllerActionServer shouldn't be instantiated directly - instead use PPCtrller.py, MPC.py, ...")