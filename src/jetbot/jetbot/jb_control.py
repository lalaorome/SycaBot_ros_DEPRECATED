
import math as m
import numpy as np
import control


def align_with_goal(goal, state, cmd_vel):
    alpha = calculate_angle(goal,state)
    angle2turn = alpha - state[2]

    if angle2turn > 0.01 : 
        rot = 0.15
    elif angle2turn < -0.01 :
        rot = -0.15
    else :
        rot = 0.
    return rot

def calculate_angle(goal,state):
    delta = state[0:2] - goal
    alpha = m.atan2(delta[1],delta[0])
    alpha = (-np.sign(alpha)*m.pi + alpha)
    return alpha

class PIDController:
    def __init__(self, P=0.0, D=0.0, I=0.0, set_point=0, angle=False, Ts=0.01):
        self.Kp = P
        self.Kd = D
        self.Ki = I
        self.set_point = set_point
        self.previous_error = 0
        self.sum_error = 0

        self.angle = angle
        self.previous_value = 0
        self.transition = False
        
    def update(self, current_value):
        
        # Check for transition -pi -> pi or pi -> -pi, by checking difference of sign and then if in the good quadrant
        # if yes : put the value between 0 and 2pi or 0 -2pi 
        if self.transition :
            value = current_value + self.sign*2*m.pi
            if abs(value) < 2*m.pi :
                current_value = value


        if np.sign(current_value) != np.sign(self.previous_value) and abs(abs(self.previous_value) - m.pi) < m.pi/2 and not self.transition:
            self.transition = True
            self.sign = np.sign(self.previous_value)
            current_value = current_value + self.sign*2*m.pi

        

        # calculate P_term, I_term and D_term
        error = self.set_point - current_value
        P_term = self.Kp*error
        D_term = self.Kd*(error - self.previous_error)
        diff = error - self.previous_error
        I_term = self.Ki*self.sum_error
        self.sum_error += error

        self.previous_error = error
        self.previous_value = current_value
        
        return P_term + D_term + I_term, self.sum_error, diff, current_value, error

    def setPoint(self, set_point, reset=True):
        self.set_point = set_point
        if reset :
            self.previous_error = 0
            self.sum_error = 0
    
    def setPID(self, P=0.0, D=0.0,I=0.0):
        self.Kp = P
        self.Kd = D
        self.Ki = I
    
    def offset_angle(self, angle):
        angle = angle + m.pi
        return angle

class LQRcontroller:
    def __init__(self, R=None, Q=None, Ts=0.01, angle_ctrl=True):
        self.A = np.array([[1,0],[Ts,1]])
        self.B = np.array([[Ts],[Ts*Ts]])
        self.R = R
        self.Q = Q
        self.Ts = Ts
        self.error = [0.]
        self.i_error = [0.]
        self.angle_ctrl = angle_ctrl
        self.previous_value = 0
        self.set_point = 0
        self.transition = False
        
    def update(self, current_value):
        
        # Check for transition -pi -> pi or pi -> -pi, by checking difference of sign and then if in the good quadrant
        # if yes : put the value between 0 and 2pi or 0 -2pi 

        K,_,_ = control.dlqr(self.A,self.B,self.Q,self.R)
        inp = m.atan2(m.sin(self.set_point-current_value), m.cos(self.set_point-current_value))
        self.error.append(inp)
        self.i_error.append(self.Ts*self.error[-1] + self.error[-2])
        return K[0,0]*self.error[-1] + K[0,1]*self.i_error[-1], self.error, self.i_error, current_value
    
    def setLQR(self, Q, R, A=None, B=None):
        if A != None : self.A=A
        if B != None : self.B=B
        self.R=R
        self.Q=Q

    def setPoint(self, set_point, reset=True):
        self.set_point = set_point
        if reset :
            self.error = [0.]
            self.i_error = [0.]
    
    def check_transition(self, current_value):
        if self.transition :
            value = current_value + self.sign*2*m.pi
            if abs(value) < 2*m.pi :
                current_value = value

        if np.sign(current_value) != np.sign(self.previous_value) and abs(abs(self.previous_value) - m.pi) < m.pi/2 and not self.transition:
            self.transition = True
            self.sign = np.sign(self.previous_value)
            current_value = current_value + self.sign*2*m.pi
        return current_value

