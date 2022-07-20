import math as m
import numpy as np
from sklearn import linear_model
import matplotlib.pyplot as plt

RIGHT_WHEEL=0
LEFT_WHEEL=1

def generate_inputs(Ts, deadzones, n=1,input='ramp', Vr_max=0.2, Vl_max=0.2, var_f = False):
    '''
    Generate velocity inputs for identification.

    arguments :
        Ts         (float)  = Sampling time
        Deadzones  (array) [R,L,-R,-L] = right and left wheels deadzones 
        n          (int)    = number of inputs to generate
        Vr_max, Vl_max     (float)  = Maximum input velocity to reach for left and right wheel
        input (string) = Type of input generation we want : 'ramp', 'step'
    ------------------------------------------------
    return :
        Vl,Vr (lists of inputs) [n] = wheel velocities
        v,w   (lists of inputs) [n] = linear (x_r) and angular (z_r) velocities (robot coordinates)
    '''
    right = []
    left = []

    #################################
    ##   Generate right,left inputs here  ##
    #################################

    
    if input=='ramp':
        for i in range(n) :
            right.append((Vr_max*(i+1)/n)+np.sign(Vr_max)*(deadzones[RIGHT_WHEEL]+0.03))
            left.append((Vl_max*(i+1)/n)+np.sign(Vl_max)*(deadzones[LEFT_WHEEL]+0.03))
    elif input=='step' :
        for i in range(n) :
            right.append(Vr_max)
            left.append(Vl_max)

    elif input=='random_sine':
        time = np.linspace(0,n*Ts, num=n)
        f1 = np.random.random()*0.3
        f2 = np.random.random()*0.3
        phase_shift = np.random.random()*2*np.pi
        right = np.sin(2 * np.pi * f1* time + phase_shift)*Vr_max + Vr_max
        left = np.sin(2 * np.pi * f2 * time)*Vl_max + Vl_max

    elif input=='random_sine_cosine':
        time = np.linspace(0,n*Ts, num=n)
        f = np.random.random()*0.6

        right = np.sin(2 * np.pi * f * time)*Vr_max*1.5
        left = np.cos(2 * np.pi * f * time)*Vl_max*1.5

    else :
        print('WRONG INPUT !')


    #################################

    return right,left

def quat2eul(q, axis='z'):
    '''
    Transform a quaternion to an euler angle.

    arguments :
        q (array) = array containing the for quaternion components [qx,qy,qz,qw]
    ------------------------------------------------
    return :
    '''
    if axis=='z' :
        t1 = 2. * (q[3]*q[2] + q[0]*q[1])
        t2 = 1. - 2.*(q[1]*q[1] + q[2]*q[2])
        angle = m.atan2(t1,t2)
    return angle

def p2vel(p, Ts, n, vel_commands):
    '''
    Transform measured positions to corresponding velocities

    arguments :
        p (numpy array) [n+1,3] : positions of the robot at each time step
    ------------------------------------------------
    return :
        vel (numpy array) [n,2] : corresponding velocities at each time step
    '''

    velocities = np.zeros((n-1,2))
    for i in range(0,n-1) :


        velocities[i,0] = np.sign(vel_commands[i,0]+vel_commands[i,1])*m.sqrt(((p[i+1,0]-p[i,0])**2 + (p[i+1,1]-p[i,1])**2))/Ts[i] # v
        velocities[i,1] = (m.atan2(m.sin(p[i+1,2]-p[i,2]),m.cos(p[i+1,2]-p[i,2])))/Ts[i] # w

    return velocities


def vel2wheelinput(v,w, R=0.060325/2, L=0.1016) :

    max_rpm = 200
    left = v - w * L / 2.0
    right = v + w * L / 2.0
    
    # convert velocities to [-1,1]
    max_speed = (max_rpm / 60.0) * 2.0 * m.pi * R

    left = np.maximum(np.minimum(left, max_speed), -max_speed) / max_speed
    right = np.maximum(np.minimum(right, max_speed), -max_speed) / max_speed

    return left, right

