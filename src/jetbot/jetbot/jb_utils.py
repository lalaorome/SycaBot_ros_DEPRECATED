import math as m
import numpy as np

SYCABOT_ID = 1

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

    velocities = np.zeros(2*(n-1))
    for i in range(0,2*(n-1),2) :
        idx = int(i/2)
        velocities[i] = np.sign(vel_commands[idx,0]+vel_commands[idx,1])*m.sqrt(((p[idx+1,0]-p[idx,0])**2 + (p[idx+1,1]-p[idx,1])**2))/Ts[idx] # v
        velocities[i+1] = (m.atan2(m.sin(p[idx+1,2]-p[idx,2]),m.cos(p[idx+1,2]-p[idx,2])))/Ts[idx] # w
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
