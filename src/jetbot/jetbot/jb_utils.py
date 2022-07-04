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