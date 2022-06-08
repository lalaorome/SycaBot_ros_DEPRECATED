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
    if axis=='x' :
        angle = 2*m.asin(q[0])
    if axis=='y' :
        angle = 2*m.asin(q[1])
    if axis=='z' :
        angle = 2*m.asin(q[2])
    
    return angle