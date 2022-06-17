
import math as m
import numpy as np


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