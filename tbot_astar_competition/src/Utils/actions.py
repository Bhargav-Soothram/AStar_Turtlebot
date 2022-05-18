# -*- coding: utf-8 -*-
"""
Created on Mon Mar 21 19:28:54 2022

@author: Bhargav Kumar
"""

import numpy as np
import math
from Utils.node import *
# from Utils.mapping import *
from Utils.mapping4competition import *


def diffDriveConstraint(crnt_state, u_l, u_r, clearance, r=3.3, L=80., dt=0.01):
    
    # Max linear velocity in cm/sec
    MAX_LIN_VEL_BURGER = 22
    # Max angular velocity in rad/sec
    MAX_ANG_VEL_BURGER = 2.84

    leftAngularVelocity = u_l * 2 * np.pi / 60.0
    rightAngularVelocity = u_r * 2 * np.pi / 60.0
    x, y, theta = crnt_state[0], crnt_state[1], crnt_state[2]                                        
    t = 0
    cost = 0
    point_set1 = list()
    point_set2 = list()
    # theta_rad = np.radians(theta)
    rotation_z = min((r/L)*(rightAngularVelocity-leftAngularVelocity), MAX_ANG_VEL_BURGER)
    
    while t < 1:
        dvx = min((r/2)*(leftAngularVelocity + rightAngularVelocity)*math.cos(theta), MAX_LIN_VEL_BURGER)
        dvy = min((r/2)*(leftAngularVelocity + rightAngularVelocity)*math.sin(theta), MAX_LIN_VEL_BURGER)
        dx = dvx*dt
        dy = dvy*dt
        dtheta = rotation_z*dt
        point_set1.append((x, y))
        point_set2.append((x+dx, y+dy))
        x += dx
        y += dy
        cost += np.sqrt(dx ** 2 + dy ** 2)
        theta += dtheta
        t += dt
        if is_ObstacleSpace(x, y, clearance):
            break
    # theta = np.degrees(theta_rad)
    if is_ObstacleSpace(x, y, clearance):
        return None, None, None, None, None
    new_state = [x, y, theta]
    return new_state, cost, point_set1, point_set2, (dvx, dvy, rotation_z)


def get_neighbors(node, clearance, ang_vels):
    crnt_state = node.get_state()
    neighbors = list()
    neighbor_plots = list()
    ang_vel_combinations = [[ang_vels[0], 0], [0, ang_vels[0]], [ang_vels[0], ang_vels[0]], [ang_vels[1], 0], 
                            [0, ang_vels[1]], [ang_vels[1], ang_vels[1]], [ang_vels[0], ang_vels[1]], [ang_vels[1], ang_vels[0]]]
    for ang_vel in ang_vel_combinations:
        updated_state, travel_cost, point_set1, point_set2, action = diffDriveConstraint(crnt_state, ang_vel[0], ang_vel[1], clearance)
        if updated_state is not None:
            neighbor = Node(updated_state, node, action, node.get_cost()+travel_cost)
            neighbor_plots.append((point_set1, point_set2))
            neighbors.append(neighbor)
    return neighbors, neighbor_plots