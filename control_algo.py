# -*- coding: utf-8 -*-
"""
author: Sylvain Bertrand, 2023

   All variables are in SI units
    
   
   Variables used by the functions of this script
    - t: time instant (s)
    - robotNo: no of the current robot for which control is coputed (0 .. nbRobots-1)
    - poses:  size (3 x nbRobots)
        eg. of use: the pose of robot 'robotNo' can be obtained by: poses[:,robotNo]
            poses[robotNo,0]: x-coordinate of robot position (in m)
            poses[robotNo,1]: y-coordinate of robot position (in m)
            poses[robotNo,2]: orientation angle of robot (in rad)   (in case of unicycle dynamics only)
"""


import numpy as np
import math
import path_finder as pf



# ==============   "GLOBAL" VARIABLES KNOWN BY ALL THE FUNCTIONS ==============
# all variables declared here will be known by functions below
# use keyword "global" inside a function if the variable needs to be modified by the function


# global toto

global firstCall   # this variable can be used to check the first call ever of a function
firstCall = True


global trajectory
global index_trajectory
global index_victim
global Victim

Victim = [True, False, False, False]
index_trajectory = 0
index_victim = 1




# =============================================================================

global trajectory_done
trajectory_done = False

# =============================================================================
def formation(t, robotNo, robots_poses):
# =============================================================================  

    global firstCall
    global Victim
    global trajectory
    global index_trajectory
    global trajectory_done
    
    # number of robots (short notation)
    N = robots_poses.shape[0]
    
    # get index of current robot  (short notation)
    i = robotNo
   
    # get positions of all robots
    x0, y0, theta0_rad = robots_poses[0, 0:3]
    x, y, theta_rad = robots_poses[i, 0:3]
    theta0 = theta0_rad * 360 / np.pi

    # control law
    vx = 0.0
    vy = 0.0

    # adjacency matrix of communication graph
    # -----------------------------------------
    if trajectory_done == False:
        grid = pf.build_grid(robots_poses, (0,0.2), [(1,1)])
        path = pf.path_maker(grid)
        trajectory = pf.grid_to_real(path)
        trajectory_done = True
        print('fiz a tragetoria só uma vez')
    A = np.ones((N, N)) - np.eye(N)
  
    if firstCall:  # print information (only once)
        print(A)
        firstCall = False
    
    # Trajectory for the leader to follow
    if index_trajectory < len(trajectory):
        x_ref, y_ref = trajectory[index_trajectory]
    else:
        x_ref, y_ref = trajectory[-1]  # Last target is the base

    print(f"Current Target: {x_ref}, {y_ref}")
    
    # initialize control input vector
    kL = 1
    kF = 1

    dx = x - x0
    dy = y - y0

    distance_firefighter_to_victim = np.sqrt((dx)**2 + (dy)**2)
    if not Victim[i] and distance_firefighter_to_victim <= 1:
        Victim[i] = True

    if Victim[i]:
        Rio_x = -1 * i * np.cos(theta0_rad)
        Rio_y = -1 * i * np.sin(theta0_rad)

        u0_x = -kL * (x0 - x_ref)
        u0_y = -kL * (y0 - y_ref)

        vx = -kF * ((x - x0) - Rio_x) + u0_x
        vy = -kF * ((y - y0) - Rio_y) + u0_y
        

    # Verify if the leader has reached the current target
    distance_firefighter_to_objective = np.sqrt((x_ref - x0)**2 + (y_ref - y0)**2)
    if distance_firefighter_to_objective < 0.5 and index_trajectory < len(trajectory) - 1:
        index_trajectory += 1

    print(f"Trajectory Index: {index_trajectory}")
    print(f"Victim: {Victim}")
    
    return vx, vy
# =============================================================================







# general template of a function defining a control law
# =============================================================================
def my_control_law(t, robotNo, robots_poses):
# =============================================================================  

    # --- example of modification of global variables ---
    # ---(updated values of global variables will be known at next call of this funtion) ---
    # global toto
    # toto = toto +1

    # number of robots
    nbOfRobots= robots_poses.shape[0]
    
    
    # control law
    vx = 0.
    vy = 0.

    # .................  TO BE COMPLETED HERE .............................
    
    return vx, vy
# =============================================================================

