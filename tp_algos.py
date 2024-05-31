#!/usr/bin/python3
'''
    CentraleSupelec TP 2A/3A
    Aarsh THAKKER,2023
    (all variables in SI unit)

###########################################################################################
============================ READ THIS BEFORE STARTING TO CODE ============================
    You ONLY modify the part that is marked "to be modified" in the functions
    variables used by the functions of this script
        - robotNo: no of the current robot of same type for which control is counted (1 .. nbRobots)
        - nbTB3: number of total tb3 robots in the fleet (>=0)
        - nbCF: number of total crazyflie nano drones in the fleet (>=0)
        - nbRMTT: number of total dji robomaster TT drones in the fleet (>=0)
        - nbRMS1: number of total dji robomaster S1 in the fleet (>=0)  (YOU CAN ONLY CONTROL THIS ROBOT MANUALLY USING YOUR MOBILE PHONE AND GET POSITION IN HERE TO BE USED)
        - nbOBSTACLE: number of total obstacle positions in the environment (>=0)
        - tb3_poses:  size (3 x nbTB3) 
            eg. of use: for robot number 'robotNo', position of the robot can be obtained by: 
                    tb3_poses[:,robotNo-1]   (indexes in Python start from 0 !)
                    tb3_poses[0,robotNo-1]: x-coordinate of robot position (in m)
                    tb3_poses[1,robotNo-1]: y-coordinate of robot position (in m)
                    tb3_poses[2,robotNo-1]: orientation angle of robot (in rad)
        - cf_poses:  size (3 x nbCF) 
            eg. of use: for robot number 'robotNo', position of the robot can be obtained by: 
                    cf_poses[:,robotNo-1]   (indexes in Python start from 0 !)
                    cf_poses[0,robotNo-1]: x-coordinate of robot position (in m)
                    cf_poses[1,robotNo-1]: y-coordinate of robot position (in m)
                    cf_poses[2,robotNo-1]: z-coordinate of robot position (in m)
        - rmtt_poses:  size (3 x nbRMTT) 
            eg. of use: for robot number 'robotNo', position of the robot can be obtained by: 
                    rmtt_poses[:,robotNo-1]   (indexes in Python start from 0 !)
                    rmtt_poses[0,robotNo-1]: x-coordinate of robot position (in m)
                    rmtt_poses[1,robotNo-1]: y-coordinate of robot position (in m)
                    rmtt_poses[2,robotNo-1]: z-coordinate of robot position (in m)
        - rms1_poses:  size (3 x nbRMS1) 
            eg. of use: for robot number 'robotNo', position of the robot can be obtained by: 
                    tb3_poses[:,robotNo-1]   (indexes in Python start from 0 !)
                    tb3_poses[0,robotNo-1]: x-coordinate of robot position (in m)
                    tb3_poses[1,robotNo-1]: y-coordinate of robot position (in m)
                    tb3_poses[2,robotNo-1]: orientation angle of robot (in rad)
        - obstacle_pose:  size (5 x nbOBSTACLE)  
                    This can be used to define cube/sphere shaped obstacle in the environment.
                    obstacle_pose[:,nbOBSTACLE-1]   (indexes in Python start from 0 !)
                    obstacle_pose[0,nbOBSTACLE-1]: x-coordinate of center position of obstacle (in m)
                    obstacle_pose[1,nbOBSTACLE-1]: y-coordinate of center position of obstacle (in m)
                    obstacle_pose[2,nbOBSTACLE-1]: z-coordinate of center position of obstacle (in m)
                    obstacle_pose[3,nbOBSTACLE-1]: size of the obstacle (from center to the side of the cube/radius of sphere)
                    obstacle_pose[4,nbOBSTACLE-1]: type of obstacle (0= sphere, 1= cube)


###########################################################################################

'''

import numpy as np
import math
import rospy
import path_finder as pf

# ==============   "GLOBAL" VARIABLES KNOWN BY ALL THE FUNCTIONS ===================
# all variables declared here will be known by functions below
# use keyword "global" inside a function if the variable needs to be modified by the function

global trajectory
global index_trajectory
global Victim
global Entry_circle
global Exit_circle

Victim = [True, False, False, False]
index_trajectory = 0
grid = [[0 for _ in range(6)] for _ in range(8)]
grid[0][0] = 3  # Start
grid[0][1] = 2  # base

#victimins
grid[3][1] = 1
grid[4][5] = 1
grid[7][5] = 1

#obstacles
#grid[2][1] = -1
#grid[2][2] = -1
#grid[3][3] = -1
#grid[5][2] = -1
trajectory = pf.path_maker(grid)
Entry_circle = [True, False, False, False]
Exit_circle = [True, False, False, False]
# ===================================================================================

# Control function for turtlebot3 ground vehicle to be modified
# should ONLY return (vx,vy) for the robot command
# max useable numbers of robots = 6 (or 1 for waffle model)
# ====================================
def tb3_control_fn(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, ros_time):
    # =============================================================================  

    global firstCall
    global Victim
    global trajectory
    global index_trajectory
    global Entry_circle
    global Exit_circle
    
    # number of robots (short notation)
    nbTB3 = tb3_poses.shape[0]
    
    # get index of current robot (short notation)
    i = robotNo - 1
   
    # get positions of all robots
    x_leader, y_leader, theta_leader = tb3_poses[0, 0:3]
    x, y, theta = tb3_poses[i, 0:3]
    theta_leader_rad = theta_leader * 360 / np.pi

    # control law
    vx = 0.0
    vy = 0.0

    # adjacency matrix of communication graph
    # -----------------------------------------
    A = np.ones((nbTB3, nbTB3)) - np.eye(nbTB3)
  
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

    dx = x - x_leader
    dy = y - y_leader

    distance_firefighter_to_victim = np.sqrt(dx**2 + dy**2)
    if not Entry_circle[i] and distance_firefighter_to_victim <= 0.5:
        Entry_circle[i] = True

    if Entry_circle[i] and distance_firefighter_to_victim > 0.5:
        Exit_circle[i] = True

    if not Victim[i] and Entry_circle[i] and Exit_circle[i]:
        Victim[i] = True

    if Victim[i]:
        Rio_x = -0.5 * i * np.cos(theta_leader)
        Rio_y = -0.5 * i * np.sin(theta_leader)

        u0_x = -kL * (x_leader - x_ref)
        u0_y = -kL * (y_leader - y_ref)

        vx = -kF * ((x - x_leader) - Rio_x) + u0_x
        vy = -kF * ((y - y_leader) - Rio_y) + u0_y

        # Apply some smoothing to the control inputs
        vx = 0.1 * vx + 0.9 * vx
        vy = 0.1 * vy + 0.9 * vy

    # Verify if the leader has reached the current target
    distance_firefighter_to_objective = np.sqrt((x_ref - x_leader)**2 + (y_ref - y_leader)**2)
    if distance_firefighter_to_objective < 0.5 and index_trajectory < len(trajectory) - 1:
        index_trajectory += 1

    print(f"Trajectory Index: {index_trajectory}")
    print(f"Victim: {Victim}")
    
    return vx, vy
# ====================================        


# ====================================        
# Control function for crazyflie nano drones to be modified
# should ONLY return (vx,vy,vz,takeoff,land) for the robot command
# max useable numbers of drones = 3
# ====================================
def cf_control_fn(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, ros_time):
# ====================================
    nbTB3= len(tb3_poses[0]) # number of total tb3 robots in the use
    nbCF = len(cf_poses[0]) # number of total crazyflie nano drones in the use
    nbRMTT = len(rmtt_poses[0]) # number of total dji rmtt drones in the use
    nbRMS1 = len(rms1_poses[0]) # number of total dji rms1 in the use
    nbOBSTACLE = len(obstacle_pose[0]) # number of total obstacle positions in the environment

    #  --- TO BE MODIFIED ---
    vx = 0.0
    vy = 0.0
    vz = 0.0
    takeoff = False
    land = False
    # -----------------------

    return vx,vy, vz, takeoff, land
# ====================================    


# ====================================        
# Control function for dji rmtt drones to be modified
# should ONLY return (vx,vy,vz,takeoff,land,led) for the robot command
# max useable numbers of drones = 1
# ====================================
def rmtt_control_fn(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, ros_time):
# ====================================
    nbTB3= len(tb3_poses[0]) # number of total tb3 robots in the use
    nbCF = len(cf_poses[0]) # number of total crazyflie nano drones in the use
    nbRMTT = len(rmtt_poses[0]) # number of total dji rmtt drones in the use
    nbRMS1 = len(rms1_poses[0]) # number of total dji rms1 in the use
    nbOBSTACLE = len(obstacle_pose[0]) # number of total obstacle positions in the environment

    #  --- TO BE MODIFIED ---
    vx = 0.0
    vy = 0.0
    vz = 0.0
    takeoff = False
    land = False
    led = [0,0,0] #RGB
    # -----------------------

    return vx,vy, vz, takeoff, land, led
# ====================================    



# ======== ! DO NOT MODIFY ! ============
def tb3_controller(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, ros_time):
    vx,vy = tb3_control_fn(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, ros_time)
    return vx,vy

def cf_controller(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, ros_time):
    vx, vy, vz, takeoff, land = cf_control_fn(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, ros_time)
    return vx, vy, vz, takeoff, land

def rmtt_controller(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, ros_time):
    vx, vy, vz, takeoff, land, led = rmtt_control_fn(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, ros_time)
    return vx, vy, vz, takeoff, land, led
# =======================================