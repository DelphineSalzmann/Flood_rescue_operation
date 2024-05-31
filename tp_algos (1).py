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
from . import path_finder as pf

# ==============   "GLOBAL" VARIABLES KNOWN BY ALL THE FUNCTIONS ===================
# all variables declared here will be known by functions below
# use keyword "global" inside a function if the variable needs to be modified by the function

global trajectory
global index_trajectory
global Victim
global Entry_circle
global Exit_circle
global index_leader

index_leader = 0

Victim = [True, False, False, False]
index_trajectory = 0
grid = [[0 for _ in range(6)] for _ in range(8)]
grid[0][0] = 3  # Start
grid[0][1] = 2  # base

#victimins
grid[1][0] = 1
grid[2][1] = 1
grid[2][3] = 1

#obstacles
#grid[2][1] = -1
#grid[2][2] = -1
#grid[3][3] = -1
#grid[5][2] = -1
trajectory = pf.path_maker(grid)
Entry_circle = [True, False, False, False]
Exit_circle = [True, False, False, False]

print(trajectory)
# ===================================================================================

# Control function for turtlebot3 ground vehicle to be modified
# should ONLY return (vx,vy) for the robot command
# max useable numbers of robots = 6 (or 1 for waffle model)
# ====================================
def tb3_control_fn(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, ros_time):
    # ====================================
    global trajectory
    global index_trajectory
    global Victim
    global Entry_circle
    global Exit_circle
    global index_leader

    nbTB3 = len(tb3_poses[0])  # number of total tb3 robots in the use
    nbCF = len(cf_poses[0])  # number of total crazyflie nano drones in the use
    nbRMTT = len(rmtt_poses[0])  # number of total dji rmtt drones in the use
    nbRMS1 = len(rms1_poses[0])  # number of total dji rms1 in the use
    nbOBSTACLE = len(obstacle_pose[0])  # number of total obstacle positions in the environment

    #  --- TO BE MODIFIED ---

    # Index of the robot
    i = robotNo - 1

    # Initialisation of the control law
    vx = 0.0
    vy = 0.0

    # gains
    k_leader = 0.2
    k_follower = 0.3

    # we fixed the firefighter as the robot number 1.
    x_leader, y_leader, angle_leader = tb3_poses[:, 0]

    # Verifying if the firefighter has reached the current target
    x_leader_target, y_leader_target = trajectory[index_trajectory]

    # Position of the current robot
    x_current, y_current, angle_current = tb3_poses[:, i]

    #reference
    x_reference, y_reference, angle_reference = tb3_poses[:, index_leader]

    distance_firefighter_to_victim = np.sqrt((x_reference - x_current) ** 2 + (y_reference - y_current) ** 2)
    if not Entry_circle[i] and distance_firefighter_to_victim <= 0.7:
        Entry_circle[i] = True

    #if Entry_circle[i] and distance_firefighter_to_victim > 0.7:
    #    Exit_circle[i] = True

    if not Victim[i] and Entry_circle[i]:# and Exit_circle[i]:
        Victim[i] = True
        index_leader += 1

    if Victim[i]:
        Rio_x = -0.5 * i * np.cos(angle_leader)
        Rio_y = -0.5 * i * np.sin(angle_leader)

        u0_x = -k_leader * (x_current - x_leader_target) + 0
        u0_y = -k_leader * (y_current - y_leader_target) + 0

        vx = -k_follower * ((x_current - x_leader) - Rio_x) + u0_x
        vy = -k_follower * ((y_current - y_leader) - Rio_y) + u0_y

    # changing the index to next point and verifying if we reached the next victim - important to change index-victim

    distance_firefighter_to_objective = np.sqrt((x_leader_target - x_leader) ** 2 + (y_leader_target - y_leader) ** 2)
    #print(f"{distance_firefighter_to_objective=} \t {index_trajectory=}")
    if distance_firefighter_to_objective < 0.7 and index_trajectory < len(trajectory)-1:
        index_trajectory += 1

    # -----------------------
    vx, vy = anti_Collision_function(robotNo, tb3_poses, vx, vy, 0.5, 0.7)

    return vx, vy


# ====================================


# ====================================
# Control function for crazyflie nano drones to be modified
# should ONLY return (vx,vy,vz,takeoff,land) for the robot command
# max useable numbers of drones = 3
# ====================================
def cf_control_fn(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, ros_time):
    # ====================================
    nbTB3 = len(tb3_poses[0])  # number of total tb3 robots in the use
    nbCF = len(cf_poses[0])  # number of total crazyflie nano drones in the use
    nbRMTT = len(rmtt_poses[0])  # number of total dji rmtt drones in the use
    nbRMS1 = len(rms1_poses[0])  # number of total dji rms1 in the use
    nbOBSTACLE = len(obstacle_pose[0])  # number of total obstacle positions in the environment

    #  --- TO BE MODIFIED ---
    vx = 0.0
    vy = 0.0
    vz = 0.0
    takeoff = False
    land = False
    # -----------------------

    return vx, vy, vz, takeoff, land


# ====================================


# ====================================
# Control function for dji rmtt drones to be modified
# should ONLY return (vx,vy,vz,takeoff,land,led) for the robot command
# max useable numbers of drones = 1
# ====================================
def rmtt_control_fn(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, ros_time):
    # ====================================
    nbTB3 = len(tb3_poses[0])  # number of total tb3 robots in the use
    nbCF = len(cf_poses[0])  # number of total crazyflie nano drones in the use
    nbRMTT = len(rmtt_poses[0])  # number of total dji rmtt drones in the use
    nbRMS1 = len(rms1_poses[0])  # number of total dji rms1 in the use
    nbOBSTACLE = len(obstacle_pose[0])  # number of total obstacle positions in the environment

    #  --- TO BE MODIFIED ---
    vx = 0.0
    vy = 0.0
    vz = 0.0
    takeoff = False
    land = False
    led = [0, 0, 0]  # RGB
    # -----------------------

    return vx, vy, vz, takeoff, land, led


# ====================================


# ======== ! DO NOT MODIFY ! ============
def tb3_controller(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, ros_time):
    vx, vy = tb3_control_fn(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, ros_time)
    return vx, vy


def cf_controller(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, ros_time):
    vx, vy, vz, takeoff, land = cf_control_fn(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose,
                                              ros_time)
    return vx, vy, vz, takeoff, land


def rmtt_controller(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, ros_time):
    vx, vy, vz, takeoff, land, led = rmtt_control_fn(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses,
                                                     obstacle_pose, ros_time)
    return vx, vy, vz, takeoff, land, led
# =======================================

# ============================================    
def anti_Collision_function(robotNo, poses, vx, vy, Min_security_zone_robot, Avoid_collusion_gain):
# ============================================
    
    vx_avoid = 0
    vy_avoid = 0
    is_robot_in_range = 0
    actual_min_dist = float('inf')
    for j in range(len(poses)):
        if robotNo-1 != j:
            distance_robotNo_Robotj = np.sqrt((poses[0,j]-poses[0,robotNo-1])**2+(poses[1,j]-poses[1,robotNo-1])**2)
            if distance_robotNo_Robotj<Min_security_zone_robot:
                vx_avoid = vx_avoid - (poses[0,j]-poses[0,robotNo-1])/distance_robotNo_Robotj**2*(1.2-distance_robotNo_Robotj/Min_security_zone_robot)
                vy_avoid = vy_avoid - (poses[1,j]-poses[1,robotNo-1])/distance_robotNo_Robotj**2*(1.2-distance_robotNo_Robotj/Min_security_zone_robot)
                if distance_robotNo_Robotj<actual_min_dist:
                    actual_min_dist = distance_robotNo_Robotj
                is_robot_in_range = 1
                
    if is_robot_in_range:
        input_norm = np.sqrt(vx**2+vy**2)
        ui_norm_min = 0.05
        wanted_norm = max(input_norm, ui_norm_min) # IF we wanted the robot to not move, it still have to avoid collision
        
        
        v_avoid_norm = np.sqrt(vx_avoid**2+vy_avoid**2)
        vx_avoid = vx_avoid/v_avoid_norm*Avoid_collusion_gain*wanted_norm
        vy_avoid = vy_avoid/v_avoid_norm*Avoid_collusion_gain*wanted_norm
        
        # the next lines, make the avoidance componant higher the closer the robots are
        vx_tot = vx*actual_min_dist + vx_avoid*(1-actual_min_dist/Min_security_zone_robot)
        vy_tot = vy*actual_min_dist + vy_avoid*(1-actual_min_dist/Min_security_zone_robot)
        
        
        # The following lines make sure the norm of the final wanted speed is the same as the input one
        
        ui_norm_tot = np.sqrt(vx_tot**2+vy_tot**2)
        vx_tot = vx_tot/ui_norm_tot*wanted_norm
        vy_tot = vy_tot/ui_norm_tot*wanted_norm
    else:
        #no modification because there aren't any robot near the current one
        vx_tot = vx
        vy_tot = vy


    return vx_tot, vy_tot
