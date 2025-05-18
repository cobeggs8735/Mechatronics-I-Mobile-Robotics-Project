#import L1_lidar as li
import L2_vector as vec
#import Lab6Template as FB #feedback
import L2_speed_control as sc
import L2_inverse_kinematics as inv
import L2_kinematics as kin
#import L2_obstacle as obs
import obstacle_furthest as of
import time
import numpy as np
import math

def cart2polar(xy):     #convert cartesian value to cart2polar
    x = xy[0]
    y = xy[1]
    r = math.sqrt(x**2 + y**2)
    theta = math.atan(y/x)
    polar = [r,theta]
    
    return polar

def FreeRoam(target,actual):
    sc.driveClosedLoop(target,actual,0) #actual = kin.getPdCurrent in rad/s
    #will only drive straight
    #incorporate SLAM?
    
    return
   
def Avoidance(near, actual):
    xDot = 0
    
    if (-90 < near[1] <= 0):
        thetaDot = 1 
        
    elif (0 < near[1] < 90):
        thetaDot = -1
    
    phiDots = inv.convert([xDot, thetaDot])
    sc.driveClosedLoop(phiDots, actual, 0)

    return

if __name__ == "__main__":
    while True:
        actual = kin.getPdCurrent()
        #print(actual)
        target = [9.7/2,9.7/2] #max speed, in rad
        #print(target)
        near = vec.getNearest()
       
        if ((0 <= near[0] <= 0.25) & (-90 < near[1] < 90)):
            #print("avoidance before")
            Avoidance(near, actual)
            #print("avoidance after")

        else: 
            #print("freeroam before")
            FreeRoam(target, actual)
            #print("freeroam after")
        
        time.sleep(0.1)

    #use furthest point angle as rotation angle
    #new_vec = vec.rotate()
    