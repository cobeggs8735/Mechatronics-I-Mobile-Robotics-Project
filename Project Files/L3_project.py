# Import important files 
import freeroam as roam                     # local library for roaming algorithm
import L1_encoder as enc                    # local library for encoders
import L2_displacement as disp              # local library for robot displacement
import L2_slam as slam                      # local library for slam algorithms
import L2_vector as vec                     # local library for obstacle vectors

# Import important packets
import numpy as np                          # library for math operations
import time                                 # library for time access
import math

# Camera values for detecting the glove
hMin = 75
hMax = 140

sMin = 65
sMax = 85

vMin = 215
vMax = 240

# clear file
slam.clearFile()

# initialize variables at zero
x_total = 0                                     # x total movement of robot
t = 0                                           # theta
y = 0                                           # chasis doesn't move in the y
encL1 = 0
encR1 = 0
robotX = 0
robotY = 0

# define kinematics
R = 0.041                                   # radius in meters
L = 0.201                                   # half of wheelbase meters
res = (360/2**14)                           # resolution of the encoders
roll = int(360/res)                         # variable for rollover logic
gap = 0.5 * roll                            # degress specified as limit for rollover

A = np.array([[R/2, R/2], [-R/(2*L), R/(2*L)]])     # This matrix relates [PDL, PDR] to [XD,TD]
wait = 0.02                                 # wait time between encoder measurements (s)


# Repeating parts of the program
if __name__ == "__main__":
    while True:
        #####           Obtain current encoder readings
        encL0 = encL1                           # transfer previous reading.
        encR0 = encR1                           # transfer previous reading.
        encoders = enc.read()                   # grabs the current encoder readings, raw
        encL1 = round(encoders[0], 1)           # reading, raw.
        encR1 = round(encoders[1], 1)           # reading, raw.
    
        # movement calculations
        travL = disp.getTravel(encL0, encL1) * res   # grabs travel of left wheel, degrees
        travL = -1 * travL                           # this wheel is inverted from the right side
        travR = disp.getTravel(encR0, encR1) * res   # grabs travel of right wheel, degrees
    
        # build an array of wheel travels in rad/s
        travs = np.array([travL, travR])        # store wheels travel in degrees
        travs = travs * 0.5                     # pulley ratio = 0.5 wheel turns per pulley turn
        travs = travs * 3.14 / 180              # convert degrees to radians
        travs = np.round(travs, decimals=3)     # round the array
        
        chass = disp.getChassis(travs)          # convert the wheel travels to chassis travel
        x_total = x_total + chass[0]                        # add the latest advancement(m) to the total
        t = t + chass[1]
        #####           The above code is from the while loop of L2_displacement.py to obtain displacement
        
        # Instantaneous travel
        x_current = chass[0]
        # Robot Global
        robotX = robotX + math.cos(t)*x_current - math.sin(t)*y  # current robot x global
        robotY = robotY + math.sin(t)*x_current + math.cos(t)*y  # current robot y global
        
        ##### Start Obstacle detection
        actual = roam.kin.getPdCurrent()
        target = [9.7/2,9.7/2] #max speed, in rad
        near = vec.getNearest()
           
        if ((0 <= near[0] <= 0.5) & (-90 < near[1] < 90)):
            roam.Avoidance(near, actual) # Turn away from obstacle
    
        else: 
            roam.FreeRoam(target, actual) # Drive forward
        ##### End Obstacle Detection
        
        ##### Start of SLAM algorithms
        slam.exportCoords(robotX, robotY, near)
        ##### End of SLAM algorithms
        
        ##### Start Hand Detectiion
        
        ##### End Hand Detection
        
        ##### Start High Five
        
        ##### End High Five
        
        
        time.sleep(.1)
    
    
    
    
    
    
    
    
    
    
    
