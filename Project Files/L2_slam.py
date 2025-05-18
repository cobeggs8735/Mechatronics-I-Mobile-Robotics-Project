# import packets
import csv                  # csv editing packet
import numpy as np          # array packet
import math                 # math operations packet

# import files
import L2_displacement as disp                  # obtain wheel displacement
import L2_inverse_kinematics as inverse         # perform kinematics
import L2_kinematics as kinematics              # perform LIDAR vector calculations
import L2_vector as vector

# Initialize Xprime and YPrime
#xPrime = 0          # the origin
#yPrime = 0          # the origin

# Function for obtaining local coordinates of objects
def getLocal(x, alpha):         # input nearest obstacle vector
    # Find local coordinates
    local_coords = np.array(vector.polar2cart(x, alpha))    # Cartesian coordinate of object
    local_coords[0] += 0.25                                  #lidar distance from origin"
    
    return local_coords

# Function for converting local to global coordinates of objects
def getGlobal(x, alpha, robotX, robotY):
    # Get Local Coordinates
    local = getLocal(x, alpha)          # Obtain Local coordinates
    
    # Convert to Global Coordinates
    xPrime = robotX + local[0]      # x global coordinate of object
    yPrime = robotY + local[1]      # y global coordinate of object
    
    return xPrime, yPrime

# Function for sending global coordinates to excel
def exportCoords(robotX, robotY, near):
    with open('map.csv', 'a') as mapping:           # Open the map file
        x = near[0]
        alpha = near[1]
        global_coords = np.array(getGlobal(x, alpha, robotX, robotY))    # Obtain global coordinates of obstacle
        globalCoords = []
        for i in global_coords:
            i = str(i)                          # Convert the coordinte to a string
            globalCoords.append(i)              # Make coordiante a list
        csv.writer(mapping).writerow(globalCoords)                 # write coordinate to csv
    mapping.close()

    return

# Function for clearing the .csv file between runs
def clearFile():
    open('map.csv', 'w').close()
    
    return
##### The above function is modeled from L2_log.clear_file()


if __name__ == "__main__": 
    while True:

        time.sleep(0.2)
        