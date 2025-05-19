# Mechatronics-I-Mobile-Robotics-Project
Final project designed using a SLAM algorithm (Simultaneous Localization and Mapping) for Mechatronics I: Mobile Robotics class (MXET 300) in the Engineering Technology and Industrial Distribution department at Texas A&amp;M University

## Summary
### freeroam.py
The file [freeroam.py](https://github.com/cobeggs8735/Mechatronics-I-Mobile-Robotics-Project/blob/main/Project%20Files/freeroam.py) is designed for autonomous robot navigation, specifically for "free roaming" and obstacle avoidance. Here’s what it does:

- It imports various modules for speed control, kinematics, inverse kinematics, and obstacle detection.
- The main functions are:
  - cart2polar: Converts Cartesian coordinates to polar coordinates.
  - FreeRoam: Drives the robot forward at target speeds using closed-loop speed control.
  - Avoidance: Stops forward motion and rotates the robot left or right to avoid nearby obstacles, based on their angle.
- In the main loop:
  - It constantly reads the robot’s current wheel speeds.
  - Sets a target speed (max speed for both wheels).
  - Gets the nearest detected obstacle (distance and angle).
  - If an obstacle is close (within 0.25 units) and in front of the robot (within ±90 degrees), it triggers the Avoidance routine to turn away.
  - Otherwise, it continues forward (FreeRoam).
  - The loop repeats every 0.1 seconds.

In summary: freeroam.py enables a robot to move forward freely unless it detects a nearby obstacle ahead, in which case it rotates to avoid the obstacle before proceeding.
### L2_slam.py
The file [L2_slam.py](https://github.com/cobeggs8735/Mechatronics-I-Mobile-Robotics-Project/blob/main/Project%20Files/L2_slam.py) is a Python script that supports Simultaneous Localization and Mapping (SLAM) for a mobile robotics project. Here’s what it does:

- Imports several modules, including custom ones for displacement, kinematics, and vector operations, along with standard libraries (numpy, math, csv).
- Provides functions to:
  - Convert LIDAR polar coordinates of detected obstacles to local Cartesian coordinates (getLocal).
  - Transform local coordinates to global coordinates based on the robot’s position (getGlobal).
  - Export global coordinates of obstacles to a CSV file for mapping (exportCoords).
  - Clear the CSV mapping file between runs (clearFile).

This file essentially helps the robot record the position of obstacles it detects as it moves, storing these positions in a CSV file for later analysis or map-building. The main loop is set up for repeated execution, but currently only includes a sleep timer, suggesting this is a module meant to be imported and used by other scripts.

### L3_project.py
The file [L3_project.py](https://github.com/cobeggs8735/Mechatronics-I-Mobile-Robotics-Project/blob/main/Project%20Files/L3_project.py) is the main control script for a mobile robotics project. It is based on a template provided for the third lab of the course, hence the name fo the file "L3_project.py". The projecct part of the name denotes its changes from the third lab of the course. Here’s what it does:

- **Imports Local and External Libraries:** It loads several custom modules for robot control (like roaming, encoders, displacement, SLAM, and obstacle detection), as well as standard libraries such as NumPy, time, and math.
- **Initializes Camera Color Thresholds:** It sets HSV (Hue, Saturation, Value) ranges, likely for detecting a colored glove with a camera (though the actual detection code is not implemented in this file).
- **Clears Previous SLAM Data:** It clears a file used by the SLAM module to reset stored mapping data.
- **Defines Robot Parameters and Variables:** It sets up wheel radius, wheelbase, encoder resolution, and initializes position variables.
- **Main Control Loop:** Inside the `if __name__ == "__main__":` block, the script enters an infinite loop where it:
  - Reads encoder values to calculate the robot’s movement and update its position.
  - Computes wheel travels and updates the robot’s global X and Y positions.
  - Detects nearby obstacles using sensors; if an obstacle is detected in front, the robot turns away, otherwise it continues moving forward.
  - Logs the robot’s updated position and nearest obstacle data for SLAM (Simultaneous Localization and Mapping).
  - Contains placeholders for hand detection and “high five” interaction features (not implemented in the code shown).
  - Waits briefly between loop iterations to control the update rate.

**In summary:**  
This file is the main operational script for a mobile robot. It reads sensors, updates position, avoids obstacles, and logs data for mapping. It also sets up for camera-based interactions, though those parts are not implemented in this file.
