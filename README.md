# Kidnapped Vehicle Particle Filter

This project implements a 2 dimensional particle filter in C++.  The Filter is then run against a simulation that provides initial location and direction, a map of local landmarks, control information, and lidar sensor data of surrounding objects for a car that has been kidnapped.  It is the task of the particle filter program to determine the location and heading of the kidnapped vehicle in real time.  The program must keep the location and heading errors within limits and complete the calculations within 100 seconds.




<br /><br />
<p align="center">
<img src="https://github.com/TheOnceAndFutureSmalltalker/extended_kalman_filter/blob/master/images/simulator.JPG" width="792px" /><br /><b>Runaway Vehicle with Landmarks</b></p>
<br />


## Results

Since the simulator provides ground truth values, I was able to calculate errors for the estimates as mean squared error - MSE.
Running the code against Data Set 1, I recorded the following MSE's for x position (Px), y position (Py), velocity in the x direction (Vx), and velocity in the y direction (Vy).

Num Particles | X Error | Y Error | Yaw Error | Time (s) | Success | 
-----|-----|-----|-----|-----|-----
10 | 0.174 | 0.152 | 0.006 | 49.02 | YES

