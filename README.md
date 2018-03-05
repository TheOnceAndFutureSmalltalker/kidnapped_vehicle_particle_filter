# Kidnapped Vehicle Particle Filter

This project implements a 2 dimensional particle filter in C++.  The Filter is then run against a simulation that provides initial location and direction, a map of local landmarks, control information, and lidar sensor data of surrounding objects for a car that has been kidnapped.  It is the task of the particle filter program to determine the location and heading of the kidnapped vehicle in real time.  The program must keep the location and heading errors within limits and complete the calculations within 100 seconds.


<br /><br />
<p align="center">
<img src="https://github.com/TheOnceAndFutureSmalltalker/kidnapped_vehicle_particle_filter/blob/master/images/kidnapped_car_simulator.JPG" width="802px" /><br /><b>Runaway Vehicle with Landmarks Simulator</b></p>
<br />


## Results

The implementation is mostly straight forward.  It is necessary that we calculate in real time so that optimal code is of a premium.  There may be a few opportunities for optimization, but not many.  The only significant improvement perhaps could be made in the nearest neighbor calculation for matching lidar observations to actual landmark locations.  This leaves number of particles as the only true parameter for tuning the filter.  Below are the various trials of number of particles the corresponding results.  I left the final result with number of particles set to 200.  This seemed to have the best trade off of time to error.   In particular, notice that 1000 particles does not do much better at all than 200 particles, yet takes more than twice as long to complete!

Num Particles | X Error | Y Error | Yaw Error | Time (s) | Success | 
-----|-----|-----|-----|-----|-----
10 | 0.174 | 0.152 | 0.006 | 49.02 | YES
100 | 0.116 | 0.108 | 0.004 | 48.98 | YES
200 | 0.109 | 0.105 | 0.003 | 49.94 | YES
500 | 0.109 | 0.100 | 0.004 | 73.58 | YES
750 | 0.106 | 0.100 | 0.003 | 89.40 | YES
1000 | 0.110 | 0.101 | 0.003 | 102.96 | NO


