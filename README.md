# Udacity-Bosch Path Planning Challenge
This challenge is to design a path planner that creates smooth, safe paths along a 3-lane highway with traffic. A successful path planner is able to keep inside the lanes, avoid hitting other cars, and pass slower moving traffic - all by using localization, sensor fusion, and map data.

# Udacity Track Release
Download the simulator and original project track: https://github.com/udacity/CarND-Path-Planning-Project

# Bosch Track Release
The Bosch track is a highway that continuously goes down the same direction but with some slight curviness. 

The competitive challenge element added to this project is a brand new highway to test on, along with a timer element that measures how quickly the target 2.75 miles can be completed without incident. 

# Project Rubric 

### Compilation
The code compiles correctly. Code compiles without errors with cmake and make. Basic Build Instructions:

1. Unzip the file to a directory and open it.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

### Valid Trajectories
**The car is able to drive at least 2.75 miles without incident**        

The top right screen of the simulator shows the current/best miles driven without incident. Incidents include exceeding acceleration/jerk/speed, collision, and driving outside of the lanes. Each incident case is also listed below in more detail.

----------------------------------------------------------------------------------------------------------------------------------------

**The car drives according to the speed limit**

The car doesn't drive faster than the speed limit of 50 miles per hour.

----------------------------------------------------------------------------------------------------------------------------------------

**Max acceleration and jerk are not exceeded**

The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.

----------------------------------------------------------------------------------------------------------------------------------------

**Car does not have collisions**

The car must not come into contact with any of the other cars or objects on the road.

----------------------------------------------------------------------------------------------------------------------------------------

**The car stays in its lane except when changing lanes**

During lane changes, the car doesn't spend more than 3 seconds outside the lane lines. During normal operation the car stays inside the lane lines with the flow of traffic.

----------------------------------------------------------------------------------------------------------------------------------------

**The car completes its trip within 360 seconds**

Any submissions that don't finish within 360 seconds will be considered invalid.

----------------------------------------------------------------------------------------------------------------------------------------

**Submission file must meet naming standards**

Your .zip file must include main.cpp and all the code required to successfully compile. In addition, you must use the following map name in your code: highway_map_bosch1.csv




