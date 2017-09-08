# Highway-Path-Planning
From Self-Driving Car Engineer Nanodegree Program, Starter Code Provided by Udacity   

![](highway_path_planning.gif)
## Overview
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. We are provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

## Implementation
### Fitting Spline
In order to get smooth paths, which will not exceed limits of acceleration and jerk, a spline is fitted over points. Five points are initially determined. For the first two, if the previous path from the simulator is available then the last two points on that path are used, otherwise the current position of the car is taken and a point one time step in the past using the angle of the car is calculated:

      prev_ref_x = car_x - 1*cos(car_yaw);
      prev_ref_y = car_y - 1*sin(car_yaw);

For the remaining 3 points, choose them in a given lane further along the s coordinate, 50 in the example below:

      vector<double> WP1 = getXY(car_s + 50, (ref_lane*4+2), map_waypoints_s, map_waypoints_x, map_waypoints_y);

A spline is then fitted to these 5 points

The coordinates are also transformed from map space to relative to the car. This makes it easier to work with them. They are converted back to the map space before passing to the simulator.

### Smooth Transition Over Each Time Step
Since we also want to have smooth transition from last simulator path to the next one, we add the missing coordinates to the last path from the simulator instead of creating a new one. E.g if the simulator returned 47 points for last path, that means in the 0.02 seconds it could only traverse 3 points. Since our path length is 50, we will only add 3 new points instead of creating 50 new points.

We then split the spline into equal intervals that are uniform and the correct distance apart. This will prevent acceleration and jerks.

### Lane changes
A lane change is only considered when the vehicle gets too close to another vehicle in the present lane. Iterating over all of the sensor_fusion data, each car's projected location, based on magnitude of speed and time interval, is calculated. If the projected car is in the same lane as the main car and its projected location will be within 20 meters of the main car then a lane change is considered.

### Which Lane to Choose
If the car is in the right most lane and there is no car within 15 meters (ahead or behind) in the middle lane, then the lane is changed.

If the car is in the left most lane and there is not car within 15 meters (ahead or behind) in the middle lane, then the lane is changed.

If the car is in the middle lane, it is first checked whether there is a car within 15 meters (ahead or behind) in either the left or right lane. If there is a car in either lane then the opposite lane is taken. If there are cars in both left and right lanes then only the speed is decreased and the car keeps its lane.

If the car is in the middle lane and there are no cars in left and right lanes, then the distance to the next car in each lane is calculated. Which ever lane has car which is further away, is taken. This serves as a cost function.


### Frequency and Speed of Lane Change
If a lane was changed within the last 2 seconds then a new lane change can't occur. The speed of the vehicle also needs to be below 45 mph for a lane change.

---

## Simulator
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time.

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
