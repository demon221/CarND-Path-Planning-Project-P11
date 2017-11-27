# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

---

## Description

The goal of this project is to design a path planning of a car on highway driving. It requires smooth, safe paths for the car to follow along a three highway with traffic, meanwhile as fast as possible, just follow the speed limit of 50 mph. Operations such as lane keeping, lane changing, as well as acceleration and deceleration will be executed for the car actions.

The coding consists of following files:

* `main.cpp`: interfaces with the simulator and invokes the path planner.
* `planner.h`: path planner functions, with highway road map data, sensor fusion data, and ego localization data, generates a planning path trajectory in world coordinates.
* `roadmap.h`: read the highway waypoints data, transfrom between Cartesian coordinate and Frenet coordinate.
* `lane.h`: lane related functions according to the highway feature.
* `utils.h`: useful utillity functions.
* `spline.h`: cubic spline library by Tino Kluge, used for trajectory generation.
* `json.hpp`: JSON library of C++ for simulator interface.

[//]: # (Image References)
[image1]: ./data/log.png "Log picture"

When the path planning is running, the log view could record the drving statues:

![alt text][image1]

---

The planner itself proceeds in following steps: 

#### 1. Get reference point of ego motion

It sets two points from the end of the previous path, for the car ego position and heading. If the previous path has been consumed, that allow to create a tangent line in the direction of the car. Meanwhile, when the car drives to a new lap, the position will renew to the beginning of the waypoints. This function will print the current lap, lane, and s,d of the ego motion.

See method `PathPlanner::get_reference`.
    
#### 2. Environment analysis, process the data from sensor fusion with prediction

The data from the sensor fusion is analysed and a table with the state of all the lanes is produced. For each car objects, a struct `car_t` including each car id and its position, speed, driving lane will be tracked. For each lane, a struct `lane_info_t` is filled containing information about the cars ahead and behind of the reference position. 

Especcially, the predict gap will be calculated based on a costant speed model. According to the current gap `front_gap`, `back_gap` and future gap `front_gap_next`, `back_gap_next`, a `feasible` state for each lane will be got for best lane calculation.
    
This is the definition of the lane info struct with is default values:
    
```C++
struct lane_info_t
{
  int front_car = -1;
  int back_car = -1;
  double front_gap = 1000;
  double front_speed = 1000;
  double front_gap_next = 1000;
  double back_gap = -1000;
  double back_speed = -1000;
  double back_gap_next = -1000;
  bool feasible = true;

  bool is_clear() const { return feasible && front_car < 0; }
};
```
A **feasibility** flag signaling if the gap in the lane is large enough for a lane change with **prediction** is computed here for simplicity in the following manner:
    
```C+
// Evaluate lane feasibility
laneinfo.feasible = (laneinfo.front_gap > lane_change_front_buffer)
        && (laneinfo.front_gap_next > lane_change_front_buffer)
        && (laneinfo.back_gap < lane_change_back_buffer)
        && (laneinfo.back_gap_next < lane_change_back_buffer);
```

See method `PathPlanner::process_sensor_fusion`.
    

#### 3. Behavior planning, create plann for target lane and speed

The behavior planner is designed as a simple state machine with states: *start*, *keep lane*, *prepare for lane change* and *lane change*. The telemetry and the lane information trigger the transitions between states. 

```C++
enum class STATE { START = 0, KEEPLANE = 1, PRELANECHANGE = 2, LANECHANGE = 3 };
```

Firstly a best lane is calculated according to the three lane states. Secondly following state machine is evaluated according to the real time traffic. For more stable driving, a `meters_in_state` value, which record how far in this state, is considered as one of the transition conditions.

The output of the planner are the target lane (variable `target_lane`) and the desired target speed (`target_speed`) that the next stages should consider.

See method `PathPlanner::create_plan`.

##### *KEEPLANE* State

On enter fix the target speed to road limit.

If there is car in front forcing a speed below the road limit and there is a faster lane, set that lane as change target (variable `changing_lane`) and transition to *PRELANECHANGE*. The faster lane is decided in the method `PathPlanner::get_best_lane`.

##### *PRELANECHANGE* State

On enter, fix the target lane as the changing lane, of the closest lane in the direction of the changing lane. If the target lane is feasible, transition to *LANECHANGE*. If not feasible and the changing lane is not the fastest, abort and transition back to *KEEPLANE*. Otherwise, wait for an opportunity to change the lane adjusting the speed if necessary.

##### *LANECHANGE* State

During lane change the target speed will increase. When current lane change completed, if it's not the final lane, transition to *PRELANECHANGE*, else to *KEEPLANE*. If a risk of collision is detected in the middle of a lane change the change will be aborted setting the lane target to the reference lane.

Lane change parameters:
```C++
// Lane change parameter
		double lane_horizon = 50; //m
		double lane_change_front_buffer = 10; //m
		double lane_change_back_buffer = -10; //m, backward minus value
		double lane_emergy_front_buffer = 5; //m
```


#### 4. Collision avoidance

A simple mechanism to avoid collisions with cars in the target lane reducing the speed. The output of this component is a maximum safe target speed. When distance between front car is too close, a `warning_collision` state will be set true.

See method `PathPlanner::collision_avoidance`.

#### 5. Speed control

A simple mechanism to accelerate or decelerate safely. When `warning_collision` is set an emergency brake will be executed. It outputs the final target speed for the trajectory generator.

See method `PathPlanner::speed_control`.

#### 6. Generate final trajectory

Generates a smooth trajectory from the reference point to the target lane and to the target speed. The trajectory is generated using a cubic spline interpolation in the manner described in the walkthrough video.

See method `PathPlanner::create_trajectory`.

---

   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

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

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

