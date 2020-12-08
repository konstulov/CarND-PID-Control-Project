# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## PID Control
Here we describe how the car is able to drive around the track using PID Control.

### Control Parameters
There are three parameters that we need to set for a PID controller to work - P (proportional), I (integral), and D (derivative).
In the code here, these parameters are stored in a C++ vector `p := {P, D, I}` and the values are the weights used to calculate the steering angle for the vehicle at each time step in the simulator according to the following equation.
```
steer_value = -p[0] * cte - p[1] * diff_cte - p[2] * int_cte;
```
where `cte` is a cross track error of the car at the current simulation time step, `diff_cte` is the difference between the current and previous CTE values and `int_cte` is the cumulative CTE (used to correct for any steering bias that would otherwise accumulate to large errors over time).
The goal here is to find good (optimal) values for the vector `p` that would result in the least overall error (normalized sum of squared CTEs) when the car drives around the track.

### Parameter Tuning
In general, we can start the parameter tuning by initializing all parameters to 0 and letting the optimization algorithm figure out the best values on its own.
However, given that the car in the simulator is driving on a track and random choices for the initial parameter values would often lead to the car driving off the track, it is best to start with some good initialization and let the algorithm tune these parameters without having the car drive off the track all the time. 
The initial parameters (`{0.1, 3.0, 0.004}`) were chosen based on the previously used parameters in another task and upon verifying that the car could drive around the track without going off wildly, the Twiddle optimization algorithm was implemented and run for several hours starting with these initial values. In addition to setting initial values for `p`, we also had to initialize vector `dp` which stores the magnitudes of increments for each parameter in vector `p`. Vector `dp` was initialized to be roughly 1/4 of the corresponding parameter value. This heuristic allowed the algorithm to proceed incrementally without taking ambitiously large steps in the early cycles that would have led the car off the track.
Running the Twiddle Algorithm for several hours (over 100 training cycles each about 1 min of track driving in the simulator), it converged to the following values `p = {0.123803, 3.3709, 0.00450544}`.
Here we can see that the largest weight is for the `D` parameter and the smallest is for the `I` parameter. This can be explained by the nature of values that `cte`, `diff_cte` and `int_cte` tend to take on. First, `cte` is the Cross Track Error, which gets the most immediate feedback (being the current deviation of the car from its desired course). Second, `diff_cte` is the change between consecutive CTEs and its value tends to be small relative to CTE. Here we note that for the purposes of reducing noise, instead of using consecutive values of CTE to compute `diff_cte`, we actually use `n_diff` (set to 4) parameter to specify how many time steps back to look to get the previous CTE value. Lastly, `int_cte` is a cumulative CTE error and it can grow very large if there's bias in the car's steering (e.g. the car tends to drive slightly left when steering angle is set to drive straight). Hence, the `I` parameter has the smallest value to avoid being the dominant component in the steering angle.

### Racing On The Track
After the Twiddle Algorithm converged to the parameters that had the lowest overall error, the next challenge was to see how fast can the car go around the track without eventually driving off.
Here, I tried a combination of approaches but in the end spent most of the time manually tuning a number of coded parameters.
First I tried incrementing the value of throttle until the car drove off the track (raching throttle value of about 0.6 when this happened). Next, I picked the min and max throttle values and made the throttle of the car depend on the steering angle of the vehicle (provided by the PID controller). The idea here is when the car's steering angle is small, the car's error is also small so we can drive faster along a (more or less) straight line. On the other hand, when the steering angle is large, this indicates that the car is either in a turn or trying to wiggle its way back to its prescribed course, hence a lower speed would help it stabilize itself.
Initially I implemented a linear dependence between the steering angle and the throttle but the car tended to drive off the track so I changed the dependence between throttle and steering angle to be inversely proportional to each other. This helped the car stabilize faster, since higher values of steering angle resulted in significantly less extra throttle than with linear dependence.
Additionally, I also set a maximum value of steering angle (another tunable parameter) to prevent the car from taking large swings back and forth when it is near the center of its desired trajectory but is going too fast to stabilize itself without winding wildly and spinning out.
There were many choices of parameters that seemed to work well and resulted in fast laps (the fastest being at about 39 seconds) but a lot of these were unstable in the sense that the car would eventually drive off the track given enough time to drive around. After several attempts to hand tune the parameters I settled on the values that gave a reasonable trade-off between speed (lap time) and overall error.

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.


