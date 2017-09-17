# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program


## My Solution Write Up

For this solution, I used the provided code, as well as some of the class exercises code to jump start it. I was very quickly able to get the Model Predictive Controller running at high speeds and completing the track without having the tire go off the track. However, for my solution work with latency I had to make some changes. I tried different approaches before realized the solution was simpler that I first expected: I use the fitted polynomial to make most predictions about the state after 100ms latency.

### Model

The first step of my model was to fit a third degree polynomial on the next 6 waypoints provided by the simulator. To facilitate, the waypoints were transformed from the global positioning of the track to the car coordinate position. The car coordinate position has the car moving forward along the x axis, while any turning would cause disaplacement along the y axis.

Next step of the model was to create a state of the vehicle so the model could optimize the actuators over the next steps to best fit along the polynomial. The state included the following values:

- x (car relative coordinates)
- y (car relative coordinates)
- psi (turning angle)
- velocity
- cross check error
- turning angle error

The goal of the model was to optimize the following actuators to best fit the waypoints:

- Throttle
- Steering Angle


#### Handling Latency

In order to properly handle the 100ms, I couldn't use the current state of the vehicle, as the actuators would change by the time it should already be at the next step, considering the 100ms step interval. In order to properly handle the car, I had to estimate the state 100ms in the future. Here's how I estimated:

First step was to predict the velocity, as the rest of the state values would depend on it:

```
Predicted_Velocity = Velocity + (Throttle * Latency)
```

After that, I had to predict the angle the car would be, which would dependon the rate at which the car was turning:

```
Predicted_Angle = Predicted_Velocity * Latency * Steering_Rate / 2
```
The division by 2 was to average the rate of change, as the Steering_Rate would have changed over time during the 100ms interval.

Using the estimated velocity and angle, estimate the x and y position:

```
# How much the car moved in the x and y axis
X_Displacement = Predicted_Velocity * Latency * cos(Predicted_Angle)
Y_Displacement = Predicted_Velocity * Latency * sin(Predicted_Angle)

# Estimate in the track global position
Global_X = Previous_X + X_Displacement * cos(Predicted_Angle) - sin(Predicted_Angle)
Global_Y = Previous_X + X_Displacement * sin(Predicted_Angle) + cos(Predicted_Angle)
```

Transforming the Global_X and Global_Y into the car coodinate space would give an x,y coordinate value in the fitted polynomial that would represent the estimated position of the car 100ms in the future.

By assuming that our polynomial is fitted to the center of the track, we can also estimate the cross track error by evaluating the x position of the vehicle and getting the expected y along the polynomial, and substracting the estimated y position. That is not the true cross track error, but works well enough for the problem.

The angle error can be estimated with the following formula:

```
Predicted_Angle_error = Predicted_Angle - atan(coeffs[1] + 2 * Predicted_X * coeffs[2] + 3 * coeffs[3] * pow(Predicted_X, 2))
```
where coeffs is the vector of coefficients of the fitted polynomial.




### Number of Steps and Steps Interval

Since the simulated latency was set to 100ms (which is the expected actuator latency on a real world scenario), I set the interval between steps to that same 100ms. Making the steps shorter than that would not bring benefits as the actuators wouldn't respond on a smaller interval than that. There was also no reason to increase the step interval to over 100ms.

For the number of steps, I experimentes with extreme short numbers and long numbers. Having a small number of steps (around 5) would case the polynomial to have unnecessary sharp curves, making the car lose control and having unnatural turning. Having a bigger number of steps (around 20) would work well for a lot of the steps, with the exception of when the cost function would rise, usually around curvers, which would cause the polynomial to fit weird shapes.

I settled on 12 steps, which didn't show any behavior that the shorter or bigger number of steps had.



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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * If challenges to installation are encountered (install script fails).  Please review this thread for tips on installing Ipopt.
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

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

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
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

