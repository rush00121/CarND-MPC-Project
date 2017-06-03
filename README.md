# CarND-Controls-MPC

Self-Driving Car Engineer Nanodegree Program
---

### The Model
   
We have used a simple kinematic model to represent the vehicle state and its actuators

The model looks like this:

State: [x,y,ψ,v]

Actuators: [δ,a]

where x,y are the vehicle coordinates, the vehicle orientation is ψ and v is the vehicle velocity. 

The vehicle has two actuators, which we'll denote as δ for steering angle and a for acceleration (throttle/brake combined).

The equations that govern the state transition for our kinematic model are as follows : 
```
  x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
  
  y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
  
  psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
  
  v_[t+1] = v[t] + a[t] * dt
  
  cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
  
  epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```


###Timestep Length and Elapsed Duration (N & dt)

The system has an inherent delay of 100 ms. Hence it is safe to say that we are sending
not more than 1 input every 100ms. Hence the time interval between 2 consecutive inputs can be safely assumed to be 100ms. 
If we take N as 10, then it means we are looking ahead 10 * 100ms = 1 second .
This is good enough for the simulation since it does not look too far ahead in the future nor too little. 
Too far is of no use since we need the ipopt the solve the  to produce 

If N = 10, then to match 100ms frequency, dt = 0.1 . 

Higher values of N means we will have more computation required to calculate the 
desired trajectory using ipopt which is not required. 

I tried with N = 20, N = 25 but N=10 produced the most stable results. 

I have also limited ipopt to return the result within 0.15 s, hence a lower value of N will ensure that ipopt will complete its computation faster.


###Polynomial Fitting and MPC Preprocessing

I have used a polynomial of order 3 to fit the ptsx and ptsy points to a trajectory. 
Using the poynomial cooefficient,we can plot a reference trajectory that can be seen in the simulation by
the yellow line.


###Handling Latency

The output actuator signals generated generate N ie 10 actuator inputs for each point on the desired 
trajectory. Since we have a latency of 100 ms, instead of using the current state actuator, I have 
used the next state point actuators. This means that when we send our signal,
the signal is intended not for the current point, but the point it will be at after 100ms. This is very easy to verify.
If we send the 0th position input, the car behaves very wobbly that signifies that the signal on which it is 
acting is the older state. If we send the 2nd position input, the car fails as well. 

I am also using rudimentary mechanisms to detect deviation from the reference trajectory to control speed.

If we deviate too much from the reference trajectory, then I change the reference velocity to a lower value.
This helps in slowing down the car, as well as accelerating it when the car is on the correct path.


###Scope for improvements 

1) Latency is not always constant. Hence N and dt should be able to handle this uncertainty
2) The mechanism for detecting deviation from the reference trajectory can be improved. It can factor in the 
   radius of curvature to detect and determine the correct speed. Currently we are using only 2 speed settings.
   This can be enhanced to determine different speed settings on different trajectory paths.
3) The current cost functions are manually tuned. It is worth exploring if the cost functions 
   can be dynamically tuned using twiddle or SGD or some other mechanism.

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).
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
