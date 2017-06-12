# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Project goals

In this project I implement a PID controller to successfully 
(and safely) drive a car around Udacity's self-driving car simulator. A 
PID controller is a control loop feedback mechanism that relies on three 
parameters - proportional (P), integral (I) and derivative (D). At every 
control iteration, the implemented controller receives the current cross-track 
error from the simulator and uses the PID parameters to update the car's 
steering angle.
 
#### Proportional (P) coefficient

The P component provides a steering angle that is proportional to the 
current cross-track error:

steering_angle = -p * cte

As a result, the car will try to stay as close as possible to a region where 
the cross-track error would be zero. Being used alone, it will likely cause the 
car to overshoot its target and drive in a constant left and right swerving 
pattern.

#### Integral (I) coefficient

This parameter uses the accumulated cross-track error to determine the 
steering angle. It's particularly useful if there's a mismatch between the 
intended steering angle and how much the car is actually steering - by 
accumulating the errors, it provides a mechanism to correct the steering 
angle in such situations. 

steering_angle = -i * sum(cte)

#### Differential (D) coefficient

The D coefficient addresses the overshooting issue posed by using a 
P-controller alone. It operates on the difference between the current 
error and the previous error. This way, the steering angle becomes less 
abrupt as we get closer to the region of zero error.

steering_angle = -d * (cte - previous_cte)

#### Implementation details and parameter tuning

To tune the values of P, I and D, I used twiddle (a.k.a. coordinate ascent). 
Twiddle can be executed by providing the number of iterations it should run 
for in every optimization loop to the pid binary. For instance, by using 
the following command line, the tool will start in twiddle mode and use 
100 steps to calculate the average error for every parameter selection 
step:

./pid 100

By running twiddle with 1000 steps (enough to get through the first curve) 
and a fixed throttle, I obtained the following coefficients:
 
* P = 1.56387
* I = 0.0079461
* D = 10.886

The implementation provided in this project uses the last 20 iterations to 
calculate the accumulated cross-track error used by the integral component 
of the controller. It's important to notice, however, that the integral 
component should not be important in this scenario, given there's 
no disconnection between the input steering angle and how much the car is 
steering in the simulator (as proven by the parameters chosen by twiddle).

To further improve parameter selection, I observed how the car behaved in 
the full track. One noticeable point is that the cross-track error provided 
by the simulator is not accurate in portions of the track, leading the car 
to swerve more than necessary. Throttle was also not being controlled in 
any meaningful way, so by adjusting the throttle it would be likely 
necessary to adjust the PID parameters.

Instead of using another PID controller for throttle I decided to use the 
following formula:

throttle = 0.5 - (0.49 * (|cte| / max_cte));

This provided relatively fast speeds while greatly slowing down the car when 
the cross track error was to big. By running twiddle once again and then 
adjusting the parameters based on the car behavior over the full track, I 
settled for the following parameters:

* P = 0.25
* I = 0.00
* D = 5.00

With all that, the car was able to reach top speeds close to 50MPH while 
still remaining safely inside the track.


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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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
