# CarND-Controls-PID
A PID Control to control the steering of a car to race around the track in a simulator. 

## Components of PID-Controller

**P (propoprtional)**: Calculates the value (steering angle in this) proportional to the passed _error_ (Cross Track Error in this case).

**D (Derivative)**: Derivative part avoids overshooting (oscilations) of the car from its _reference track_, assuming `delta=1` in this project.

**I (Integral)**: Handles any systematic bias (wheel alignment error, wind pressure, slippery road etc.)

For example, following video was recorded of simulator with following values for each part (P, I, D) of the PID.
`
```
Proportional gain = 0.2
Integral gain = 0.0 (since no or negligible systematic bias in this simulator)
Derivative gain = 4.0
```

![animation](visualization/animated.gif)


## Parameter Fine Tunning

I used a mix of manual guess work and Twiddle algorithm. You can find the Twiddle code in repo.

- For speed = 30mph I used following values

P gain = 0.2
I gain = 0.0
D gain = 4.0

- For speed = 50mph I used following values

P gain = 0.1
I gain = 0.0
D gain = 1.5


## Getting Started

There is a detailed example of code in main.cpp that you can refer to for how to connect to the simulator and how to use PID-Controller. Following is a high level use of PID-Controller code.

```
PID pid;

//initialize gians Kp, Ki, Kd
pid.Init(0.2, 0.0, 4.0);

//pass cross track error (cte) to pid
pid.UpdateError(cte);

//get the resultant steering angle
double steer_value = pid.TotalError();

//as steering angle is in [1, -1] so make sure boundaries 
if (steer_value > 1) {
steer_value = 1;
} else if (steer_value < -1) {
steer_value = -1;
}

//pass the steer_value to simulator or whatever hardware/software you want to
```


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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://github.com/udacity/CarND-PID-Control-Project/releases) in the classroom.

## Basic Build Instructions

Running `./do.sh` on Linux or Mac should build and run the PID controller. Following are the commands in that file. 

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 


