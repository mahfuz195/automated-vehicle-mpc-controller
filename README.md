# Term 2 Project 5: Model Predictive Control (MPC)  

The goal of this project is to implement Model Predictive Control to drive the car around the track.

[image1]: model.PNG "Vehicle Model"
[image2]: cost.gif  "Cost Function"


### Model

As per the lecture seris, I used a global kinematic model, which is a simplification of a dynamic model that ignores tire forces, gravity and mass. The state model is represented by the vehicles position, orientation angle (in radians) and velocity. The global kinematic model is as follows:

![alt text][image1]

Using Model Predictive Controller(MPC), the Vehicle model can be converted into an optimization problem, where the goal is to reduce the cross track error(CTE), and orientaiton error(epsi). The formula for optimization is as follows: 

![alt text][image2]

I tuned the weights manually as follows: 

|   Cost Components     | Value   |
| ------:|-------:|
| CTE     	| 2000 |
|  EPSI     	| 2000 |
| Speed	 		| 1 |
| Steering angle	 	| 25 |
| Throttle	 		| 25 |
| Rate of chnage of steering angle	| 400000 |
| Rate of chnage of throttle		 	| 8000 |


### Selection of N and dT

Selecting the value of N and dT was crucial. As, A higher N adds to more computational since more calculations are needed if dt remains the same. Same goes for a lower value of dt, if N remains same. First, I selected the prediction horizeon as 0.5 second, As 0.5-2 seconds are a resonable horizon of time prediction for a moving car. So, Initially I selected the value dt=0.05 and N=10. It gave me a good result, as follows: 

[![IMAGE ALT TEXT](http://img.youtube.com/vi/MoWPw6ZHvQg/0.jpg)](https://youtu.be/MoWPw6ZHvQg "MPC Output with N=10, dT= 0.05")

Then I increased the value of N=20 and dt=0.05, making a time horizon of 1 sec. But the result was erratic and the car went of the track. The output is as below:

[![IMAGE ALT TEXT](http://img.youtube.com/vi/hY0dwCbiBdg/0.jpg)](https://youtu.be/hY0dwCbiBdg "MPC Output with N=20, dT= 0.05")

Finally, I selected N=10, and dt=0.1, having a time horizon of 1 sec. The choice of 0.1s for dt results in actuations every 100ms which is good enough for this exercise and produced a stable behavior on the track. The final output is as shown below:


[![IMAGE ALT TEXT](http://img.youtube.com/vi/1na6xRt9zgQ/0.jpg)](https://youtu.be/1na6xRt9zgQ "MPC Output with N=20, dT= 0.1")


### Polynomial Fitting and MPC Preprocessing

The model was implemented in the code ```MPC.cpp``` in the lines 111-118:
```
AD<double> c0 = coeffs[0];
AD<double> c1 = coeffs[1];
AD<double> c2 = coeffs[2];
AD<double> c3 = coeffs[3];

AD<double> f0 = c0 + (c1 * x0) + (c2*CppAD::pow(x0,2))+ (c3*CppAD::pow(x0, 3));
AD<double> psides0 = CppAD::atan(c1 + (2*c2*x0) + (3*c3*CppAD::pow(x0,2)));

```

### Model Predictive Control with Latency

To handle actuator latency, the state values are calculated using the model and the delay interval. These values are used instead of the initial one. The code implementing that could be found at ./src/main.cpp from line 110 to line 213.

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

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

