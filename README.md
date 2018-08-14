# Term 2 Project 5: Model Predictive Control (MPC)  

---
The goal of this project is to implement Model Predictive Control to drive the car around the track.

### Project output

[![IMAGE ALT TEXT](http://img.youtube.com/vi/MoWPw6ZHvQg/0.jpg)](https://youtu.be/MoWPw6ZHvQg "MPC Output with N=10, dT= 0.05")

[![IMAGE ALT TEXT](http://img.youtube.com/vi/hY0dwCbiBdg/0.jpg)](https://youtu.be/hY0dwCbiBdg "MPC Output with N=20, dT= 0.05")

[![IMAGE ALT TEXT](http://img.youtube.com/vi/1na6xRt9zgQ/0.jpg)](https://youtu.be/1na6xRt9zgQ "MPC Output with N=20, dT= 0.1")


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

### Model

The model I implemted is as follows: 
```
// Initial state.
const double x0 = 0;
const double y0 = 0;
const double psi0 = 0;
const double cte0 = coeffs[0];
const double epsi0 = -atan(coeffs[1]);

// Kinematic model is used to predict vehicle state at the actual
// moment of control (current time + delay dt)

  // State after delay.
double x_delay = x0 + ( v * cos(psi0) * dT );
double y_delay = y0 + ( v * sin(psi0) * dT );
double psi_delay = psi0 - ( v * delta * dT / Lf);
double v_delay = v + a * dT;
double cte_delay = cte0 + ( v * sin(epsi0) * dT );
double epsi_delay = epsi0 - ( v * atan(coeffs[1]) * dT / Lf );     
```

where ```dT``` is the delay set to 100ms. 

```a``` is the the throlttle value

```delta``` is the steering angle.

### Selection of N and dT

I used the value (N,dT) = (20, 0.05) initially but if produced erratic behavior. Then is used (N,dT) = (10, 0.1) and produced a stable behavior on the track. 

### Polynomial Fitting and MPC Preprocessing

The model was implemented in the code ```MPC.cpp``` in the lines 132-206. and used a plynomial fitting of 3rd degree at line in ```main.cpp``` : 

```
auto coeffs = polyfit(ptsx_transformed,ptsy_transformed,3);
```

### Model Predictive Control with Latency

To handle actuator latency, the state values are calculated using the model and the delay interval. These values are used instead of the initial one. The code implementing that could be found at ./src/main.cpp from line 110 to line 201.
