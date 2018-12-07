# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)

[image1]: ./plot/nis_lidar.png "LiDAR NIS"
[image2]: ./plot/nis_radar.png "Radar NIS"

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the programs that need to be written to accomplish the project are src/ukf.cpp, src/ukf.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` Previous versions use i/o from text files.  The current state uses i/o
from the simulator.

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

This information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/c3eb3583-17b2-4d83-abf7-d852ae1b9fff/concepts/f437b8b0-f2d8-43b0-9662-72ac4e4029c1)
for instructions and the project rubric.

## Unscented Kalman Filter

A Kalman filter is used to validate the certainity of a dynamic system with linear predictive model. However, for a gaussian distribution, applying a non-linear predictive model does not work. Hence, for weak non-linearity the EKF is applied. However, if there is strong non-linearity in the predictive function, then [UKF](https://www.seas.harvard.edu/courses/cs281/papers/unscented.pdf) works better. In the case of the UKF, generally the kinematic model which is chosen is the CTRV for the prediction step. The main point is that we have a transform a distribution through a non-linear transform with minimal error propagation in the measurements. One more difference between the `predict state` and the `update state` is the use of non-linear noise in the `predict state`. However, in the `update state` we can measure the noise directly from the sensor data which is generally considered to be additive. Process noise is non-linear and that is why augmentation step was required in the `predict step`.

**We have two sensors:**
The simulation tool generates sensor data for a single tracked object.
- A lidar sensor that measures a tracked object's position in cartesian-coordinates `(x, y)`
- A radar sensor that measures a tracked object's position and relative velocity in polar coordinates `(rho, phi, rho_dot)` with process covariance noise `Q` consisting of speed noise and yaw rate noise, which generally the radar sensor supplier provides

**From the UKF we would like to get the following parameters of the tracked object:**
- The position in cartesian co-ordinates `x, y`
- The velocity `v`
- The yaw angle in rad `yaw`
- The yaw rate  in rad/second `yaw_rate`

**Algorithm**
  * Get sensor data [L/R Data, Timestamp]
   * If first measurement then initialize state vector [x] and state covariance matrix [P]
   * Else compute the time difference [dt] between current sensor data timestamp and the previsous timestamp
  
  * Predict [x, P, Q, dt]
   * Generate sigma points from state vector
     Input: State vector [x] and State covariance matrix [P]
     Output: Sigma points [x x+sqrt((lambda+nx)*P) x-sqrt((lambda+nx)*P)]

   * Generate augmented sigma points
     Input: Augmented State vector [x, Speed noise, Yaw rate noise] and Augmented State covariance matrix [P, Q]
     Output: Augmented sigma points [x_aug x_aug+sqrt((lambda+n_aug)*P_aug) x_aug-sqrt((lambda+n_aug)*P_aug)] 

   * Predict sigma points from augmented sigma points using CTRV model
     Input: Augmented sigma points [x_sigma_aug] and CTRV model
     Output: Predicted sigma points [x_sigma_pred] 

   * Convert predicted sigma points to mean and covariance
     Input: Predicted sigma points [x_sigma_pred] and Weights [w]
     Output: Predicted state mean [x_sigma_pred_mean] and predicted state covariance [x_sigma_pred_covariance]

  * Update [L/R Data, dt, w]
   * Calculate cross correlation between the predicted state and the sensor data measurement
     Input: Predicted state [x_sigma_pred], sensor data measurement [L/R data], timestamp [t + dt]
     Output:  - Cross correlation [Sum (weights * (difference of predicted sigma points in state space) * (difference of predicted sigma points in measurement space)')]
              - Kalman Gain
              - NIS (Noise measurement intuition)

## Evaluation of the noise parameters

The [Noise measurement intuition](https://www.youtube.com/watch?v=S4fX3X_9oik) parameters helps to check the estimation of the process noise via comparision with [Chi-squared_distribution](https://en.wikipedia.org/wiki/Chi-squared_distribution) for certain degrees of freedom (DOF). The process noise I have chosen experimentally are as follows:
```
// Process noise standard deviation longitudinal acceleration in m/s^2
std_a_ = 1.0;

// Process noise standard deviation yaw acceleration in rad/s^2
std_yawdd_ = .9;
```
In case of LiDAR there are 2 DOF as we predict two variables `p_x` and `p_y`. The Chi-squared_distribution for 2DOF at 95% is = `7.815`.

![Noise measurement intuition - LiDAR][image1]

And in case of RADAR the DOF is 3 as we measure three variables `rho`, `rho_dot` and `phi`. The Chi-squared_distribution for 32DOF at 95% is = `5.991`.

![Noise measurement intuition - RADAR][image2]

The final RMSE which I get is: `RMSE_x: 0.0660999, RMSE_y: 0.0826775 RMSE_vx: 0.295983 RMSE_vy: 0.270642`.

