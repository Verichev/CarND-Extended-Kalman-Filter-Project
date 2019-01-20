# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

[//]: # (References) 
[dataset1]: ./dataset1.png

The server that runs Extended Kalman Filter gets radar and laser data from simulator and feeds the position of the vehicle to the client. Dependent on data source the program runs the extended KF implementation (for radar data) or standard KF implementation (lidar data).

![Tracking car with EKF][dataset1]

Red circles are lidar measurements. Blue circles are radar measurements. Green markers are the estimations of the KF. The RMSE errors represent the errors for estimated values against real values.
