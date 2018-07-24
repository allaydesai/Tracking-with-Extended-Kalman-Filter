# Extended Kalman Filter Project Starter Code

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Self-Driving Car Engineer Nanodegree Program

Overview

---

[//]: # (Image References)

[image1]: ./images/simulator.PNG "Simulator Image"

This project utilizes a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. To acheive this I used a Kalman filter, lidar measurements and radar measurements to track the bicycle's position and velocity around a vehicle. Goal of the project is to obtain RMSE values that are lower than the desired tolerance. 

**Project Goals**

The goals / steps of this project are the following:
* Build a Kalman Filter (KF) for tracking using Lidar measurments
* Build an Extended Kalman Filter(EKF) for tracking using Radar measurments
* Test algorithm against Dataset-1 in the simulator
* Output coordinates px, py, vx, and vy RMSE should be less than or equal to the values [.11, .11, 0.52, 0.52]

**Project Files**

The repository consists of the following files: 
* ./Docs/ - Supporting documentation
* ./data/obj_pose-laser-radar-synthetic-input.txt - Text file contating measurements from the two sensors
* ./ide_profiles - Editor profiles
* ./src/Eigen - Eigen Library
* ./src/CMakeLists.txt - Code compilation
* ./src/main.cpp - communicates with the Term 2 Simulator receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE
* ./src/FusionEKF.cpp -  Initializes the filter, calls the predict function, calls the update function
* ./src/tools.cpp - function to calculate RMSE and the Jacobian matrix
* ./src/json.cpp - Structuring to communicate with simulator
* ./src/kalman_filter.cpp - Defines the predict function, the update function for lidar, and the update function for radar
* ./src/FusionEKF.h - Header file for FusionEKF.cpp
* ./src/kalman_filter.h - Header file for kalman_filter.cpp
* ./src/measurement_package.h - Header file for measurement_package.cpp
* ./src/tools.h - Header file for tolls.cpp

**Project Dependencies**

* cmake: 3.5
* make: 4.1 (Linux and Mac), 3.81 (Windows)
* gcc/g++: 5.4
* uWebSocketIO: Use install-ubuntu.sh

**Project Simulator**

This project involves Tracking Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

Lidar measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles.

![alt text][image1]

**Project Build Instructions**

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

---

**Dataset**

The dataset consists of simulated lidar and radar measurements detecting a bicycle that travels around the vehicle.

Data file:
* obj_pose-laser-radar-synthetic-input.txt

Data Flow:
1) The measuremennt processor/matlab simulator is generating the FUSION .txt file:
	"data/obj_pose-laser-radar-synthetic-ukf-input.txt";
	OR
	"../matlab_examples/obj_pose-laser-radar-synthetic-ukf-input.txt";

The Input file format is:
#L(for laser) meas_px meas_py timestamp gt_px gt_py gt_vx gt_vy
#R(for radar) meas_rho meas_phi meas_rho_dot timestamp gt_px gt_py gt_vx gt_vy

Example:
R	8.60363	0.0290616	-2.99903	1477010443399637	8.6	0.25	-3.00029	0
L	8.45	0.25	1477010443349642	8.45	0.25	-3.00027	0
	
2) The EKF Algorithm reads form file reads all the lines and generates measurement structures
3) The MeasurementProcessor() is called with individual measurements (one by one). The results are saved
(Attention: no file processing routines are used inside MeasurementProcessor() all the file processing routines are in the main function
So the data read/write is decoupled from the algorithm
4) The results are saved in an output file:
"data/obj_pose-laser-radar-ekf-output.txt"

Output file format:
est_px est_py est_vx est_vy meas_px meas_py gt_px gt_py gt_vx gt_vy

Example:
4.53271	0.279	-0.842172	53.1339	4.29136	0.215312	2.28434	0.226323
43.2222	2.65959	0.931181	23.2469	4.29136	0.215312	2.28434	0.226323


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

# Results

The project goal of keeping RMSE value less than or equal to the values [.11, .11, 0.52, 0.52] was succesfully acheived.

