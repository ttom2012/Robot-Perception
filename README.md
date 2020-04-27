# IMU calibration and orientation tracking

### This code can be executed in matlab

### This code is mainly divided into two parts: 1) calibrate the raw data; 2) compare three basic fusion algorithms

### 1. Calibrate raw data
	Run the file: calibrateddata.m
    ### The parameter used in calibrateddata is calculated by file: calibratedpara.m


### 2. Compare three fusion algorithms
	Run the file: comparison.m


### 3. Data flow

       Raw data --> Fusion algorithm --> orientation tracking
       Raw data --> calibration --> Fusion algorithm --> orientation tracking

### 4. Data structure
	time, accelerometer, gyroscope, magnetometer 


### 5. In order to apply these codes for different data set, you can change the two row codes in above two files:

       For calibrate the raw data
		load('filename')
      		Rdata=filename

       For compare three fusion algorithms
		load('filename')
     		our_data11=filename
### 6. Dependencies

	The EKF algorithm is based on the paper:Sabatelli, Simone, et al. "A double-stage Kalman filter for orientation tracking with an integrated processor in 9-D IMU." IEEE Transactions on Instrumentation and Measurement 62.3 (2012): 590-598
	
	The Mahony algorithm is inspired by: Mahony, Robert, Tarek Hamel, and Jean-Michel Pflimlin. "Nonlinear complementary filters on the special orthogonal group." IEEE Transactions on automatic control 53.5 (2008): 1203-1218.
	The calibration process is refer the paper: Tedaldi, David, Alberto Pretto, and Emanuele Menegatti. "A robust and easy to implement method for IMU calibration without external equipments." 2014 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2014.
	Compariosn process refer the blog: Shike Shen: 9 Axis IMU Calibration [https://blog.csdn.net/shenshikexmu/article/details/80013444]


