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

### Data structure: time, accelerometer, gyroscope, magnetometer 


### In order to apply these codes for different data set, you can change the two row codes in above two files:

       For calibrate the raw data
		load('filename')
      		Rdata=filename

       For compare three fusion algorithms
		load('filename')
     		our_data11=filename


