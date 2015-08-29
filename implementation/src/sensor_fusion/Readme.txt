This folder contains the sensor fusion for obstacle grid and odometry.

Folders:
* AADC_ObstacleGrid: 	This filter merges the given sensor data into an opencv matrix using sensor values and odometry data.
* MTUM_DepthGrid:       This filter transforms the given depth image to an obstacle grid like map.
* MTUM_MotionEstimator: This filter calculates the car odometry, e.g. the movement (driven distance and orientation change) since the last update from this filter.

Files:
* CMakeLists.txt:	Datei der CMake kette
* Readme.txt:		Diese Datei