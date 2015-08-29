This folder contains all computer vision filters. Please note that all those filters are based on a top-down view and use machine learning methods.

Folders:
* AADC_ImageProcessing:         Calculates the top-down view of the road ahead using an inverse mapping (IP mapping).
* AADC_LaneDetection:           This filter provides lane-curvature and offset of the car from right lane markings.
* MTUM_CrossingDetection_ML:    This filter detects crossings (using signs) and calculates the orientation of the crossing in respect to the car, as well as the distance between car and crossing.
* MTUM_MarkerDetection:         This is a modified version of the MarkerDetection-filter provided by Audi, which also provides the pose of the sign.
* MTUM_MarkerDetection_FreeDemo:This ia a modified version of the MTUM_MarkerDetection used for the free demonstration
* MTUM_ParkingDirection:        This filter determines the type of parking lot the car is currently in. That information is used to select the correct pull out maneuver.
* MTUM_ParkingSpaceDetection:   This filter detects and tracks the nearest parking lot (distance and orientation in respect to the car).
* MTUM_PylonDetection:          This filter detects pose of the next pylon used for the free demonstration.

Files:
* CMakeLists.txt:	Datei der CMake kette
* Readme.txt:		Diese Datei