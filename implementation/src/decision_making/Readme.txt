This folder contains all planning and control filters.

Folders:
* AADC_LaneFollower: 	    This filter represents the steering control during lane-following, parking-lot-detection and crossing-approaching.
* AADC_SegmentDriver:       This filter executes some scripts, selected via the current maneuver (jury module!), and provides path-following for pre-calculated trajectories (turning and parking).
* AADC_SpeedRecommender:    This filter extracts max. driving speed from obstacle grid and limitations set from outside.
* MTUM_CarController:       This filter communicates with the jury and maintains the maneuver list.

Files:
* CMakeLists.txt:	Datei der CMake kette
* Readme.txt:		Diese Datei