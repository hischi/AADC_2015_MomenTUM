/**
Copyright (c)
Audi Autonomous Driving Cup. Team MomenTUM. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */


/**********************************************************************
 * $Author:: MomenTUM $  $Date:: 2015-03-15 13:29:48#$ $Rev:: 26104  $*
 **********************************************************************/

/*! \brief SegmentDriver
 *
 *  This filter executes some scripts, selected via the current maneuver (jury module!), and provides path-following for
 *  pre-calculated trajectories (turning and parking).
 */

#ifndef _SEGMENTDRIVER_H_
#define _SEGMENTDRIVER_H_

#include "momenTUM_const.h"
#include <pthread.h>


#define OID_ADTF_SEGMENTDRIVER "adtf.aadc.SegmentDriver"

/*!
 * This enumeration contains all availible script operations and their opcode (used to store the scripts internally).
 */
enum SCRIPT_OPERATIONS {NOP=0, SET_CURV=1, SET_SPEED=2, WAITFORORIENTATION=3, CHECKROW=4, WAITFORDISTREAR=5, SEARCHPARKINGLOT=6, WAITFORDISTDRIVEN=7,
                        WAITFORDISTCROSSING=8, WAITFORTIME=9, LFENABLE=10, WAITFORSIGN=11, SETINDICATOR=12, INIT_ORIENTATION=13, CHECKOT=14, HSENABLE=15, IGNORESENS=16};

/*!
 * Struct to store the opcode and parameters of a script operation (single line).
 */
struct ScriptOP{
    tInt op;
    vector<tFloat32> args;
};

/*!
 * Struct to store a script for a single maneuver. Each maneuver should have its own script!
 */
struct ManeuverScript{
    tInt id;
    vector<ScriptOP*> ops;
};

/*!
 * This filter executes some scripts, selected via the current maneuver (jury module!), and provides path-following for
 * pre-calculated trajectories (turning and parking).
 *
 * During filter-initialisation a xml-file (decribed by a ADTF-filter-parameter) is loaded and compiled (stored in the
 * script struct). The filter produces an ADTF-output which is the decompiled version of the loaded scripts. That way
 * one can check the correctness of the compilation.
 *
 * The filter consists of two parts:
 *   - the filter-interface which accepts media-samples, decodes them and stores the information in thread-safe,
 *     variables, each protected by a semaphore (mutex), and
 *   - a pthread, which is not sheduled by ADTF but directly by the operating system (with realtime-requirements). This
 *     thread executes the script line by line. If an operation requires additional information from a filter-input, the
 *     thread waits on a special wait-mutex, which is released by a new media-sample. Thus, the thread does not consume
 *     computation time during it waits for arriving media-samples but needs to be as fast as possible during script-
 *     operation.
 *
 * Example script for turning left:
 * lane_following(1);		    // enable lane-following
 * speed(25);  high_speed(1);	// set speed (max. speed which can be set by the speed-control)
 * wait_sign(2.3); 		        // drive until there is a sign nearer than 2.3 meters
 * speed(22); indicator(0,1); 	// slow down and set left inicator on
 * lane_following(0); 		    // disable lane-following
 * wait_dist_crossing(80); 	    // drive until the crossing is 80 centimeters ahead
 * init_orientation(1); 		// remember current orientation
 * check_row(…); 			    // check right of way (first time before entering the crossing), blocks until everything is free
 * curv(-18); speed(25); 	    // set curvature and speed
 * wait_orientation(-7); 	    // drive until we have an orientation change to the init. orientation of 7 deg.
 * check_ot(); 			        // check right of way (second time)
 * speed(25); 			        // increase speed
 * wait_orientation(-90); 	    // drive until we finish the left-turn (90 deg.)
 * curv(0); indicator(0,0); 	// set indicator off
 * lane_following(1); 		    // reactivate lane-following
 * speed(25); high_speed(1);	// set speed and enable the high-speed-mode.
 *
 */
class SegmentDriver : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SEGMENTDRIVER, "AADC Segment Driver", OBJCAT_DataFilter, "Segment Driver", 1, 0, 0, "<Beta Version");


    cInputPin m_inputPin_state;             /*!< input pin for the state signal to active filters >*/
    cInputPin m_inputPin_maneuver;          /*!< input pin for the current active sub-segment >*/
    cInputPin m_inputPin_crossingType;      /*!< input pin for the current detected crossing-type >*/
    cInputPin m_inputPin_rightOfWay;        /*!< input pin for the current detected row >*/
    cInputPin m_inputPin_distToCrossing;    /*!< input pin for the current dist to crossing >*/
    cInputPin m_inputPin_distToParking;     /*!< input pin for the current dist to next parking-lot >*/
    cInputPin m_inputPin_parkingDirection;  /*!< input pin for the current type of parking-lot >*/

    cVideoPin m_inputPin_ObstacleGrid;      /*!< input pin for the obstacle-map >*/
    cVideoPin m_inputPin_DepthGrid;         /*!< input pin for the obstacle-map (depth-image based) >*/

    cInputPin m_inputPin_motionData;        /*!< input pin for the odometry >*/
    cInputPin m_inputPin_yaw;               /*!< input pin for the current yaw >*/

    cVideoPin m_outputPin_DebugGrid;        /*!< output pin for the debug outputs >*/

    cOutputPin	m_outputPin_maneuverFinished;   /*!< feedback to CarController >*/
    cOutputPin	m_outputPin_LFenable;           /*!< output pin to enable/disable lane-following >*/
    cOutputPin	m_outputPin_PSenable;           /*!< output pin to enable/disable parking-space-detection >*/
    cOutputPin	m_outputPin_SDenable;           /*!< output pin to enable/disable sign-detection >*/
    cOutputPin	m_outputPin_CDenable;           /*!< output pin to enable/disable crossing-detection >*/
    cOutputPin	m_outputPin_PMenable;           /*!< output pin to set strictness during parking  >*/
    cOutputPin	m_outputPin_PDenable;           /*!< output pin to enable/disable parking-type-detection >*/
    cOutputPin	m_outputPin_OTenable;           /*!< output pin to enable/disable overtaking >*/
    cOutputPin	m_outputPin_HSenable;           /*!< output pin to enable/disable high-speed >*/
    cOutputPin	m_outputPin_segmentSpeed;       /*!< output pin to set max. allowed speed >*/
    cOutputPin	m_outputPin_segmentSteering;    /*!< output pin to set steering >*/

    cOutputPin	m_outputPin_turn_left_enabled;  /*!< output pin to enable/disable left-indicator >*/
    cOutputPin	m_outputPin_turn_right_enabled; /*!< output pin to enable/disable right-indicator >*/
    cOutputPin  m_outputPin_hazzard_enabled;    /*!< output pin to enable/disable hazzard-indicator >*/


    public:
        SegmentDriver(const tChar* __info);
        virtual ~SegmentDriver();
	
    protected: // overwrites cFilter
        tResult Init(tInitStage eStage, __exception = NULL);
        tResult Start(__exception = NULL);
        tResult Stop(__exception = NULL);
        tResult Shutdown(tInitStage eStage, __exception = NULL);
        tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);


    private:
		/*! creates all the input Pins*/
		tResult CreateInputPins(__exception = NULL);
		/*! creates all the output Pins*/
		tResult CreateOutputPins(__exception = NULL);

        /*! loads the given file, compiles the script and stores it in the script-structure */
        tResult LoadManeuverScripts(cFilename m_fileConfig);

    public: // since the thread is not in private access of the class, one can solve this issue better!!!
        /*! codes a signal-value (float) media-sample and sends it through the given output pin*/
        tResult SendSignalValueMessage(cObjectPtr<cOutputPin> pOutputPin, tFloat32 value, tUInt32 timeStamp);
        /*! codes a signal-bool (boolean) media-sample and sends it through the given output pin*/
        tResult SendBoolSignalValueMessage(cObjectPtr<cOutputPin> pOutputPin, bool value, tUInt32 timeStamp);
        /*! decodes a signal-value (float) media-sample with the given descriptor*/
        tResult ReceiveSignalValueMessage(IMediaSample* pMediaSample, cObjectPtr<IMediaTypeDescription> pDesc, tFloat32 *pValue, tUInt32 *pTimeStamp);
        /*! decodes a motion-data media-sample*/
        tResult ReceiveMotionDataMessage(IMediaSample* pMediaSample, tFloat32 *pDY, tFloat32 *pPhi, tUInt32 *pTimeStamp);
        /*! decodes a crossing-data media-sample*/
        tResult ReceiveCrossingDataMessage(IMediaSample* pMediaSample, tFloat32 *pDistToCrossing, tFloat32 *pOrientation, tInt8 *pCrossingType);
        /*! decodes a parking media-sample*/
        tResult ReceiveParkingMessage(IMediaSample* pMediaSample, tFloat32 *dist, tFloat32* angle, tInt32* lotID, tUInt32* pTimeStamp );
        /*! decodes a road-sign media-sample*/
        tResult ReceiveRoadSignMessage(IMediaSample *pMediaSample, tInt8 *pID, tFloat32 *pTransX, tFloat32 *pTransY, tFloat32 *pTransZ, tFloat32 *pRotX, tFloat32 *pRotY, tFloat32 *pRotZ);
        /*! returns the current time*/
        tTimeStamp GetTime();

        /*! draws the given polygon in the debug-output and fills it with the given colour*/
        void drawPoly(vector<Point2f> ppt, Vec3b color);
        /*! intersects the two vectors X+a*x and Y+b*y and returns the intersection point*/
        Point2f intersectLines(Point2f X, Point2f x, Point2f Y, Point2f y);

        /*! checks the oncoming-traffic at crossings */
        bool CheckFront(Point2f A, Point2f B, Point2f C);
        /*! checks the traffic coming from right at crossings */
        bool CheckRight(Point2f A, Point2f B, Point2f C);
        /*! checks the traffic coming from left at crossings */
        bool CheckLeft();
        /*! calculates the error value of dept-/obstacle-grid inside the given polygon */
        tResult CalcErrorValue(vector<Point2f> polypoints, tUInt8 refVal, tFloat32 *pErrValue, tInt32 mode = USE_BOTH);

    // Methods called by input pins:
        /*! the car controller requests a new maneuver */
        tResult RequestNewManeuver(tFloat32 newManeuver);
        /*! updates the locally stored orientation */
        tResult UpdateOrientation(tFloat32 dOrientation);
        /*! updates the locally stored driven-distance */
        tResult UpdateDrivenDist(tFloat32 dDist);
        /*! updates the locally stored crossing-information */
        tResult UpdateCrossingInfo(tFloat32 dDist, tFloat32 dOrientation, tInt8 iType);
        /*! updates the locally stored parking-information */
        tResult UpdateParkingDist(tFloat32 dDist, tFloat32 dAngle, tInt32 lotID);
        /*! updates the locally stored parking-lot-type */
        tResult UpdateParkingDirection(tFloat32 dDirection);
        /*! updates the locally stored nearest sign */
        tResult ProcessSignUpdate(IMediaSample* pMediaSample);

    // Methods called by thread (always in the cycle of LoadManeuver > ScriptMapper > NextOperation > LoadManeruver > ...:
        /*! if there is a next operation in the current maneuver-script execute this operation, otherwise report end of
         * aneuver-script to CarController */
        tResult NextOperation();
        /*! if there is a new maneuver requested, load the corresponding maneuver-script */
        tResult LoadManeuver();
        /*! detect the type of parking-lot the car is currently in*/
        tResult DetectParkingLotType();
        /*! maps the current script-operation to the corresponding filter-method */
        tResult ScriptMapper();

    // Methods representing the various script-operrations:
        /*! sets the steering angle*/
        tResult SetCurv(tFloat32 curv);
        /*! sets the maximum allowed speed*/
        tResult SetSpeed(tFloat32 speed);
        /*! sets the given indicator*/
        tResult SetIndicator(tFloat32 id, tFloat32 val);
        /*! stores the current orientation (used as base for wait-orientation)*/
        tResult InitOrientation(tFloat32 useCrossingOrientation);
        /*! wait until the car-orientation changed about the given value*/
        tResult WaitForOrientation(tFloat32 orientation);
        /*! wait until the distance to a object behind the car is inside dist+-delta*/
        tResult WaitForDistRear(tFloat32 dist, tFloat32 delta);
        /*! wait until the car drove the given distance*/
        tResult WaitForDistDriven(tFloat32 dist);
        /*! wait until the crossing is dist cm ahead*/
        tResult WaitForDistCrossing(tFloat32 dist, tFloat32 planB=300);
        /*! enable/disable lane-following mode*/
        tResult LaneFollowingEnable(tFloat32 value);
        /*! enable/disable high-speed mode*/
        tResult HighSpeedEnable(tFloat32 value);
        /*! set sensor strictness*/
        tResult SensorsEnable(tFloat32 value);
        /*! wait for the given duration (attention: active waiting!!)*/
        tResult WaitForTime(tFloat32 duration);
        /*! drives along the parking-lots and searches a free one*/
        tResult SearchParkingLot(tFloat32 dist, tFloat32 lotLength, tFloat32 lotWidth, tFloat32 parkingType);
        /*! unused method #1*/
        tResult ResetSearchParkingLot(tFloat32 speed);
        /*! corrects car-orientation after right-of-way check*/
        tResult EndRoW(tFloat32 curv, tFloat32 orientation, tFloat32 delta);
        /*! changes the car-orientation in a way that we can check the right-of-way as best as possible*/
        tResult PrepareRoW(tFloat32 curv, tFloat32 orientation, tFloat32 delta);
        /*! check the right-of-way, decodes the traffic rules and the crossing-type*/
        tResult CheckRightOfWay(tInt maneuverType, tFloat32 curv, tFloat32 orientation, tFloat32 delta, tFloat32 speed, tFloat32 crossingType);
        /*! waits untill the necessary crossing-parts are free */
        tResult WaitForTraffic(tBool careLeft, tBool careFront, tBool careRight);
        /*! waits untill the oncoming traffic is clear */
        tResult CheckOncomingTraffic();

    // Thread mutexes and variables:
        bool m_StopFilterRequested;
        pthread_mutex_t m_orientation_wait_mux;
        pthread_mutex_t m_dist_wait_mux;
        pthread_mutex_t m_crossing_wait_mux;
        pthread_mutex_t m_sign_wait_mux;
        pthread_mutex_t m_maneuver_wait_mux;
        pthread_mutex_t m_parking_wait_mux;
        pthread_mutex_t m_parking_direction_wait_mux;
        pthread_mutex_t m_grid_wait_mux;
        pthread_mutex_t m_orientation_mux;
        pthread_mutex_t m_dist_mux;
        pthread_mutex_t m_crossing_mux;
        pthread_mutex_t m_sign_mux;
        pthread_mutex_t m_grid_mux;
        pthread_mutex_t m_maneuver_mux;
        pthread_mutex_t m_parking_mux;
        pthread_mutex_t m_parking_direction_mux;

    // The thread:
        pthread_t m_Thread;
        tResult *m_ThreadReturn;

    // Locally stored information (some accessed via semaphores!):
        tUInt8 m_state;
        bool m_DebugEnable;

        Mat m_ObstacleGrid;
        Mat m_DepthGrid;
        Mat m_DebugGrid;
        tBitmapFormat	m_sBitmapFormatGrid;
        tBitmapFormat   m_sDebugFormatGrid;
        tInt32 m_gridScale;


        bool m_ManeuverFinished;
        bool m_ManeuverRequested;
        tFloat32 m_RequestedManeuverID;
        tInt32 m_CurrentRoW;

        vector<ManeuverScript*> m_Scripts;
        tInt m_ManeuverIdx;
        tInt m_ScriptIdx;

        tUInt8 m_CurrSignID;
        tInt m_SignDetectionCounter;
        tFloat32 m_DistToSign;

        tUInt8 m_CurrPD;
        tInt m_PDDetectionCounter;

        tInt m_ParkingType;
        tFloat32 m_LastCurv;
        tFloat32 m_LastSpeed;
        tFloat32 m_Orientation;
        tFloat32 m_DrivenDist;
        tFloat32 m_CrossingDist;
        tFloat32 m_CrossingOrientation;
        tInt8 m_CrossingType;

        tFloat32 m_ParkingDist;
        tFloat32 m_ParkingAngle;
        tInt32 m_ParkingLotID;


        Point2f m_K, m_kl, m_kr, m_IR_R, m_IR_L;


		/*! Coder Descriptor for the pins*/
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal_send;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal_receive_maneuver;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal_receive_yaw;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal_receive_pd;

        cObjectPtr<IMediaTypeDescription> m_pCoderDescMotionData;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescCrossing;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescBoolSignal;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescRoadSign;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescParking;

        tResult WaitForSign(tFloat32 dist);
        void drawCrossing2Debug(Point2f A, Point2f B, Point2f C);
        void sendDebugGrid();
};

void* Cycle(void *vp);


//*************************************************************************************************

#endif // _SEGMENTDRIVER_H_

