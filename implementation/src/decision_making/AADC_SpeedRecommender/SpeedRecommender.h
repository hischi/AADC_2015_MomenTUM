/**
 * Audi Autonomous Driving Cup. ADTF filter sources.
 * Copyright (C) 2015 MomenTUM
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


/**********************************************************************
 * $Author:: MomenTUM $  $Date:: 2015-03-15 13:29:48#$ $Rev:: 26104  $*
 **********************************************************************/

/*! \brief Speed Recommender
 *
 *  Extracts max driving speed from obstacle grid, limitations set from outside; includes checking along curves using current steering feedback
 */


#ifndef _SPEEDRECOMMENDER_H_
#define _SPEEDRECOMMENDER_H_

#include "stdafx.h"
#include "math.h"
#include "../../sensor_fusion/AADC_ObstacleGrid/cCubic.h"
#include "momenTUM_const.h"
#include <pthread.h>

#define OID_MTUM_SPEEDRECOMMENDER "adtf.mtum.SpeedRecommender"

#define HIST_DEPTH 5
#define CAMERADIST2FRONT 20.0
#define OT_HIST_SIZE 5

/*!
 *  This filter extracts max. driving speed from obstacle grid and limitations set from outside. It includes checking
 *  along curves using current steering feedback. Additionally the filter checks if an overtaking-process should be
 *  started and maintains the rear lights (reverse light, brake light). There is no speed-control loop!
 */
class SpeedRecommender : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_MTUM_SPEEDRECOMMENDER, "MTUM Speed Recommender", OBJCAT_DataFilter, "Speed Recommender", 1, 0, 0, "<Beta Version");

    cInputPin m_inputPin_enable;        /*!< input pin to enable/disable/reset the filter >*/
    cInputPin m_inputPin_OTenable;      /*!< input pin to allow/forbid overtaking >*/
    cInputPin m_inputPin_HSenable;      /*!< input pin to enable/disable high-speed-mode >*/
    cInputPin m_inputPin_steering;      /*!< input pin for the current steering (read back from servo) >*/
    cInputPin m_inputPin_segmentSpeed;  /*!< input pin for the current speed limitation >*/
    cInputPin m_inputPin_laneInfo;      /*!< input pin for the current lane-information, currently unused >*/
    cInputPin m_inputPin_motionData;    /*!< input pin to enable/disable high-speed-mode >*/
    cInputPin m_inputPin_reducedSpeed;  /*!< input pin to reduce speed >*/
    cInputPin m_inputPin_parking;       /*!< input pin to enable/disable parking-mode >*/
    cInputPin m_inputPin_voltageAct;    /*!< input pin for the current driving-battery voltage >*/


    cVideoPin m_inputPin_ObstacleGrid;  /*!< input pin for the obstacle-grid >*/
    cVideoPin m_inputPin_DepthGrid;     /*!< input pin for the depth-grid >*/


    cOutputPin	m_outputPin_accelerate;         /*!< output pin for the speed >*/
    cOutputPin  m_outputPin_useOtherLane;       /*!< output pin for the lane-detection to calculate overtaking path >*/
    cOutputPin	m_outputPin_drivingDirection;   /*!< output pin for the current driving direction (used by MotionEstimator) >*/
    cOutputPin	m_outputPin_dist2Obst;          /*!< currently unused >*/
    cOutputPin  m_outputPin_brakeLight;         /*!< output pin for the brake lights >*/
    cOutputPin	m_outputPin_reverseLight;       /*!< output pin for the reverse lights >*/

    cOutputPin	m_outputPin_turn_left_enabled;  /*!< output pin for the left indicator >*/
    cOutputPin	m_outputPin_turn_right_enabled; /*!< output pin for the right indicator >*/

    cVideoPin m_outputPin_DebugGrid;            /*!< output pin for debug information >*/



    public:
        SpeedRecommender(const tChar* __info);
        virtual ~SpeedRecommender();
	
    protected: // overwrites cFilter
        tResult LoadConfigurationData(cFilename m_fileConfig);
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

        /*! sends a signal-value (float) media-sample through the given output pin using the given media-descriptor*/
        tResult SendSignalValueMessage(cObjectPtr<cOutputPin> pOutputPin,cObjectPtr<IMediaTypeDescription> mediaDesc, tFloat32 value, tUInt32 timeStamp);
        /*! sends a bool-value media-sample through the given output pin */
        tResult SendBoolSignalValueMessage(cObjectPtr<cOutputPin> pOutputPin, bool value, tUInt32 timeStamp);
        /*! decodes a signal-value (float) media-sample using the given media-descriptor*/
        tResult ReceiveSignalValueMessage(IMediaSample* pMediaSample,cObjectPtr<IMediaTypeDescription> mediaDesc, tFloat32 *pValue, tUInt32 *pTimeStamp);
        /*! decodes a motion-data media-sample*/
        tResult ReceiveMotionDataMessage(IMediaSample* pMediaSample, tFloat32 *pDY, tFloat32 *pDPhi,tFloat32 *pVel, tUInt32 *pTimeStamp);
        /*! decodes a lane-information media-sample*/
        tResult ReceiveLaneInfoMessage(IMediaSample* pMediaSample, tFloat32 *pDX, tFloat32 *pOrientation, tFloat32 *pCurvature, tUInt32 *pTimeStamp);

        /*! generates an arc consisting of trapezoidal polygons*/
        tResult GenPolyArc(Point2f center, tFloat32 radiusInner, tFloat32 radiusOuter, tFloat32 arcStartInner, tFloat32 arcStartOuter, tFloat32 arcEndInner, tFloat32 arcEndOuter, tInt sampleCount, vector<vector<Point2f> > &pts);
        /*! calculates the error value of obstacle- and/or depth-grid inside the given polygon*/
        tResult CalcErrorValue(vector<Point2f> ppts, tUInt8 refVal, tFloat32 *pErrValue, tInt32 mode = USE_BOTH);
        /*! calculates the allowed forward speed*/
        tFloat32 calcForwardSpeed();
        /*! calculates the allowed backwards speed*/
        tFloat32 calcBackwardsSpeed();
        /*! calculates the allowed parking speed*/
        tFloat32 calcParkingSpeed(tInt32 dir);
        /*! uses negative speed-values to brake more 'actively'*/
        tResult breaking(tFloat32* speed);

    // Helper:
        tBool checkDeadBlock();

        tResult TryStartOvertaking();
        tResult TryEndOvertaking();


        Mat m_obstacleGrid;
        Mat m_depthGrid;
        Mat m_debugGrid;

        tBitmapFormat	m_sBitmapFormatGrid;

        tFloat32 m_curvature;
        tInt32 m_otCounterStart;
        tFloat32 m_curvatureLaneDetect;
        tBool m_enabled;
        tBool m_OTenabled;
        tBool m_PDenabled;
        tBool m_HSenabled;
        tFloat32 m_highSpeedDist;
        tFloat32 m_afterParkingDist;
        tUInt8 m_intState;
        tUInt8 m_overtakingState;
        tFloat32 m_maxAccValue;
        tBool m_reduceSpeed;
        tBool m_parkingEnabled;
        tFloat32 m_bonusVoltageAcc;

        tFloat32 m_dist2RightIn;
        tFloat32 m_dist2ObstHist[OT_HIST_SIZE];
        tInt32 m_dist2ObstHistCounter;
        tFloat32 m_overtakingDY;

        tInt32 m_drivingDirection;
        tFloat32 m_dyBack;
        tInt32 m_obstBackwardsCounter;
        tFloat32 m_lastDist2Obst;
        tBool m_checkObstBack;
        tBool m_lanechangeReq;
        tBool m_overtakingRdy;
        tInt32 m_startOvertakingCounter;

        tFloat32 m_preSpeed;
        tFloat32 m_viewDist;

        tFloat32 m_distHist[HIST_DEPTH];
        tUInt32 m_distHistTime[HIST_DEPTH];
        tInt32 m_distHistCounter;
        tFloat32 m_currSpeedEst;
        tFloat32 m_backDistEst;

        tInt32 m_testSamplePointCount;
        Cubic* m_interpolation;
        tInt32 m_breakCounterForward;
        tInt32 m_breakCounterBack;
        tInt32 m_sendCounter;
        tFloat32 m_preBreakSpeed;
        tFloat32 m_postBreakSpeed;

        tFloat32 m_maxRangeScaling;
        tFloat32 m_testingRangeModMin;
        tFloat32 m_testingRangeModMax;

		/*! Coder Descriptor for the pins*/
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalOut;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalOutEnable;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescLaneInfo;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescMotion;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescBoolSignal;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalSteering;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalSegSpeed;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalLaneDet;

        /*! internal states*/
        enum intStates {IS_FORWARD=1,IS_BACKWARDS, IS_BACKOBST};
        /*! steps of overtaking*/
        enum overtakingStates {OS_NORMAL=1,OS_OVERTAKING,OS_START_OVERTAKING,OS_FINISH_OVERTAKING};
        /*! driving direction*/
        enum directions {FORWARD = 1, BACKWARD = -1};

		
};



//*************************************************************************************************

#endif // _SPEEDRECOMMENDER_H_

