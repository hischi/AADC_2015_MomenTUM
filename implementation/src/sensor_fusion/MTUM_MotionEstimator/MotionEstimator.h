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


/*! \brief Motion Estimator
 *  This filter calculates the car odometry, e.g. the movement (driven distance and orientation change) since the last
 *  update from this filter.
 */
#ifndef _MTUMMOTIONESTIMATOR_H_
#define _MTUMMOTIONESTIMATOR_H_

#include "momenTUM_const.h"

#define OID_ADTF_MTUMMOTIONESTIMATOREXT "adtf.aadc.mtum.MotionEstimator"
#define max(a,b) (a>b)?a:b;

/*!
 * This filter calculates the car odometry, e.g. the movement (driven distance and orientation change) since the last
 * update from this filter. This information is used by the ObstacleGrid, DepthGrid, SpeedRecommender, SegemntDriver, ...
 *
 * The calculation is based on a simple Kalman filter using both wheel-encoders, acceleration and orientation by the IMU
 * and the motor-control-value (labled: acceleration).
 *
 * Since the gyroscope takes some time to calibrate after starting the ADTF-configuartion, this filter will report if it
 * is ready to the CarController.
 */
class MotionEstimatorExt : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_MTUMMOTIONESTIMATOREXT, "MTUM Motion Estimator", OBJCAT_DataFilter, "Motion Estimator", 1, 0, 0, "<Beta Version");



    /*!input pin for the the rpm of the left wheel */
    cInputPin m_inputPin_RPM_Left;
    /*!input pin for the the rpm of the right wheel */
    cInputPin m_inputPin_RPM_Right;
    /*!input pin for the the vehicle yaw (z-rotation) */
    cInputPin m_inputPin_yaw;
    /*!input for the acceleration actuator */
    cInputPin m_InputPin_acc_act;
    /*!input for the current driving direction */
    cInputPin m_InputPin_dir;
    /*!input for the measured acceleration by IMU */
    cInputPin m_inputPin_Yacc;
    /*!input to reset the filter */
    cInputPin m_InputPin_reset;


    /*!output pin for the motion data  */
    cOutputPin m_outputPin_motion;
    cOutputPin m_outputPin_status;




    public:
        MotionEstimatorExt(const tChar* __info);
        virtual ~MotionEstimatorExt();

protected: // overwrites cFilter
        tResult Init(tInitStage eStage, __exception = NULL);
        tResult Start(__exception = NULL);
        tResult Stop(__exception = NULL);
        tResult Shutdown(tInitStage eStage, __exception = NULL);
        tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

        tResult SendMotionDataMessage(cObjectPtr<cOutputPin> pOutputPin, tFloat32 drivenDist, tFloat32 angelChange, tUInt32 timeStamp);
        tResult ReceiveSignalValueMessage(IMediaSample* pMediaSample, tFloat32 *pValue, tUInt32 *pTimeStamp);
        //tResult ProcessGridUpdate(tSensInfo *m_pSensData, tFloat32 m_sensValue);




        struct Pair{
            float xValue;
            float yValue;
        };
        vector<Pair> m_values;
private:
		/*! creates all the input Pins*/
		tResult CreateInputPins(__exception = NULL);
		/*! creates all the output Pins*/
		tResult CreateOutputPins(__exception = NULL);

        /*! loads a xml configuration file*/
        tResult LoadConfigurationData();
        /*! codes a signal-value (float) media-sample and sends it through the given output pin*/
        tResult SendSignalValueMessage(cObjectPtr<cOutputPin> pOutputPin, tFloat32 value, tUInt32 timeStamp);

        /*! checks if the car has already stopped*/
        void checkWaitStop(float v);

    // some helpers:
        void insertAccMean(tFloat32 value);
        void fillMat(Mat_<float> &measurement, adtf_util::cDOMElementRefList &oElems);
        void printMat(Mat_<float> measurement);
        float getAccMean();
        tTimeStamp GetTime()
        {
            return (_clock != NULL) ? _clock->GetTime(): cSystem::GetTime();
        }

    // mutexes to be thread-safe (different threads access the OnPinEvent method!)
        pthread_mutex_t m_dir_mux;
        pthread_mutex_t m_waitStop_mux;
        pthread_mutex_t m_reset_mux;

        //temp sensor data storage
        float m_lastl;
        float m_ldist;
        float m_lastr;
        float m_rdist;
        bool m_waitStop;
        tUInt m_standCounter;
        tUInt m_maxstandCounter;

        vector<float> m_lastaccs;

        tFloat32 m_yaw_mean_pre_pre;


        const tFloat32 m_ticks2Dist;

        bool m_switched_to_enabled;
        tFloat32 m_offset;

        tFloat32 m_accy;
        tFloat32 m_dy;
        int m_n;
        float m_M2;
        tFloat32 m_dir;

        tFloat32 m_yaw_mean;
        tFloat32 m_yaw_mean_pre;

        Mat_<float> m_estimated;

        tInt32 m_initEnableCounter;

        tUInt8 m_state;

		/*! Coder Descriptor for the pins*/
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalCAM;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalARDU;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalSCPT;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescMotion;

        // The Kalman filter

        tTimeStamp m_t;
        KalmanFilter* kalman;
        bool m_debug;
        bool m_dirPinActive;



};



//*************************************************************************************************

#endif // _MTUMMOTIONESTIMATOR_H_

