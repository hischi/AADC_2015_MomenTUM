/**
Copyright (c)
Audi Autonomous Driving Cup. Team MomenTUM . All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 **/

/**********************************************************************
 * $Author:: MomenTUM $  $Date:: 2015-01-15 13:29:48#$ $Rev:: 26104  $*
 **********************************************************************/

/*! \brief Pylon Detection
 *         
 *  Detects the pylons used for the free demonstration.
 */

#ifndef _pylonDetection_H_
#define _pylonDetection_H_

#include "stdafx.h"
#include "momenTUM_const.h"
#include "PylonDetector.h"

#define OID_MTUM_PYLON_DETECTION "adtf.mtum.pylonDetection"

struct pylonData{
//    int type;
    double distance;
    double angle;
};

class pylonDetection : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_MTUM_PYLON_DETECTION, "MTUM Pylon Detection", OBJCAT_DataFilter, "Pylon Detection", 1, 0, 0, "<Beta Version");

        cVideoPin  m_inputPin_colorImage;
        cVideoPin  m_inputPin_depthImage;
        cInputPin  m_inputPin_motion;
        cInputPin  m_inputPin_signLocation;

        //Calibration Input Pins
        cInputPin  m_inputPin_0;
        cInputPin  m_inputPin_1;
        cInputPin  m_inputPin_2;
        cInputPin  m_inputPin_3;
        cInputPin  m_inputPin_4;
        cInputPin  m_inputPin_5;
        
        cVideoPin  m_outputPin_depthImage;
        cVideoPin  m_outputPin_debugImage;
        cOutputPin m_outputPin_PylonInfo;
        cOutputPin m_oLaneDataOutputPin;
        cOutputPin	m_outputPin_speed;

    public:
        pylonDetection(const tChar* __info);
        virtual ~pylonDetection();
	
    protected: // overwrites cFilter
        tResult Init(tInitStage eStage, __exception = NULL);
        tResult Start(__exception = NULL);
        tResult Stop(__exception = NULL);
        tResult Shutdown(tInitStage eStage, __exception = NULL);        
        tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);
	private:
        tResult sendPylonInfo(float *distArray, float *angleArray);
		/*! creates all the input Pins*/
		tResult CreateInputPins(__exception = NULL);
		/*! creates all the output Pins*/
		tResult CreateOutputPins(__exception = NULL);
        /*! creates all the filter propertys*/

        tResult ReceiveSignalValueMessage(IMediaSample* pMediaSample, tFloat32 *pValue, tUInt32 *pTimeStamp);

        tResult ReceiveRoadSign(IMediaSample* pMediaSample, tInt8 *pId, Mat *pTvec, Mat* pRvec, tUInt32 *pTimeStamp);
        tResult ProcessSign(IMediaSample* pSample);
        tResult LoadConfigurationColorData(cFilename m_fileConfig);
        tResult LoadConfigurationCameraData(cFilename m_fileConfig);

        Mat m_depthImage;
        Mat m_debugImage;

        tBitmapFormat	m_sBitmapFormatColorIn;
        tBitmapFormat	m_sBitmapFormatDepthIn;
        tBitmapFormat	m_sBitmapFormatDebugOut;
        tBitmapFormat	m_sBitmapFormatDepthOut;

		/*! Coder Descriptor for the pins*/
        cObjectPtr<IMediaTypeDescription> m_pCoderDescMotion;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescLaneInfo;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescPylonInfo;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSign;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;

        PylonDetector pDetector;
        tResult SendLaneInfo(tFloat32 curvature, tFloat32 lanePos, tFloat32 carOrientation, tUInt32 timeStamp);
        tResult SendSignalValueMessage(cObjectPtr<cOutputPin> pOutputPin, cObjectPtr<IMediaTypeDescription> mediaDesc, tFloat32 value, tUInt32 timeStamp);
        double getPylonDist(Rect pylonRect);


        Point2d targetPylon;
        int pylonLost;
        int sign;
        bool turn_around;
        bool is_turning;
        bool flip_sign;

        int   type_sign;

        double car_angle;
        double m_dY;

        bool waitForInit;

        clock_t lastSeen;

        Scalar orange_from;
        Scalar orange_to;
        int ohl,ohh,osl,osh,ovl,ovh;
        tInt debugImageSelection;

        float m_fu,m_cx;

        int speed_driving;
        int speed_turning;
        float curvature_gain;
        float angular_gain;

        int pylonDetectionCounter;
        int signDetectionCounter;
        int frameCounter;
};

#endif // _pylonDetection_H_

