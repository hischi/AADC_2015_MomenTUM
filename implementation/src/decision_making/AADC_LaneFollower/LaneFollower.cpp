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

#include "stdafx.h"
#include "LaneFollower.h"
#include <iostream>
#include "math.h"

ADTF_FILTER_PLUGIN("AADC Lane Follower", OID_ADTF_LANEFOLLOWER, LaneFollower)


LaneFollower::LaneFollower(const tChar* __info) : cFilter(__info)
{
    SetPropertyFloat("curvature fact",1.0);
    SetPropertyFloat("steering alpha pos",1);
    SetPropertyFloat("steering alpha orientation",1);

    m_steering = 0;
    m_lf_enabled = true;
    m_cd_enabled = false;
    m_ps_enabled = false;

}

LaneFollower::~LaneFollower()
{

}

tResult LaneFollower::CreateInputPins(__exception)
{
    RETURN_IF_FAILED(m_inputPin_LF_enable.Create("lf_enable", new cMediaType(0, 0, 0, "tSignalValue")  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_LF_enable));

    RETURN_IF_FAILED(m_inputPin_LaneInfo.Create("laneInfo", new cMediaType(0, 0, 0, "tLaneInfo")  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_LaneInfo));

    RETURN_IF_FAILED(m_inputPin_CD_enable.Create("cd_enable", new cMediaType(0, 0, 0, "tSignalValue")  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_CD_enable));

    RETURN_IF_FAILED(m_inputPin_PS_enable.Create("ps_enable", new cMediaType(0, 0, 0, "tSignalValue")  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_PS_enable));

    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    tChar const * strDesc = pDescManager->GetMediaDescription("tCrossingInfo");
    RETURN_IF_POINTER_NULL(strDesc);
    cObjectPtr<IMediaType> pType = new cMediaType(0, 0, 0, "tCrossingInfo", strDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescCrossingInfo));

    RETURN_IF_FAILED(m_inputPin_CrossingInfo.Create("CrossingInfo", pType , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_CrossingInfo));

	RETURN_NOERROR;
}

tResult LaneFollower::CreateOutputPins(__exception)
{ 
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal));
    pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal_cd));

    tChar const * strDescLaneInfoValue = pDescManager->GetMediaDescription("tLaneInfo");
    RETURN_IF_POINTER_NULL(strDescLaneInfoValue);
    cObjectPtr<IMediaType> pTypeLaneInfoValue = new cMediaType(0, 0, 0, "tLaneInfo", strDescLaneInfoValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeLaneInfoValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescLaneInfo));

    RETURN_IF_FAILED(m_outputPin_steer_angle.Create("steer_angle", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_steer_angle));


    RETURN_NOERROR;
}

tResult LaneFollower::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
	{
		CreateInputPins(__exception_ptr);
		CreateOutputPins(__exception_ptr);
	}
    else if (eStage == StageNormal)
    {
        m_lf_enabled = true;
        m_cd_enabled = false;
        m_ps_enabled = false;
    }

    RETURN_NOERROR;
}

tResult LaneFollower::Start(__exception)
{
	return cFilter::Start(__exception_ptr);
}

tResult LaneFollower::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult LaneFollower::Shutdown(tInitStage eStage, __exception)
{
	return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult LaneFollower::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    RETURN_IF_POINTER_NULL(pMediaSample);

    if(pSource == &m_inputPin_LaneInfo)
    {
        cObjectPtr<IMediaCoder> pCoderInput;
        RETURN_IF_FAILED(m_pCoderDescLaneInfo->Lock(pMediaSample, &pCoderInput));

        //write values with zero
        tFloat32 curvature = 0;
        tFloat32 lanePos = 0;
        tFloat32 carOrientation = 0;
        tUInt32 timeStamp = 0;

        //get values from media sample
        pCoderInput->Get("f32Curvature", (tVoid*)&curvature);
        pCoderInput->Get("f32LanePos", (tVoid*)&lanePos);
        pCoderInput->Get("f32CarOrientation", (tVoid*)&carOrientation);
        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
        m_pCoderDescLaneInfo->Unlock(pCoderInput);

       // LOG_INFO(cString::Format("LaneFollow: received c=%f  x=%f  phi=%f",curvature,lanePos,carOrientation));


        if(m_lf_enabled)
        {
            //FLORIANS CONTROLLER


            double curvature_factor = 1.0;
           // double angle_factor = 0.5;
            // double distance_factor = 1.0;
            double angle_factor = 0.4;
            double distance_factor = 1.2;

            //steering based on curvature
            m_steering = curvature_factor*(90.0 - (atan2(1,curvature * CARSIZE_AXISDIST) /(2 * PI) * 360.0));

            //m_steering = 0;

            //correction if not close to lane center
            //m_steering += m_steeringAlphaPos * (lanePos - (tFloat32)LANE_WIDTH/2) - m_steeringAlphaAngle * carOrientation /(2 * PI) * 360.0;
            if(m_ps_enabled)
                m_steering += distance_factor * (lanePos + 5 - (tFloat32)LANE_WIDTH/2.0) - angle_factor * carOrientation /(2 * PI) * 360.0;
            else
                m_steering += distance_factor * (lanePos - (tFloat32)LANE_WIDTH/2.0) - angle_factor * carOrientation /(2 * PI) * 360.0;

            tFloat32 valueOut = m_steering;

            tFloat32 timeStampOut = 0;


            //create new media sample
            cObjectPtr<IMediaSample> pMediaSampleOut;
            AllocMediaSample((tVoid**)&pMediaSampleOut);

            //allocate memory with the size given by the descriptor
            cObjectPtr<IMediaSerializer> pSerializer;
            m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
            tInt nSize = pSerializer->GetDeserializedSize();
            pMediaSampleOut->AllocBuffer(nSize);

            //write date to the media sample with the coder of the descriptor
            cObjectPtr<IMediaCoder> pCoderOutput = NULL;
            m_pCoderDescSignal->WriteLock(pMediaSampleOut, &pCoderOutput);

            pCoderOutput->Set("f32Value", (tVoid*)&(valueOut));
            pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&(timeStampOut));
            m_pCoderDescSignal->Unlock(pCoderOutput);

            //transmit media sample over output pin
            tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
            pMediaSampleOut->SetTime(tmStreamTime);
            m_outputPin_steer_angle.Transmit(pMediaSampleOut);
        }
    }
    if(pSource == &m_inputPin_LF_enable)
    {cObjectPtr<IMediaCoder> pCoderInput;
        RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoderInput));

        //write values with zero
        tFloat32 value = 0;
        tUInt32 timeStamp = 0;

        //get values from media sample
        pCoderInput->Get("f32Value", (tVoid*)&value);
        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
        m_pCoderDescSignal->Unlock(pCoderInput);

        m_lf_enabled = (value == 1);
    }
    if(pSource == &m_inputPin_CD_enable)
    {cObjectPtr<IMediaCoder> pCoderInput;
        RETURN_IF_FAILED(m_pCoderDescSignal_cd->Lock(pMediaSample, &pCoderInput));

        //write values with zero
        tFloat32 value = 0;
        tUInt32 timeStamp = 0;

        //get values from media sample
        pCoderInput->Get("f32Value", (tVoid*)&value);
        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
        m_pCoderDescSignal_cd->Unlock(pCoderInput);

        m_cd_enabled = (value == 1);
    }
    if(pSource == &m_inputPin_PS_enable)
    {cObjectPtr<IMediaCoder> pCoderInput;
        RETURN_IF_FAILED(m_pCoderDescSignal_cd->Lock(pMediaSample, &pCoderInput));

        //write values with zero
        tFloat32 value = 0;
        tUInt32 timeStamp = 0;

        //get values from media sample
        pCoderInput->Get("f32Value", (tVoid*)&value);
        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
        m_pCoderDescSignal_cd->Unlock(pCoderInput);

        m_ps_enabled = (value == 1);
    }
    if(pSource == &m_inputPin_CrossingInfo)
    {
        tFloat32 dist;
        tInt8 type;

        tFloat32 lanePos = 0;
        tFloat32 carOrientation = 0;

        ReceiveCrossingDataMessage(pMediaSample, &dist, &carOrientation, &type);

       // LOG_INFO(cString::Format("LaneFollow: received c=%f  x=%f  phi=%f",curvature,lanePos,carOrientation));


        if(m_cd_enabled)
        {
            //FLORIANS CONTROLLER
            cout << "LaneFollower crossing: " << carOrientation << endl;


           // double angle_factor = 0.5;
            // double distance_factor = 1.0;
            double angle_factor = 1;
            double distance_factor = 0;

            //steering based on curvature
            m_steering = 0;//curvature_factor*(90.0 - (atan2(1,curvature * CARSIZE_AXISDIST) /(2 * PI) * 360.0));


            //m_steering = 0;

            //correction if not close to lane center
            //m_steering += m_steeringAlphaPos * (lanePos - (tFloat32)LANE_WIDTH/2) - m_steeringAlphaAngle * carOrientation /(2 * PI) * 360.0;
            m_steering += distance_factor * (lanePos - (tFloat32)LANE_WIDTH/2.0) - angle_factor * carOrientation /(2 * PI) * 360.0;

            tFloat32 valueOut = m_steering;

            tFloat32 timeStampOut = 0;


            //create new media sample
            cObjectPtr<IMediaSample> pMediaSampleOut;
            AllocMediaSample((tVoid**)&pMediaSampleOut);

            //allocate memory with the size given by the descriptor
            cObjectPtr<IMediaSerializer> pSerializer;
            m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
            tInt nSize = pSerializer->GetDeserializedSize();
            pMediaSampleOut->AllocBuffer(nSize);

            //write date to the media sample with the coder of the descriptor
            cObjectPtr<IMediaCoder> pCoderOutput = NULL;
            m_pCoderDescSignal->WriteLock(pMediaSampleOut, &pCoderOutput);

            pCoderOutput->Set("f32Value", (tVoid*)&(valueOut));
            pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&(timeStampOut));
            m_pCoderDescSignal->Unlock(pCoderOutput);

            //transmit media sample over output pin
            tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
            pMediaSampleOut->SetTime(tmStreamTime);
            m_outputPin_steer_angle.Transmit(pMediaSampleOut);
        }
    }

	RETURN_NOERROR;
}

tResult LaneFollower::ReceiveCrossingDataMessage(IMediaSample* pMediaSample, tFloat32 *pDistToCrossing, tFloat32 *pOrientation, tInt8 *pCrossingType)
{
    // read-out the incoming Media Sample
    cObjectPtr<IMediaCoder> pCoderInput;
    RETURN_IF_FAILED(m_pCoderDescCrossingInfo->Lock(pMediaSample, &pCoderInput));

    *pDistToCrossing = 1234;
    *pCrossingType = 0;
    *pOrientation = 0;
    tUInt32 ts = 0;

    //get values from media sample
    pCoderInput->Get("i8Type", (tVoid*) pCrossingType);
    pCoderInput->Get("f32Distance", (tVoid*) pDistToCrossing);
    pCoderInput->Get("f32Orientation", (tVoid*) pOrientation);
    pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&ts);
    m_pCoderDescCrossingInfo->Unlock(pCoderInput);

    RETURN_NOERROR;
}
