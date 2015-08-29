/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: forchhe#$  $Date:: 2014-09-16 09:45:24#$ $Rev:: 26079   $
**********************************************************************/

#include "stdafx.h"
#include "userAttitudeConverter.h"


ADTF_FILTER_PLUGIN("AADC User Attitude Converter", OID_ADTF_USER_ATTITUDE_FILTER, cUserAttitudeFilter)

cUserAttitudeFilter::cUserAttitudeFilter(const tChar* __info) : cFilter(__info)
{

	m_bDebugModeEnabled = tFalse;
	SetPropertyBool("DebugOutput",              m_bDebugModeEnabled);
	
	/*! total of 24 different euler systems. We support only one, yet. */
	m_EulerAnglesType = 0;
	SetPropertyInt("EulerAngles", m_EulerAnglesType);
	SetPropertyStr("EulerAngles" NSSUBPROP_VALUELISTNOEDIT, "0@YawPitchRoll");
	SetPropertyInt("EulerAngles"  NSSUBPROP_REQUIRED, tTrue);

	qX = .0;
	qY = .0;
	qZ = .0;
	qW = .0;
}

cUserAttitudeFilter::~cUserAttitudeFilter()
{
}


//****************************
tResult cUserAttitudeFilter::CreateInputPins(__exception)
{
    RETURN_IF_FAILED(m_oInput_qW.Create("quaternion_gyro_w", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_oInput_qW));

	RETURN_IF_FAILED(m_oInput_qX.Create("quaternion_gyro_x", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_oInput_qX));

	RETURN_IF_FAILED(m_oInput_qY.Create("quaternion_gyro_y", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_oInput_qY));

	RETURN_IF_FAILED(m_oInput_qZ.Create("quaternion_gyro_z", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_oInput_qZ));

	 RETURN_NOERROR;
}

//*****************************************************************************

tResult cUserAttitudeFilter::CreateOutputPins(__exception)
{
	cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
    
	tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);        
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);	
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal)); 
    
	RETURN_IF_FAILED(m_oOutputYaw.Create("yaw", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_oOutputYaw));

	RETURN_IF_FAILED(m_oOutputPitch.Create("pitch", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_oOutputPitch));

	RETURN_IF_FAILED(m_oOutputRoll.Create("roll", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_oOutputRoll));


    RETURN_NOERROR;
}

tResult cUserAttitudeFilter::TransmitAngles(tTimeStamp sampleTimeStamp, const tTimeStamp timeStampValue, const tFloat32 yaw, const tFloat32 pitch, const tFloat32 roll)
{	
	//create new media sample
    cObjectPtr<IMediaSample> pMediaSampleYaw;
    cObjectPtr<IMediaSample> pMediaSamplePitch;
    cObjectPtr<IMediaSample> pMediaSampleRoll;
    AllocMediaSample((tVoid**)&pMediaSampleYaw);
    AllocMediaSample((tVoid**)&pMediaSamplePitch);
    AllocMediaSample((tVoid**)&pMediaSampleRoll);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleYaw->AllocBuffer(nSize);
    pMediaSamplePitch->AllocBuffer(nSize);
    pMediaSampleRoll->AllocBuffer(nSize);
    
    cObjectPtr<IMediaCoder> pCoderOutput;
       
    //write date to the media sample with the coder of the descriptor
    m_pCoderDescSignal->WriteLock(pMediaSampleYaw, &pCoderOutput);	
    pCoderOutput->Set("f32Value", (tVoid*)&(yaw));	
    pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStampValue);    
    m_pCoderDescSignal->Unlock(pCoderOutput);
    //transmit media sample over output pin
    pMediaSampleYaw->SetTime(sampleTimeStamp);
    m_oOutputYaw.Transmit(pMediaSampleYaw);
    
    //write date to the media sample with the coder of the descriptor
    m_pCoderDescSignal->WriteLock(pMediaSamplePitch, &pCoderOutput);	
    pCoderOutput->Set("f32Value", (tVoid*)&(pitch));	
    pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStampValue);    
    m_pCoderDescSignal->Unlock(pCoderOutput);
    //transmit media sample over output pin
    pMediaSamplePitch->SetTime(sampleTimeStamp);
    m_oOutputPitch.Transmit(pMediaSamplePitch);
    
    //write date to the media sample with the coder of the descriptor
    m_pCoderDescSignal->WriteLock(pMediaSampleRoll, &pCoderOutput);	
    pCoderOutput->Set("f32Value", (tVoid*)&(roll));	
    pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStampValue);    
    m_pCoderDescSignal->Unlock(pCoderOutput);
    //transmit media sample over output pin
    pMediaSampleRoll->SetTime(sampleTimeStamp);
    m_oOutputRoll.Transmit(pMediaSampleRoll);

	RETURN_NOERROR;
}



//*****************************************************************************

tResult cUserAttitudeFilter::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
    {
		CreateInputPins(__exception_ptr);
		CreateOutputPins(__exception_ptr);
    }
    else if (eStage == StageNormal)
    {
		m_EulerAnglesType = GetPropertyInt("EulerAngles");
	}
		
    RETURN_NOERROR;
}

tResult cUserAttitudeFilter::Start(__exception)
{
	return cFilter::Start(__exception_ptr);
}

tResult cUserAttitudeFilter::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult cUserAttitudeFilter::Shutdown(tInitStage eStage, __exception)
{
	return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cUserAttitudeFilter::calulateEulerAngles(tFloat32 &yaw_out, tFloat32 &pitch_out, tFloat32 &roll_out){
		switch(m_EulerAnglesType){
		    /*! total of 24 different euler systems. We support only one, yet. */
			/*! For details please see "Graphic Germs IV", Ken Shoemake, 1993 */
			default:
                pitch_out = atan2(2*(qW*qX + qY*qZ),1 - 2*(qX*qX + qY*qY));
				roll_out = asin(2*(qW*qY - qZ*qX));
				yaw_out = atan2(2*(qW*qZ + qX*qY),1 - 2*(qY*qY + qZ*qZ));	
                //LOG_INFO(cString::Format("roll: %10.4f   pitch: %10.4f   yaw:%10.4f",roll_out,pitch_out,yaw_out));
//original		
                //roll_out = atan2(2*qY*qW - 2*qX*qZ, 1 - 2*qY*qY - 2*qZ*qZ);
				//pitch_out = atan2(2*qX*qW - 2*qY*qZ, 1 - 2*qX*qX - 2*qZ*qZ);
				//yaw_out = asin(2*qX*qY + 2*qZ*qW); 
				break;
		}
		
		RETURN_NOERROR;
}

tResult cUserAttitudeFilter::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
	
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pMediaSample != NULL && m_pCoderDescSignal != NULL)
    {

		RETURN_IF_POINTER_NULL( pMediaSample);	
        tUInt32 timeStamp = 0;
		if (pSource == &m_oInput_qW)
		    {	
			    // read-out the incoming Media Sample
                cObjectPtr<IMediaCoder> pCoderInput;
                RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoderInput));
           
                //get values from media sample        
                pCoderInput->Get("f32Value", (tVoid*)&qW);
                pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
                m_pCoderDescSignal->Unlock(pCoderInput);       
		    }
		else if (pSource == &m_oInput_qX)
		    {
                // read-out the incoming Media Sample
                cObjectPtr<IMediaCoder> pCoderInput;
                RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoderInput));
           
                //get values from media sample        
                pCoderInput->Get("f32Value", (tVoid*)&qX);
                pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
                m_pCoderDescSignal->Unlock(pCoderInput);       
		     }
		else if (pSource == &m_oInput_qY)
			{
                // read-out the incoming Media Sample
                cObjectPtr<IMediaCoder> pCoderInput;
                RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoderInput));
           
                //get values from media sample        
                pCoderInput->Get("f32Value", (tVoid*)&qY);
                pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
                m_pCoderDescSignal->Unlock(pCoderInput);       
		    }
        else if (pSource == &m_oInput_qZ)
			{
                // read-out the incoming Media Sample
                cObjectPtr<IMediaCoder> pCoderInput;
                RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoderInput));
           
                //get values from media sample        
                pCoderInput->Get("f32Value", (tVoid*)&qZ);
                pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
                m_pCoderDescSignal->Unlock(pCoderInput);
                
                tFloat32 roll, pitch, yaw;
				
                calulateEulerAngles(yaw,pitch,roll);
				
				TransmitAngles(pMediaSample->GetTime(),timeStamp,yaw,pitch,roll);      
		    }		    
	}
    RETURN_NOERROR;
}
