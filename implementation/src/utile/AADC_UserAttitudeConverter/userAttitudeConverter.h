/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Extension by team MomenTUM:
We changed the calculation of the euler angles:
 	pitch_out = atan2(2*(qW*qX + qY*qZ),1 - 2*(qX*qX + qY*qY));
	roll_out = asin(2*(qW*qY - qZ*qX));
	yaw_out = atan2(2*(qW*qZ + qX*qY),1 - 2*(qY*qY + qZ*qZ));

originally:
	//roll_out = atan2(2*qY*qW - 2*qX*qZ, 1 - 2*qY*qY - 2*qZ*qZ);
	//pitch_out = atan2(2*qX*qW - 2*qY*qZ, 1 - 2*qX*qX - 2*qZ*qZ);
	//yaw_out = asin(2*qX*qY + 2*qZ*qW);

All other parts of this source code should be the same as provided by Audi!


**********************************************************************
* $Author:: spiesra $  $Date:: 2014-09-16 13:29:48#$ $Rev:: 26104   $
**********************************************************************/


#ifndef _USER_ATTITUDE_FILTER_H_
#define _USER_ATTITUDE_FILTER_H_

#define OID_ADTF_USER_ATTITUDE_FILTER "adtf.aadc.userAttitudeFilter"

using namespace adtf;
/*!
* This is filter converts the attitude from the quaternions system to the euler angles
*/
class cUserAttitudeFilter : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_USER_ATTITUDE_FILTER, "AADC User Attitude Converter", OBJCAT_DataFilter, "User Attitude Converter", 1, 0, 0, "Beta Version");
	
    public:
        cUserAttitudeFilter(const tChar* __info);
        virtual ~cUserAttitudeFilter();
	
    protected: // overwrites cFilter
        tResult Init(tInitStage eStage, __exception = NULL);
        tResult Start(__exception = NULL);
        tResult Stop(__exception = NULL);
        tResult Shutdown(tInitStage eStage, __exception = NULL);
        tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
	
	private:
	    tResult CreateOutputPins(__exception = NULL);
		tResult CreateInputPins(__exception = NULL);
		tFloat32 qW;
		tFloat32 qX;
		tFloat32 qY;
		tFloat32 qZ;
		tResult calulateEulerAngles(tFloat32 &yaw_out, tFloat32 &pitch_out, tFloat32 &roll_out); 

		tResult TransmitAngles(tTimeStamp sampleTimeStamp, const tTimeStamp timeStampValue, const tFloat32 yaw, const tFloat32 pitch, const tFloat32 roll);

		cInputPin m_oInput_qW;
		cInputPin m_oInput_qX;
		cInputPin m_oInput_qY;
		cInputPin m_oInput_qZ;
		cOutputPin m_oOutputPitch;
		cOutputPin m_oOutputYaw;
		cOutputPin m_oOutputRoll;
		tInt m_EulerAnglesType; 
		
		tBool m_bDebugModeEnabled;
		
		/*! Coder Descriptor for the pins*/
	    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
};



//*************************************************************************************************

#endif // _USER_ATTITUDE_FILTER_H_
