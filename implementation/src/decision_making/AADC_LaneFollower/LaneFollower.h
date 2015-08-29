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

/*! \brief LaneFollower
 *         
 *  This filter represents the steering control during lane-following, parking-lot-detection and crossing-approaching.
 */

#ifndef _LANEFOLLOWER_H_
#define _LANEFOLLOWER_H_

#include "momenTUM_const.h"

#define OID_ADTF_LANEFOLLOWER "adtf.aadc.LaneFollower"

/*!
 * This filter represents the steering control during lane-following, parking-lot-detection and crossing-approaching.
 *
 * The filter-mode can be changed by sending a 1.0 to the corresponding input pin:
 *    - LF_enable for lane-following,
 *    - PS_enable for parking-lot-detection mode and
 *    - CD_enable for crossing-approaching mode.
 * All other input values on this ports will deactivate the corresponding mode. If no mode is selected, the filter will
 * not produce any outputs (required for path-following, where the steering angle is set by a different filter).
 *
 * The filter reacts in each mode as following:
 *
 * Lane-following:
 * The filter takes the lane-information (lane-curvature, lane-position, car-orientation) from the LaneInfo pin.
 * It controls the car orientation and distance to the lane-markings to keep the car in the middle of the lane. Please
 * note that this control is without steering-feedback. Thus, higher speeds will destabilize the lane-following!
 *
 * Parking-lot-detection mode:
 * During parking lot detection the behaviour is the same as during lane-following with the exception that the car
 * tries to keep as right as possible in the lane. That way we generate a repeatable starting situation for parking.
 *
 * Crossing-approaching:
 * Approx. 0.5 meter in front of a crossing the lane-following is disabled. The filter takes during crossing-approaching
 * the crossing-information (crossing-type, distance to crossing, car-crossing-orientation) from the CrossingInfo pin.
 * Aim of this control is to generate repeatable starting conditions for turning (same orientation, distance, ...).
 * The control is done like during lane-following but focuses more on the car-orientation and distance to the right lane
 * marking than on curvature.
 *
 */


class LaneFollower : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_LANEFOLLOWER, "AADC Lane Follower", OBJCAT_DataFilter, "Lane Follower", 1, 0, 0, "<Beta Version");

    cInputPin m_inputPin_LF_enable;     /*!< input pin from SegmentDriver  */
    cInputPin m_inputPin_LaneInfo;      /*!< input pin from LaneDetection  */
    cInputPin m_inputPin_CrossingInfo;  /*!< input pin from CrossingDetection  */
    cInputPin m_inputPin_CD_enable;     /*!< input pin from SegmentDriver  */
    cInputPin m_inputPin_PS_enable;     /*!< input pin from SegmentDriver  */

    cOutputPin	m_outputPin_steer_angle;/*!< output pin to BaseConfig  */



    public:
        LaneFollower(const tChar* __info);
        virtual ~LaneFollower();
	
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

        /*! decodes a crossing-data media-sample*/
        tResult ReceiveCrossingDataMessage(IMediaSample* pMediaSample, tFloat32 *pDistToCrossing, tFloat32 *pOrientation, tInt8 *pCrossingType);



    tFloat32 m_steering;


        tBool m_lf_enabled;
        tBool m_cd_enabled;
        tBool m_ps_enabled;

		/*! Coder Descriptor for the pins*/
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal_cd;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescCrossingInfo;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescLaneInfo;
		
};



//*************************************************************************************************

#endif // _LANEFOLLOWER_H_

