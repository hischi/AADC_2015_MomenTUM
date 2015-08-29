/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2014-09-16 13:29:48#$ $Rev:: 26104   $
**********************************************************************/

/*! \brief Calibration Scaling
 *         
 *  Mit diesem Filter koennen Sensor- oder Aktorwerte linear skaliert werden. Dabei wird der Eingangswert  mit dem in den Eigenschaften gegebenen Wert multipliziert.
 */

#ifndef _SEGMENTPATH_H_
#define _SEGMENTPATH_H_

#include "momenTUM_const.h"

/*!
This filter can be used to scale data with a simple scale factor
*/


class SegmentPath
{
    public:
        SegmentPath();
        SegmentPath(tInt32 maneuver);
        ~SegmentPath();

        tResult SetManeuverType(tInt32 maneuver);
        tResult AddSegment(tFloat32 dist, tFloat32 curv);

        bool IsApplicable(tInt32 maneuver);

        bool StartPath(tFloat32 drivenDist, tFloat32 *pCurv);
        bool IteratePath(tFloat32 drivenDist, tFloat32 *pCurv);
	
	private:
        vector<tFloat32> m_DistVector;
        vector<tFloat32> m_CurvVector;

        tFloat32 m_StartDist;
        tInt32 m_CurrentSegment;

        tInt32 m_Maneuver;
		
};



//*************************************************************************************************

#endif // _SEGMENTPATH_H_

