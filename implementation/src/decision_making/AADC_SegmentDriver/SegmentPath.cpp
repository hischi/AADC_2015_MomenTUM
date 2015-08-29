/**
Copyright (c) 
Audi Autonomous_Driving Cup. All rights reserved.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous_Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: forchhe#$  $Date:: 2014-09-11 10:10:54#$ $Rev:: 25921   $
**********************************************************************/

// arduinofilter.cpp : Definiert die exportierten Funktionen für die DLL-Anwendung.
//
#include "stdafx.h"

#include "math.h"
#include "SegmentPath.h"

SegmentPath::SegmentPath()
{
    m_Maneuver = 0;
    m_CurrentSegment = -1;
}

SegmentPath::SegmentPath(tInt32 maneuver)
{
    m_Maneuver = maneuver;
}

SegmentPath::~SegmentPath()
{

}
tResult SegmentPath::SetManeuverType(tInt32 maneuver)
{
    m_Maneuver = maneuver;
}

tResult SegmentPath::AddSegment(tFloat32 dist, tFloat32 curv)
{
    m_DistVector.push_back(dist);
    m_CurvVector.push_back(curv);

    RETURN_NOERROR;
}

bool SegmentPath::StartPath(tFloat32 drivenDist, tFloat32 *pCurv)
{
    LOG_INFO("Start Pfad");


    m_CurrentSegment = 0;
    m_StartDist = drivenDist;

    if(m_DistVector.empty())
    {
        LOG_INFO("Pfad ist leer");

        m_CurrentSegment = -1;
        return true;
    }

    *pCurv = m_CurvVector[m_CurrentSegment];
    return false;
}

bool SegmentPath::IteratePath(tFloat32 drivenDist, tFloat32 *pCurv)
{
    if(m_CurrentSegment < 0)
        return true;

    if(m_StartDist+m_DistVector[m_CurrentSegment] > drivenDist)
    {
        *pCurv = m_CurvVector[m_CurrentSegment];
        return false;
    }

    m_StartDist = m_StartDist+m_DistVector[m_CurrentSegment];
    m_CurrentSegment++;

    if(m_CurrentSegment >= m_DistVector.size())
    {
        m_CurrentSegment = -1;
        return true;
    }

    *pCurv = m_CurvVector[m_CurrentSegment];
    return false;
}

bool SegmentPath::IsApplicable(tInt32 maneuver)
{
    return (m_Maneuver == maneuver);
}
