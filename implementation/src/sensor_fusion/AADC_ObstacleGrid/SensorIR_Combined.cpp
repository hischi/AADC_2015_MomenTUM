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

#include "SensorIR_Combined.h"


SensorIR_Combined::SensorIR_Combined() : Sensor(), m_SensorLong()
{
    m_ShortArrived = false;
    m_LongArrived = false;
}

SensorIR_Combined::~SensorIR_Combined()
{
}

tResult SensorIR_Combined::LoadConfigurationData(cFilename m_fileConfig_Short, cFilename m_fileConfig_Long)
{
    Sensor::LoadConfigurationData(m_fileConfig_Short);
    m_SensorLong.LoadConfigurationData(m_fileConfig_Long);

    RETURN_NOERROR;
}

tResult SensorIR_Combined::UpdateGrid(tFloat32 value, Mat obstacleGrid, bool shortRange)
{
    if(shortRange)
    {
        m_LastShortValue = value;
        m_ShortArrived = true;
    }
    else
    {
        m_LastLongValue = value;
        m_LongArrived = true;
    }

    if(m_ShortArrived && m_LongArrived)
    {
        m_ShortArrived = false;
        m_LongArrived = false;

        vector<tFloat32> XShortVector,XLongVector, VarVector;
        GetXValues(m_LastShortValue, &XShortVector, &VarVector);
        m_SensorLong.GetXValues(m_LastLongValue, &XLongVector, &VarVector);

        tFloat32 carXBase = m_xPos/GRID_CM_PER_UNIT+TILE_COUNT_RIGHT;
        tFloat32 carYBase = m_yPos/GRID_CM_PER_UNIT+TILE_COUNT_REAR;

        Point endPoint;
        Point endPointObst;
        tFloat32 xLMax;
        tFloat32 xLMin;

        if((int)(XShortVector.size()) < 1 || (int)(XLongVector.size()) < 1)
        {
            LOG_INFO(cString::Format("SENSOR DATA LOOKUP ERROR - to few data long: %d short: %d",(int)(XShortVector.size()),(int)(XLongVector.size())));
            RETURN_NOERROR;
        }
        if((int)(XLongVector.size()) == 2)
        {
            //LOG_INFO(cString::Format("in  l : %f  ins %f  xs : %f  xl1: %f  xl2: %f",m_LastLongValue,m_LastShortValue,XShortVector[0],XLongVector[0],XLongVector[1]));
            xLMax = MAX(XLongVector[0],XLongVector[1]);
            xLMin = MIN(0,MIN(XLongVector[0],XLongVector[1]));
        }
        else
        {
            //LOG_INFO(cString::Format("in  l : %f  ins %f  xs : %f  xl1: %f  ",m_LastLongValue,m_LastShortValue,XShortVector[0],XLongVector[0]));
            xLMax = XLongVector[0];
            xLMin = MIN(XLongVector[0],0);
        }

        if(m_LastShortValue > m_LastLongValue && m_LastShortValue > m_minThreshold && (int)XLongVector.size() > 1)
        { //vorne
            //LOG_INFO(cString::Format("vorne - xLMin"));
            endPoint = Point((cos(m_phi)*xLMin + m_xPos)/GRID_CM_PER_UNIT+TILE_COUNT_RIGHT, (sin(m_phi)*xLMin+m_yPos)/GRID_CM_PER_UNIT+TILE_COUNT_REAR);
        }
        else if(XShortVector[0] < m_rangeMax && m_LastLongValue > m_SensorLong.getMinThreshold())
        { //mitte
            //LOG_INFO(cString::Format("mitte - xS"));
            endPoint = Point((cos(m_phi)*XShortVector[0] + m_xPos)/GRID_CM_PER_UNIT+TILE_COUNT_RIGHT, (sin(m_phi)*XShortVector[0]+m_yPos)/GRID_CM_PER_UNIT+TILE_COUNT_REAR);
        }
        else if(xLMax < m_SensorLong.getRangeMax())
        { //hinten
            //LOG_INFO(cString::Format("vorne - xLMax"));
            endPoint = Point((cos(m_phi)*xLMax + m_xPos)/GRID_CM_PER_UNIT+TILE_COUNT_RIGHT, (sin(m_phi)*xLMax+m_yPos)/GRID_CM_PER_UNIT+TILE_COUNT_REAR);
        }
        else
        {
            //LOG_INFO(cString::Format("out of range"));
            endPoint = Point((cos(m_phi)*m_SensorLong.getRangeMax() + m_xPos)/GRID_CM_PER_UNIT+TILE_COUNT_RIGHT, (sin(m_phi)*m_SensorLong.getRangeMax()+m_yPos)/GRID_CM_PER_UNIT+TILE_COUNT_REAR);
        }

        endPointObst = endPoint + Point(cos(m_phi)* m_obstacleSize/GRID_CM_PER_UNIT,sin(m_phi)*m_obstacleSize/GRID_CM_PER_UNIT);
        updateLine(0.75,obstacleGrid,Point(carXBase, carYBase),endPoint);
        updateLine(0.1,obstacleGrid,endPoint,endPointObst);
    }

    RETURN_NOERROR;
}
