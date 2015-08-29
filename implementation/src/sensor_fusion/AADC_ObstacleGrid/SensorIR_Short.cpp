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

#include "SensorIR_Short.h"


SensorIR_Short::SensorIR_Short() : Sensor()
{
}

SensorIR_Short::~SensorIR_Short()
{
}

tResult SensorIR_Short::UpdateGrid(tFloat32 value, Mat obstacleGrid)
{
    vector<tFloat32> XVector, VarVector;
    GetXValues(value, &XVector, &VarVector);
    tFloat32 carXBase = m_xPos/GRID_CM_PER_UNIT+TILE_COUNT_RIGHT;
    tFloat32 carYBase = m_yPos/GRID_CM_PER_UNIT+TILE_COUNT_REAR;
    if((int)XVector.size() < 1)
        RETURN_NOERROR;
    tFloat32 xVal = MIN(XVector[0],m_rangeMax);
    Point endPoint;
    Point endPointObst;

    endPoint = Point((cos(m_phi)*xVal + m_xPos)/GRID_CM_PER_UNIT+TILE_COUNT_RIGHT, (sin(m_phi)*xVal+m_yPos)/GRID_CM_PER_UNIT+TILE_COUNT_REAR);
    endPointObst = endPoint + Point(cos(m_phi)* m_obstacleSize/GRID_CM_PER_UNIT,sin(m_phi)*m_obstacleSize/GRID_CM_PER_UNIT);//Point(endPoint.x + cos(m_phi)*8/GRID_CM_PER_UNIT,)
    updateLine(0.9,obstacleGrid,Point(carXBase, carYBase),endPoint);
    if(xVal < m_rangeMax-m_obstacleSize)
    {
        updateLine(0.1,obstacleGrid,endPoint,endPointObst);
    }

    RETURN_NOERROR;
}
