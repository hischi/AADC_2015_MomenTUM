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

/*! \brief Sensor
 *
 *  This class models a sensor using probabilistic characteristica and merges updates with values, which are already
 *  in the obstacle/depth grid.
 */
#ifndef SENSINFO_TYPES_H
#define SENSINFO_TYPES_H


#include "stdafx.h"
#include "cCubic.h"
#include "momenTUM_const.h"

/*!
 *  This class models a sensor using probabilistic characteristica and merges updates with values, which are already
 *  in the obstacle/depth grid. The characteristica can be loaded from a xml configuration file (selected in ADTF).
 *
 *  An update will fire the following operations:
 *      1. UpdateGrid:  Calculates the distance (and variance) from sensor to the detected obstacle (GetXValue). After
 *                      that it determines those pixels in the grid, which represents the obstalce (inc. its inaccurancy),
 *                      as a polygon.
 *
 *      2. UpdatePolygon: Calls for every pixel inside the given polygon the value in the grid
 *
 *      3. UpdateGridPoint: Merges old value and new pixel value, which depends on the accuracy of the sensor at the
 *                          detected distance, its range and our trust in the sensor-type.
 */
class Sensor
{
public:
    Sensor();
    ~Sensor();

    tResult LoadConfigurationData(cFilename m_fileConfig);

    tResult UpdateGrid(tFloat32 value, Mat pObstacleGrid);

    tResult GetXValues(tFloat32 yValue, vector<tFloat32> *pXVector, vector<tFloat32> *pVarVector);
    void UpdatePolygon(tFloat32 value, Mat obstacleGrid, vector<Point> ppt);
    void updateLine(tFloat32 value, Mat obstacleGrid, Point p1, Point p2);
    tFloat32 getRangeMin();
    tFloat32 getRangeMax();
    tInt32 getMinThreshold();


protected:

    tUInt8 UpdateGridPoint(tUInt8 last, tUInt8 newVal);

    vector<Cubic*> m_Interpolations; //list of measureable support points
    vector<Cubic*> m_Variances;      //list of variance of support points

    tInt32 m_xPos;       //distance from vehicle 0:0 to sensor position in cross direction
    tInt32 m_yPos;       //distance from vehicle 0:0 to sensor position in driving direction
    tFloat32 m_phi;      //orientation of the sensor in relation to driving direction 0
    tFloat32 m_delta;    //opening angle of the sensor
    tInt32 m_openMax;   //distance the sensor area changes from triangle to rectangularform
    tFloat32 m_rangeMin; //minimal range the sensor can detect objects at correct distance
    tFloat32 m_rangeMax; //maximal range the sensor can detect objects at correct distance
    tFloat32 m_yD;
    tFloat32 m_xD;
    tInt32 m_obstacleSize;
    tInt32 m_lineWidth;
    tInt32 m_minThreshold;
};



#endif // SENSINFO_TYPES_H







