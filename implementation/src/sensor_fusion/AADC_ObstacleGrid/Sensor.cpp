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

#include "Sensor.h"

Sensor::Sensor()
{
    m_xPos = 0;
    m_yPos = 0;
    m_phi = 0;
    m_delta = 0;
    m_openMax = 0;
    m_rangeMin = 0;
    m_rangeMax = 200;
    m_obstacleSize = 16;
    m_lineWidth = 1;
    m_minThreshold = 250;
}


Sensor::~Sensor()
{
    for(int i = 0; i < (int)m_Interpolations.size(); i++)
    {
        if(m_Interpolations.at(i) != NULL)
        {
            delete(m_Interpolations.at(i));
        }
    }
    m_Interpolations.clear();

    for(int i = 0; i < (int)m_Variances.size(); i++)
    {
        if(m_Variances.at(i) != NULL)
        {
            delete(m_Variances.at(i));
        }
    }
    m_Variances.clear();
}

tResult Sensor::LoadConfigurationData(cFilename m_fileConfig)
{
    if (m_fileConfig.IsEmpty())
    {
        LOG_ERROR("Sensor: Configured configuration file not found");
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    //Load file, parse configuration, print the data
    if (cFileSystem::Exists(m_fileConfig))
    {
        cDOM oDOM;
        oDOM.Load(m_fileConfig);
        cDOMElementRefList oInterpolations;
        cDOMElementRefList oElems;
        cDOMElement* pConfigElement;


        if (IS_OK(oDOM.FindNode("sensor/xPos", pConfigElement)))
            m_xPos = int(cString(pConfigElement->GetData()).AsInt());

        if (IS_OK(oDOM.FindNode("sensor/yPos", pConfigElement)))
            m_yPos = int(cString(pConfigElement->GetData()).AsInt());

        if (IS_OK(oDOM.FindNode("sensor/phi", pConfigElement)))
            m_phi = float(cString(pConfigElement->GetData()).AsFloat64());
            m_phi = (m_phi+90)/360*2*PI;

        if (IS_OK(oDOM.FindNode("sensor/delta", pConfigElement)))
        {
            m_delta = float(cString(pConfigElement->GetData()).AsFloat64());
            m_delta = m_delta/2;
        }

        if (IS_OK(oDOM.FindNode("sensor/open_max", pConfigElement)))
            m_openMax = int(cString(pConfigElement->GetData()).AsInt());

        if (IS_OK(oDOM.FindNode("sensor/range_max", pConfigElement)))
            m_rangeMax = int(cString(pConfigElement->GetData()).AsInt());

        if (IS_OK(oDOM.FindNode("sensor/range_min", pConfigElement)))
            m_rangeMin = int(cString(pConfigElement->GetData()).AsInt());

        if (IS_OK(oDOM.FindNode("sensor/obstSize", pConfigElement)))
            m_obstacleSize = int(cString(pConfigElement->GetData()).AsInt());

        if (IS_OK(oDOM.FindNode("sensor/lineWidth", pConfigElement)))
            m_lineWidth = int(cString(pConfigElement->GetData()).AsInt());

        if (IS_OK(oDOM.FindNode("sensor/minThreshold", pConfigElement)))
            m_minThreshold = int(cString(pConfigElement->GetData()).AsInt());

        if(IS_OK(oDOM.FindNodes("sensor/interpolation", oInterpolations)))
        {

            for (cDOMElementRefList::iterator itInterpolations = oInterpolations.begin(); itInterpolations != oInterpolations.end(); ++itInterpolations)
            {
                float yStart=0, yStop=10000;
                vector<tFloat32> xValues, yValues, yVarValues;

                if (IS_OK((*itInterpolations)->FindNode("yStart", pConfigElement)))
                {
                    yStart = (tFloat32)(cString(pConfigElement->GetData()).AsFloat64());
                }
                if (IS_OK((*itInterpolations)->FindNode("yStop", pConfigElement)))
                {
                    yStop = (tFloat32)(cString(pConfigElement->GetData()).AsFloat64());
                }


                if(IS_OK((*itInterpolations)->FindNodes("point", oElems)))
                {
                    for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem)
                    {

                        pConfigElement = NULL;

                        if (IS_OK((*itElem)->FindNode("xValue", pConfigElement)))
                        {
                            xValues.push_back(tFloat32(cString(pConfigElement->GetData()).AsFloat64()));
                        }
                        if (IS_OK((*itElem)->FindNode("yValue", pConfigElement)))
                        {
                            yValues.push_back(tFloat32(cString(pConfigElement->GetData()).AsFloat64()));
                        }
                        if (IS_OK((*itElem)->FindNode("yVarValue", pConfigElement)))
                        {
                            yVarValues.push_back(tFloat32(cString(pConfigElement->GetData()).AsFloat64()));
                        }
                    }
                }

                if(xValues.size() > 0 && xValues.size() == yValues.size() && yValues.size() == yVarValues.size())
                {
                    if(yValues[0] > yValues[1])
                    {
                        tInt mid = xValues.size()/2;
                        tInt length = xValues.size() - 1;
                        for(int i = 0; i < mid;i++)
                        {
                            tFloat32 help = xValues[i];
                            xValues[i] = xValues[length-i];
                            xValues[length-i] = help;

                            help = yValues[i];
                            yValues[i] = yValues[length-i];
                            yValues[length-i] = help;

                            help = yVarValues[i];
                            yVarValues[i] = yVarValues[length-i];
                            yVarValues[length-i] = help;
                        }
                    }

                    Cubic *pCubic = new Cubic(yValues.size(), yValues, xValues, yStart, yStop);
                    Cubic *pVarCubic = new Cubic(yValues.size(), yValues, yVarValues, yStart, yStop);

                    m_Interpolations.push_back(pCubic);
                    m_Variances.push_back(pVarCubic);
                }
                else
                {
                    LOG_ERROR("Sensor: no supporting points in given file found!");
                    RETURN_ERROR(ERR_INVALID_FILE);
                }
            }
        }

        // Calculate helper variables xE / yE:
        float xA, yA, xE, yE;

        yE = sin(m_phi) * m_openMax;
        xE = cos(m_phi) * m_openMax;
        xA = (xE/tan(m_phi) + yE) / (tan(m_phi-m_delta)+1/tan(m_phi));
        yA = xA * tan(m_phi-m_delta);

        m_xD = xA - xE;
        m_yD = yA - yE;
    }
    else
    {
        LOG_ERROR("SensorCalibration: Configured configuration file not found (yet)");
        RETURN_ERROR(ERR_INVALID_FILE);
    }
    RETURN_NOERROR;
}

tFloat32 Sensor::getRangeMin()
{
    return m_rangeMin;
}

tFloat32 Sensor::getRangeMax()
{
    return m_rangeMax;
}

tInt32 Sensor::getMinThreshold()
{
    return m_minThreshold;
}

tResult Sensor::GetXValues(tFloat32 yValue, vector<tFloat32> *pXVector, vector<tFloat32> *pVarVector)
{
    for(tUInt i = 0; i < m_Interpolations.size(); i++)
    {
        Cubic *pCubicInterpolation = m_Interpolations[i];
        Cubic *pCubicVariance = m_Variances[i];

        if(!pCubicInterpolation->isInRange(yValue))
            continue;

        tFloat32 xValue = pCubicInterpolation->getValue(yValue);
        tFloat32 varValue = pCubicVariance->getValue(yValue);

        if(xValue < m_rangeMin)
            xValue = 0;
        if(xValue > m_rangeMax)
            xValue = m_rangeMax+1;
        if(varValue < 0)
            varValue = 0;

        pXVector->push_back(xValue);
        pVarVector->push_back(varValue);
    }

    RETURN_NOERROR;
}

void Sensor::UpdatePolygon(tFloat32 value, Mat obstacleGrid, vector<Point> ppt)
{
    tInt xMax = 0;
    tInt xMin = 100000;
    tInt yMax = 0;
    tInt yMin = 100000;

    for(int i = 0; i < (int)ppt.size(); i++)
    {
        xMin = MIN(ppt[i].x, xMin);
        yMin = MIN(ppt[i].y, yMin);
        xMax = MAX(ppt[i].x, xMax);
        yMax = MAX(ppt[i].y, yMax);
    }
    xMin = MAX(xMin,0);
    yMin = MAX(yMin,0);

    xMax = MIN(xMax,obstacleGrid.cols-1);
    yMax = MIN(yMax,obstacleGrid.rows-1);

    for(int x = xMin; x <= xMax; x++)
    {
        for(int y = yMin; y <= yMax; y++)
        {
            if(pointPolygonTest(ppt, Point(x,y),false) >= 0)
            {
                tUInt8 last = obstacleGrid.at<tUInt8>(Point(x,y));
                obstacleGrid.at<tUInt8>(Point(x,y)) = UpdateGridPoint(last,(tUInt8)(value*255));
            }
        }
    }
}

void Sensor::updateLine(tFloat32 value, Mat obstacleGrid, Point p1, Point p2)
{
    LineIterator it(obstacleGrid, p1,p2);
    int x = 0;
    for(; x < it.count; x++, ++it)
    {
        tUInt8 last = *(tUInt8*)*it;
        for(int i = -(m_lineWidth/GRID_CM_PER_UNIT/2);i <= m_lineWidth/GRID_CM_PER_UNIT/2;i++)
        {
            Point pShift(abs(sin(m_phi))*i,abs(cos(m_phi))*i);
            obstacleGrid.at<tUInt8>(it.pos() + pShift) = UpdateGridPoint(last,(tUInt8)(value*255));
        }
    }
}

tResult Sensor::UpdateGrid(tFloat32 value, Mat obstacleGrid)
{
    Point basePoint,linePoint1, linePoint2, linePoint3, linePoint4,linePoint5,linePoint6,linePointO1,linePointO2;

    vector<tFloat32> XVector, VarVector;

    GetXValues(value, &XVector, &VarVector);

    for(int j = 0; j < (int)(XVector.size()); j++)
    {

        tFloat32 x = XVector[j];
        tFloat32 yL,yLO;
        tFloat32 xL,xLO;
        tFloat32 xC;
        tFloat32 xCO;

        basePoint = Point((m_xPos)/GRID_CM_PER_UNIT+TILE_COUNT_RIGHT, (m_yPos)/GRID_CM_PER_UNIT+TILE_COUNT_REAR);

        if(x < (m_rangeMin+1))
            x = m_rangeMin*0.8;

        if(x <= m_openMax)
        {
            yL = sin(m_phi) * x;
            xL = cos(m_phi) * x;
            yLO = sin(m_phi) * (x+m_obstacleSize);
            xLO = cos(m_phi) * (x+m_obstacleSize);
            xC = tan(m_delta) * x;
            if(x+m_obstacleSize<=m_openMax)
                xCO = tan(m_delta) * (x+m_obstacleSize);
            else
                xCO = tan(m_delta) * m_openMax;
        }
        else
        {
            yL = sin(m_phi) * m_openMax;
            xL = cos(m_phi) * m_openMax;
            xC = tan(m_delta) * m_openMax;
            xCO = xC;
        }

        tFloat32 xA = cos(PI/2-m_phi)*xC;
        tFloat32 yA = sin(PI/2-m_phi)*xC;
        tFloat32 xAO = cos(PI/2-m_phi)*xCO;
        tFloat32 yAO = sin(PI/2-m_phi)*xCO;

        linePoint1 = Point((m_xPos + xL + xA)/GRID_CM_PER_UNIT + TILE_COUNT_RIGHT,(m_yPos + yL - yA)/GRID_CM_PER_UNIT + TILE_COUNT_REAR);
        linePoint2 = Point((m_xPos + xL - xA)/GRID_CM_PER_UNIT + TILE_COUNT_RIGHT,(m_yPos + yL + yA)/GRID_CM_PER_UNIT + TILE_COUNT_REAR);

        if(x <= m_openMax)
        {
            linePoint3 = linePoint1;
            linePoint4 = linePoint2;
        }
        else
        {
            yL = sin(m_phi) * x;
            xL = cos(m_phi) * x;
            yLO = sin(m_phi) * (x+m_obstacleSize);
            xLO = cos(m_phi) * (x+m_obstacleSize);

            xA = cos(PI/2-m_phi)*xC;
            yA = sin(PI/2-m_phi)*xC;

            linePoint3 = Point((m_xPos + xL + xA)/GRID_CM_PER_UNIT + TILE_COUNT_RIGHT,(m_yPos + yL - yA)/GRID_CM_PER_UNIT + TILE_COUNT_REAR);
            linePoint4 = Point((m_xPos + xL - xA)/GRID_CM_PER_UNIT + TILE_COUNT_RIGHT,(m_yPos + yL + yA)/GRID_CM_PER_UNIT + TILE_COUNT_REAR);
        }

        linePoint5 = Point((m_xPos + xLO + xAO)/GRID_CM_PER_UNIT + TILE_COUNT_RIGHT,(m_yPos + yLO - yAO)/GRID_CM_PER_UNIT + TILE_COUNT_REAR);
        linePoint6 = Point((m_xPos + xLO - xAO)/GRID_CM_PER_UNIT + TILE_COUNT_RIGHT,(m_yPos + yLO + yAO)/GRID_CM_PER_UNIT + TILE_COUNT_REAR);

        vector<Point> polygon;

        if(x > (m_rangeMin*1.15))
        {
            polygon.clear();
            polygon.push_back(basePoint);
            polygon.push_back(linePoint1);
            polygon.push_back(linePoint3);
            polygon.push_back(linePoint4);
            polygon.push_back(linePoint2);

            UpdatePolygon(0.7, obstacleGrid, polygon);
        }

        if(x <= m_rangeMax)
        {
            polygon.clear();
            polygon.push_back(linePoint3);
            polygon.push_back(linePoint5);
            polygon.push_back(linePoint6);
            polygon.push_back(linePoint4);

            UpdatePolygon(0.1, obstacleGrid, polygon);
        }
    }
    RETURN_NOERROR;
}

tUInt8 Sensor::UpdateGridPoint(tUInt8 last, tUInt8 newVal)
{
    tFloat32 new_prop = (tFloat32)last + (tFloat32)(newVal - 128)*0.75;
    new_prop = MIN(new_prop, 255);
    new_prop = MAX(new_prop, 0);
    return new_prop;
}
