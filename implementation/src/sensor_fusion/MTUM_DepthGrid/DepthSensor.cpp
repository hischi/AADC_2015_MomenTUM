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

#include "DepthSensor.h"


DepthSensor::DepthSensor() : Sensor()
{

}

DepthSensor::~DepthSensor()
{
}

tResult DepthSensor::LoadConfigurationData(cFilename m_fileConfig,tBitmapFormat bitmapFormat)
{
    m_depthImageBase = *(new Mat(bitmapFormat.nHeight,1,CV_16UC1));

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
        cDOMElement* pConfigElement;


        if (IS_OK(oDOM.FindNode("sensor/ground_values", pConfigElement)))
        {
            cString str = cString(pConfigElement->GetData());

            cStringList values;
            str.Split(values, ',');
            if(values.GetItemCount() != m_depthImageBase.rows)
            {
                LOG_ERROR(cString::Format("ObstacleGrid - depthImage: file format missmatch; %d values loaded -> should be %d",values.GetItemCount(),m_depthImageBase.rows));
                RETURN_NOERROR;
            }

            for(int i = 0; i < values.GetItemCount(); i++)
            {
                m_depthImageBase.at<tUInt16>(i) = (tUInt16) values[i].AsUInt32();
            }
        }
    }
    else
    {
        LOG_ERROR("SensorCalibration: Configured configuration file for depth sensor not found (yet)");
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    Sensor::LoadConfigurationData(m_fileConfig);

    m_phiShift = m_delta - PI/2;
    m_phiColFact = m_delta*2/bitmapFormat.nWidth;

    RETURN_NOERROR;
}



tResult DepthSensor::ProcessDepthImage(Mat depthImage, Mat obstacleGrid, Mat depthGrid, tBool processDepth)
{
    tUInt16 tempMinValue = 65535;
    tFloat32 tempDistValue = 0;
    tFloat32 colPhi = 0;
    tFloat32 invalidColIndex = 0;
    tInt32 foundGroundCount = 0;


    for(int col = DEPTH_IGNORE_LEFT_SIDE;col < depthImage.cols-DEPTH_IGNORE_RIGHT_SIDE;col+=1)   //cols
    {
        //preset col dependent variables
        colPhi = (col-DEPTH_IGNORE_RIGHT_SIDE)*m_phiColFact - m_phiShift;
        tempMinValue = 65535;
        invalidColIndex = 0;
        //scan col for min
        for(int rowEle = (int)(DEPTH_IGNORE_ROWS_UP);rowEle < depthImage.rows - DEPTH_IGNORE_ROWS_DOWN;rowEle+=1) //rows from HEIGHTRATIO * rows to bottom
        {
            if(depthImage.at<tUInt8>(rowEle,col) < (tUInt8)DEPTH_IGNORE_THRESHOLD)
                invalidColIndex += pow(rowEle/100,2);
            else if(depthImage.at<tUInt8>(rowEle,col) < (m_depthImageBase.at<tUInt16>(rowEle) - (tUInt8)DEPTH_BASE_OFFSET) && //differ object from ground
                     depthImage.at<tUInt8>(rowEle,col) < tempMinValue)         //new min value
                tempMinValue = depthImage.at<tUInt8>(rowEle,col);
            if(depthImage.at<tUInt8>(rowEle,col) > (m_depthImageBase.at<tUInt16>(rowEle))) //determine ground pixels found
                foundGroundCount++;
        }

        //ignore line if invalid value is to high && not enough ground pixels have been found => black = close object
        if(invalidColIndex>INVALID_COLINDEX_MAX)// || foundGroundCount <= 10)
            continue;
    //------------finished scanning col for min----------------

        tempDistValue = (tFloat32)(((tFloat32)tempMinValue - 17021.0) * 0.00315604761845 + 50.0); //convert colorval to dist
        Point tempEndPoint = Point((cos(colPhi)*(tempDistValue+m_obstacleSize)+m_xPos)/GRID_CM_PER_UNIT+TILE_COUNT_RIGHT, (tempDistValue+m_obstacleSize+m_yPos)/GRID_CM_PER_UNIT+TILE_COUNT_REAR);

        LineIterator it(obstacleGrid,
                        Point(m_xPos/GRID_CM_PER_UNIT+TILE_COUNT_RIGHT,m_yPos/GRID_CM_PER_UNIT+TILE_COUNT_REAR),
                        Point(tempEndPoint));

        tFloat32 last = 0;
        int x = 0;
        for(; x < it.count; x++, ++it)
        {
            last = float(*(tUInt8*)*it);

            if(x < tempDistValue/GRID_CM_PER_UNIT){
                obstacleGrid.at<tUInt8>(it.pos()) = UpdateGridPoint(last,(tUInt8)(0.9*255));
            }else if(x < m_rangeMax/GRID_CM_PER_UNIT){
            obstacleGrid.at<tUInt8>(it.pos()) = UpdateGridPoint(last,(tUInt8)(0.1*255));
            }
        }
        line(depthImage, Point(0,depthImage.rows - DEPTH_IGNORE_ROWS_DOWN), Point(depthImage.cols,depthImage.rows - DEPTH_IGNORE_ROWS_DOWN), Scalar(255), 1);
        line(depthImage, Point(0,DEPTH_IGNORE_ROWS_UP), Point(depthImage.cols,DEPTH_IGNORE_ROWS_UP), Scalar(255), 1);

    }
    RETURN_NOERROR;
}
