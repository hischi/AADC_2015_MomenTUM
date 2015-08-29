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
// SpeedRecommender.cpp : Determins the maximum drivable speed and handles slight breaking support
//                        Recommends Lane Change to lane detection if obstacle is avoidable and contraflow is free
//

#include "SpeedRecommender.h"

ADTF_FILTER_PLUGIN("MTUM Speed Recommender", OID_MTUM_SPEEDRECOMMENDER, SpeedRecommender)

SpeedRecommender::SpeedRecommender(const tChar* __info) : cFilter(__info)
{
    SetPropertyStr("Configurationfile","/home/odroid/AADC/calibration_files/calibrationAccIn.xml");
    SetPropertyBool("Configurationfile" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Configurationfile" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");

    SetPropertyInt("Initial State", 1);
    SetPropertyStr("Initial State" NSSUBPROP_VALUELISTNOEDIT, "1@IDLE|3@LANEFOLLOWING");

    m_curvature = 0;
    m_curvatureLaneDetect = 0;
    m_reduceSpeed = false;
    m_enabled = false;
    m_OTenabled = false;
    m_PDenabled = false;
    m_HSenabled = false;
    m_highSpeedDist = 0;
    m_afterParkingDist = 0;
    m_intState = IS_FORWARD;
    m_overtakingState = OS_NORMAL;
    m_otCounterStart = 0;
    m_dyBack = 0;
    m_maxAccValue = 0;
    m_distHistCounter = 0;
    m_obstBackwardsCounter = 0;
    m_drivingDirection = 1;
    m_checkObstBack = false;
    m_lanechangeReq = false;
    m_parkingEnabled = false;
    m_lastDist2Obst = 9999;
    m_viewDist = 9999;
    m_sendCounter = 0;

    m_dist2RightIn = LANE_WIDTH/2;
    for(int i = 0; i< OT_HIST_SIZE;i++)
    {
        m_dist2ObstHist[i] = 9999;
    }
    m_dist2ObstHistCounter = 0;
    m_startOvertakingCounter = 0;


    m_testingRangeModMin = 0.7;
    m_testingRangeModMax = 1.8;
    m_maxRangeScaling = 1.195;

    m_testSamplePointCount = TEST_SAMPLE_POINT_COUNT;


    m_breakCounterForward = 0;
    m_breakCounterBack = 0;
    m_postBreakSpeed = 0;
    m_preBreakSpeed = 0;
    m_preSpeed = 0;
}

SpeedRecommender::~SpeedRecommender()
{

}

tResult SpeedRecommender::CreateInputPins(__exception)
{
    RETURN_IF_FAILED(m_inputPin_enable.Create("SR_enable", new cMediaType(0, 0, 0, "tSignalValue")  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_enable));

    RETURN_IF_FAILED(m_inputPin_OTenable.Create("OT_enable", new cMediaType(0, 0, 0, "tSignalValue")  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_OTenable));

    RETURN_IF_FAILED(m_inputPin_HSenable.Create("HS_enable", new cMediaType(0, 0, 0, "tSignalValue")  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_HSenable));

    RETURN_IF_FAILED(m_inputPin_segmentSpeed.Create("segmentSpeed", new cMediaType(0, 0, 0, "tSignalValue")  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_segmentSpeed));

    RETURN_IF_FAILED(m_inputPin_parking.Create("parkingEnable", new cMediaType(0, 0, 0, "tSignalValue")  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_parking));

    RETURN_IF_FAILED(m_inputPin_reducedSpeed.Create("reducedSpeed", new cMediaType(0, 0, 0, "tSignalValue")  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_reducedSpeed));

    RETURN_IF_FAILED(m_inputPin_steering.Create("steering", new cMediaType(0, 0, 0, "tSignalValue")  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_steering));

    RETURN_IF_FAILED(m_inputPin_voltageAct.Create("voltage_power", new cMediaType(0, 0, 0, "tSignalValue")  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_voltageAct));

    RETURN_IF_FAILED(m_inputPin_motionData.Create("motionData", new cMediaType(0, 0, 0, "tMotionDataExt")  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_motionData));

    RETURN_IF_FAILED(m_inputPin_ObstacleGrid.Create("obstacle_grid", adtf::IPin::PD_Input  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_ObstacleGrid));

    RETURN_IF_FAILED(m_inputPin_DepthGrid.Create("depth_grid", adtf::IPin::PD_Input  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_DepthGrid));

    RETURN_NOERROR;
}

tResult SpeedRecommender::CreateOutputPins(__exception)
{ 
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    tChar const * strDescSignalValueOutEnable = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValueOutEnable);
    cObjectPtr<IMediaType> pTypeSignalValueOutEnable = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValueOutEnable,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValueOutEnable->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalOutEnable));

    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal));

    tChar const * strDescSignalValueSTeer = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValueSTeer);
    cObjectPtr<IMediaType> pTypeSignalValueSteer = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValueSTeer,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValueSteer->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalSteering));

    tChar const * strDescSignalValueLane = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValueLane);
    cObjectPtr<IMediaType> pTypeSignalValueLane = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValueLane,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValueLane->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalLaneDet));

    tChar const * strDescSignalValueSegS = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValueSegS);
    cObjectPtr<IMediaType> pTypeSignalValueSegS = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValueSegS,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValueSegS->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalSegSpeed));

    tChar const * strDescSignalValueOut = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValueOut);
    cObjectPtr<IMediaType> pTypeSignalValueOut = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValueOut,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValueOut->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalOut));

    tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(strDescBoolSignalValue);
    cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescBoolSignal));

    tChar const * strDescMotionDataValue = pDescManager->GetMediaDescription("tMotionDataExt");
    RETURN_IF_POINTER_NULL(strDescMotionDataValue);
    cObjectPtr<IMediaType> pTypeMotionDataValue = new cMediaType(0, 0, 0, "tMotionDataExt", strDescMotionDataValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeMotionDataValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescMotion));

    tChar const * strDescLaneInfo = pDescManager->GetMediaDescription("tLaneInfo");
    RETURN_IF_POINTER_NULL(strDescLaneInfo);
    cObjectPtr<IMediaType> pTypeLaneInfo = new cMediaType(0, 0, 0, "tLaneInfo", strDescLaneInfo,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeLaneInfo->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescLaneInfo));


    RETURN_IF_FAILED(m_outputPin_accelerate.Create("accelerate", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_accelerate));

    RETURN_IF_FAILED(m_outputPin_drivingDirection.Create("divingDirection", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_drivingDirection));

    RETURN_IF_FAILED(m_outputPin_useOtherLane.Create("useOtherLane", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_useOtherLane));

//    RETURN_IF_FAILED(m_outputPin_dist2Obst.Create("dist2Obstacle", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
//    RETURN_IF_FAILED(RegisterPin(&m_outputPin_dist2Obst));

    RETURN_IF_FAILED(m_outputPin_brakeLight.Create("brakeLight", pTypeBoolSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_brakeLight));

    RETURN_IF_FAILED(m_outputPin_turn_left_enabled.Create("blinker_left", pTypeBoolSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_turn_left_enabled));
    RETURN_IF_FAILED(m_outputPin_turn_right_enabled.Create("blinker_right", pTypeBoolSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_turn_right_enabled));

    RETURN_IF_FAILED(m_outputPin_reverseLight.Create("reverseLight", pTypeBoolSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_reverseLight));

#ifdef DEBUG
    RETURN_IF_FAILED(m_outputPin_DebugGrid.Create("debug_obstacle_grid", adtf::IPin::PD_Output  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_DebugGrid));
#endif

    RETURN_NOERROR;
}

tResult SpeedRecommender::LoadConfigurationData(cFilename m_fileConfig)
{
    if (m_fileConfig.IsEmpty())
    {
        LOG_ERROR("SR: Configured configuration file not found");
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

        if(IS_OK(oDOM.FindNodes("acc/points", oInterpolations)))
        {
            LOG_INFO("-------");

            for (cDOMElementRefList::iterator itInterpolations = oInterpolations.begin(); itInterpolations != oInterpolations.end(); ++itInterpolations)
            {
                vector<tFloat32> accValues, speed;

                if(IS_OK((*itInterpolations)->FindNodes("point", oElems)))
                {
                    for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem)
                    {

                        pConfigElement = NULL;

                        if (IS_OK((*itElem)->FindNode("accValue", pConfigElement)))
                        {
                            accValues.push_back(tFloat32(cString(pConfigElement->GetData()).AsFloat64()));
                        }
                        if (IS_OK((*itElem)->FindNode("speed", pConfigElement)))
                        {
                            speed.push_back(tFloat32(cString(pConfigElement->GetData()).AsFloat64()));
                        }
                    }
                }

                if(accValues.size() > 0 && accValues.size() == speed.size())
                {
                    m_interpolation = new Cubic(speed.size(), speed, accValues, 0, 100);
                }
                else
                {
                    LOG_ERROR("SR: no supporting points in given file found!");
                    RETURN_ERROR(ERR_INVALID_FILE);
                }
            }
        }

    }
    else
    {
        LOG_ERROR("SpeedRecommender: Configured configuration file not found (yet)");
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    RETURN_NOERROR;
}

tResult SpeedRecommender::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

            if (eStage == StageFirst)
    {
        CreateInputPins(__exception_ptr);
        CreateOutputPins(__exception_ptr);
    }
    else if (eStage == StageNormal)
    {
        LoadConfigurationData(GetPropertyStr("Configurationfile"));
        const tBitmapFormat* pSourceFormat = m_inputPin_ObstacleGrid.GetFormat();
        if (pSourceFormat != NULL)
        {
            memcpy(&m_sBitmapFormatGrid, pSourceFormat, sizeof(tBitmapFormat));
        }

        if (m_sBitmapFormatGrid.nWidth == 0 ||
                m_sBitmapFormatGrid.nHeight == 0)
        {
            m_sBitmapFormatGrid.nWidth = TILE_COUNT_LEFT+TILE_COUNT_RIGHT;
            m_sBitmapFormatGrid.nHeight = TILE_COUNT_FRONT+TILE_COUNT_REAR;
            m_sBitmapFormatGrid.nBitsPerPixel = 8;
            m_sBitmapFormatGrid.nPixelFormat = cImage::PF_GREYSCALE_8;
            m_sBitmapFormatGrid.nBytesPerLine = m_sBitmapFormatGrid.nWidth;
            m_sBitmapFormatGrid.nSize = m_sBitmapFormatGrid.nWidth*m_sBitmapFormatGrid.nHeight;
            m_sBitmapFormatGrid.nPaletteSize = 0;
        }
        m_inputPin_ObstacleGrid.SetFormat(&m_sBitmapFormatGrid, NULL);
        m_outputPin_DebugGrid.SetFormat(&m_sBitmapFormatGrid, NULL);
        m_obstacleGrid = *(new Mat(m_sBitmapFormatGrid.nHeight,m_sBitmapFormatGrid.nWidth,CV_8UC1));
        m_depthGrid = *(new Mat(m_sBitmapFormatGrid.nHeight,m_sBitmapFormatGrid.nWidth,CV_8UC1));
#ifdef DEBUG
        m_debugGrid = *(new Mat(m_sBitmapFormatGrid.nHeight,m_sBitmapFormatGrid.nWidth,CV_8UC1));		//DEBUG
#endif
        //m_enabled = (tUInt8)GetPropertyInt("Initial State");
    }
    else if (eStage == StageGraphReady)
    {
        m_overtakingState = OS_NORMAL;
        SendSignalValueMessage(&m_outputPin_accelerate,m_pCoderDescSignalOut, 0, 0);
    }

    RETURN_NOERROR;
}

tResult SpeedRecommender::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult SpeedRecommender::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult SpeedRecommender::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult SpeedRecommender::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    RETURN_IF_POINTER_NULL(pMediaSample);

    if(pSource == &m_inputPin_DepthGrid)
    {

        if(m_enabled)
        {
            tFloat32 speed = 0;
            tInt drivingDirPre = m_drivingDirection;
            const tVoid* l_pSrcBuffer;
            if (IS_OK(pMediaSample->Lock(&l_pSrcBuffer)))
            {
                m_depthGrid = Mat(m_sBitmapFormatGrid.nHeight,m_sBitmapFormatGrid.nWidth,CV_8UC1,(tVoid*)l_pSrcBuffer,m_sBitmapFormatGrid.nBytesPerLine);

#ifdef DEBUG
                m_obstacleGrid.copyTo(m_debugGrid);		//DEBUG
#endif

                if(m_intState == IS_BACKWARDS)
                    m_drivingDirection = -1;
                else
                    m_drivingDirection = 1;

#ifdef DEBUG
                //putText(m_debugGrid,cv::String(cString::Format("%d",m_intState).GetPtr()),Point2f(TILE_COUNT_RIGHT - 100,TILE_COUNT_REAR),CV_FONT_HERSHEY_PLAIN,0.75,Scalar(0));
#endif


                if(m_parkingEnabled)
                {
                    speed = m_maxAccValue;
                }
                else if(m_intState == IS_FORWARD)
                {
                    speed = calcForwardSpeed();
                }
                else if(m_intState == IS_BACKWARDS)
                {

                    speed = calcBackwardsSpeed();
                }
//                else if (m_intState == IS_BACKOBST)
//                {
//                    Point2f basePoint = Point2f(TILE_COUNT_RIGHT - (CARSIZE_CROSS) / GRID_CM_PER_UNIT / 2, TILE_COUNT_REAR - (CARSIZE_DRIVING - CARSIZE_CENTER_TO_FRONT) / GRID_CM_PER_UNIT);

//                    tFloat32 errVal = 0;
//                    vector<Point2f> checkarea;
//                    checkarea.push_back(basePoint + Point2f(4,0));
//                    checkarea.push_back(basePoint + Point2f(0,-15));
//                    checkarea.push_back(basePoint + Point2f((CARSIZE_CROSS+1)/GRID_CM_PER_UNIT,-15));
//                    checkarea.push_back(basePoint + Point2f((CARSIZE_CROSS-3)/GRID_CM_PER_UNIT,0));

//                    CalcErrorValue(checkarea,TEST_REF_GREY_VALUE,&errVal);

//                    if(errVal < TEST_MAX_ERROR_VALUE)
//                        speed = -SPEED_BASE_VALUE;
//                    else
//                        speed = 0;
//                }


                if(speed > 10)
                {
                    speed += m_bonusVoltageAcc;
                }
                else if (speed < -10)
                    speed -= m_bonusVoltageAcc;

                pMediaSample->Unlock(l_pSrcBuffer);
            }

            breaking(&speed);

            speed = MIN(MAX_ACC,MAX(speed,-MAX_ACC));

            m_sendCounter++;
            if(abs(m_preSpeed-speed)!= 0 || m_sendCounter >= 20)
            {
                SendSignalValueMessage(&m_outputPin_accelerate,m_pCoderDescSignalOut, speed, 0);
                m_preSpeed = speed;
                m_sendCounter = 0;
            }
            if(m_drivingDirection != drivingDirPre)
            {
                SendSignalValueMessage(&m_outputPin_drivingDirection,m_pCoderDescSignalOut, (tFloat32)m_drivingDirection, 0);

            }
#ifdef DEBUG
            if (m_outputPin_DebugGrid.IsConnected())
            {
                cObjectPtr<IMediaSample> pNewDepthSample;
                if (IS_OK(AllocMediaSample(&pNewDepthSample)))
                {
                    tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
                    pNewDepthSample->Update(tmStreamTime, m_debugGrid.data, tInt32(m_debugGrid.rows*m_debugGrid.cols), 0);
                    m_outputPin_DebugGrid.Transmit(pNewDepthSample);
                }
            }
#endif
        }
    }

    if(pSource == &m_inputPin_ObstacleGrid)
    {
        const tVoid* l_pSrcBuffer;
        if (IS_OK(pMediaSample->Lock(&l_pSrcBuffer)))
        {
            adtf_util::cImage oImage;
            oImage.SetBits((tUInt8*) l_pSrcBuffer, &m_sBitmapFormatGrid);
            memcpy(m_obstacleGrid.data,oImage.GetBitmap(),(int)m_obstacleGrid.total());
            pMediaSample->Unlock(l_pSrcBuffer);
        }
    }

    if(pSource == &m_inputPin_segmentSpeed)
    {
        tFloat32 value = 0;
        tUInt32 timeStamp = 0;

        ReceiveSignalValueMessage(pMediaSample,m_pCoderDescSignalSegSpeed, &value, &timeStamp);
        LOG_INFO(cString::Format("segspeed val = %f",value));
        m_HSenabled = false;
        if(value < 0.0)
        {
            SendBoolSignalValueMessage(&m_outputPin_reverseLight,true,0);
            m_intState = IS_BACKWARDS;
            m_maxAccValue = value;
        }
        else
        {
            SendBoolSignalValueMessage(&m_outputPin_reverseLight,false,0);
            m_intState = IS_FORWARD;
            m_maxAccValue = value;
        }
    }


    if(pSource == &m_inputPin_motionData)
    {
        tFloat32 dY = 0, dPhi = 0, vel = 0;
        tUInt32 timeStamp = 0;

        ReceiveMotionDataMessage(pMediaSample,&dY,&dPhi,&vel,&timeStamp);
        m_overtakingDY += dY;
        m_highSpeedDist += dY;

        if(m_HSenabled && m_maxAccValue > 0 && m_highSpeedDist > HIGHSPEED_MIN_DIST)
        {
            //LOG_INFO("HIGH SPEED");
            m_maxAccValue = MAX(m_maxAccValue,27);//MAX_ACC;
        }

        if(m_parkingEnabled)
            m_afterParkingDist = 0;
        else
            m_afterParkingDist += dY;
//        if(m_intState == IS_BACKOBST)
//        {
//            m_dyBack += dY;
//            if(m_dyBack < -30.0)
//            {
//                m_intState = IS_FORWARD;
//                m_overtakingState = OS_NORMAL;
//            }
//        }

    }

    if(pSource == &m_inputPin_steering)
    {
        tFloat32 value = 0;
        tUInt32 timeStamp = 0;
        ReceiveSignalValueMessage(pMediaSample,m_pCoderDescSignalSteering, &value, &timeStamp);
        m_curvature = 1/(CARSIZE_AXISDIST/tan((value)/360*2*PI));
        if(abs(m_curvature) > 0.012)
            m_curvature = 0;
    }

    if(pSource == &m_inputPin_reducedSpeed)
    {
        tFloat32 value = 0;
        tUInt32 timeStamp = 0;
        ReceiveSignalValueMessage(pMediaSample,m_pCoderDescSignalLaneDet, &value, &timeStamp);
        m_reduceSpeed = (value==1);
    }

    if(pSource == &m_inputPin_voltageAct)
    {
        tFloat32 value = 0;
        tUInt32 timeStamp = 0;
        ReceiveSignalValueMessage(pMediaSample,m_pCoderDescSignal, &value, &timeStamp);
        m_bonusVoltageAcc = (MIN(MAX(8.5-value,0),7)*2.5);
    }

    if(pSource == &m_inputPin_parking)
    {
        tFloat32 value = 0;
        tUInt32 timeStamp = 0;
        ReceiveSignalValueMessage(pMediaSample,m_pCoderDescSignalOutEnable, &value, &timeStamp);
        m_parkingEnabled = (value==1);
    }

    if(pSource == &m_inputPin_enable)
    {
        tFloat32 value = 0;
        tUInt32 timeStamp = 0;
        ReceiveSignalValueMessage(pMediaSample,m_pCoderDescSignalOutEnable, &value, &timeStamp);
        m_enabled = (value==1);
        if(!m_enabled)
        {
			SendSignalValueMessage(&m_outputPin_accelerate,m_pCoderDescSignalOutEnable, 0, 0);
        }
        m_preSpeed = 0;

        m_OTenabled = false;
        m_HSenabled = false;
        m_dist2ObstHistCounter = 0;
        m_otCounterStart = 0;
        m_overtakingRdy = false;
        m_overtakingState = OS_NORMAL;
        m_intState = IS_FORWARD;

        m_breakCounterBack = 0;
        m_breakCounterForward = 0;


    }

    if(pSource == &m_inputPin_OTenable)
    {
        tFloat32 value = 0;
        tUInt32 timeStamp = 0;
        ReceiveSignalValueMessage(pMediaSample,m_pCoderDescSignalOutEnable, &value, &timeStamp);
        m_OTenabled = (value==1);
        if(!m_OTenabled)
        {
            LOG_INFO("reset OT");
            m_dist2ObstHistCounter = 0;
            m_overtakingRdy = false;
            m_overtakingState = OS_NORMAL;
            SendSignalValueMessage(&m_outputPin_useOtherLane,m_pCoderDescSignalOutEnable,0,0);
        }
    }

    if(pSource == &m_inputPin_HSenable)
    {
        tFloat32 value = 0;
        tUInt32 timeStamp = 0;
        ReceiveSignalValueMessage(pMediaSample,m_pCoderDescSignalOutEnable, &value, &timeStamp);
        m_HSenabled = (value==1);
        if(!m_HSenabled)
        {
            LOG_INFO("reset HS");
            //SendSignalValueMessage(&m_outputPin_useOtherLane,m_pCoderDescSignalOutEnable,0,0);
        }
        m_highSpeedDist = 0;
    }

    RETURN_NOERROR;
}

tResult SpeedRecommender::SendSignalValueMessage(cObjectPtr<cOutputPin> pOutputPin, cObjectPtr<IMediaTypeDescription> mediaDesc, tFloat32 value, tUInt32 timeStamp)
{
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleOut;
    AllocMediaSample((tVoid**)&pMediaSampleOut);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    mediaDesc->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleOut->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    cObjectPtr<IMediaCoder> pCoderOutput;
    RETURN_IF_FAILED(mediaDesc->WriteLock(pMediaSampleOut, &pCoderOutput));

    pCoderOutput->Set("f32Value", (tVoid*)&(value));
    pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
    mediaDesc->Unlock(pCoderOutput);

    //transmit media sample over output pin
    tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
    pMediaSampleOut->SetTime(tmStreamTime);
    pOutputPin->Transmit(pMediaSampleOut);

    RETURN_NOERROR;
}

tResult SpeedRecommender::ReceiveSignalValueMessage(IMediaSample* pMediaSample,cObjectPtr<IMediaTypeDescription> mediaDesc, tFloat32 *pValue, tUInt32 *pTimeStamp)
{
    // read-out the incoming Media Sample
    cObjectPtr<IMediaCoder> pCoderInput;
    RETURN_IF_FAILED(mediaDesc->Lock(pMediaSample, &pCoderInput));

    *pValue = 0;
    *pTimeStamp = 0;

    //get values from media sample
    pCoderInput->Get("f32Value", (tVoid*)pValue);
    pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)pTimeStamp);
    mediaDesc->Unlock(pCoderInput);

    RETURN_NOERROR;
}

tResult SpeedRecommender::ReceiveMotionDataMessage(IMediaSample* pMediaSample, tFloat32 *pDY, tFloat32 *pDPhi,tFloat32 *pVel, tUInt32 *pTimeStamp)
{
    // read-out the incoming Media Sample
    cObjectPtr<IMediaCoder> pCoderInput;
    RETURN_IF_FAILED(m_pCoderDescMotion->Lock(pMediaSample, &pCoderInput));

    *pDY = 0;
    *pDPhi = 0;
    *pVel = 0;
    *pTimeStamp = 0;

    //get values from media sample
    pCoderInput->Get("f32dY", (tVoid*)pDY);
    pCoderInput->Get("f32dPhi", (tVoid*)pDPhi);
    pCoderInput->Get("f32vel", (tVoid*)pVel);
    pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)pTimeStamp);
    m_pCoderDescMotion->Unlock(pCoderInput);

    RETURN_NOERROR;
}

tResult SpeedRecommender::GenPolyArc(Point2f center, tFloat32 radiusInner, tFloat32 radiusOuter, tFloat32 arcStartInner, tFloat32 arcStartOuter, tFloat32 arcEndInner, tFloat32 arcEndOuter, tInt sampleCount, vector<vector<Point2f> > &pts)
{
    tFloat32 curArcIn = arcStartInner*DEG2RAD;
    tFloat32 curArcOut = arcStartOuter*DEG2RAD;
    for(int i = 0;i < sampleCount+1;i++)
    {
        tFloat32 stepIn;
        tFloat32 stepOut;
        stepIn = ((arcEndInner - arcStartInner)/(sampleCount/(m_testingRangeModMin+i*(m_testingRangeModMax-m_testingRangeModMin)/(sampleCount))))*DEG2RAD;
        stepOut = ((arcEndOuter - arcStartOuter)/(sampleCount/(m_testingRangeModMin+i*(m_testingRangeModMax-m_testingRangeModMin)/(sampleCount))))*DEG2RAD;
        if(i==0)
        {
            stepIn = (stepIn/abs(stepIn))*TEST_FIRST_AREA_LENGTH/radiusInner;
            stepOut = (stepIn/abs(stepIn))*TEST_FIRST_AREA_LENGTH/radiusOuter;
        }

        pts.push_back(vector<Point2f>());
        pts[i].push_back(center + Point2f(radiusOuter / GRID_CM_PER_UNIT*cos(curArcOut),radiusOuter / GRID_CM_PER_UNIT*sin(curArcOut)));
        pts[i].push_back(center + Point2f(radiusOuter / GRID_CM_PER_UNIT*cos(curArcOut+stepOut),radiusOuter / GRID_CM_PER_UNIT*sin(curArcOut+stepOut)));
        pts[i].push_back(center + Point2f(radiusInner / GRID_CM_PER_UNIT*cos(curArcIn+stepIn),radiusInner / GRID_CM_PER_UNIT*sin(curArcIn+stepIn)));
        pts[i].push_back(center + Point2f(radiusInner / GRID_CM_PER_UNIT*cos(curArcIn),radiusInner / GRID_CM_PER_UNIT*sin(curArcIn)));
        if(i==0)
        {
            if(center.x < (tFloat32)TILE_COUNT_RIGHT)
            {
                pts[(int)pts.size() - 1][0].x = TILE_COUNT_RIGHT + CAMERADIST2FRONT * tan(29.0*DEG2RAD)-2;
                pts[(int)pts.size() - 1][3].x = TILE_COUNT_RIGHT - CAMERADIST2FRONT * tan(29.0*DEG2RAD)+2;
            }
            else
            {
                pts[(int)pts.size() - 1][0].x = TILE_COUNT_RIGHT - CAMERADIST2FRONT * tan(29.0*DEG2RAD)+2;
                pts[(int)pts.size() - 1][3].x = TILE_COUNT_RIGHT + CAMERADIST2FRONT * tan(29.0*DEG2RAD)-2;
            }
        }
        curArcIn += stepIn;
        curArcOut += stepOut;
    }
    RETURN_NOERROR;
}

tResult SpeedRecommender::CalcErrorValue(vector<Point2f> polypoints, tUInt8 refVal, tFloat32 *pErrValue,tInt32 mode)
{
    tInt xMax = 0;
    tInt xMin = 100000;
    tInt yMax = 0;
    tInt yMin = 100000;
    tInt pixCount = 0;
    for(int i = 0; i < (int)polypoints.size(); i++)
    {
        xMin = MIN(polypoints[i].x, xMin);
        yMin = MIN(polypoints[i].y, yMin);
        xMax = MAX(polypoints[i].x, xMax);
        yMax = MAX(polypoints[i].y, yMax);
    }
    xMin = MAX(xMin,0);
    yMin = MAX(yMin,0);
    xMax = MIN(xMax,m_obstacleGrid.cols-1);
    yMax = MIN(yMax,m_obstacleGrid.rows-1);

    *pErrValue = 0;

    for(int row = yMin; row <= yMax; row++)
    {
        for(int col = xMin; col <= xMax; col++)
        {
            if(pointPolygonTest(polypoints, Point(col,row),false) >= 0)
            {
                tFloat32 tempErrVal,tempErrValDepth;
                if(mode==USE_BOTH)
                {
                    tempErrVal = pow(MAX((tFloat32)(refVal - m_obstacleGrid.at<tUInt8>(Point(col,row))),0),2.5);
                    tempErrValDepth = pow(MAX((tFloat32)(refVal - m_depthGrid.at<tUInt8>(Point(col,row))),0),2.5);
                }
                else if(mode == USE_DEPTH_ONLY)
                {
                    tempErrValDepth = pow(MAX((tFloat32)(refVal - m_depthGrid.at<tUInt8>(Point(col,row))),0),2.5);
                    tempErrVal = tempErrValDepth;
                }
                else
                {
                    tempErrVal = pow(MAX((tFloat32)(refVal - m_obstacleGrid.at<tUInt8>(Point(col,row))),0),2.5);
                    tempErrValDepth = tempErrVal;
                }
                pixCount++;
                *pErrValue += (tempErrVal + tempErrValDepth)/2;
            }
        }
    }

    if(pixCount > 0)
        *pErrValue = *pErrValue/pixCount;
    else
        *pErrValue = TEST_MAX_ERROR_VALUE+1;
    RETURN_NOERROR;
}

tFloat32 SpeedRecommender::calcForwardSpeed()
{
//    if(m_parkingEnabled)
//        return calcParkingSpeed(FORWARD);

    tFloat32 radiusInner = 0, radiusOuter = 0, radiusExt = 0;
    tFloat32 arcShiftInner = 0, arcShiftOuter = 0;
    tFloat32 arc = MAX(MIN(MAX_TESTING_RANGE/m_maxRangeScaling * m_curvature * RAD2DEG, 90), -90);
    Point2f circleCenter;
    vector<vector<Point2f> > segmentPoints;

    tFloat32 speed = 0;
    tInt32 x = 0;

    //--------------------------------Select Detection Area--------------------------------
    if(abs(m_curvature) < CURVATURE_SMALL_EPSILON)	//straight
    {
        Point2f basePoint = Point2f(TILE_COUNT_RIGHT - (CARSIZE_CROSS) / GRID_CM_PER_UNIT / 2 -1, TILE_COUNT_REAR + (CARSIZE_CENTER_TO_FRONT) / GRID_CM_PER_UNIT);
        for (int i = 0; i < m_testSamplePointCount; i++)
        {
            tFloat32 yShift = (tFloat32)MAX_TESTING_RANGE/m_maxRangeScaling / (m_testSamplePointCount/(m_testingRangeModMin+i*(m_testingRangeModMax-m_testingRangeModMin)/m_testSamplePointCount)) / GRID_CM_PER_UNIT;
            if(i==0)
                yShift = MAX(TEST_FIRST_AREA_LENGTH,yShift);
            segmentPoints.push_back(vector<Point2f>());
            segmentPoints[(int)segmentPoints.size() - 1].push_back(basePoint);
            segmentPoints[(int)segmentPoints.size() - 1].push_back(basePoint + Point2f(0,yShift));
            segmentPoints[(int)segmentPoints.size() - 1].push_back(basePoint + Point2f((CARSIZE_CROSS+2)/GRID_CM_PER_UNIT,yShift));
            segmentPoints[(int)segmentPoints.size() - 1].push_back(basePoint + Point2f((CARSIZE_CROSS+2)/GRID_CM_PER_UNIT,0));

            if(i==0)
            {
                segmentPoints[(int)segmentPoints.size() - 1][0].x = TILE_COUNT_RIGHT - CAMERADIST2FRONT * tan(29.0*DEG2RAD)+2;
                segmentPoints[(int)segmentPoints.size() - 1][3].x = TILE_COUNT_RIGHT + CAMERADIST2FRONT * tan(29.0*DEG2RAD)-2;
            }


            basePoint = segmentPoints[(int)segmentPoints.size() - 1][1];
        }
    }
    else if(m_curvature > 0)	//right curve
    {
        radiusInner = MAX(1 / m_curvature - (CARSIZE_CROSS+2) / 2,0);
        radiusOuter = MAX(1 / m_curvature + (CARSIZE_CROSS+2) / 2,0);
        radiusExt = sqrt(pow(radiusOuter, 2) + pow((CARSIZE_CENTER_TO_FRONT+1), 2));

        arcShiftInner = asin((CARSIZE_CENTER_TO_FRONT+1)/radiusInner)*RAD2DEG;
        arcShiftOuter = asin((CARSIZE_CENTER_TO_FRONT+1)/radiusExt)*RAD2DEG;

        circleCenter = Point2f(TILE_COUNT_RIGHT - (CARSIZE_CROSS+2) / GRID_CM_PER_UNIT / 2 - radiusInner / GRID_CM_PER_UNIT, TILE_COUNT_REAR);

        GenPolyArc(circleCenter, radiusInner, radiusExt, arcShiftInner, arcShiftOuter, arc + arcShiftInner, arc + arcShiftOuter, m_testSamplePointCount, segmentPoints);
    }
    else	//left curve
    {
        radiusInner = abs(MIN(1 / m_curvature + (CARSIZE_CROSS+2) / 2, 0));
        radiusOuter = abs(MIN(1 / m_curvature - (CARSIZE_CROSS+2) / 2, 0));
        radiusExt = sqrt(pow(radiusOuter, 2) + pow(CARSIZE_CENTER_TO_FRONT+1, 2));

        arcShiftInner = -asin((CARSIZE_CENTER_TO_FRONT+1)/ radiusInner) *RAD2DEG;
        arcShiftOuter = -asin((CARSIZE_CENTER_TO_FRONT+1)/ radiusExt) *RAD2DEG;

        circleCenter = Point2f(TILE_COUNT_RIGHT + (CARSIZE_CROSS+2) / GRID_CM_PER_UNIT / 2 + radiusInner / GRID_CM_PER_UNIT, TILE_COUNT_REAR);

        GenPolyArc(circleCenter, radiusInner, radiusExt, 180 + arcShiftInner, 180 + arcShiftOuter, 180 + arc + arcShiftInner, 180 + arc + arcShiftOuter, m_testSamplePointCount, segmentPoints);
    }

#ifdef DEBUG
    //debug drawing
    for (int i = 0; i < (int)segmentPoints.size(); i++)
    {
        for (int j = 0; j < (int)segmentPoints[i].size() - 1; j++)
        {
            line(m_debugGrid, segmentPoints[i][j], segmentPoints[i][j+1], Scalar(0), 1);
        }
        line(m_debugGrid, segmentPoints[i][0], segmentPoints[i][segmentPoints[i].size() - 1], Scalar(0), 1);
    }
#endif

    //--------------------------------Determin View Dist + calc speed--------------------------------
    for(x = 0; x < (int)segmentPoints.size(); x++)
    {
        tFloat32 tempErrVal = 0;
        CalcErrorValue(segmentPoints[x],TEST_REF_GREY_VALUE,&tempErrVal);

#ifdef DEBUG
        //putText(m_debugGrid,cv::String(cString::Format("%.2f",tempErrVal).GetPtr()),Point2f(30,TILE_COUNT_REAR+ 15*x),CV_FONT_HERSHEY_PLAIN,0.75,Scalar(0));
#endif

        if (tempErrVal > TEST_MAX_ERROR_VALUE)
        {
            break;
        }
    }

    if(x<1)
    {
        m_viewDist = 0;
        speed = 0;
    }
    else
    {
        m_viewDist = (segmentPoints[x-1][1].y - segmentPoints[0][0].y);//*(1+m_curvature*20);   //TODO handle curve
        speed = SPEED_MIN_VALUE / 100 + m_viewDist / (VIEW_MAX_SPEED_DIST / (0.5 - SPEED_BASE_VALUE/100));
    }

    if(m_reduceSpeed && m_viewDist > 10)
    {
        speed = MAX(SPEED_BASE_VALUE/100,speed*CURVATURE_SPEED_REDUCTION_RATIO);
    }

    //--------------------------------Overtaking--------------------------------
    m_dist2ObstHist[m_dist2ObstHistCounter%OT_HIST_SIZE] = m_viewDist;
    m_dist2ObstHistCounter++;

#ifdef DEBUG
    //putText(m_debugGrid,cv::String(cString::Format("ot state %d",m_overtakingState).GetPtr()),Point2f(20,90),CV_FONT_HERSHEY_PLAIN,0.75,Scalar(0));	//DEBUG
#endif

    if(!m_overtakingRdy && m_dist2ObstHistCounter > 40 && m_afterParkingDist > 100)
        m_overtakingRdy = true;

#ifdef DEBUG
    putText(m_debugGrid,cv::String(cString::Format("cuLa %3.5f",m_curvatureLaneDetect*1000000).GetPtr()),Point2f(TILE_COUNT_RIGHT + 30,100),CV_FONT_HERSHEY_PLAIN,0.75,Scalar(0));	//DEBUG
    putText(m_debugGrid,cv::String(cString::Format("otst %d",m_overtakingState).GetPtr()),Point2f(TILE_COUNT_RIGHT + 30,110),CV_FONT_HERSHEY_PLAIN,0.75,Scalar(0));	//DEBUG
#endif

    if(m_HSenabled && m_OTenabled &&  m_overtakingState == OS_NORMAL && m_overtakingRdy)
    {
        tFloat32 currDist = 0;
        for(int i = 0;i<OT_HIST_SIZE;i++)
        {
            currDist = MAX(currDist,m_dist2ObstHist[i]);
        }

#ifdef DEBUG
        putText(m_debugGrid,cv::String(cString::Format("dist %3.5f",currDist).GetPtr()),Point2f(TILE_COUNT_RIGHT + 30,90),CV_FONT_HERSHEY_PLAIN,0.75,Scalar(0));	//DEBUG
#endif
        if(currDist < MIN_DIST2OBST_BEFORE_OVERTAKING && abs(m_curvatureLaneDetect) < CURVATURE_SMALL_EPSILON*25)
        {
                TryStartOvertaking();
        }
    }
    else if(m_overtakingState == OS_START_OVERTAKING)
    {
#ifdef DEBUG
        putText(m_debugGrid,cv::String(cString::Format("otdy %d",m_overtakingDY).GetPtr()),Point2f(TILE_COUNT_RIGHT + 30,120),CV_FONT_HERSHEY_PLAIN,0.75,Scalar(0));	//DEBUG
#endif

        if(m_overtakingDY > 50.0)
        {
            SendBoolSignalValueMessage(&m_outputPin_turn_left_enabled,false,0);
            m_overtakingState = OS_OVERTAKING;
        }

    }
    else if(m_overtakingState == OS_OVERTAKING)
    {
        TryEndOvertaking();
    }
    else if(m_overtakingState == OS_FINISH_OVERTAKING)
    {
        if(m_overtakingDY > 50.0)
        {
            SendBoolSignalValueMessage(&m_outputPin_turn_right_enabled,false,0);
            m_overtakingState = OS_NORMAL;
        }
    }

    //--------------------------------limit speed + breaking controller--------------------------------
    if (speed < ((SPEED_BASE_VALUE-3)/100))
        speed = 0;
    else if(speed < SPEED_BASE_VALUE/100)
        speed += abs(m_curvature * 4);


    speed = MIN(speed*100,m_maxAccValue);

    //--------------------------------going back for overtake--------------------------------
//    if(speed == 0 && m_maxAccValue > 0)
//    {
//        if(m_obstBackwardsCounter < 20)
//            m_obstBackwardsCounter++;
//        else
//        {
//            m_obstBackwardsCounter = 0;
//            m_intState = IS_BACKOBST;
//            m_dyBack = 0;
//        }
//    }

#ifdef DEBUG
    putText(m_debugGrid,cv::String(cString::Format("spre %3.2f",speed).GetPtr()),Point2f(TILE_COUNT_RIGHT + 30,10),CV_FONT_HERSHEY_PLAIN,0.75,Scalar(0));		//DEBUG
#endif

//    if((m_preSpeed - speed > 2 /*&& m_preSpeed > SPEED_BASE_VALUE-20*/) || (m_breakCounterForward > 0 && m_postBreakSpeed > speed)) //initial OR retrigger break counter
//    {
//        if(m_breakCounterForward == 0)
//            m_preBreakSpeed = m_preSpeed;
//        m_postBreakSpeed = speed;
//        m_breakCounterForward = BREAK_COUNTER_START;
//        SendBoolSignalValueMessage(&m_outputPin_brakeLight,1,0);
//    }
//    if(m_breakCounterForward > 0)
//    {
//        m_breakCounterForward--;
//        if(m_breakCounterForward < 1)
//            SendBoolSignalValueMessage(&m_outputPin_brakeLight,0,0);
//        if(m_preBreakSpeed < SPEED_MIN_VALUE +2 && m_postBreakSpeed <= 0)
//            speed -= MAX(m_preBreakSpeed - m_postBreakSpeed-SPEED_BASE_VALUE,0)*m_breakCounterForward / BREAK_COUNTER_START;
//        else
//            speed -= (m_preBreakSpeed - m_postBreakSpeed)*m_breakCounterForward / BREAK_COUNTER_START;
//    }

#ifdef DEBUG
    putText(m_debugGrid,cv::String(cString::Format("brco %d",m_breakCounterForward).GetPtr()),Point2f(TILE_COUNT_RIGHT + 30,30),CV_FONT_HERSHEY_PLAIN,0.75,Scalar(0));	//DEBUG
    putText(m_debugGrid,cv::String(cString::Format("sout %.2f",speed).GetPtr()),Point2f(TILE_COUNT_RIGHT + 30,40),CV_FONT_HERSHEY_PLAIN,0.75,Scalar(0));			//DEBUG
    putText(m_debugGrid,cv::String(cString::Format("pres %.2f",m_preSpeed).GetPtr()),Point2f(TILE_COUNT_RIGHT + 30,50),CV_FONT_HERSHEY_PLAIN,0.75,Scalar(0));		//DEBUG
    putText(m_debugGrid,cv::String(cString::Format("preb %.2f",m_preBreakSpeed).GetPtr()),Point2f(TILE_COUNT_RIGHT + 30,60),CV_FONT_HERSHEY_PLAIN,0.75,Scalar(0));//DEBUG
    //putText(m_debugGrid,cv::String(cString::Format("brfa %.2f",m_breakFact).GetPtr()),Point2f(185,70),CV_FONT_HERSHEY_PLAIN,0.75,Scalar(0));	//DEBUG

    putText(m_debugGrid,cv::String(cString::Format("sada %3.2f",(m_preBreakSpeed-m_postBreakSpeed)*m_breakCounterForward/45).GetPtr()),Point2f(TILE_COUNT_RIGHT + 30,20),CV_FONT_HERSHEY_PLAIN,0.75,Scalar(0));	//DEBUG
    putText(m_debugGrid,cv::String(cString::Format("vdis %3.2f",m_viewDist).GetPtr()),Point2f(TILE_COUNT_RIGHT + 30,70),CV_FONT_HERSHEY_PLAIN,0.75,Scalar(0));	//DEBUG
    putText(m_debugGrid,cv::String(cString::Format("curv %3.5f",abs(m_curvature*1000)).GetPtr()),Point2f(TILE_COUNT_RIGHT + 30,80),CV_FONT_HERSHEY_PLAIN,0.75,Scalar(0));	//DEBUG
#endif
    return speed;

}

tFloat32 SpeedRecommender::calcBackwardsSpeed()
{
//    if(m_parkingEnabled)
//        return calcParkingSpeed(BACKWARD);

    Point2f basePoint = Point2f(TILE_COUNT_RIGHT - (CARSIZE_CROSS) / GRID_CM_PER_UNIT / 2, TILE_COUNT_REAR - (CARSIZE_DRIVING - CARSIZE_CENTER_TO_FRONT) / GRID_CM_PER_UNIT);

    tFloat32 errVal = 0;
    vector<Point2f> checkarea;
    checkarea.push_back(basePoint + Point2f(4,0));
    checkarea.push_back(basePoint + Point2f(0,-15));
    checkarea.push_back(basePoint + Point2f((CARSIZE_CROSS+1)/GRID_CM_PER_UNIT,-15));
    checkarea.push_back(basePoint + Point2f((CARSIZE_CROSS-3)/GRID_CM_PER_UNIT,0));

#ifdef DEBUG
    //debug drawing
    for (int j = 0; j < (int)checkarea.size() - 1; j++)
    {
        line(m_debugGrid, checkarea[j], checkarea[j+1], Scalar(0), 1);
    }
    line(m_debugGrid, checkarea[0], checkarea[checkarea.size() - 1], Scalar(0), 1);
#endif

    CalcErrorValue(checkarea,TEST_REF_GREY_VALUE,&errVal);
#ifdef DEBUG
    putText(m_debugGrid,cv::String(cString::Format("errV %.2f",errVal).GetPtr()),Point2f(TILE_COUNT_RIGHT + 30,40),CV_FONT_HERSHEY_PLAIN,0.75,Scalar(0));			//DEBUG
#endif
    if(errVal < TEST_MAX_ERROR_VALUE)
        return m_maxAccValue;
    else
        return 0;


}

tBool SpeedRecommender::checkDeadBlock()
{
    vector<Point2f> polyBAreaLeft, polyBAreaRight;

    tFloat32 innerX = CAMERADIST2FRONT * tan(29.0*DEG2RAD);
    tFloat32 outerX = CARSIZE_CROSS/2+1;//1/m_curvature - sqrt(radiusInner*radiusInner - CARSIZE_CENTER_TO_FRONT * CARSIZE_CENTER_TO_FRONT);
    tFloat32 outerY = tan(61.0*DEG2RAD)*outerX;

    Point2f camPoint(TILE_COUNT_RIGHT,TILE_COUNT_REAR+CARSIZE_CENTER_TO_FRONT-CAMERADIST2FRONT);

    polyBAreaLeft.clear();
    polyBAreaLeft.push_back(camPoint + Point2f(innerX,CAMERADIST2FRONT));
    polyBAreaLeft.push_back(camPoint + Point2f(outerX,CAMERADIST2FRONT));
    polyBAreaLeft.push_back(camPoint + Point2f(outerX,outerY));

    polyBAreaRight.clear();
    polyBAreaRight.push_back(camPoint + Point2f(-innerX,CAMERADIST2FRONT));
    polyBAreaRight.push_back(camPoint + Point2f(-outerX,CAMERADIST2FRONT));
    polyBAreaRight.push_back(camPoint + Point2f(-outerX,outerY));

#ifdef DEBUG
    for (int j = 0; j < (int)polyBAreaLeft.size() - 1; j++)
    {
        line(m_debugGrid, polyBAreaLeft[j], polyBAreaLeft[j+1], Scalar(0), 1);
    }
    line(m_debugGrid, polyBAreaLeft[0], polyBAreaLeft[polyBAreaLeft.size() - 1], Scalar(0), 1);
    for (int j = 0; j < (int)polyBAreaRight.size() - 1; j++)
    {
        line(m_debugGrid, polyBAreaRight[j], polyBAreaRight[j+1], Scalar(0), 1);
    }
    line(m_debugGrid, polyBAreaRight[0], polyBAreaRight[polyBAreaRight.size() - 1], Scalar(0), 1);
#endif
    tFloat32 lErr = 0, rErr=0;
    CalcErrorValue(polyBAreaLeft,TEST_REF_GREY_VALUE,&lErr);
    CalcErrorValue(polyBAreaRight,TEST_REF_GREY_VALUE,&rErr);
#ifdef DEBUG
    putText(m_debugGrid,cv::String(cString::Format("%.2f",lErr).GetPtr()),Point2f(TILE_COUNT_RIGHT - 50,TILE_COUNT_REAR+20),CV_FONT_HERSHEY_PLAIN,0.75,Scalar(0));

    putText(m_debugGrid,cv::String(cString::Format("%.2f",rErr).GetPtr()),Point2f(TILE_COUNT_RIGHT - 50,TILE_COUNT_REAR+35),CV_FONT_HERSHEY_PLAIN,0.75,Scalar(0));
#endif
    if((lErr+rErr)/2 > TEST_MAX_ERROR_VALUE)
        return true;
    else
        return false;
}

tResult SpeedRecommender::TryStartOvertaking()
{
    //find ot dist


    vector<Point2f> contraflowCheckArea;
    contraflowCheckArea.push_back(Point2f(TILE_COUNT_RIGHT-CARSIZE_CROSS/2 - m_dist2RightIn+LANE_WIDTH*1.5+1,TILE_COUNT_REAR+CARSIZE_CENTER_TO_FRONT));
    contraflowCheckArea.push_back(Point2f(TILE_COUNT_RIGHT-CARSIZE_CROSS/2 - m_dist2RightIn+LANE_WIDTH*1.5+1,TILE_COUNT_REAR+CARSIZE_CENTER_TO_FRONT + OVERTAKING_TEST_RANGE));
    contraflowCheckArea.push_back(Point2f(TILE_COUNT_RIGHT+CARSIZE_CROSS/2 - m_dist2RightIn+LANE_WIDTH*1.5+3,TILE_COUNT_REAR+CARSIZE_CENTER_TO_FRONT + OVERTAKING_TEST_RANGE));
    contraflowCheckArea.push_back(Point2f(TILE_COUNT_RIGHT+CARSIZE_CROSS/2 - m_dist2RightIn+LANE_WIDTH*1.5+3,TILE_COUNT_REAR+CARSIZE_CENTER_TO_FRONT));

#ifdef DEBUG
    for (int j = 0; j < (int)contraflowCheckArea.size() - 1; j++)
    {
        line(m_debugGrid, contraflowCheckArea[j], contraflowCheckArea[j+1], Scalar(0), 1);
    }
    line(m_debugGrid, contraflowCheckArea[0], contraflowCheckArea[contraflowCheckArea.size() - 1], Scalar(0), 1);
    putText(m_debugGrid,cv::String(cString::Format("d2rl: %.2f",m_dist2RightIn).GetPtr()),Point2f(TILE_COUNT_RIGHT - 50,TILE_COUNT_REAR+55),CV_FONT_HERSHEY_PLAIN,0.75,Scalar(0));

#endif

    tFloat32 tempErrVal = 0;
    CalcErrorValue(contraflowCheckArea,TEST_REF_GREY_VALUE,&tempErrVal,USE_DEPTH_ONLY);

    if(tempErrVal < TEST_MAX_ERROR_VALUE)
    {
        if(m_otCounterStart < 3)
            m_otCounterStart++;
        else
        {
            tUInt32 timestamp = 0;
            //contraflow in sightrange free -> start overtaking
            SendBoolSignalValueMessage(&m_outputPin_turn_left_enabled,true,timestamp);
            SendSignalValueMessage(&m_outputPin_useOtherLane,m_pCoderDescSignalOut,1.0,timestamp);

            m_otCounterStart = 0;
            m_overtakingDY = 0;
            m_overtakingState = OS_START_OVERTAKING;
        }
    }
    RETURN_NOERROR;
}

tResult SpeedRecommender::TryEndOvertaking()
{
    vector<Point2f> normalLaneArea;
    normalLaneArea.push_back(Point2f(TILE_COUNT_RIGHT-CARSIZE_CROSS/2 - m_dist2RightIn-LANE_WIDTH/2 -3,TILE_COUNT_REAR+CARSIZE_CENTER_TO_FRONT/2));
    normalLaneArea.push_back(Point2f(TILE_COUNT_RIGHT-CARSIZE_CROSS/2 - m_dist2RightIn-LANE_WIDTH/2 -3,TILE_COUNT_REAR+CARSIZE_CENTER_TO_FRONT*2));
    normalLaneArea.push_back(Point2f(TILE_COUNT_RIGHT+CARSIZE_CROSS/2 - m_dist2RightIn-LANE_WIDTH/2 -1,TILE_COUNT_REAR+CARSIZE_CENTER_TO_FRONT*2));
    normalLaneArea.push_back(Point2f(TILE_COUNT_RIGHT+CARSIZE_CROSS/2 - m_dist2RightIn-LANE_WIDTH/2 -1,TILE_COUNT_REAR+CARSIZE_CENTER_TO_FRONT/2));


#ifdef DEBUG
    for (int j = 0; j < (int)normalLaneArea.size() - 1; j++)
    {
        line(m_debugGrid, normalLaneArea[j], normalLaneArea[j+1], Scalar(0), 1);
    }
    line(m_debugGrid, normalLaneArea[0], normalLaneArea[normalLaneArea.size() - 1], Scalar(0), 1);
#endif

    tFloat32 tempErrVal = 0;
    CalcErrorValue(normalLaneArea,TEST_REF_GREY_VALUE,&tempErrVal,USE_BOTH);
#ifdef DEBUG
    //putText(m_debugGrid,cv::String(cString::Format("%.2f",tempErrVal).GetPtr()),Point2f(TILE_COUNT_RIGHT,TILE_COUNT_REAR+-50),CV_FONT_HERSHEY_PLAIN,0.75,Scalar(0));
#endif
    if(tempErrVal < TEST_MAX_ERROR_VALUE)
    {
        tUInt32 timestamp = 0;
        SendBoolSignalValueMessage(&m_outputPin_turn_right_enabled,true,timestamp);
        SendSignalValueMessage(&m_outputPin_useOtherLane,m_pCoderDescSignalOut,0.0,timestamp);
        m_overtakingState = OS_FINISH_OVERTAKING;
        m_overtakingDY = 0;
    }
    RETURN_NOERROR;
}

tResult SpeedRecommender::ReceiveLaneInfoMessage(IMediaSample* pMediaSample, tFloat32 *pDX, tFloat32 *pOrientation, tFloat32 *pCurvature, tUInt32 *pTimeStamp)
{
    // read-out the incoming Media Sample
    cObjectPtr<IMediaCoder> pCoderInput;
    RETURN_IF_FAILED(m_pCoderDescLaneInfo->Lock(pMediaSample, &pCoderInput));

    *pDX = 0;
    *pOrientation = 0;
    *pCurvature = 0;
    *pTimeStamp = 0;

    //get values from media sample
    pCoderInput->Get("f32Curvature", (tVoid*)pCurvature);
    pCoderInput->Get("f32LanePos", (tVoid*)pDX);
    pCoderInput->Get("f32CarOrientation", (tVoid*)pOrientation);
    pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)pTimeStamp);
    m_pCoderDescLaneInfo->Unlock(pCoderInput);

    RETURN_NOERROR;
}

tFloat32 SpeedRecommender::calcParkingSpeed(tInt32 dir)
{
    Point startP,endP;
    if(dir == FORWARD)
    {
        startP = Point(TILE_COUNT_RIGHT,TILE_COUNT_REAR + CARSIZE_CENTER_TO_FRONT);
        endP = Point(TILE_COUNT_RIGHT,TILE_COUNT_REAR + CARSIZE_CENTER_TO_FRONT + PARKING_TESTING_RANGE);
    }
    else if(dir == BACKWARD)
    {
        startP = Point(TILE_COUNT_RIGHT,TILE_COUNT_REAR - CARSIZE_CENTER_TO_REAR);
        endP = Point(TILE_COUNT_RIGHT,TILE_COUNT_REAR - CARSIZE_CENTER_TO_REAR - PARKING_TESTING_RANGE);
    }
    else
    {
        LOG_ERROR("calcParkingSpeed - invalid direction");
        return 0;
    }

    LineIterator it(m_obstacleGrid, startP,endP);
    int x = 0;
    for(; x < it.count; x++, ++it)
    {
        tUInt8 last = *(tUInt8*)*it;
        if(last < (tUInt8)TEST_REF_GREY_VALUE)
            break;
    }
    if(x > PARKING_TESTING_RANGE-1)
    {
        return m_maxAccValue;
    }
    else if(x > PARKING_TESTING_RANGE/2)
    {
        if(dir == FORWARD)
            return MAX(SPEED_MIN_VALUE,m_maxAccValue-3);
        else
            return MIN(-SPEED_MIN_VALUE,m_maxAccValue+3);
    }
    else
    {
        return 0;
    }
}

tResult SpeedRecommender::SendBoolSignalValueMessage(cObjectPtr<cOutputPin> pOutputPin, bool value, tUInt32 timeStamp)
{
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleOut;
    AllocMediaSample((tVoid**)&pMediaSampleOut);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescBoolSignal->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleOut->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    cObjectPtr<IMediaCoder> pCoderOutput;
    m_pCoderDescBoolSignal->WriteLock(pMediaSampleOut, &pCoderOutput);

    pCoderOutput->Set("bValue", (tVoid*)&(value));
    pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
    m_pCoderDescBoolSignal->Unlock(pCoderOutput);

    //transmit media sample over output pin
    pMediaSampleOut->SetTime(pMediaSampleOut->GetTime());
    pOutputPin->Transmit(pMediaSampleOut);

    RETURN_NOERROR;
}

tResult SpeedRecommender::breaking(tFloat32* speed)
{
 //break forward
    if(((m_preSpeed - *speed  > 2 && m_preSpeed > 0 )/*&& m_preSpeed > SPEED_BASE_VALUE-20*/) || (m_breakCounterForward > 0 && m_postBreakSpeed > *speed)) //initial OR retrigger break counter
    {
        if(m_breakCounterForward == 0)
            m_preBreakSpeed = m_preSpeed;
        m_postBreakSpeed = *speed;
        m_breakCounterForward = BREAK_COUNTER_START;
        SendBoolSignalValueMessage(&m_outputPin_brakeLight,1,0);
    }
    if(m_breakCounterForward > 0)
    {
        m_breakCounterForward--;
        if(m_breakCounterForward < 1)
            SendBoolSignalValueMessage(&m_outputPin_brakeLight,0,0);
        if(m_preBreakSpeed < SPEED_MIN_VALUE +2 && m_postBreakSpeed <= 0)
            *speed -= MAX(m_preBreakSpeed - m_postBreakSpeed - SPEED_MIN_VALUE,0)*m_breakCounterForward / BREAK_COUNTER_START;
        else if(m_preBreakSpeed < SPEED_BASE_VALUE +2)
            *speed -= (m_preBreakSpeed - m_postBreakSpeed)*m_breakCounterForward / BREAK_COUNTER_START;
        else
            *speed -= (m_preBreakSpeed - m_postBreakSpeed + 13)*m_breakCounterForward / BREAK_COUNTER_START;

//        *speed -= MAX(m_preBreakSpeed - m_postBreakSpeed + SPEED_BASE_VALUE,0)*m_breakCounterForward / BREAK_COUNTER_START;
    }

//break backward
//    if(((m_preSpeed - *speed < -2 && m_preSpeed < 0) /*&& m_preSpeed > SPEED_BASE_VALUE-20*/) || (m_breakCounterBack > 0 && m_postBreakSpeed < *speed)) //initial OR retrigger break counter
//    {
//        if(m_breakCounterBack == 0)
//            m_preBreakSpeed = m_preSpeed;
//        m_postBreakSpeed = *speed;
//        m_breakCounterBack = BREAK_COUNTER_START;
//        //SendBoolSignalValueMessage(&m_outputPin_brakeLight,1,0);
//    }
//    if(m_breakCounterBack > 0)
//    {
//        m_breakCounterBack--;
//       // *speed -= SPEED_BACKWARDS*m_breakCounterBack / BREAK_COUNTER_START;

//        if(m_preBreakSpeed > -SPEED_MIN_VALUE -2 && m_postBreakSpeed <= 0)
//            *speed += MIN(m_preBreakSpeed - m_postBreakSpeed+SPEED_BACKWARDS,0)*m_breakCounterBack / BREAK_COUNTER_START;
//        else
//            *speed += (m_preBreakSpeed - m_postBreakSpeed)*m_breakCounterBack / BREAK_COUNTER_START;
//    }
    RETURN_NOERROR;
}












