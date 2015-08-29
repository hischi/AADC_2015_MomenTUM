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

#include "stdafx.h"

#include "math.h"
#include "SegmentDriver.h"


#define RETURN_ROW goto ROW
#define RETURN_NO_ROW goto NO_ROW

ADTF_FILTER_PLUGIN("AADC Segment Driver", OID_ADTF_SEGMENTDRIVER, SegmentDriver)


SegmentDriver::SegmentDriver(const tChar* __info) : cFilter(__info)
{
    SetPropertyStr("Configurationfile","/home/odroid/AADC/calibration_files/maneuver_script.xml");
    SetPropertyBool("Configurationfile" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Configurationfile" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");

    m_K = Point2f(TILE_COUNT_RIGHT - (CARSIZE_CROSS) / GRID_CM_PER_UNIT / 2, TILE_COUNT_REAR + (30) / GRID_CM_PER_UNIT);
    m_kr = Point2f(-cos(0.506145483078+PI/4), sin(0.506145483078));
    m_kl = Point2f(cos(0.506145483078+PI/4), sin(0.506145483078));
    m_IR_R = Point2f(TILE_COUNT_RIGHT-CARSIZE_CROSS, TILE_COUNT_REAR + (47) / GRID_CM_PER_UNIT);
    m_IR_L = Point2f(TILE_COUNT_RIGHT, TILE_COUNT_REAR + (47) / GRID_CM_PER_UNIT);
}

SegmentDriver::~SegmentDriver()
{
    for(int i = 0; i < m_Scripts.size(); i++)
    {
        if(m_Scripts[i] != NULL)
        {
            vector<ScriptOP*> ops = m_Scripts[i]->ops;
            for(int j = 0; j < ops.size(); j++)
            {
                if(m_Scripts[i]->ops[j] != NULL)
                    delete(m_Scripts[i]->ops[j]);
                m_Scripts[i]->ops[j] = NULL;
            }
            delete(m_Scripts[i]);
            m_Scripts[i] = NULL;
        }
    }

}

void printOP(tInt op)
{
    if(op == SET_CURV)
        LOG_INFO("    curv()");
    else if(op == SET_SPEED)
        LOG_INFO("    speed()");
    else if(op == WAITFORORIENTATION)
        LOG_INFO("    wait_orientation()");
    else if(op == CHECKROW)
        LOG_INFO("    check_row()");
    else if(op == WAITFORDISTREAR)
        LOG_INFO("    wait_dist_back()");
    else if(op == SEARCHPARKINGLOT)
        LOG_INFO("    search_parking_lot()");
    else if(op == WAITFORDISTDRIVEN)
        LOG_INFO("    wait_dist_driven()");
    else if(op == WAITFORDISTCROSSING)
        LOG_INFO("    wait_dist_crossing()");
    else if(op == WAITFORTIME)
        LOG_INFO("    wait_time()");
    else if(op == LFENABLE)
        LOG_INFO("    lane_following()");
    else if(op == WAITFORSIGN)
        LOG_INFO("    wait_sign()");
    else if(op == SETINDICATOR)
        LOG_INFO("    indicator()");
    else if(op == INIT_ORIENTATION)
        LOG_INFO("    init_orientation()");
    else if(op == CHECKOT)
        LOG_INFO("    check_ot()");
    else if(op == HSENABLE)
        LOG_INFO("    high_speed()");
    else if(op == IGNORESENS)
        LOG_INFO("    ignore_sensors()");
}

void printScript(vector<ManeuverScript*> m_Scripts)
{
    for(int i = 0; i < m_Scripts.size(); i++)
    {
        LOG_INFO(cString::Format("%d", m_Scripts[i]->id));
        if(m_Scripts[i] != NULL)
        {
            vector<ScriptOP*> ops = m_Scripts[i]->ops;
            for(int j = 0; j < ops.size(); j++)
            {
                if(m_Scripts[i]->ops[j] != NULL)
                {
                    printOP(m_Scripts[i]->ops[j]->op);
                }
            }
        }
    }
}

tResult SegmentDriver::CreateInputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    RETURN_IF_FAILED(m_inputPin_state.Create("state", new cMediaType(0, 0, 0, "tSignalValue")  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_state));

    RETURN_IF_FAILED(m_inputPin_maneuver.Create("maneuver", new cMediaType(0, 0, 0, "tSignalValue")  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_maneuver));

    RETURN_IF_FAILED(m_inputPin_yaw.Create("yaw", new cMediaType(0, 0, 0, "tSignalValue")  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_yaw));

    RETURN_IF_FAILED(m_inputPin_parkingDirection.Create("parkingDirection", new cMediaType(0, 0, 0, "tSignalValue")  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_parkingDirection));

    tChar const * strDesc = pDescManager->GetMediaDescription("tCrossingInfo");
    RETURN_IF_POINTER_NULL(strDesc);
    cObjectPtr<IMediaType> pType = new cMediaType(0, 0, 0, "tCrossingInfo", strDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescCrossing));

    RETURN_IF_FAILED(m_inputPin_crossingType.Create("crossingInfo", new cMediaType(0, 0, 0, "tCrossingInfo")  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_crossingType));

    tChar const * strDescRoadSign = pDescManager->GetMediaDescription("tRoadSign");
    RETURN_IF_POINTER_NULL(strDescRoadSign);
    cObjectPtr<IMediaType> pTypeRoadSign = new cMediaType(0, 0, 0, "tRoadSign", strDescRoadSign, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(m_inputPin_rightOfWay.Create("Road_Sign", pTypeRoadSign, this));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_rightOfWay));
    RETURN_IF_FAILED(pTypeRoadSign->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescRoadSign));

    RETURN_IF_FAILED(m_inputPin_distToCrossing.Create("distToCrossing", new cMediaType(0, 0, 0, "tSignalValue")  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_distToCrossing));


    tChar const * strDescMotionData = pDescManager->GetMediaDescription("tMotionDataExt");
    RETURN_IF_POINTER_NULL(strDescMotionData);
    cObjectPtr<IMediaType> pTypeMotionData = new cMediaType(0, 0, 0, "tMotionDataExt", strDescMotionData,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeMotionData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescMotionData));

    RETURN_IF_FAILED(m_inputPin_motionData.Create("motionData", pTypeMotionData  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_motionData));

    RETURN_IF_FAILED(m_inputPin_ObstacleGrid.Create("obstacle_grid", adtf::IPin::PD_Input  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_ObstacleGrid));

    RETURN_IF_FAILED(m_inputPin_DepthGrid.Create("depth_grid", adtf::IPin::PD_Input  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_DepthGrid));


    tChar const * strDescParking = pDescManager->GetMediaDescription("tParkingData");
    RETURN_IF_POINTER_NULL(strDescParking);
    cObjectPtr<IMediaType> pTypeParking = new cMediaType(0, 0, 0, "tParkingData", strDescParking, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeParking->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescParking));

    RETURN_IF_FAILED(m_inputPin_distToParking.Create("DistToLot", new cMediaType(0, 0, 0, "tParkingData")  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_distToParking));

	RETURN_NOERROR;
}

tResult SegmentDriver::CreateOutputPins(__exception)
{ 
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);

    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal_send));

    pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal_receive_yaw));
    pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal_receive_maneuver));
    pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal_receive_pd));

    RETURN_IF_FAILED(m_outputPin_LFenable.Create("LF_enable", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_LFenable));

    RETURN_IF_FAILED(m_outputPin_PSenable.Create("PS_enable", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_PSenable));

    RETURN_IF_FAILED(m_outputPin_SDenable.Create("SD_enable", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_SDenable));

    RETURN_IF_FAILED(m_outputPin_CDenable.Create("CD_enable", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_CDenable));

    RETURN_IF_FAILED(m_outputPin_PMenable.Create("PM_enable", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_PMenable));

    RETURN_IF_FAILED(m_outputPin_PDenable.Create("PD_enable", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_PDenable));

    RETURN_IF_FAILED(m_outputPin_OTenable.Create("OT_enable", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_OTenable));

    RETURN_IF_FAILED(m_outputPin_HSenable.Create("HS_enable", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_HSenable));

    RETURN_IF_FAILED(m_outputPin_segmentSpeed.Create("segmentSpeed", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_segmentSpeed));

    RETURN_IF_FAILED(m_outputPin_segmentSteering.Create("segmentSteering", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_segmentSteering));

    RETURN_IF_FAILED(m_outputPin_maneuverFinished.Create("maneuverFinished", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_maneuverFinished));

    RETURN_IF_FAILED(m_outputPin_DebugGrid.Create("debug_obstacle_grid", adtf::IPin::PD_Output  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_DebugGrid));

    tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(strDescBoolSignalValue);
    cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescBoolSignal));

    RETURN_IF_FAILED(m_outputPin_turn_left_enabled.Create("blinker_left", pTypeBoolSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_turn_left_enabled));
    RETURN_IF_FAILED(m_outputPin_turn_right_enabled.Create("blinker_right", pTypeBoolSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_turn_right_enabled));
    RETURN_IF_FAILED(m_outputPin_hazzard_enabled.Create("hazzard_light", pTypeBoolSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_hazzard_enabled));

    RETURN_NOERROR;
}

tResult SegmentDriver::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
	{
		CreateInputPins(__exception_ptr);
		CreateOutputPins(__exception_ptr);
	}
    else if (eStage == StageNormal)
    {
        LOG_INFO("In Init-Normal");

        LoadManeuverScripts(GetPropertyStr("Configurationfile"));
        printScript(m_Scripts);

        const tBitmapFormat* pSourceFormat = m_inputPin_ObstacleGrid.GetFormat();
        if (pSourceFormat != NULL)
        {
            // copy it
            memcpy(&m_sBitmapFormatGrid, pSourceFormat, sizeof(tBitmapFormat));
        }

        // if the source format is currently not set, initialize it with default values.
        if (m_sBitmapFormatGrid.nWidth == 0 || m_sBitmapFormatGrid.nHeight == 0)
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
        m_ObstacleGrid = *(new Mat(m_sBitmapFormatGrid.nHeight,m_sBitmapFormatGrid.nWidth,CV_8UC1));
        m_DepthGrid = *(new Mat(m_sBitmapFormatGrid.nHeight,m_sBitmapFormatGrid.nWidth,CV_8UC1));


        // if the source format is currently not set, initialize it with default values.
        m_sDebugFormatGrid.nWidth = TILE_COUNT_LEFT+TILE_COUNT_RIGHT;
        m_sDebugFormatGrid.nHeight = TILE_COUNT_FRONT+TILE_COUNT_REAR;
        m_sDebugFormatGrid.nBitsPerPixel = 24;
        m_sDebugFormatGrid.nPixelFormat = cImage::PF_RGB_888;
        m_sDebugFormatGrid.nBytesPerLine = m_sDebugFormatGrid.nWidth * 3;
        m_sDebugFormatGrid.nSize = m_sDebugFormatGrid.nBytesPerLine*m_sDebugFormatGrid.nHeight;
        m_sDebugFormatGrid.nPaletteSize = 0;

        m_outputPin_DebugGrid.SetFormat(&m_sDebugFormatGrid, NULL);
        m_DebugGrid = *(new Mat(m_sDebugFormatGrid.nHeight,m_sDebugFormatGrid.nWidth,CV_8UC3, Scalar(0,0,0)));		//DEBUG

        pthread_mutex_init(&m_orientation_mux, NULL);// PTHREAD_MUTEX_INITIALIZER;
        pthread_mutex_init(&m_dist_wait_mux, NULL);
        pthread_mutex_init(&m_crossing_wait_mux, NULL);
        pthread_mutex_init(&m_sign_wait_mux , NULL);
        pthread_mutex_init(&m_maneuver_wait_mux, NULL);
        pthread_mutex_init(&m_parking_wait_mux, NULL);
        pthread_mutex_init(&m_parking_direction_wait_mux, NULL);
        pthread_mutex_init(&m_grid_wait_mux, NULL);
        pthread_mutex_init(&m_orientation_mux, NULL);
        pthread_mutex_init(&m_dist_mux , NULL);
        pthread_mutex_init(&m_crossing_mux , NULL);
        pthread_mutex_init(&m_sign_mux , NULL);
        pthread_mutex_init(&m_grid_mux , NULL);
        pthread_mutex_init(&m_maneuver_mux, NULL);
        pthread_mutex_init(&m_parking_mux, NULL);
        pthread_mutex_init(&m_parking_direction_mux, NULL);


        pthread_mutex_lock(&m_maneuver_wait_mux);

        m_state = STATE_IDLE;

        m_gridScale = GRID_CM_PER_UNIT;

        m_ManeuverFinished = true;
        m_ManeuverRequested = false;
        m_LastCurv = 100000;
        m_LastSpeed = 0;
        m_Orientation = 0;

        m_ManeuverIdx = 100;
        m_StopFilterRequested = false;

        m_DebugEnable = true;
        m_ParkingType = 0;

        m_SignDetectionCounter = 3;
        m_PDDetectionCounter = 30;
    }

    RETURN_NOERROR;
}


tResult SegmentDriver::Start(__exception)
{    
    LOG_INFO("In Start");
    sched_param param;
    param.sched_priority = 0;

    m_StopFilterRequested = false;

    pthread_create(&m_Thread, NULL, Cycle, this);

    pthread_setschedparam(m_Thread, SCHED_FIFO, &param);

    return cFilter::Start(__exception_ptr);
}

tResult SegmentDriver::Stop(__exception)
{
    LOG_INFO("In Stop");
    // Give all mux free
    m_StopFilterRequested = true;

    pthread_mutex_unlock(&m_orientation_mux);
    pthread_mutex_unlock(&m_grid_mux);
    pthread_mutex_unlock(&m_dist_mux);
    pthread_mutex_unlock(&m_sign_mux);
    pthread_mutex_unlock(&m_crossing_mux);
    pthread_mutex_unlock(&m_maneuver_mux);
    pthread_mutex_unlock(&m_parking_mux);
    pthread_mutex_unlock(&m_parking_direction_mux);


    pthread_mutex_unlock(&m_orientation_wait_mux);
    pthread_mutex_unlock(&m_grid_wait_mux);
    pthread_mutex_unlock(&m_dist_wait_mux);
    pthread_mutex_unlock(&m_sign_wait_mux);
    pthread_mutex_unlock(&m_crossing_wait_mux);
    pthread_mutex_unlock(&m_maneuver_wait_mux);
    pthread_mutex_unlock(&m_parking_wait_mux);
    pthread_mutex_unlock(&m_parking_direction_wait_mux);

    pthread_join(m_Thread, (void**) &m_ThreadReturn);

    return cFilter::Stop(__exception_ptr);
}

tResult SegmentDriver::Shutdown(tInitStage eStage, __exception)
{
    if(eStage == StageNormal)
    {
        LOG_INFO("In Shutdown");
        pthread_mutex_destroy(&m_orientation_mux);
        pthread_mutex_destroy(&m_grid_mux);
        pthread_mutex_destroy(&m_dist_mux);
        pthread_mutex_destroy(&m_sign_mux);
        pthread_mutex_destroy(&m_crossing_mux);
        pthread_mutex_destroy(&m_maneuver_mux);
        pthread_mutex_destroy(&m_parking_mux);
        pthread_mutex_destroy(&m_parking_direction_mux);


        pthread_mutex_destroy(&m_orientation_wait_mux);
        pthread_mutex_destroy(&m_grid_wait_mux);
        pthread_mutex_destroy(&m_dist_wait_mux);
        pthread_mutex_destroy(&m_sign_wait_mux);
        pthread_mutex_destroy(&m_crossing_wait_mux);
        pthread_mutex_destroy(&m_maneuver_wait_mux);
        pthread_mutex_destroy(&m_parking_wait_mux);
        pthread_mutex_destroy(&m_parking_direction_wait_mux);

    }

    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult SegmentDriver::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    if (pMediaSample != NULL && m_pCoderDescSignal_send != NULL)
    {
        tUInt32 timeStamp;

        if(pSource == &m_inputPin_ObstacleGrid)
        {
            adtf_util::cImage oImage;
            __sample_read_lock(pMediaSample, tUInt8, pSrcBuffer);
            oImage.SetBits((tUInt8*) pSrcBuffer, &m_sBitmapFormatGrid);

            pthread_mutex_lock(&m_grid_mux);
            memcpy(m_ObstacleGrid.data,oImage.GetBitmap(),(int)m_ObstacleGrid.total());
                //m_ObstacleGrid.data = oImage.GetBitmap();
            pthread_mutex_unlock(&m_grid_mux);

            // Notify waiting threads about grid change:
            pthread_mutex_unlock(&m_grid_wait_mux);
        }
        if(pSource == &m_inputPin_DepthGrid)
        {
            adtf_util::cImage m_oImageDepth;
            __sample_read_lock(pMediaSample, tUInt8, pSrcBuffer);
            m_oImageDepth.SetBits((tUInt8*) pSrcBuffer, &m_sBitmapFormatGrid);

            pthread_mutex_lock(&m_grid_mux);
                memcpy(m_DepthGrid.data,m_oImageDepth.GetBitmap(),(int)m_DepthGrid.total());
            pthread_mutex_unlock(&m_grid_mux);

            // Notify waiting threads about grid change:
            pthread_mutex_unlock(&m_grid_wait_mux);
        }
        if(pSource == &m_inputPin_rightOfWay)
        {
            ProcessSignUpdate(pMediaSample);
            //ReceiveSignalValueMessage(pMediaSample, &value, &timeStamp);
        }

        if(pSource == &m_inputPin_state)
            RETURN_NOERROR;

        if(pSource == &m_inputPin_maneuver)
        {
            tFloat32 value;
            ReceiveSignalValueMessage(pMediaSample, m_pCoderDescSignal_receive_maneuver, &value, &timeStamp);

            LOG_INFO("New maneuver requested");
            RequestNewManeuver(value);

        }
        else if(pSource == &m_inputPin_motionData)
        {
            tFloat32 dY, dPhi;
            ReceiveMotionDataMessage(pMediaSample, &dY, &dPhi, &timeStamp);

            UpdateOrientation(-1 * dPhi * 360/2/PI);
            UpdateDrivenDist(abs(dY));
        }
        else if(pSource == &m_inputPin_parkingDirection)
        {
            tFloat32 value;
            ReceiveSignalValueMessage(pMediaSample, m_pCoderDescSignal_receive_pd, &value, &timeStamp);

            UpdateParkingDirection(value);

        }
        else if(pSource == &m_inputPin_crossingType)
        {
            tFloat32 distToCrossing;
            tFloat32 orientation;
            tInt8 type;
            tUInt32 timestamp;
            ReceiveCrossingDataMessage(pMediaSample,&distToCrossing, &orientation, &type);

            UpdateCrossingInfo(distToCrossing, orientation, type);
        }
        else if(pSource == &m_inputPin_distToParking)
        {
            tFloat32 dist,angle;
            tInt32 lotID;
            ReceiveParkingMessage(pMediaSample, &dist, &angle, &lotID,&timeStamp);

            UpdateParkingDist(dist,angle,lotID);
        }
    }

	RETURN_NOERROR;
}

tResult SegmentDriver::RequestNewManeuver(tFloat32 newManeuver)
{
    pthread_mutex_lock(&m_maneuver_mux);
        m_ManeuverRequested = true;
        m_RequestedManeuverID = newManeuver;
    pthread_mutex_unlock(&m_maneuver_mux);

    pthread_mutex_unlock(&m_orientation_wait_mux);
    pthread_mutex_unlock(&m_grid_wait_mux);
    pthread_mutex_unlock(&m_dist_wait_mux);
    pthread_mutex_unlock(&m_sign_wait_mux);
    pthread_mutex_unlock(&m_crossing_wait_mux);
    pthread_mutex_unlock(&m_parking_wait_mux);

    // Notify waiting threads about orientation change:
    pthread_mutex_unlock(&m_maneuver_wait_mux);

    RETURN_NOERROR;
}


tResult SegmentDriver::UpdateOrientation(tFloat32 dOrientation)
{
    pthread_mutex_lock(&m_orientation_mux);
        m_Orientation += dOrientation;
        //cout << m_Orientation << " " << dOrientation << endl;
    pthread_mutex_unlock(&m_orientation_mux);

    // Notify waiting threads about orientation change:
    pthread_mutex_unlock(&m_orientation_wait_mux);
    RETURN_NOERROR;
}

tResult SegmentDriver::UpdateDrivenDist(tFloat32 dDist)
{
    pthread_mutex_lock(&m_dist_mux);
        m_DrivenDist += dDist;
    pthread_mutex_unlock(&m_dist_mux);

    // Notify waiting threads about driven-dist change:
    pthread_mutex_unlock(&m_dist_wait_mux);
    RETURN_NOERROR;
}

tResult SegmentDriver::UpdateCrossingInfo(tFloat32 dDist, tFloat32 dOrientation, tInt8 iType)
{
    pthread_mutex_lock(&m_crossing_mux);
        m_CrossingDist = dDist;
        m_CrossingOrientation = dOrientation;
        m_CrossingType = iType;
    pthread_mutex_unlock(&m_crossing_mux);

    //cout << "crossing dist " << dDist <<  endl;

    // Notify waiting threads about crossing-dist change:
    pthread_mutex_unlock(&m_crossing_wait_mux);
    RETURN_NOERROR;
}

tResult SegmentDriver::UpdateParkingDist(tFloat32 dDist, tFloat32 dAngle, tInt32 lotID)
{
    pthread_mutex_lock(&m_parking_mux);
        m_ParkingDist = dDist;
        m_ParkingAngle = dAngle;
        m_ParkingLotID = lotID;
    pthread_mutex_unlock(&m_parking_mux);

    // Notify waiting threads about parking-dist change:
    pthread_mutex_unlock(&m_parking_wait_mux);
    RETURN_NOERROR;
}

tResult SegmentDriver::UpdateParkingDirection(tFloat32 dDirection)
{
    pthread_mutex_lock(&m_parking_direction_mux);

        if(m_CurrPD != dDirection)  // New direction? Then set detection counter back to threshold
        {
            m_CurrPD = dDirection;
            m_PDDetectionCounter = 30;
            pthread_mutex_unlock(&m_parking_direction_mux);
        }
        else if(m_PDDetectionCounter > 0)  // Same direction? Was it already n-times detected
        {
            m_PDDetectionCounter--;
            pthread_mutex_unlock(&m_parking_direction_mux);
        }
        else
        {
            pthread_mutex_unlock(&m_parking_direction_mux);

            // Notify waiting threads about parking-direction change:
            pthread_mutex_unlock(&m_parking_direction_wait_mux);
        }
    RETURN_NOERROR;
}

tResult SegmentDriver::ProcessSignUpdate(IMediaSample* pMediaSample)
{
    tInt8 ID = 0;
    tFloat32 TransX = 0;
    tFloat32 TransY = 0;
    tFloat32 TransZ = 0;
    tFloat32 RotX = 0;
    tFloat32 RotY = 0;
    tFloat32 RotZ = 0;


    ReceiveRoadSignMessage(pMediaSample, &ID, &TransX, &TransY, &TransZ, &RotX, &RotY, &RotZ);

    if(ID)
    {
        pthread_mutex_lock(&m_sign_mux);
            m_DistToSign = TransZ;

            if(m_CurrSignID != ID)  // New sign? Then set detection counter back to threshold
            {
                m_CurrSignID = ID;
                m_SignDetectionCounter = 1;
                pthread_mutex_unlock(&m_sign_mux);
            }
            else if(m_SignDetectionCounter > 0)  // Same sign? Was it already n-times detected
            {
                m_SignDetectionCounter--;
                pthread_mutex_unlock(&m_sign_mux);
            }
            else
            {
                pthread_mutex_unlock(&m_sign_mux);

                // Notify waiting threads about sign-dist change:
                pthread_mutex_unlock(&m_sign_wait_mux);
            }
    }

    RETURN_NOERROR;
}


void* Cycle(void *vp)
{    
    SegmentDriver *that = (SegmentDriver*) vp;

    while(!that->m_StopFilterRequested)
    {
        // Check for new maneuver
        that->LoadManeuver();

        // Do the current script operation
        if(IS_FAILED(that->ScriptMapper()))
            continue;

        that->NextOperation();
    }
}

tResult SegmentDriver::NextOperation()
{
    m_ScriptIdx++;
    if(m_ManeuverIdx >= m_Scripts.size() ||  m_ScriptIdx >= m_Scripts[m_ManeuverIdx]->ops.size())
    {
        pthread_mutex_lock(&m_maneuver_mux);
            m_ManeuverFinished = true;
        pthread_mutex_unlock(&m_maneuver_mux);
            SendSignalValueMessage(&m_outputPin_maneuverFinished, 1, 0);

            if(m_ManeuverIdx < m_Scripts.size())
            {
                if(m_Scripts[m_ManeuverIdx]->id == ACTION_CROSS_PARKING)
                {
                    m_ParkingType = 1;
                }
                else if(m_Scripts[m_ManeuverIdx]->id == ACTION_PARALLEL_PARKING)
                {
                    m_ParkingType = 2;
                }
            }

        // Wait for next maneuver:
        LOG_INFO("Wait for next maneuver");
        pthread_mutex_lock(&m_maneuver_wait_mux); // Wait for next orientation change
        LOG_INFO("Wake-up for next maneuver");
    }
    RETURN_NOERROR;
}

tResult SegmentDriver::LoadManeuver()
{
    pthread_mutex_lock(&m_maneuver_mux);

        if(m_ManeuverRequested)
        {
            m_ManeuverRequested = false;
            m_ManeuverFinished = false;

            SetIndicator(0,0);
            SetIndicator(1,0);
            SetIndicator(2,0);
            LaneFollowingEnable(0);
            HighSpeedEnable(0);
            SensorsEnable(0);
            SendSignalValueMessage(&m_outputPin_CDenable, 0, 0);
            SendSignalValueMessage(&m_outputPin_SDenable, 0, 0);
            SendSignalValueMessage(&m_outputPin_PSenable, 0, 0);


            if(m_RequestedManeuverID == ACTION_PULL_OUT_RIGHT)
            {
                m_ParkingType = 0;
                if(m_ParkingType == 0)
                    DetectParkingLotType();

                if(m_ParkingType == 1)
                {
                    m_RequestedManeuverID = ACTION_PULL_OUT_RIGHT_CP;
                    LOG_INFO("This is a cross parking lot");
                }
                else if(m_ParkingType == 2)
                {
                    m_RequestedManeuverID = ACTION_PULL_OUT_RIGHT_PP;
                    LOG_INFO("This is a parallel parking lot");
                }
                else
                {
                    m_RequestedManeuverID = ACTION_STOP;
                    LOG_INFO("No parking lot detected");
                }
            }

            m_ManeuverIdx = 100;
            for(int i = 0; i < m_Scripts.size(); i++)
            {
                if(m_Scripts[i] != NULL && m_Scripts[i]->id == m_RequestedManeuverID)
                {
                    m_ManeuverIdx = i;
                    break;
                }
            }

            m_ScriptIdx = 0;
        }

    pthread_mutex_unlock(&m_maneuver_mux);

    m_LastSpeed = -1000;
    m_LastCurv = -1000;

    RETURN_NOERROR;
}

tResult SegmentDriver::DetectParkingLotType()
{
    tInt parkingTypeSuggestion = 0;

    SendSignalValueMessage(&m_outputPin_PDenable, 1, 0);
    LOG_INFO("Type detection started");
    pthread_mutex_lock(&m_parking_direction_mux);
        m_PDDetectionCounter = 30;
    pthread_mutex_unlock(&m_parking_direction_mux);

    while(!m_StopFilterRequested && !m_ManeuverRequested)
    {
        pthread_mutex_lock(&m_parking_direction_mux);
            if(m_PDDetectionCounter <= 0)
            {
                parkingTypeSuggestion = m_CurrPD;
                pthread_mutex_unlock(&m_parking_direction_mux);
                break;
            }
        pthread_mutex_unlock(&m_parking_direction_mux);

            pthread_mutex_lock(&m_parking_direction_wait_mux); // Wait for next parking_direction change
    }

    SendSignalValueMessage(&m_outputPin_PDenable, 0, 0);

    vector<Point2f> ppt;
    ppt.clear();
    ppt.push_back(m_IR_L);
    ppt.push_back(m_IR_R);
    ppt.push_back(m_IR_R + Point2f(0,30));
    ppt.push_back(m_IR_L + Point2f(0,30));

    tFloat32 errVal = 0;
    CalcErrorValue(ppt, 168, &errVal, USE_SENSOR_ONLY);

    if (errVal > TEST_MAX_ERROR_VALUE)
    {
        m_ParkingType = 2;
    }
    else
    {
        m_ParkingType = parkingTypeSuggestion;
    }

    RETURN_NOERROR;
}

tResult SegmentDriver::ScriptMapper()
{
    if(m_ManeuverIdx < m_Scripts.size() && m_ScriptIdx < m_Scripts[m_ManeuverIdx]->ops.size())
    {
        printOP(m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->op);

        switch(m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->op)
        {
        case NOP:
            break;

        case SET_CURV:
            return SetCurv(m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->args[0]);

        case SET_SPEED:
            return SetSpeed(m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->args[0]);

        case WAITFORORIENTATION:
            return WaitForOrientation(m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->args[0]);

        case CHECKROW:
            return CheckRightOfWay(m_Scripts[m_ManeuverIdx]->id, m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->args[0], m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->args[1], m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->args[2], m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->args[3],m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->args[4]);

        case WAITFORDISTREAR:
            return WaitForDistRear(m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->args[0], m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->args[1]);

        case SEARCHPARKINGLOT:
            return SearchParkingLot(m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->args[0], m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->args[1], m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->args[2], m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->args[3]);

        case WAITFORDISTDRIVEN:
            return WaitForDistDriven(m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->args[0]);

        case WAITFORDISTCROSSING:
            return WaitForDistCrossing(m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->args[0], m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->args[1]);

        case WAITFORTIME:
            return WaitForTime(m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->args[0]);

        case LFENABLE:
            return LaneFollowingEnable(m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->args[0]);

        case WAITFORSIGN:
            return WaitForSign(m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->args[0]);

        case SETINDICATOR:
            return SetIndicator(m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->args[0], m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->args[1]);

        case INIT_ORIENTATION:
            return InitOrientation(m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->args[0]);

        case CHECKOT:
            return CheckOncomingTraffic();

        case HSENABLE:
            return HighSpeedEnable(m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->args[0]);

        case IGNORESENS:
            return SensorsEnable(m_Scripts[m_ManeuverIdx]->ops[m_ScriptIdx]->args[0]);

        default:
            break;
        }
    }
    RETURN_NOERROR;
}

tResult SegmentDriver::SetCurv(tFloat32 curv)
{
    if(m_LastCurv != curv)
    {
        m_LastCurv = curv;
        SendSignalValueMessage(&m_outputPin_segmentSteering, curv, 0);
        WaitForTime(0.015);
        SendSignalValueMessage(&m_outputPin_segmentSteering, curv, 0);
    }
    RETURN_NOERROR;
}

tResult SegmentDriver::SetSpeed(tFloat32 speed)
{
    if(m_LastSpeed != speed)
    {
        m_LastSpeed = speed;
        SendSignalValueMessage(&m_outputPin_segmentSpeed, speed, 0);
    }
    RETURN_NOERROR;
}

tResult SegmentDriver::SetIndicator(tFloat32 id, tFloat32 val)
{
    if(id == 0)
        SendBoolSignalValueMessage(&m_outputPin_turn_left_enabled, (val == 1), 0);
    else if(id == 1)
        SendBoolSignalValueMessage(&m_outputPin_turn_right_enabled, (val == 1), 0);
    else
        SendBoolSignalValueMessage(&m_outputPin_hazzard_enabled, (val == 1), 0);

    RETURN_NOERROR;
}

tResult SegmentDriver::InitOrientation(tFloat32 useCrossingOrientation)
{
    pthread_mutex_lock(&m_orientation_mux);
        pthread_mutex_lock(&m_crossing_mux);
            if(useCrossingOrientation == 1 && m_CrossingDist < 1000)
                m_Orientation = m_CrossingOrientation;
            else
                m_Orientation = 0;
        pthread_mutex_unlock(&m_crossing_mux);
    pthread_mutex_unlock(&m_orientation_mux);

    RETURN_NOERROR;
}

tResult SegmentDriver::WaitForOrientation(tFloat32 orientation)
{
    tFloat32 currentOrientation = 0;
    tFloat32 lastOrientation = 0;

    pthread_mutex_lock(&m_orientation_mux);
        currentOrientation = m_Orientation;
        lastOrientation = m_Orientation;
    pthread_mutex_unlock(&m_orientation_mux);

    while(!m_StopFilterRequested && !m_ManeuverRequested)
    {
        lastOrientation = currentOrientation;
        pthread_mutex_lock(&m_orientation_mux);
            currentOrientation = m_Orientation;
        pthread_mutex_unlock(&m_orientation_mux);

        if((lastOrientation <= orientation && orientation <= currentOrientation) || (currentOrientation <= orientation && orientation <= lastOrientation))
            RETURN_NOERROR;
        else
            pthread_mutex_lock(&m_orientation_wait_mux); // Wait for next orientation change
    }
    RETURN_ERROR(1);
}

tResult SegmentDriver::WaitForDistRear(tFloat32 dist, tFloat32 delta)
{
    while(!m_StopFilterRequested && !m_ManeuverRequested)
    {
        pthread_mutex_lock(&m_grid_mux);
            //LineIterator it(m_ObstacleGrid, p1,p2);

        int x = 0;
        int y = TILE_COUNT_REAR - (12);
        for(; y >= 0; y--)
        {
            tUInt8 last = m_ObstacleGrid.at<tUInt8>(Point(TILE_COUNT_RIGHT - (CARSIZE_CROSS) / GRID_CM_PER_UNIT / 2, y));
            if(last < 168)
                break;
        }
        pthread_mutex_unlock(&m_grid_mux);

        if(dist-delta <= x && x <= dist+delta)
            RETURN_NOERROR;
        else
            pthread_mutex_lock(&m_grid_wait_mux); // Wait for next grid change
    }
    RETURN_ERROR(1);
}

tResult SegmentDriver::WaitForDistDriven(tFloat32 dist)
{
    tFloat32 lastDrivenDist = 0;

    // Set driven-dist to 0
    pthread_mutex_lock(&m_dist_mux);
        m_DrivenDist = 0;
    pthread_mutex_unlock(&m_dist_mux);

    while(!m_StopFilterRequested && !m_ManeuverRequested)
    {
        pthread_mutex_lock(&m_dist_mux);
            lastDrivenDist = m_DrivenDist;
        pthread_mutex_unlock(&m_dist_mux);

        if(dist <= lastDrivenDist)
            RETURN_NOERROR;
        else
            pthread_mutex_lock(&m_dist_wait_mux); // Wait for next dist change
    }
    RETURN_ERROR(1);
}

tResult SegmentDriver::WaitForDistCrossing(tFloat32 dist, tFloat32 planB)
{
    tFloat32 lastCrossingDist = 0;
    tFloat32 lastDrivenDist = 0;
    SendSignalValueMessage(&m_outputPin_CDenable, 1, 0);
    SendSignalValueMessage(&m_outputPin_SDenable, 1, 0);

    pthread_mutex_lock(&m_crossing_mux);
        m_CrossingDist = 1234;
    pthread_mutex_unlock(&m_crossing_mux);

    pthread_mutex_lock(&m_dist_mux);
        pthread_mutex_lock(&m_sign_mux);
            if(m_SignDetectionCounter <= 0)
                m_DrivenDist = -100 * m_DistToSign + planB;
            else
                m_DrivenDist = -1 * planB;
        pthread_mutex_unlock(&m_sign_mux);
    pthread_mutex_unlock(&m_dist_mux);


    while(!m_StopFilterRequested && !m_ManeuverRequested)
    {
        pthread_mutex_lock(&m_crossing_mux);
            lastCrossingDist = m_CrossingDist;
        pthread_mutex_unlock(&m_crossing_mux);

        pthread_mutex_lock(&m_dist_mux);
            lastDrivenDist = m_DrivenDist;
        pthread_mutex_unlock(&m_dist_mux);

        if(dist <= lastDrivenDist)
            RETURN_NOERROR;

        if(dist >= lastCrossingDist)
        {
            SendSignalValueMessage(&m_outputPin_SDenable, 0, 0);
            SendSignalValueMessage(&m_outputPin_CDenable, 0, 0);
            RETURN_NOERROR;
        }
        else if(0 <= lastDrivenDist)
        {
            LOG_INFO("Plan-B!");
            SendSignalValueMessage(&m_outputPin_SDenable, 0, 0);
            SendSignalValueMessage(&m_outputPin_CDenable, 0, 0);
            RETURN_NOERROR;
        }
        else
            pthread_mutex_lock(&m_crossing_wait_mux); // Wait for next crossing change
    }
    SendSignalValueMessage(&m_outputPin_SDenable, 0, 0);
    SendSignalValueMessage(&m_outputPin_CDenable, 0, 0);
    RETURN_ERROR(1);
}

tResult SegmentDriver::WaitForSign(tFloat32 dist)
{
    tFloat32 lastSignDist = 0;
    tUInt8 lastSignID = 0;

    SendSignalValueMessage(&m_outputPin_SDenable, 1, 0);

    pthread_mutex_lock(&m_sign_mux);
        m_SignDetectionCounter = 1;
    pthread_mutex_unlock(&m_sign_mux);

    while(!m_StopFilterRequested && !m_ManeuverRequested)
    {
        pthread_mutex_lock(&m_sign_mux);
            if(m_SignDetectionCounter <= 0)
            {
                lastSignDist = m_DistToSign;
                lastSignID = m_CurrSignID;
            }
            else
            {
                lastSignDist = 100000000;
                lastSignID = 0;
            }
        pthread_mutex_unlock(&m_sign_mux);

        if(lastSignID > 0 && lastSignDist <= dist && (((m_Scripts[m_ManeuverIdx]->id == ACTION_CROSS_PARKING || m_Scripts[m_ManeuverIdx]->id == ACTION_PARALLEL_PARKING) && lastSignID == 4) || (m_Scripts[m_ManeuverIdx]->id != ACTION_CROSS_PARKING && lastSignID != 4)))
        {
            SendSignalValueMessage(&m_outputPin_SDenable, 0, 0);
            RETURN_NOERROR;
        }
        else
            pthread_mutex_lock(&m_sign_wait_mux); // Wait for next crossing change
    }
    SendSignalValueMessage(&m_outputPin_SDenable, 0, 0);
    RETURN_ERROR(1);
}

tResult SegmentDriver::LaneFollowingEnable(tFloat32 value)
{
    SendSignalValueMessage(&m_outputPin_LFenable, value, 0);

    SendSignalValueMessage(&m_outputPin_OTenable, value, 0);

    RETURN_NOERROR;
}

tResult SegmentDriver::HighSpeedEnable(tFloat32 value)
{
    SendSignalValueMessage(&m_outputPin_HSenable, value, 0);

    RETURN_NOERROR;
}

tResult SegmentDriver::SensorsEnable(tFloat32 value)
{
    SendSignalValueMessage(&m_outputPin_PMenable, value, 0);

    RETURN_NOERROR;
}


tResult SegmentDriver::WaitForTime(tFloat32 duration)
{
    tTimeStamp m_StopTimeStart = GetTime() + duration * 1000000;

    while(GetTime() < m_StopTimeStart)
    {
        if(m_ManeuverRequested || m_StopFilterRequested)
            RETURN_ERROR(1);
    }
    RETURN_NOERROR;
}
inline void Convert_World2Grid(vector<Point2f>& points){
    for(int i = 0;i < points.size();i++){
        points[i].x = TILE_COUNT_RIGHT - ( points[i].x) / GRID_CM_PER_UNIT ;
        points[i].y =  TILE_COUNT_REAR + (points[i].y) / GRID_CM_PER_UNIT;
    }
}

tResult SegmentDriver::SearchParkingLot(tFloat32 dist,tFloat32 lotLength, tFloat32 lotWidth, tFloat32 parkingType)
{
    tFloat32 lastParkingDist = 0;
    tInt32 CurrLotID = 1;

    SendSignalValueMessage(&m_outputPin_PSenable, parkingType, 0);
    SendSignalValueMessage(&m_outputPin_OTenable, 0, 0);

    pthread_mutex_lock(&m_parking_wait_mux); // Wait for next parking-dist change
    pthread_mutex_lock(&m_parking_wait_mux); // Wait for next parking-dist change
    //LOG_INFO(cString::Format("filterrequest %b",m_StopFilterRequested));

    while(!m_StopFilterRequested && !m_ManeuverRequested)
    {
        pthread_mutex_lock(&m_parking_mux);
            lastParkingDist = m_ParkingDist;
            //LOG_INFO(cString::Format("%f %f", lastParkingDist, dist));
        pthread_mutex_unlock(&m_parking_mux);

#ifdef DEBUG
        pthread_mutex_lock(&m_grid_mux);
        cvtColor(m_ObstacleGrid,m_DebugGrid,CV_GRAY2RGB);
        pthread_mutex_unlock(&m_grid_mux);
#endif

        if((CurrLotID >= 5 && parkingType == 1) || (CurrLotID >= 6 && parkingType == 2))
        {
            SendSignalValueMessage(&m_outputPin_PSenable, 0, 0);
//            return ResetSearchParkingLot;
            //SendSignalValueMessage(&m_outputPin_SDenable, 1, 0);
            RETURN_ERROR(1);
        }

        //LOG_INFO(cString::Format("PD received: dist %f   lotid %d   neededid %d    type %.0f",lastParkingDist,m_ParkingLotID,CurrLotID,parkingType));
        if(m_ParkingLotID > CurrLotID)
            CurrLotID = m_ParkingLotID;

        if(lastParkingDist < dist && CurrLotID == m_ParkingLotID)
        {
            tFloat32 distLotX = 20;
            vector<Point2f> ppt;
            ppt.push_back(Point2f(distLotX,lastParkingDist));
            ppt.push_back(Point2f(distLotX,lastParkingDist+lotLength));
            ppt.push_back(Point2f(distLotX+lotWidth,lastParkingDist+lotLength));
            ppt.push_back(Point2f(distLotX+lotWidth,lastParkingDist));

            tFloat32 errVal = 0;
            Convert_World2Grid(ppt);
            CalcErrorValue(ppt, 208, &errVal,USE_SENSOR_ONLY);

#ifdef DEBUG
            putText(m_DebugGrid,cv::String(cString::Format("%d",m_ParkingLotID).GetPtr()),ppt[0],CV_FONT_HERSHEY_PLAIN,1,Scalar(0,0,255));
            putText(m_DebugGrid,cv::String(cString::Format("%f",errVal).GetPtr()),ppt[1],CV_FONT_HERSHEY_PLAIN,1,Scalar(0,0,255));


            if (errVal > TEST_MAX_ERROR_VALUE)
            {
                line(m_DebugGrid,ppt[0],ppt[1],Scalar(0,0,255));
                line(m_DebugGrid,ppt[1],ppt[2],Scalar(0,0,255));
                line(m_DebugGrid,ppt[2],ppt[3],Scalar(0,0,255));
                line(m_DebugGrid,ppt[3],ppt[0],Scalar(0,0,255));

                //drawPoly(ppt, Vec3b(0,0,255));
            }else{
                line(m_DebugGrid,ppt[0],ppt[1],Scalar(0,255,0));
                line(m_DebugGrid,ppt[1],ppt[2],Scalar(0,255,0));
                line(m_DebugGrid,ppt[2],ppt[3],Scalar(0,255,0));
                line(m_DebugGrid,ppt[3],ppt[0],Scalar(0,255,0));
                //drawPoly(ppt, Vec3b(0,255,0));
            }
            cObjectPtr<IMediaSample> pNewDebugSample;
            if (IS_OK(AllocMediaSample(&pNewDebugSample)))
            {
                tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
                pNewDebugSample->Update(tmStreamTime, m_DebugGrid.data, tInt32(m_DebugGrid.rows*m_DebugGrid.cols*3), 0);
                m_outputPin_DebugGrid.Transmit(pNewDebugSample);
            }
#endif

            if(errVal < TEST_MAX_ERROR_VALUE){
                SendSignalValueMessage(&m_outputPin_PSenable, 0, 0);
                //SendSignalValueMessage(&m_outputPin_SDenable, 1, 0);
                RETURN_NOERROR;
            }
            CurrLotID++;
        }
        else
            pthread_mutex_lock(&m_parking_wait_mux); // Wait for next parking-dist change
    }
    SendSignalValueMessage(&m_outputPin_PSenable, 0, 0);
    RETURN_ERROR(1);
}

tResult SegmentDriver::ResetSearchParkingLot(tFloat32 speed)
{
    RETURN_NOERROR;

}

tResult SegmentDriver::PrepareRoW(tFloat32 curv, tFloat32 orientation, tFloat32 delta)
{
    SetCurv(curv);
    WaitForOrientation(orientation);
    SetSpeed(0);
    RETURN_NOERROR;
}

tResult SegmentDriver::EndRoW(tFloat32 curv, tFloat32 orientation, tFloat32 delta)
{
    SetCurv(-curv);
    SetSpeed(25);
    WaitForOrientation(-orientation);
    RETURN_NOERROR;
}

tResult SegmentDriver::CheckRightOfWay(tInt maneuverType, tFloat32 curv, tFloat32 orientation, tFloat32 delta, tFloat32 speed, tFloat32 crossingType)
{
    tUInt8 signID = 0;
//    tUInt8 crossingType = 0;
    tBool careRight = true;
    tBool careFront = true;
    tBool careLeft = true;

    pthread_mutex_lock(&m_crossing_mux);
        switch(m_CrossingType)
        {
        case 0:
        case 1:
            break;
        case 2:
            careFront = false;
            break;
        case 3:
            careRight = false;
            break;
        case 4:
            careLeft = false;
            break;
        }

    pthread_mutex_unlock(&m_crossing_mux);

    pthread_mutex_lock(&m_sign_mux);
        signID = m_CurrSignID;
    pthread_mutex_unlock(&m_sign_mux);

    // Preperations:


    // RoW decisions:
    if(signID == 3)
    {
        SetSpeed(0);
        WaitForTime(3);
        SetSpeed(speed);
    }

    if(maneuverType == ACTION_RIGHT)
    {
        if(signID == 2 || signID == 6 || ((signID == 1 || signID == 3) && !careLeft)) // Nicht nachschauen
        {
            //WaitForDistDriven(10);
            RETURN_NOERROR;
        }
        else
        {
            PrepareRoW(curv, orientation,delta);
            WaitForTraffic(careLeft, false, false);
            SetCurv(0);
            SetSpeed(speed);
            //WaitForDistDriven(20);
            RETURN_NOERROR;
        }
    }
    else if(maneuverType == ACTION_STRAIGHT)
    {
        if(signID == 2 || (signID == 6 && !careRight)) // Nicht nachschauen
        {
            WaitForDistDriven(15);
            RETURN_NOERROR;
        }
        else
        {
            PrepareRoW(curv, orientation,delta);
            WaitForTraffic(careLeft, false, careRight);
            EndRoW(curv, 0,delta);
            RETURN_NOERROR;
        }
    }
    else if(maneuverType == ACTION_LEFT)
    {
        if((signID == 2 && !careFront) || !careRight) // Nicht nachschauen
        {
            WaitForDistDriven(15);
            RETURN_NOERROR;
        }
        else
        {
            PrepareRoW(curv, orientation,delta);
            WaitForTraffic(careLeft && signID != 2,careFront,careRight && signID != 2);
            EndRoW(curv, 0,delta);
            RETURN_NOERROR;
        }
    }
    RETURN_ERROR(1);
}

tResult SegmentDriver::WaitForTraffic(tBool careLeft, tBool careFront, tBool careRight)
{
    tFloat32 phi = 0;
    tFloat32 offsetRight = 0;
    tFloat32 offsetFront = 0;

    tInt iterations = 4;

    tTimeStamp timeMax = GetTime() + 10 * 1000000;

    for(int i = 0; i < iterations && !m_StopFilterRequested && !m_ManeuverRequested; i++)
    {
        while(!m_StopFilterRequested && !m_ManeuverRequested)
        {
            pthread_mutex_lock(&m_orientation_mux);
                phi = m_Orientation / 180 * PI;
            pthread_mutex_unlock(&m_orientation_mux);

            Point2f m = Point2f(cos(-phi+PI/2), sin(-phi+PI/2));
            Point2f n = Point2f(cos(-phi), sin(-phi));

            Point2f X = m_IR_R - Point2f(offsetRight, 0);
            Point2f Y = m_IR_L + Point2f(0, offsetFront);

            Point2f A = intersectLines(X,m,Y,n);
            Point2f B = A + 90*m;
            Point2f C = B + 90*n;

            drawCrossing2Debug(A,B,C);

            bool noTraffic = true;
            if(careLeft)
                noTraffic &= CheckLeft();
            if(careFront)
                noTraffic &= CheckFront(A,B,C);
            if(careRight)
                noTraffic &= CheckRight(A,B,C);

            sendDebugGrid();

            if(noTraffic || GetTime() > timeMax)
                break;
        }
    }

    if(!m_StopFilterRequested && !m_ManeuverRequested)
        RETURN_NOERROR;
    else
        RETURN_ERROR(1);
}

tResult SegmentDriver::CheckOncomingTraffic()
{
    tFloat32 phi = 0;
    tFloat32 offsetRight = 20;
    tFloat32 offsetFront = -35;
    tBool careFront = true;
    tFloat32 speed = m_LastSpeed;

    tTimeStamp timeMax = GetTime() + 10 * 1000000;

    pthread_mutex_lock(&m_crossing_mux);
        if(m_CrossingType == 2)
            careFront = false;
    pthread_mutex_unlock(&m_crossing_mux);


    if(!careFront)
        RETURN_NOERROR;

    while(!m_StopFilterRequested && !m_ManeuverRequested)
    {
        SetSpeed(0);

        pthread_mutex_lock(&m_orientation_mux);
            phi = m_Orientation / 180 * PI;
        pthread_mutex_unlock(&m_orientation_mux);

        Point2f m = Point2f(cos(-phi+PI/2), sin(-phi+PI/2));
        Point2f n = Point2f(cos(-phi), sin(-phi));

        Point2f X = m_IR_R - Point2f(offsetRight, 0);
        Point2f Y = m_IR_L + Point2f(0, offsetFront);

        Point2f A = intersectLines(X,m,Y,n);
        Point2f B = A + 90*m;
        Point2f C = B + 90*n;

        drawCrossing2Debug(A,B,C);

        bool noTraffic = true;
        if(careFront)
            noTraffic &= CheckFront(A,B,C);

        sendDebugGrid();


        if(noTraffic || GetTime() > timeMax)
            break;
    }

    if(!m_StopFilterRequested && !m_ManeuverRequested)
    {
        SetSpeed(23);
        RETURN_NOERROR;
    }
    else
        RETURN_ERROR(1);
}

bool SegmentDriver::CheckFront(Point2f A, Point2f B, Point2f C)
{
    Point2f m = B-A;
    Point2f n = C-B;

    Point2f I = B + 0.5*n;
    Point2f H = intersectLines(I,n, m_K, m_kl);
    Point2f G = intersectLines(C,m,m_K,m_kl);
    Point2f F = intersectLines(C,m, Point2f(0,500), Point2f(1,0));
    Point2f J = intersectLines(I,m, Point2f(0,500), Point2f(1,0));

    vector<Point2f> ppt;
    ppt.clear();
    ppt.push_back(I);
    ppt.push_back(J);
    ppt.push_back(F);
    ppt.push_back(G);
    ppt.push_back(H);

    tFloat32 errVal = 0;
    CalcErrorValue(ppt, 168, &errVal, USE_DEPTH_ONLY);

    if (errVal > TEST_MAX_ERROR_VALUE)
    {
        drawPoly(ppt, Vec3b(0,0,255));
        return false;
    }
    else
    {
        drawPoly(ppt, Vec3b(0,255,0));
        return true;
    }
}

bool SegmentDriver::CheckRight(Point2f A, Point2f B, Point2f C)
{
    Point2f m = B-A;
    Point2f n = C-B;

    Point2f U = A + 0.5*m;
    Point2f W = intersectLines(B,n,m_K,m_kr);//Y - beta*n;
    Point2f V = intersectLines(U,n,m_K,m_kr);

    vector<Point2f> ppt;
    ppt.clear();
    ppt.push_back(V);
    ppt.push_back(W);
    ppt.push_back(B);
    ppt.push_back(U);

    tFloat32 errVal = 0;
    CalcErrorValue(ppt, 168, &errVal, USE_BOTH);

    if (errVal > TEST_MAX_ERROR_VALUE)
    {
        drawPoly(ppt, Vec3b(0,0,255));
        return false;
    }
    else
    {
        drawPoly(ppt, Vec3b(0,255,0));
        return true;
    }
}

bool SegmentDriver::CheckLeft()
{
    tUInt8 refVal = 168;
    tFloat32 errVal = 0;
    tInt iterations = 10;
    tInt cols;
    tBool free = true;

    pthread_mutex_lock(&m_grid_mux);
        cols = m_ObstacleGrid.cols;
    pthread_mutex_unlock(&m_grid_mux);

    for(tInt j = 0; j < iterations && !m_StopFilterRequested && !m_ManeuverRequested; j++)
    {
        tInt x = m_IR_L.x;
        tInt i = 0;

        for(i = 0; x < cols && i < 140 && !m_StopFilterRequested&& !m_ManeuverRequested; x++, i++)
        {
            tUInt8 val = 0;
            pthread_mutex_lock(&m_grid_mux);
                val = m_ObstacleGrid.at<tUInt8>(Point(x,m_IR_L.y));
            pthread_mutex_unlock(&m_grid_mux);
            errVal += pow(MAX((tFloat32)(refVal - val),0),2.5);
        }

        if(i > 0)
            errVal /= ((tFloat32) i);
        else
            errVal = TEST_MAX_ERROR_VALUE + 1;

        free &= (errVal <= TEST_MAX_ERROR_VALUE);
        if(!free)
            break;

        pthread_mutex_lock(&m_grid_wait_mux); // Wait for next grid update
    }

    if (!free)
    {
        line(m_DebugGrid, m_IR_L, Point(cols-1, m_IR_L.y), Scalar(0,0,255), 1);
        return false;
    }
    else
    {
        line(m_DebugGrid, m_IR_L, Point(cols-1, m_IR_L.y), Scalar(0,255,0), 1);
        return true;
    }

}


tResult SegmentDriver::CalcErrorValue(vector<Point2f> polypoints, tUInt8 refVal, tFloat32 *pErrValue, tInt32 mode)
{ tInt xMax = 0;
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
    xMax = MIN(xMax,m_ObstacleGrid.cols-1);
    yMax = MIN(yMax,m_ObstacleGrid.rows-1);

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
                    tempErrVal = pow(MAX((tFloat32)(refVal - m_ObstacleGrid.at<tUInt8>(Point(col,row))),0),2.5);
                    tempErrValDepth = pow(MAX((tFloat32)(refVal - m_DepthGrid.at<tUInt8>(Point(col,row))),0),2.5);
                }
                else if(mode == USE_DEPTH_ONLY)
                {
                    tempErrValDepth = pow(MAX((tFloat32)(refVal - m_DepthGrid.at<tUInt8>(Point(col,row))),0),2.5);
                    tempErrVal = tempErrValDepth;
                }
                else
                {
                    tempErrVal = pow(MAX((tFloat32)(refVal - m_ObstacleGrid.at<tUInt8>(Point(col,row))),0),2.5);
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


tResult SegmentDriver::SendSignalValueMessage(cObjectPtr<cOutputPin> pOutputPin, tFloat32 value, tUInt32 timeStamp)
{
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleOut;
    AllocMediaSample((tVoid**)&pMediaSampleOut);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSignal_send->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleOut->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    cObjectPtr<IMediaCoder> pCoderOutput;
    m_pCoderDescSignal_send->WriteLock(pMediaSampleOut, &pCoderOutput);

    pCoderOutput->Set("f32Value", (tVoid*)&(value));
    pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
    m_pCoderDescSignal_send->Unlock(pCoderOutput);

    //transmit media sample over output pin
    tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
    pMediaSampleOut->SetTime(tmStreamTime);
    pOutputPin->Transmit(pMediaSampleOut);

    RETURN_NOERROR;
}

tResult SegmentDriver::SendBoolSignalValueMessage(cObjectPtr<cOutputPin> pOutputPin, bool value, tUInt32 timeStamp)
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
    tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
    pMediaSampleOut->SetTime(tmStreamTime);
    pOutputPin->Transmit(pMediaSampleOut);

    RETURN_NOERROR;
}

tResult SegmentDriver::ReceiveSignalValueMessage(IMediaSample* pMediaSample, cObjectPtr<IMediaTypeDescription> pDesc, tFloat32 *pValue, tUInt32 *pTimeStamp)
{
    // read-out the incoming Media Sample
    cObjectPtr<IMediaCoder> pCoderInput;
    RETURN_IF_FAILED(pDesc->Lock(pMediaSample, &pCoderInput));

    *pValue = 0;
    *pTimeStamp = 0;

    //get values from media sample
    pCoderInput->Get("f32Value", (tVoid*)pValue);
    pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)pTimeStamp);
    pDesc->Unlock(pCoderInput);

    RETURN_NOERROR;
}
tResult SegmentDriver::ReceiveParkingMessage(IMediaSample* pMediaSample, tFloat32 *pDist, tFloat32 *pAngle, tInt32 *plotID, tUInt32 *pTimeStamp)
{
    // read-out the incoming Media Sample
    cObjectPtr<IMediaCoder> pCoderInput;
    RETURN_IF_FAILED(m_pCoderDescParking->Lock(pMediaSample, &pCoderInput));

    *pDist = 0;
    *pAngle =0;
    *plotID =0;
    *pTimeStamp = 0;

    //get values from media sample
    pCoderInput->Get("f32dist", (tVoid*)pDist);
    pCoderInput->Get("f32angle",(tVoid*)pAngle);
    pCoderInput->Get("i32lotID",(tVoid*)plotID);
    pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)pTimeStamp);
    m_pCoderDescParking->Unlock(pCoderInput);

    RETURN_NOERROR;
}

tResult SegmentDriver::ReceiveMotionDataMessage(IMediaSample* pMediaSample, tFloat32 *pDY, tFloat32 *pPhi, tUInt32 *pTimeStamp)
{
    // read-out the incoming Media Sample
    cObjectPtr<IMediaCoder> pCoderInput;
    RETURN_IF_FAILED(m_pCoderDescMotionData->Lock(pMediaSample, &pCoderInput));

    *pDY = 0;
    *pPhi = 0;
    *pTimeStamp = 0;

    //get values from media sample
    pCoderInput->Get("f32dY", (tVoid*)pDY);
    pCoderInput->Get("f32dPhi", (tVoid*)pPhi);
    pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)pTimeStamp);
    m_pCoderDescMotionData->Unlock(pCoderInput);

    RETURN_NOERROR;
}

tResult SegmentDriver::ReceiveCrossingDataMessage(IMediaSample* pMediaSample, tFloat32 *pDistToCrossing, tFloat32 *pOrientation, tInt8 *pCrossingType)
{
    // read-out the incoming Media Sample
    cObjectPtr<IMediaCoder> pCoderInput;
    RETURN_IF_FAILED(m_pCoderDescCrossing->Lock(pMediaSample, &pCoderInput));

    *pDistToCrossing = 1234;
    *pCrossingType = 0;
    *pOrientation = 0;
    tUInt32 ts = 0;

    //get values from media sample
    pCoderInput->Get("i8Type", (tVoid*) pCrossingType);
    pCoderInput->Get("f32Distance", (tVoid*) pDistToCrossing);
    pCoderInput->Get("f32Orientation", (tVoid*) pOrientation);
    pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&ts);
    m_pCoderDescCrossing->Unlock(pCoderInput);

    RETURN_NOERROR;
}

tResult SegmentDriver::ReceiveRoadSignMessage(IMediaSample* pMediaSample, tInt8 *pID, tFloat32 *pTransX, tFloat32 *pTransY, tFloat32 *pTransZ, tFloat32 *pRotX, tFloat32 *pRotY, tFloat32 *pRotZ )
{
    // read-out the incoming Media Sample
    cObjectPtr<IMediaCoder> pCoderInput;
    RETURN_IF_FAILED(m_pCoderDescRoadSign->Lock(pMediaSample, &pCoderInput));

    *pID = 0;
    *pTransX = 0;
    *pTransY = 0;
    *pTransZ = 0;
    *pRotX = 0;
    *pRotY = 0;
    *pRotZ = 0;

    //get values from media sample
    pCoderInput->Get("i8Identifier", (tVoid*) pID);
    pCoderInput->Get("fl32TransX", (tVoid*)pTransX);
    pCoderInput->Get("fl32TransY", (tVoid*)pTransY);
    pCoderInput->Get("fl32TransZ", (tVoid*)pTransZ);
    pCoderInput->Get("fl32RotX", (tVoid*)pRotX);
    pCoderInput->Get("fl32RotY", (tVoid*)pRotY);
    pCoderInput->Get("fl32RotZ", (tVoid*)pRotZ);


    m_pCoderDescRoadSign->Unlock(pCoderInput);

    RETURN_NOERROR;
}

tTimeStamp SegmentDriver::GetTime()
{
    return (_clock != NULL) ? _clock->GetStreamTime () : cHighResTimer::GetTime();
}

void SegmentDriver::drawPoly(vector<Point2f> ppt, Vec3b color)
{
    if (!(m_outputPin_DebugGrid.IsConnected() && m_DebugEnable))
        return;


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

    xMax = MIN(xMax,m_DebugGrid.cols-1);
    yMax = MIN(yMax,m_DebugGrid.rows-1);

    for(int x = xMin; x <= xMax; x++)
    {
        for(int y = yMin; y <= yMax; y++)
        {
            if(pointPolygonTest(ppt, Point(x,y),false) >= 0)
            {
                m_DebugGrid.at<Vec3b>(Point(x,y)) = color;
            }
        }
    }
}

Point2f SegmentDriver::intersectLines(Point2f X, Point2f x, Point2f Y, Point2f y)
{
    x *= (1.0/sqrt(x.x*x.x+x.y*x.y));
    y *= (1.0/sqrt(y.x*y.x+y.y*y.y));

    tFloat32 beta = (Y.y-X.y - x.y/x.x*(Y.x-X.x)) / (y.y-x.y/x.x*y.x);
    return Y - beta*y;
}

tInt String2Action(cString str)
{
    if(str.Compare("left") == 0)
        return ACTION_LEFT;
    else if(str.Compare("straight") == 0)
        return ACTION_STRAIGHT;
    //else if(str.Compare("straight_wo_row") == 0)
      //  return ACTION_STRAIGHT_WO_ROW;
    else if(str.Compare("right") == 0)
        return ACTION_RIGHT;
    else if(str.Compare("cross_parking") == 0)
        return ACTION_CROSS_PARKING;
    else if(str.Compare("parallel_parking") == 0)
        return ACTION_PARALLEL_PARKING;
    else if(str.Compare("pull_out_left") == 0)
        return ACTION_PULL_OUT_LEFT;
    else if(str.Compare("pull_out_right") == 0)
        return ACTION_PULL_OUT_RIGHT;
    else if(str.Compare("pull_out_right_cp") == 0)
        return ACTION_PULL_OUT_RIGHT_CP;
    else if(str.Compare("pull_out_right_pp") == 0)
        return ACTION_PULL_OUT_RIGHT_PP;
    else if(str.Compare("stop") == 0)
        return ACTION_STOP;
    else
        return ACTION_STOP;
}


tResult SegmentDriver::LoadManeuverScripts(cFilename m_fileConfig)
{
    if (m_fileConfig.IsEmpty())
    {
        LOG_ERROR("SD: Configured configuration file not found");
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    //Load file, parse configuration, print the data
    if (cFileSystem::Exists(m_fileConfig))
    {
        cDOM oDOM;
        oDOM.Load(m_fileConfig);
        cDOMElementRefList oInterpolations;
        cDOMElement* pConfigElement;

        if(IS_OK(oDOM.FindNodes("maneuverlist/maneuver", oInterpolations)))
        {
            LOG_INFO("-------");

            for (cDOMElementRefList::iterator itInterpolations = oInterpolations.begin(); itInterpolations != oInterpolations.end(); ++itInterpolations)
            {
                //vector<tFloat32> accValues, speed;
                ManeuverScript *maneuverScript = new ManeuverScript();
                pConfigElement = NULL;

                if(IS_OK((*itInterpolations)->FindNode("name", pConfigElement)))
                {
                    maneuverScript->id = String2Action(pConfigElement->GetData());
                }

                if(IS_OK((*itInterpolations)->FindNode("script", pConfigElement)))
                {
                    cString script = pConfigElement->GetData();
                    cStringList scriptList;
                    script.Split(scriptList, ";");

                    cStringListIterator scriptLine;
                    if(scriptList.GetFirstString(scriptLine))
                    {
                        do{
                            ScriptOP *operation = new ScriptOP();

                            cStringList argList;
                            scriptLine->SplitToken(argList, "(,)");

                            if(argList.GetItemCount() <= 0) // there must be at least one element in this list
                                continue;

                            cStringListIterator element;
                            argList.GetFirstString(element);


                            if(element->Find("curv") >= 0)
                                operation->op = SET_CURV;
                            else if(element->Find("high_speed") >= 0)
                                operation->op = HSENABLE;
                            else if(element->Find("speed") >= 0)
                                operation->op = SET_SPEED;
                            else if(element->Find("wait_orientation") >= 0)
                                operation->op = WAITFORORIENTATION;
                            else if(element->Find("check_row") >= 0)
                                operation->op = CHECKROW;
                            else if(element->Find("wait_dist_rear") >= 0)
                                operation->op = WAITFORDISTREAR;
                            else if(element->Find("search_parking_lot") >= 0)
                                operation->op = SEARCHPARKINGLOT;
                            else if(element->Find("wait_dist_driven") >= 0)
                                operation->op = WAITFORDISTDRIVEN;
                            else if(element->Find("wait_dist_crossing") >= 0)
                                operation->op = WAITFORDISTCROSSING;
                            else if(element->Find("wait_time") >= 0)
                                operation->op = WAITFORTIME;
                            else if(element->Find("lane_following") >= 0)
                                operation->op = LFENABLE;
                            else if(element->Find("wait_sign") >= 0)
                                operation->op = WAITFORSIGN;
                            else if(element->Find("indicator") >= 0)
                                operation->op = SETINDICATOR;
                            else if(element->Find("init_orientation") >= 0)
                                operation->op = INIT_ORIENTATION;
                            else if(element->Find("check_ot") >= 0)
                                operation->op = CHECKOT;
                            else if(element->Find("ignore_sensors") >= 0)
                                operation->op = IGNORESENS;


                            // Parse arguments
                            while(argList.GetNextString(element))
                            {
                                operation->args.push_back((tFloat32) element->AsFloat64());
                            }

                            // Insert in operation list for this maneuver
                            maneuverScript->ops.push_back(operation);

                        }while(scriptList.GetNextString(scriptLine));
                    }
                    else //empty maneuver!! -> setSpeed(0)
                    {
                        ScriptOP *operation = new ScriptOP();
                        operation->op = 2;
                        operation->args.push_back(0);
                    }
                }

                // Insert in maneuver list
                m_Scripts.push_back(maneuverScript);

            }
        }

    }
    else
    {
        LOG_ERROR("SegmentDriver: Configured configuration file not found (yet)");
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    RETURN_NOERROR;
}


void SegmentDriver::drawCrossing2Debug(Point2f A, Point2f B, Point2f C)
{
    if (m_outputPin_DebugGrid.IsConnected() && m_DebugEnable)
    {
        Point2f n = C-B;
        Point2f D = A + n;
        Point2f KR = m_K + 100*m_kr;
        Point2f KL = m_K + 100*m_kl;


        line(m_DebugGrid, m_K, KR, Scalar(255,255,255), 1);
        line(m_DebugGrid, m_K, KL, Scalar(255,255,255), 1);
        line(m_DebugGrid, m_IR_R, m_IR_L, Scalar(255,255,255), 1);

        line(m_DebugGrid, A, B, Scalar(255,255,255), 1);
        line(m_DebugGrid, B, C, Scalar(255,255,255), 1);
        line(m_DebugGrid, C, D, Scalar(255,255,255), 1);
        line(m_DebugGrid, D, A, Scalar(255,255,255), 1);
    }
}

void SegmentDriver::sendDebugGrid()
{
    m_DebugEnable = true;
    if (m_outputPin_DebugGrid.IsConnected() && m_DebugEnable)
    {
        cObjectPtr<IMediaSample> pNewDebugSample;
        if (IS_OK(AllocMediaSample(&pNewDebugSample)))
        {
            tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
            pNewDebugSample->Update(tmStreamTime, m_DebugGrid.data, tInt32(m_DebugGrid.rows*m_DebugGrid.cols*3), 0);
            m_outputPin_DebugGrid.Transmit(pNewDebugSample);
        }

        m_DebugGrid = 0 * m_DebugGrid;
    }
}
