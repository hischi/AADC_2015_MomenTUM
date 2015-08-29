/**
Copyright (c)
Audi Autonomous Driving Cup. Team MomenTUM . All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 **/

/**********************************************************************
 * $Author:: MomenTUM $  $Date:: 2015-01-15 13:29:48#$ $Rev:: 26104  $*
 **********************************************************************/

// pylonDetection.cpp : Transforms information from the depth image to an obstacle map
//

#include "pylonDetection.h"
#include <iomanip>


//enable to show the detected pylons drawn in the depth image
//#define SHOW_DEPTH_TRIANGLE

ADTF_FILTER_PLUGIN("MTUM Pylon Detection", OID_MTUM_PYLON_DETECTION, pylonDetection)

#define IMAGE_WIDTH 320
#define IMAGE_HEIGHT 240

#define CAMERA_X_OFFSET 5 //in cm

#define IMAGE_WIDTH_INPUT 640
#define IMAGE_HEIGHT_INPUT 480

#define DESIRED_PYLON_DIST_LEFT (50+CAMERA_X_OFFSET)
#define DESIRED_PYLON_DIST_RIGHT (50-CAMERA_X_OFFSET)

#define TRACKING_THRESHOLD 30

bool pylonComparator(pylonData p1,pylonData p2) {
    return p1.distance < p2.distance;
}

tResult pylonDetection::ReceiveSignalValueMessage(IMediaSample* pMediaSample, tFloat32 *pValue, tUInt32 *pTimeStamp)
{
    // read-out the incoming Media Sample
    cObjectPtr<IMediaCoder> pCoderInput;
    RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoderInput));

    *pValue = 0;
    *pTimeStamp = 0;

    //get values from media sample
    pCoderInput->Get("f32Value", (tVoid*)pValue);
    pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)pTimeStamp);
    m_pCoderDescSignal->Unlock(pCoderInput);

    RETURN_NOERROR;
}

pylonDetection::pylonDetection(const tChar* __info) : cFilter(__info)
{
    targetPylon = Point2d(0,-1000);
//    pylonCertainty = 0;
    pylonLost = 0;
    sign = -1;

    turn_around = false;
    flip_sign = false;
    is_turning = false;

    type_sign  = -1;

    car_angle = 0.0;
    pylonDetectionCounter = 0;
    signDetectionCounter = 0;
    frameCounter = 0;
    SetPropertyStr("Color Calibration Parameters","/home/odroid/AADC/calibration_files/pylonCalibration.xml");
    SetPropertyBool("Color Calibration Parameters" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Color Calibration Parameters" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyStr("Camera Calibration Parameters","/home/odroid/AADC/calibration_files/cameraParameters.xml");
    SetPropertyBool("Camera Calibration Parameters" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Camera Calibration Parameters" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyInt("Debug Image",3);
    SetPropertyStr("Debug Image" NSSUBPROP_VALUELIST, "1@Threshold|2@Color|3@Model|4@Off|");

    SetPropertyInt("Speed turning",35);
    SetPropertyInt("Speed driving",35);
    SetPropertyFloat("Curvature Gain",1.0);
    SetPropertyFloat("Curvature Gain" NSSUBPROP_REQUIRED, tTrue);
    SetPropertyFloat("Angular Gain",2.0);
    SetPropertyFloat("Angular Gain" NSSUBPROP_REQUIRED, tTrue);
}

pylonDetection::~pylonDetection()
{

}

tResult pylonDetection::CreateInputPins(__exception)
{	
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
    tChar const * strDescMotion = pDescManager->GetMediaDescription("tMotionDataExt");
    RETURN_IF_POINTER_NULL(strDescMotion);
    cObjectPtr<IMediaType> pTypeMotion = new cMediaType(0, 0, 0, "tMotionDataExt", strDescMotion,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeMotion->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescMotion));

    //rgb image input pin
    RETURN_IF_FAILED(m_inputPin_colorImage.Create("rgb_image", adtf::IPin::PD_Input  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_colorImage));

    //depth image input pin
    RETURN_IF_FAILED(m_inputPin_depthImage.Create("depth_image_in", adtf::IPin::PD_Input  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_depthImage));

    //motion data input pin
    RETURN_IF_FAILED(m_inputPin_motion.Create("motion_data", pTypeMotion, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_motion));

    //sign data input pin
    tChar const * strMarkerInfoValue = pDescManager->GetMediaDescription("tRoadSign");
    RETURN_IF_POINTER_NULL(strMarkerInfoValue);
    cObjectPtr<IMediaType> pTypeSignInfoValue = new cMediaType(0, 0, 0, "tRoadSign", strMarkerInfoValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignInfoValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSign));
    RETURN_IF_FAILED(m_inputPin_signLocation.Create("RoadSign", pTypeSignInfoValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_signLocation));

    //Color calib input pins
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal));
    RETURN_IF_FAILED(m_inputPin_0.Create("hlow", new cMediaType(0, 0, 0, "tSignalValue") , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_0));
    RETURN_IF_FAILED(m_inputPin_1.Create("hhigh", new cMediaType(0, 0, 0, "tSignalValue") , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_1));
    RETURN_IF_FAILED(m_inputPin_2.Create("slow", new cMediaType(0, 0, 0, "tSignalValue") , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_2));
    RETURN_IF_FAILED(m_inputPin_3.Create("shigh", new cMediaType(0, 0, 0, "tSignalValue") , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_3));
    RETURN_IF_FAILED(m_inputPin_4.Create("vlow", new cMediaType(0, 0, 0, "tSignalValue") , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_4));
    RETURN_IF_FAILED(m_inputPin_5.Create("vhigh", new cMediaType(0, 0, 0, "tSignalValue") , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_5));

	RETURN_NOERROR;
}

tResult pylonDetection::CreateOutputPins(__exception)
{
    //debug image otput pin
    RETURN_IF_FAILED(m_outputPin_debugImage.Create("debug_image", adtf::IPin::PD_Output  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_debugImage));

    //depth depth output pin
    RETURN_IF_FAILED(m_outputPin_depthImage.Create("depth_image_out", adtf::IPin::PD_Output  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_depthImage));

    //Pylon Info output pin
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
    tChar const * strDesc = pDescManager->GetMediaDescription("tPylonInfo");
    if(!strDesc) LOG_ERROR(cString(OIGetInstanceName()) + ": Could not load mediadescription tPylonInfo, check path");
    RETURN_IF_POINTER_NULL(strDesc);
    cObjectPtr<IMediaType> pType = new cMediaType(0, 0, 0, "tPylonInfo", strDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(m_outputPin_PylonInfo.Create("PylonInfo", pType, this));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_PylonInfo));
    RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescPylonInfo));
 
    //Lane Info
    tChar const * strDescLaneInfoValue = pDescManager->GetMediaDescription("tLaneInfo");
    RETURN_IF_POINTER_NULL(strDescLaneInfoValue);
    cObjectPtr<IMediaType> pTypeLaneInfoValue = new cMediaType(0, 0, 0, "tLaneInfo", strDescLaneInfoValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeLaneInfoValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescLaneInfo));

    RETURN_IF_FAILED(m_oLaneDataOutputPin.Create("laneInfo", pTypeLaneInfoValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oLaneDataOutputPin));

    //acceleration pin
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);

    RETURN_IF_FAILED(m_outputPin_speed.Create("accelerate", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_speed));

    RETURN_NOERROR;
}

tResult pylonDetection::sendPylonInfo(float *distArray, float *angleArray)
{
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescPylonInfo->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();

    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
    tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
    cObjectPtr<IMediaCoder> pCoder;
    RETURN_IF_FAILED(m_pCoderDescPylonInfo->WriteLock(pMediaSample, &pCoder));
    pCoder->Set("f32Dist", (tVoid*) distArray);
    pCoder->Set("f32Angle", (tVoid*)angleArray);
    pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&tmStreamTime);
    RETURN_IF_FAILED(m_pCoderDescPylonInfo->Unlock(pCoder));

    RETURN_IF_FAILED(pMediaSample->SetTime(tmStreamTime));
    RETURN_IF_FAILED(m_outputPin_PylonInfo.Transmit(pMediaSample));
    RETURN_NOERROR;
}

tResult pylonDetection::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
	{
		CreateInputPins(__exception_ptr);
		CreateOutputPins(__exception_ptr);
	}
    else if (eStage == StageNormal)
    {
        m_debugImage = Mat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC3,Scalar(0,0,0));

        //format debug output pin
        m_sBitmapFormatDebugOut.nWidth = IMAGE_WIDTH;
        m_sBitmapFormatDebugOut.nHeight = IMAGE_HEIGHT;
        m_sBitmapFormatDebugOut.nBitsPerPixel = 24;
        m_sBitmapFormatDebugOut.nPixelFormat = cImage::PF_RGB_888;
        m_sBitmapFormatDebugOut.nBytesPerLine = IMAGE_WIDTH*3;
        m_sBitmapFormatDebugOut.nSize = IMAGE_WIDTH*IMAGE_HEIGHT*3;
        m_sBitmapFormatDebugOut.nPaletteSize = 0;

        //format rgb input pin
        m_sBitmapFormatColorIn.nWidth = IMAGE_WIDTH_INPUT;
        m_sBitmapFormatColorIn.nHeight = IMAGE_HEIGHT_INPUT;
        m_sBitmapFormatColorIn.nBitsPerPixel = 24;
        m_sBitmapFormatColorIn.nPixelFormat = cImage::PF_RGB_888;
        m_sBitmapFormatColorIn.nBytesPerLine = IMAGE_WIDTH_INPUT * 3;
        m_sBitmapFormatColorIn.nSize = 3* IMAGE_WIDTH_INPUT * IMAGE_HEIGHT_INPUT;
        m_sBitmapFormatColorIn.nPaletteSize = 0;

        //format depth input pin
        m_sBitmapFormatDepthIn.nWidth = 320;
        m_sBitmapFormatDepthIn.nHeight = 240;
        m_sBitmapFormatDepthIn.nBitsPerPixel = 16;
        m_sBitmapFormatDepthIn.nPixelFormat = cImage::PF_GREYSCALE_16;
        m_sBitmapFormatDepthIn.nBytesPerLine = 640;
        m_sBitmapFormatDepthIn.nSize = 640*240;
        m_sBitmapFormatDepthIn.nPaletteSize = 0;

        //format depth output pin
        m_sBitmapFormatDepthOut.nWidth = 320;
        m_sBitmapFormatDepthOut.nHeight = 240;
        m_sBitmapFormatDepthOut.nBitsPerPixel = 16;
        m_sBitmapFormatDepthOut.nPixelFormat = cImage::PF_GREYSCALE_16;
        m_sBitmapFormatDepthOut.nBytesPerLine = 640;
        m_sBitmapFormatDepthOut.nSize = 640*240;
        m_sBitmapFormatDepthOut.nPaletteSize = 0;

        //apply bitmap format to video pins
        m_inputPin_depthImage.SetFormat(&m_sBitmapFormatDepthIn, NULL);
        m_outputPin_depthImage.SetFormat(&m_sBitmapFormatDepthOut, NULL);
        m_outputPin_debugImage.SetFormat(&m_sBitmapFormatDebugOut, NULL);

        m_depthImage = Mat(m_sBitmapFormatDepthIn.nHeight,m_sBitmapFormatDepthIn.nWidth,CV_16UC1);

        cFilename colorCalib = GetPropertyStr("Color Calibration Parameters");
        RETURN_IF_FAILED(LoadConfigurationColorData(colorCalib));
        cFilename cameraCalib = GetPropertyStr("Camera Calibration Parameters");
        RETURN_IF_FAILED(LoadConfigurationCameraData(cameraCalib));

        speed_driving = GetPropertyInt("Speed driving");
        speed_turning = GetPropertyInt("Speed turning");
        curvature_gain = GetPropertyFloat("Curvature Gain");
        angular_gain = GetPropertyFloat("Angular Gain");

        debugImageSelection = GetPropertyInt("Debug Image");
    }
    RETURN_NOERROR;
}

tResult pylonDetection::Start(__exception)
{
    waitForInit = true;
    return cFilter::Start(__exception_ptr);
}

tResult pylonDetection::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult pylonDetection::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}
tResult pylonDetection:: OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample)
{
    if(pSource == &m_inputPin_motion)
    {
        if(waitForInit) waitForInit = false;

        // read-out the incoming Media Sample of tMotionDataExt type
        cObjectPtr<IMediaCoder> pCoderInput;
        RETURN_IF_FAILED(m_pCoderDescMotion->Lock(pMediaSample, &pCoderInput));
        tFloat32 phi = 0;
        tFloat32 inDY = 0;
        tUInt32 timeStamp = 0;
        pCoderInput->Get("f32dY", (tVoid*)&inDY);
        pCoderInput->Get("f32dPhi", (tVoid*)&phi);
        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
        m_pCoderDescMotion->Unlock(pCoderInput);

        //update global car angle with motion data
        car_angle += phi;
        if(car_angle > 2*M_PI)car_angle -= 2*M_PI;
        if(car_angle < -2*M_PI)car_angle += 2*M_PI;

        if(is_turning)
        {
            targetPylon.y = -1000;
            RETURN_NOERROR;
        }
        //update pylon y position with car angle (this only applies if the pylon is not seen at the moment)
        targetPylon.y -= inDY;

        double s = sin(-phi);
        double c = cos(-phi);

        double y_offset = 29;

        targetPylon.x = targetPylon.x * c - (targetPylon.y + y_offset)* s;
        targetPylon.y = targetPylon.x * s + (targetPylon.y + y_offset)* c;
        targetPylon.y  -= y_offset;

    } else if (pSource == &m_inputPin_depthImage)
    {
        RETURN_IF_POINTER_NULL(pMediaSample);

        const tVoid* l_pSrcBuffer;
        if (IS_OK(pMediaSample->Lock(&l_pSrcBuffer)))
        {
            Mat m_depthImageTemp = Mat(m_sBitmapFormatDepthIn.nHeight,m_sBitmapFormatDepthIn.nWidth,CV_16UC1,(tVoid*)l_pSrcBuffer,m_sBitmapFormatDepthIn.nBytesPerLine);
            m_depthImage = m_depthImageTemp.clone();
            pMediaSample->Unlock(l_pSrcBuffer);
        }

    } else if(pSource == &m_inputPin_signLocation)
    {
            ProcessSign(pMediaSample);
    } else if (pSource == &m_inputPin_colorImage)
    {
        float timeSinceLastSeen = (clock()-lastSeen)/CLOCKS_PER_SEC;

        if(waitForInit || (timeSinceLastSeen >= 5.0 && !is_turning))
        {
            //STOP
            SendSignalValueMessage(&m_outputPin_speed,m_pCoderDescSignal, 0, 0);
        }
        else if(is_turning)
        {
            //Send turning speed
            SendSignalValueMessage(&m_outputPin_speed,m_pCoderDescSignal, speed_turning , 0);
        }
        else
        {
           //Send driving speed
            SendSignalValueMessage(&m_outputPin_speed,m_pCoderDescSignal, speed_driving, 0);
        }

        RETURN_IF_POINTER_NULL(pMediaSample);

        //read media sample and write to depth image data (openCV Mat)
        const tVoid* l_pSrcBuffer;
        if (IS_OK(pMediaSample->Lock(&l_pSrcBuffer)))
        {
            Mat colorImage = Mat(m_sBitmapFormatColorIn.nHeight,m_sBitmapFormatColorIn.nWidth,CV_8UC3,(tVoid*)l_pSrcBuffer,m_sBitmapFormatColorIn.nBytesPerLine).clone();
            pMediaSample->Unlock(l_pSrcBuffer);

            resize(colorImage,colorImage,Size(320,240));

#ifdef SHOW_DEPTH_TRIANGLE
            Mat depthImageCopy(Size(320,240),CV_8UC1);
            m_depthImage.convertTo(depthImageCopy,CV_8UC1,1.0/255.0);
            cvtColor(depthImageCopy,depthImageCopy,COLOR_GRAY2BGR);
#endif

            //Detect pylons on the image
            vector<Rect> foundOrangePylons = pDetector.detect(colorImage, orange_from, orange_to);

            if(turn_around && targetPylon.y < -10)
            {
                if(car_angle >= M_PI/2 && car_angle <= 1.5*M_PI) car_angle -= 2*M_PI;
                is_turning = true;
                turn_around = false;
            }

            if(is_turning && foundOrangePylons.size() > 0)
            {
                double minDistance = 100000;
                for(int i = 0;i < (int)foundOrangePylons.size();i++)
                {
                        double currDistance = getPylonDist(foundOrangePylons.at(i));
                        if(currDistance < 0)continue;

                        if(currDistance < minDistance)
                            minDistance = currDistance;
                }
                if(minDistance < 2.5)
                {
                    is_turning = false;
                    flip_sign ^= true;
                    sign *= -1;
                }
            }

            if(is_turning)
            {
//                LOG_INFO(cString::Format("FlipSign: %s, sign: %d, car_angle: %f",flip_sign?"true":"false",sign, car_angle));
                if(!flip_sign && sign == -1) //Going away, clockwise
                {
                    if(car_angle>-10*M_PI/9)
                         SendLaneInfo(1.0/100,22.5,0,0);
                    else
                        SendLaneInfo(0,22.5,0,0);
                    RETURN_NOERROR;
                }
                else if(!flip_sign && sign == +1) //Going away, counterclockwise
                {
                    if(car_angle<+10*M_PI/9)
                         SendLaneInfo(-1.0/100,22.5,0,0);
                    else
                        SendLaneInfo(0,22.5,0,0);
                    RETURN_NOERROR;
                }
                else if(flip_sign && sign == 1) //Coming back, clockwise
                {
                    if(car_angle > -M_PI/9)
                         SendLaneInfo(1.0/100,22.5,0,0);
                    else if(car_angle < -M_PI/2)
                         SendLaneInfo(1.0/100,22.5,0,0);
                    else
                        SendLaneInfo(0,22.5,0,0);
                    RETURN_NOERROR;
                }
                else
                {
                    if(car_angle < +M_PI/9) //Coming back, counterclockwise
                         SendLaneInfo(-1.0/100,22.5,0,0);
                    else
                        SendLaneInfo(0,22.5,0,0);
                    RETURN_NOERROR;
                }
            }

            if(foundOrangePylons.size()>0)
                lastSeen = clock();

            Mat orangeThresh;
            orangeThresh = pDetector.thresholdImageDebug;
            cvtColor(orangeThresh,orangeThresh,COLOR_GRAY2BGR);

            vector<pylonData> allPylons;

            for(size_t i = 0;i < foundOrangePylons.size();i++)
            {
                Rect pylonRect = foundOrangePylons.at(i);
                rectangle(colorImage,pylonRect,Scalar(0,0,255),1);

                //draw pylon triangle
                Point p1(pylonRect.x+pylonRect.width/2,pylonRect.y+0.1*pylonRect.height);
                Point p2(pylonRect.x+0.1*pylonRect.width,pylonRect.y+pylonRect.height-0.1*pylonRect.height);
                Point p3(pylonRect.x+pylonRect.width-0.1*pylonRect.width,pylonRect.y+pylonRect.height-0.1*pylonRect.height);

                line(colorImage,p1,p2,Scalar(0,255,0),2);
                line(colorImage,p2,p3,Scalar(0,255,0),2);
                line(colorImage,p3,p1,Scalar(0,255,0),2);

                line(orangeThresh,p1,p2,Scalar(0,255,0),2);
                line(orangeThresh,p2,p3,Scalar(0,255,0),2);
                line(orangeThresh,p3,p1,Scalar(0,255,0),2);

#ifdef SHOW_DEPTH_TRIANGLE
                line(depthImageCopy,p1*0.5,p2*0.5,Scalar(0,255,0),2);
                line(depthImageCopy,p2*0.5,p3*0.5,Scalar(0,255,0),2);
                line(depthImageCopy,p3*0.5,p1*0.5,Scalar(0,255,0),2);
#endif
                //convert to distance value [m] (linear conversion)
                double tempDistValue = getPylonDist(pylonRect);
                if(tempDistValue < 0)continue;

                //print distance
                std::ostringstream strs;
                strs << "Dist: " << tempDistValue;
                std::string str = strs.str();
                putText(colorImage,str,Point(100,25+i*50),FONT_HERSHEY_COMPLEX_SMALL,0.4,Scalar(0,0,255),1,CV_AA);

                double pylon_x = pylonRect.x+pylonRect.width/2;
                double pylonAngle = atan2(m_fu/2,(pylon_x-m_cx/2));
                pylonAngle = M_PI_2-pylonAngle;

                //print angle
                std::ostringstream strs2;
                strs2 << "Angle: " << pylonAngle*RAD2DEG;
                std::string str2 = strs2.str();
                putText(colorImage,str2,Point(100,50+i*50),FONT_HERSHEY_COMPLEX_SMALL,0.4,Scalar(0,0,255),1,CV_AA);

                pylonData newPylon;
                newPylon.distance = tempDistValue;
                newPylon.angle = pylonAngle;
                allPylons.push_back(newPylon);
            }

            //sort all pylons
            std::sort(allPylons.begin(),allPylons.end(),pylonComparator);

#ifdef SHOW_DEPTH_TRIANGLE
            depthImageCopy.copyTo(colorImage(Rect(0,0,depthImageCopy.cols,depthImageCopy.rows)));
#endif

            bool sawPylon = false;
            if(allPylons.size() > 0 && allPylons.at(0).distance < 2.5)
            {
                Point2d pylonPoint(tan(allPylons.at(0).angle)*allPylons.at(0).distance*100,allPylons.at(0).distance*100);

                Point2d distanceOld = targetPylon-pylonPoint;
                double dist = sqrt(distanceOld.x*distanceOld.x + distanceOld.y*distanceOld.y);         
                if(dist < 70){
                    sawPylon = true;
                    targetPylon = pylonPoint;
//                    pylonCertainty = 0;
                }
                else if(targetPylon.y < 10)
                {
                  //  pylonCertainty++;
                 //   if(pylonCertainty == 5)
                  //  {
                       // pylonCertainty = 0;
                        targetPylon = pylonPoint;
                        pylonLost = 0;

                        if(pylonPoint.y > 100)sign *= -1;
                  //  }
                  //  else
//                    {
                  //      RETURN_NOERROR;
                  //  }
                }
            }
            else if(targetPylon.y > TRACKING_THRESHOLD)
            {
                //Too far, do nothing
            }
            else
            {
                if(pylonLost < 5)
                    pylonLost++;
                else
                {
                    if( !flip_sign )
                    {
                        //Straight is 0
                        if(car_angle>-M_PI && car_angle<M_PI)
                            SendLaneInfo(0, 22.5, -car_angle, 0);
                        else if(car_angle >= M_PI)
                            SendLaneInfo(0, 22.5, -(car_angle-2*M_PI), 0);
                        else
                            SendLaneInfo(0, 22.5, -(car_angle+2*M_PI), 0);
                    }
                    else
                    {
                        //Straight is 180
                        if(car_angle < 0)
                            SendLaneInfo(0, 22.5, -(car_angle+M_PI), 0);
                        else
                            SendLaneInfo(0, 22.5, -(car_angle-M_PI), 0);
                    }
                }
            }

            double a_param = 0;
            double b_param = 0;
            double c_param = 0;

            Point2d p1Draw(0,0);
            Point2d p2Draw(0,0);

            if(sawPylon || targetPylon.y > TRACKING_THRESHOLD)
            {
                Point2d p1(targetPylon.x,targetPylon.y);
                Point2d p2;
                Point2d direction;
                double start_linearization = 200;
                double linearization_dist = 65;
                if(p1.y < start_linearization-linearization_dist)
                {
                    if(sign == 1)
                    {
                        p1.x += sign*DESIRED_PYLON_DIST_RIGHT*cos(-car_angle);
                        p1.y += sign*DESIRED_PYLON_DIST_RIGHT*sin(-car_angle);
                    }
                    else
                    {
                        p1.x += sign*DESIRED_PYLON_DIST_LEFT*cos(-car_angle);
                        p1.y += sign*DESIRED_PYLON_DIST_LEFT*sin(-car_angle);
                    }

                    direction = Point2d(sin(-car_angle),-cos(-car_angle));
                    p2 = p1 + 5*direction;
                }
                else
                {  
                    double desired_dist;
                    if(sign == 1)
                    {
                        desired_dist = (-DESIRED_PYLON_DIST_RIGHT/linearization_dist)*(p1.y-(start_linearization-linearization_dist))+DESIRED_PYLON_DIST_RIGHT;
                        if(p1.y > start_linearization)desired_dist = 0;
                    }
                    else
                    {
                        desired_dist = (-DESIRED_PYLON_DIST_LEFT/linearization_dist)*(p1.y-(start_linearization-linearization_dist))+DESIRED_PYLON_DIST_LEFT;
                        if(p1.y > start_linearization)desired_dist = 0;
                    }

                    p1.x += sign*desired_dist*cos(-car_angle);
                    p1.y += sign*desired_dist*sin(-car_angle);
                    direction = Point2d(sin(-car_angle),-cos(-car_angle));
                    p2 = p1 + 5*direction;
                }
                p1Draw = p1;
                p2Draw = p2;

                Point2d p3(0,0);

                Mat A = (Mat_<double>(3,3) <<      p1.y*p1.y,p1.y,1,
                         p2.y*p2.y,p2.y,1,
                         p3.y*p3.y,p3.y,1);

                Mat b = (Mat_<double>(3,1) <<   p1.x,p2.x,p3.x);

                Mat sol;
                solve(A, b, sol);

                a_param = sol.at<double>(0,0);
                b_param = sol.at<double>(1,0);
                c_param = sol.at<double>(2,0);

                //get curvature estimate
                double curvature = 0.0;
                for(int i = 0;i < 100;i++) //was 50
                {
                    curvature += (2*a_param)/pow((1+pow(a_param*i+b_param,2.0)),1.5);
                }
                curvature /= 100;

                //angular offset on road
                Point2d carP1(a_param*(-10)*(-10)+b_param*(-10)+c_param,-10);
                Point2d carP2(c_param,0);
                Point2d tangentVec = carP2-carP1;

                double tan_angle = tangentVec.x / tangentVec.y;
                double ang = -atan(tan_angle);

                SendLaneInfo(curvature_gain*curvature, 22.5, angular_gain*ang, 0);
            }

            Mat modelMat(IMAGE_HEIGHT,IMAGE_WIDTH,CV_8UC3,Scalar(0,0,0));
            Point2d carOrigin(IMAGE_WIDTH/2,IMAGE_HEIGHT-40);
            line(modelMat,Point2d(0,IMAGE_HEIGHT-40),Point2d(IMAGE_WIDTH,IMAGE_HEIGHT-40),Scalar(255,255,255),1);
            line(modelMat,Point2d(IMAGE_WIDTH/2,0),Point2d(IMAGE_WIDTH/2,IMAGE_HEIGHT),Scalar(255,255,255),1);

            Point2d targetPylonDraw(targetPylon.x,-targetPylon.y);

            if(sawPylon)
                circle(modelMat,targetPylonDraw+carOrigin,3,Scalar(0,255,0),-1);
            else if(type_sign != -1)
                circle(modelMat,targetPylonDraw+carOrigin,3,Scalar(255,0,255),-1);
            else
                circle(modelMat,targetPylonDraw+carOrigin,3,Scalar(0,0,255),-1);

            circle(modelMat,Point2d(p1Draw.x,-p1Draw.y)+carOrigin,3,Scalar(255,0,255),-1);
            circle(modelMat,Point2d(p2Draw.x,-p2Draw.y)+carOrigin,3,Scalar(255,0,255),-1);

            for(int y = -10;y < 200;y++)
            {
                double x1 = a_param*y*y+b_param*y+c_param;
                double x2 = a_param*(y+1)*(y+1)+b_param*(y+1)+c_param;
                line(modelMat,carOrigin+Point2d(x1,-y),carOrigin+Point2d(x2,-y-1),Scalar(0,255,0));

            }

            if(debugImageSelection != 4)
            {
                if(debugImageSelection == 3) {
                    if(sign == 1)
                    {
                        stringstream ss;
                        ss << "Sign +1 (to right): " << type_sign;
                        putText(modelMat,ss.str(),Point(200,25),FONT_HERSHEY_SIMPLEX,0.3,Scalar(0,255,0),1,CV_AA);
                    }
                    else
                    {
                        stringstream ss;
                        ss << "Sign -1 (to left): " << type_sign;
                        putText(modelMat,ss.str(),Point(200,25),FONT_HERSHEY_SIMPLEX,0.3,Scalar(0,255,0),1,CV_AA);
                    }

                    if(frameCounter>=20) {
                        frameCounter=pylonDetectionCounter=signDetectionCounter=0;
                    }
                    frameCounter++;
                    if(foundOrangePylons.size()>0)
                        pylonDetectionCounter++;
                    if(type_sign>=0)
                        signDetectionCounter++;
                    stringstream ss;
                    float pylonDetectionPercent = (pylonDetectionCounter*100.0/frameCounter);
                    float signDetectionPercent = (signDetectionCounter*100.0/frameCounter);
                    ss << "Pylon Detection: " << setfill(' ') << setw(3) << (int)pylonDetectionPercent << "%";
                    putText(modelMat,ss.str(),Point(200,50),FONT_HERSHEY_SIMPLEX,0.3,Scalar(0,(int)2.55*pylonDetectionPercent,(int)255 - 2.55*pylonDetectionPercent),1,CV_AA);
                    ss.str("");
                    ss << "Sign Detection: " << setfill(' ') << setw(3) << (int)signDetectionPercent << "%";
                    putText(modelMat,ss.str(),Point(200,60),FONT_HERSHEY_SIMPLEX,0.3,Scalar(0,(int)2.55*signDetectionPercent,(int)255 - 2.55*signDetectionPercent),1,CV_AA);

                    ss.str("");
                    ss << "Angle: " << car_angle*RAD2DEG << " f_s: " << (flip_sign ? "true" : "false");
                    putText(modelMat,ss.str(),Point(200,70),FONT_HERSHEY_SIMPLEX,0.3,Scalar(255,255,0),1,CV_AA);
                }

                cObjectPtr<IMediaSample> pNewRGBSample;
                if (IS_OK(AllocMediaSample(&pNewRGBSample)))
                {
                    tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
                    if(debugImageSelection == 1)
                        pNewRGBSample->Update(tmStreamTime, orangeThresh.data, tInt32(3*IMAGE_WIDTH*IMAGE_HEIGHT), 0);
                    else if(debugImageSelection == 2)
                        pNewRGBSample->Update(tmStreamTime, colorImage.data, tInt32(3*IMAGE_WIDTH*IMAGE_HEIGHT), 0);
                    else if (debugImageSelection == 3)
                        pNewRGBSample->Update(tmStreamTime, modelMat.data, tInt32(3*IMAGE_WIDTH*IMAGE_HEIGHT), 0);
                    RETURN_IF_FAILED(m_outputPin_debugImage.Transmit(pNewRGBSample));
                }
            }
        }
    } else if (pSource == &m_inputPin_0)
    {
        RETURN_IF_POINTER_NULL(pMediaSample);
        tFloat32 value = 0;
        tUInt32 timeStamp = 0;
        ReceiveSignalValueMessage(pMediaSample, &value, &timeStamp);
        ohl = (int)(value*255.0/100.0);
        LOG_WARNING(adtf_util::cString::Format("orange_from = Scalar(%d, %d, %d) orange_to = Scalar(%d, %d, %d)",ohl,osl,ovl,ohh,osh,ovh));
        orange_from = Scalar(ohl, osl, ovl);
    } else if (pSource == &m_inputPin_1)
    {
        RETURN_IF_POINTER_NULL(pMediaSample);
        tFloat32 value = 0;
        tUInt32 timeStamp = 0;
        ReceiveSignalValueMessage(pMediaSample, &value, &timeStamp);
        ohh = (int)(value*255.0/70.0 + 255.0/2.0);
        LOG_WARNING(adtf_util::cString::Format("orange_from = Scalar(%d, %d, %d) orange_to = Scalar(%d, %d, %d)",ohl,osl,ovl,ohh,osh,ovh));
        orange_to = Scalar(ohh, osh, ovh);
    } else if (pSource == &m_inputPin_2)
    {
        RETURN_IF_POINTER_NULL(pMediaSample);
        tFloat32 value = 0;
        tUInt32 timeStamp = 0;
        ReceiveSignalValueMessage(pMediaSample, &value, &timeStamp);
        osl = (int)(value*255.0/100.0);
        LOG_WARNING(adtf_util::cString::Format("orange_from = Scalar(%d, %d, %d) orange_to = Scalar(%d, %d, %d)",ohl,osl,ovl,ohh,osh,ovh));
        orange_from = Scalar(ohl, osl, ovl);
    } else if (pSource == &m_inputPin_3)
    {
        RETURN_IF_POINTER_NULL(pMediaSample);
        tFloat32 value = 0;
        tUInt32 timeStamp = 0;
        ReceiveSignalValueMessage(pMediaSample, &value, &timeStamp);
        osh = (int)(value*255.0/70.0 + 255.0/2.0);
        LOG_WARNING(adtf_util::cString::Format("orange_from = Scalar(%d, %d, %d) orange_to = Scalar(%d, %d, %d)",ohl,osl,ovl,ohh,osh,ovh));
        orange_to = Scalar(ohh, osh, ovh);
    } else if (pSource == &m_inputPin_4)
    {
        RETURN_IF_POINTER_NULL(pMediaSample);
        tFloat32 value = 0;
        tUInt32 timeStamp = 0;
        ReceiveSignalValueMessage(pMediaSample, &value, &timeStamp);
        ovl = (int)(value*255.0/100.0);
        LOG_WARNING(adtf_util::cString::Format("orange_from = Scalar(%d, %d, %d) orange_to = Scalar(%d, %d, %d)",ohl,osl,ovl,ohh,osh,ovh));
        orange_from = Scalar(ohl, osl, ovl);
    } else if (pSource == &m_inputPin_5)
    {
        RETURN_IF_POINTER_NULL(pMediaSample);
        tFloat32 value = 0;
        tUInt32 timeStamp = 0;
        ReceiveSignalValueMessage(pMediaSample, &value, &timeStamp);
        ovh = (int)(value*255.0/70.0 + 255.0/2.0);
        LOG_WARNING(adtf_util::cString::Format("orange_from = Scalar(%d, %d, %d) orange_to = Scalar(%d, %d, %d)",ohl,osl,ovl,ohh,osh,ovh));
        orange_to = Scalar(ohh, osh, ovh);
    }
        RETURN_NOERROR;
}

tResult pylonDetection::LoadConfigurationColorData(cFilename m_fileConfig)
{
    if (m_fileConfig.IsEmpty())
    {
        LOG_ERROR("PylonDetection: Calibration file not found");
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    //Load file, parse configuration, print the data
    if (cFileSystem::Exists(m_fileConfig))
    {
        cDOM oDOM;
        oDOM.Load(m_fileConfig);
        cDOMElement* pConfigElement;

        int h_low_orange=-1.0, h_high_orange=-1.0,s_low_orange=-1.0, s_high_orange=-1.0,v_low_orange=-1.0, v_high_orange=-1.0;
        bool error=false;
        if (IS_OK(oDOM.FindNode("color/orange/h/low", pConfigElement))) {
            h_low_orange = int(cString(pConfigElement->GetData()).AsInt32());
        } else error=true;
        if (IS_OK(oDOM.FindNode("color/orange/h/high", pConfigElement))) {
            h_high_orange = int(cString(pConfigElement->GetData()).AsInt32());
        } else error=true;
        if (IS_OK(oDOM.FindNode("color/orange/s/low", pConfigElement))) {
            s_low_orange = int(cString(pConfigElement->GetData()).AsInt32());
        } else error=true;
        if (IS_OK(oDOM.FindNode("color/orange/s/high", pConfigElement))) {
            s_high_orange = int(cString(pConfigElement->GetData()).AsInt32());
        } else error=true;
        if (IS_OK(oDOM.FindNode("color/orange/v/low", pConfigElement))) {
            v_low_orange = int(cString(pConfigElement->GetData()).AsInt32());
        } else error=true;
        if (IS_OK(oDOM.FindNode("color/orange/v/high", pConfigElement))) {
            v_high_orange = int(cString(pConfigElement->GetData()).AsInt32());
        } else error=true;
        if(error)
        {
            LOG_ERROR("PylonDetection: Calibration file is missing parameters");
            RETURN_ERROR(ERR_INVALID_FILE);
        }
        orange_from = Scalar(h_low_orange, s_low_orange, v_low_orange);
        orange_to = Scalar(h_high_orange, s_high_orange, v_high_orange);

        ohl=h_low_orange;ohh=h_high_orange;osl=s_low_orange;osh=s_high_orange;ovl=v_low_orange;ovh=v_high_orange;
    }
    else
    {
        LOG_ERROR("PylonDetection: Calibration file not found");
        RETURN_ERROR(ERR_INVALID_FILE);
    }
    RETURN_NOERROR;
}

tResult pylonDetection::SendLaneInfo(tFloat32 curvature, tFloat32 lanePos, tFloat32 carOrientation, tUInt32 timeStamp)
{
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleOut;
    AllocMediaSample((tVoid**)&pMediaSampleOut);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescLaneInfo->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleOut->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    cObjectPtr<IMediaCoder> pCoderOutput;
    m_pCoderDescLaneInfo->WriteLock(pMediaSampleOut, &pCoderOutput);

    pCoderOutput->Set("f32Curvature", (tVoid*)&curvature);
    pCoderOutput->Set("f32LanePos", (tVoid*)&lanePos);
    pCoderOutput->Set("f32CarOrientation", (tVoid*)&carOrientation);
    pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
    m_pCoderDescLaneInfo->Unlock(pCoderOutput);

    //transmit media sample over output pin
    tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
    pMediaSampleOut->SetTime(tmStreamTime);
    m_oLaneDataOutputPin.Transmit(pMediaSampleOut);

    RETURN_NOERROR;
}

tResult pylonDetection::ProcessSign(IMediaSample* pSample)
{
    RETURN_IF_POINTER_NULL(pSample);
    tInt8 id;
    cv::Mat tVec = cv::Mat::zeros(3,1,CV_32FC1);
    cv::Mat rVec = cv::Mat::zeros(3,1,CV_32FC1);
    tUInt32 timeStamp;
    ReceiveRoadSign(pSample, &id, &tVec, &rVec, &timeStamp);
    //LOG_WARNING(adtf_util::cString::Format("(%d) T: (%.03f,%.03f,%.03f) R: (%.03f,%.03f,%.03f)", id, tVec.at<float>(0),tVec.at<float>(1),tVec.at<float>(2), rVec.at<float>(0),rVec.at<float>(1),rVec.at<float>(2)));

    double distToSign = norm(Point2d(tVec.at<float>(0)*100,tVec.at<float>(2)*100));
//    double angleToSign = atan2(tVec.at<float>(0),tVec.at<float>(2));
    double distToPylon = sqrt(targetPylon.x*targetPylon.x+targetPylon.y*targetPylon.y);

    if(id == 0 || distToSign > 300 || abs(distToPylon-distToSign) > 100)
    {
        type_sign = -1;
        RETURN_NOERROR;
    }

    if (id == 7)
    {
        turn_around = true;
    }

//    dist_sign = distToSign;
//    angle_sign = angleToSign;
    type_sign = id;

    RETURN_NOERROR;
}

tResult pylonDetection::ReceiveRoadSign(IMediaSample* pMediaSample, tInt8 *pId, Mat *pTvec, Mat* pRvec, tUInt32 *pTimeStamp)
{
    cObjectPtr<IMediaCoder> pCoderInput;
    RETURN_IF_FAILED(m_pCoderDescSign->Lock(pMediaSample, &pCoderInput));
    pCoderInput->Get("i8Identifier", (tVoid*)pId);
    pCoderInput->Get("fl32TransX", (tVoid*)&pTvec->at<float>(0));
    pCoderInput->Get("fl32TransY", (tVoid*)&pTvec->at<float>(1));
    pCoderInput->Get("fl32TransZ", (tVoid*)&pTvec->at<float>(2));
    pCoderInput->Get("fl32RotX", (tVoid*)&pRvec->at<float>(0));
    pCoderInput->Get("fl32RotY", (tVoid*)&pRvec->at<float>(1));
    pCoderInput->Get("fl32RotZ", (tVoid*)&pRvec->at<float>(2));
    pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)pTimeStamp);
    m_pCoderDescSign->Unlock(pCoderInput);
    RETURN_NOERROR;
}

tResult pylonDetection::LoadConfigurationCameraData(cFilename m_fileConfig)
{
    if (m_fileConfig.IsEmpty())
    {
        LOG_ERROR("PylonDetection: Camera Calibration file not found");
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    //Load file, parse configuration, print the data
    if (cFileSystem::Exists(m_fileConfig))
    {
        cDOM oDOM;
        oDOM.Load(m_fileConfig);
        cDOMElement* pConfigElement;

        float fu=-1.0, cx=-1.0;

        if (IS_OK(oDOM.FindNode("camera/fu", pConfigElement))) {
            fu = float(cString(pConfigElement->GetData()).AsFloat64());
        }
        if (IS_OK(oDOM.FindNode("camera/cx", pConfigElement))) {
            cx = float(cString(pConfigElement->GetData()).AsFloat64());
        }
        if(fu<0 || cx<0)
        {
            LOG_ERROR("PylonDetection: Camera Calibration file is missing parameters");
            RETURN_ERROR(ERR_INVALID_FILE);
        }
        m_fu = fu;
        m_cx = cx;
    }
    else
    {
        LOG_ERROR("PylonDetection: Camera Calibration file not found");
        RETURN_ERROR(ERR_INVALID_FILE);
    }
    RETURN_NOERROR;
}

tResult pylonDetection::SendSignalValueMessage(cObjectPtr<cOutputPin> pOutputPin, cObjectPtr<IMediaTypeDescription> mediaDesc, tFloat32 value, tUInt32 timeStamp)
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

double pylonDetection::getPylonDist(Rect pylonRect)
{
    vector<Point> contour;
    Point p1(pylonRect.x+pylonRect.width/2,pylonRect.y+0.1*pylonRect.height);
    Point p2(pylonRect.x+0.1*pylonRect.width,pylonRect.y+pylonRect.height-0.1*pylonRect.height);
    Point p3(pylonRect.x+pylonRect.width-0.1*pylonRect.width,pylonRect.y+pylonRect.height-0.1*pylonRect.height);
    contour.push_back(p1);
    contour.push_back(p2);
    contour.push_back(p3);

    //get depth values in triangle
    double avg_depth_value = 0.0;
    int num = 0;
    int totalCount = 0;
    for(int xi = pylonRect.x;xi < pylonRect.x+pylonRect.width;xi++)
    {
        for(int yi = pylonRect.y;yi < pylonRect.y+pylonRect.height;yi++)
        {
            if(pointPolygonTest(contour, Point(xi,yi),false) < 0)
            {
                continue;
            }
            double currDepthValue = m_depthImage.at<tUInt16>(yi,xi);

            //reject 0 pixels
            totalCount++;
            if(currDepthValue == 0)continue;

            avg_depth_value += currDepthValue;
            num++;
        }
    }

    if(num/(totalCount*1.0) < 0.5)return -1;

    //get average depth image value
    avg_depth_value /= num;

    //convert to distance value [m] (linear conversion)
    double tempDistValue = 0.00003296391*avg_depth_value-0.002953348-0.05;
    return tempDistValue;
}
