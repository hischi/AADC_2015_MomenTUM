/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: forchhe#$  $Date:: 2014-12-12 13:19:18#$ $Rev:: 29898   $
**********************************************************************/

#include "stdafx.h"
#include "cMarkerDetectFilter.h"
const double RAD_TO_DEG=180/M_PI;


ADTF_FILTER_PLUGIN("MTUM Maker Detection Free Demo", OID_MTUM_MARKERDETECTFILTER_FREEDEMO, cMarkerDetectFilter)



cMarkerDetectFilter::cMarkerDetectFilter(const tChar* __info):cFilter(__info)
{
    UCOM_REGISTER_TIMING_SPOT(cString(OIGetInstanceName()) + "::Process::Start", m_oProcessStart);

    SetPropertyBool("Debug Output", false);
    //initialize values for aruco detection
    m_TheMarkerSize = 0.12; //Side in [m] //m_TheMarkerSize = -1;

    SetPropertyStr("Dictionary File For Markers","/home/odroid/AADC/calibration_files/roadsign.yml");
    SetPropertyBool("Dictionary File For Markers" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Dictionary File For Markers" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "YML Files (*.yml)");

    //SetPropertyBool("downsample image",false);

    SetPropertyInt("compute every X frame",3);

    SetPropertyInt("Video Output Pin",2);
    SetPropertyStr("Video Output Pin" NSSUBPROP_VALUELIST, "1@None|2@Erkannte Zeichen|");

    SetPropertyStr("Camera Calibration Parameters","/home/odroid/AADC/calibration_files/cameraParameters.xml");
    SetPropertyBool("Camera Calibration Parameters" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Camera Calibration Parameters" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");

    m_bDownsampleImage = false; // If true use 320x240 image
    computeEveryXFrame=3; //0: Compute every frame, 1: Compute every other frame, ...
    frameCounter=0;

    UCOM_REGISTER_TIMING_SPOT(cString(OIGetInstanceName()) + "::Process::End", m_oProcessEnd);
    int markers[] = {46, 82, 140, 166, 306, 340, 371, 376, 466, 484, 491};
    validMarkers = std::set<int>(markers,markers+sizeof(markers)/sizeof(int));
}

cMarkerDetectFilter::~cMarkerDetectFilter()
{
}

tResult cMarkerDetectFilter::LoadConfigurationData(cFilename m_fileConfig)
{
    if (m_fileConfig.IsEmpty())
    {
        LOG_ERROR("MarkerDetectFilter: Camera Calibration file not found");
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    //Load file, parse configuration, print the data
    if (cFileSystem::Exists(m_fileConfig))
    {
        cDOM oDOM;
        oDOM.Load(m_fileConfig);
        cDOMElement* pConfigElement;

        float fu=-1.0, fv=-1.0, cx=-1.0, cy=-1.0, k1=0.0, k2=0.0, p1=0.0, p2=0.0, k3=0.0;

        if (IS_OK(oDOM.FindNode("camera/fu", pConfigElement))) {
            fu = float(cString(pConfigElement->GetData()).AsFloat64());
        }
        if (IS_OK(oDOM.FindNode("camera/fv", pConfigElement))) {
            fv = float(cString(pConfigElement->GetData()).AsFloat64());
        }
        if (IS_OK(oDOM.FindNode("camera/cx", pConfigElement))) {
            cx = float(cString(pConfigElement->GetData()).AsFloat64());
        }
        if (IS_OK(oDOM.FindNode("camera/cy", pConfigElement))) {
            cy = float(cString(pConfigElement->GetData()).AsFloat64());
        }
        if (IS_OK(oDOM.FindNode("camera/distortion/k1", pConfigElement))) {
            k1 = float(cString(pConfigElement->GetData()).AsFloat64());
        }
        if (IS_OK(oDOM.FindNode("camera/distortion/k2", pConfigElement))) {
            k2 = float(cString(pConfigElement->GetData()).AsFloat64());
        }
        if (IS_OK(oDOM.FindNode("camera/distortion/p1", pConfigElement))) {
            p1 = float(cString(pConfigElement->GetData()).AsFloat64());
        }
        if (IS_OK(oDOM.FindNode("camera/distortion/p2", pConfigElement))) {
            p2 = float(cString(pConfigElement->GetData()).AsFloat64());
        }
        if (IS_OK(oDOM.FindNode("camera/distortion/k3", pConfigElement))) {
            k3 = float(cString(pConfigElement->GetData()).AsFloat64());
        }
        if(fu<0 || fv<0 || cx<0 || cy<0)
        {
            LOG_ERROR("MarkerDetectFilter: Camera Calibration file is missing parameters");
            RETURN_ERROR(ERR_INVALID_FILE);
        }

        if(m_bDownsampleImage) {
            Mat cameraMatrix = (Mat_<float>(3,3) << fu/2.0,0.0,cx/2.0,0.0,fv/2.0,cy/2.0,0.0,0.0,1.0);
            cv::Mat distortion = cv::Mat::zeros(4,1,CV_32FC1);
            m_TheCameraParameters.setParams(cameraMatrix,distortion,cv::Size(320,240));
        } else {
            Mat cameraMatrix = (Mat_<float>(3,3) << fu,0.0,cx,0.0,fv,cy,0.0,0.0,1.0);
            cv::Mat distortion = (Mat_<float>(4,1) << k1, k2, p1, p2);
            m_TheCameraParameters.setParams(cameraMatrix,distortion,cv::Size(640,480));
        }
    }
    else
    {
        LOG_ERROR("MarkerDetectFilter: Camera Calibration file not found");
        RETURN_ERROR(ERR_INVALID_FILE);
    }
    RETURN_NOERROR;
}

tResult cMarkerDetectFilter::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));
    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));


        RETURN_IF_FAILED(m_inputPin_enable.Create("EnablePin", new cMediaType(0, 0, 0, "tSignalValue")  , static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_inputPin_enable));

        //this is a VideoPin
        RETURN_IF_FAILED(m_oPinInputVideo.Create("Video_RGB_input",IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oPinInputVideo));

        //this is a VideoPin
        RETURN_IF_FAILED(m_oPinOutputVideo.Create("Video_RGB_output", IPin::PD_Output, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oPinOutputVideo));



        //this is a OutputPin
        tChar const * strDesc = pDescManager->GetMediaDescription("tRoadSign");
        if(!strDesc)
            LOG_ERROR(cString(OIGetInstanceName()) + ": Could not load mediadescription tRoadSign, check path");
        RETURN_IF_POINTER_NULL(strDesc);
        cObjectPtr<IMediaType> pType = new cMediaType(0, 0, 0, "tRoadSign", strDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_oPinRoadSign.Create("RoadSign", pType, this));
        RETURN_IF_FAILED(RegisterPin(&m_oPinRoadSign));
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescRoadSign));

        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal));

//        LOG_INFO("muh");
//        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
//        RETURN_IF_POINTER_NULL(strDescSignalValue);
//        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
//        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal));

//        RETURN_IF_FAILED(m_inputPin_enable.Create("Enable", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
//        RETURN_IF_FAILED(RegisterPin(&m_inputPin_enable));
//        LOG_INFO("MUHMUH");

    }
    else if (eStage == StageNormal)
    {

        m_outputMode = GetPropertyInt("Video Output Pin");

        //m_bDownsampleImage = GetPropertyBool("downsample image");
        computeEveryXFrame = GetPropertyInt("compute every X frame");

        m_bDebugModeEnabled = GetPropertyBool("Debug Output");


        //Get path of configuration file
        cFilename fileConfig = GetPropertyStr("Dictionary File For Markers");

        ADTF_GET_CONFIG_FILENAME(fileConfig);
        fileConfig = fileConfig.CreateAbsolutePath(".");

        if (fileConfig.IsEmpty() || !(cFileSystem::Exists(fileConfig)))
        {
            LOG_ERROR("Dictionary File For Markers not found");
            RETURN_ERROR(ERR_INVALID_FILE);
        }


        if(m_Dictionary.fromFile(string(fileConfig))==false) {
            LOG_ERROR("Dictionary File For Markers not found");
        };

        if(m_Dictionary.size()==0) {
            LOG_ERROR("Dictionary File For Markers not found");
        };

        HighlyReliableMarkers::loadDictionary(m_Dictionary);

        cFilename cameraCalib = GetPropertyStr("Camera Calibration Parameters");
        RETURN_IF_FAILED(LoadConfigurationData(cameraCalib));
    }
    else if (eStage == StageGraphReady)
    {
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(m_oPinInputVideo.GetMediaType(&pType));

        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));


        UpdateInputImageFormat(pTypeVideo->GetFormat());
        UpdateOutputImageFormat(pTypeVideo->GetFormat());

        m_oPinInputVideo.SetFormat(&m_sInputFormat, NULL);

        m_MDetector.setMakerDetectorFunction(aruco::HighlyReliableMarkers::detect);
        m_MDetector.setThresholdParams( 21, 7);
        m_MDetector.setCornerRefinementMethod(aruco::MarkerDetector::LINES);
        m_MDetector.setWarpSize((m_Dictionary[0].n()+2)*8);
        m_MDetector.setMinMaxSize(0.005, 0.5);

        doDetect = true;

    }
    RETURN_NOERROR;
}

tResult cMarkerDetectFilter::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cMarkerDetectFilter::OnPinEvent(IPin* pSource,
                                        tInt nEventCode,
                                        tInt nParam1,
                                        tInt nParam2,
                                        IMediaSample* pMediaSample)
{
    switch (nEventCode)
    {
    case IPinEventSink::PE_MediaSampleReceived:
    {
        if (pSource == &m_oPinInputVideo)
        {
            UCOM_TIMING_SPOT(m_oProcessStart);
            if(frameCounter>=computeEveryXFrame && doDetect) {
                ProcessVideo(pMediaSample);
                frameCounter=0;
            } else {
                frameCounter++;
            }

            UCOM_TIMING_SPOT(m_oProcessEnd);
        } else if(pSource == &m_inputPin_enable){

            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoderInput));

            //write values with zero
            tFloat32 enable = 0;
            tUInt32 timeStamp = 0;

            //get values from media sample
            pCoderInput->Get("f32Value", (tVoid*)&enable);
            pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderDescSignal->Unlock(pCoderInput);

            if(enable == 1)
            {
                doDetect = true;
            }
            else
            {
                doDetect = false;
            }
         }
        break;
    }
    case IPinEventSink::PE_MediaTypeChanged:
    {
        if (pSource == &m_oPinInputVideo)
        {
            cObjectPtr<IMediaType> pType;
            RETURN_IF_FAILED(m_oPinInputVideo.GetMediaType(&pType));

            cObjectPtr<IMediaTypeVideo> pTypeVideo;
            RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));

            UpdateInputImageFormat(m_oPinInputVideo.GetFormat());
            UpdateOutputImageFormat(m_oPinInputVideo.GetFormat());
        }
        break;
    }
    default:
    {
        break;
    }
    }
    RETURN_NOERROR;
}

void cMarkerDetectFilter::fixVideoFormat(cv::Mat image)
{
    if(image.cols == m_sOutputFormat.nWidth && image.rows == m_sOutputFormat.nHeight && (image.channels()*8==m_sOutputFormat.nBitsPerPixel))
        return;
    LOG_WARNING(adtf_util::cString::Format("Changing image format: [%dx%d], %d channels",image.cols,image.rows,image.channels()));
    m_sOutputFormat.nWidth = image.cols;
    m_sOutputFormat.nHeight = image.rows;
    m_sOutputFormat.nBitsPerPixel = 8 * image.channels();
    m_sOutputFormat.nPixelFormat = (image.channels() == 3) ? cImage::PF_RGB_888 : cImage::PF_GREYSCALE_8;
    m_sOutputFormat.nBytesPerLine = image.cols * image.channels();
    m_sOutputFormat.nSize = m_sOutputFormat.nBytesPerLine * image.rows;
    m_sOutputFormat.nPaletteSize = 0;
    m_oPinOutputVideo.SetFormat(&m_sOutputFormat, NULL);
}



tResult cMarkerDetectFilter::ProcessVideo(adtf::IMediaSample* pISample)
{

    RETURN_IF_POINTER_NULL(pISample);

    //creating new media sample for output
    cObjectPtr<IMediaSample> pNewSample;
    RETURN_IF_FAILED(_runtime->CreateInstance(OID_ADTF_MEDIA_SAMPLE, IID_ADTF_MEDIA_SAMPLE, (tVoid**) &pNewSample));
    RETURN_IF_FAILED(pNewSample->AllocBuffer(m_sOutputFormat.nSize));

    //creating new pointer for input data
    const tVoid* l_pSrcBuffer;
    Mat TheInputImage;
    Mat downsample;
    //receiving data from inputsample, and saving to inputFrame
    if (IS_OK(pISample->Lock(&l_pSrcBuffer)))
    {
        //convert to mat
        TheInputImage  = Mat(m_sInputFormat.nHeight,m_sInputFormat.nWidth,CV_8UC3,(tVoid*)l_pSrcBuffer,m_sInputFormat.nBytesPerLine);
        if(m_bDownsampleImage) {
            downsample = Mat (cv::Size(320,240), CV_8UC3);
            resize(TheInputImage,downsample,downsample.size());
            TheInputImage = downsample;
        }
        m_MDetector.detect(TheInputImage,m_TheMarkers,m_TheCameraParameters,m_TheMarkerSize);
        pISample->Unlock(l_pSrcBuffer);

        //print marker info and draw the markers in image
        short closestValidMarkerIndex=-1;
        double closestValidMarkerDistance=std::numeric_limits<double>::max();
        for (unsigned int i=0;i<m_TheMarkers.size();i++)
        {
            if (m_bDebugModeEnabled) LOG_INFO(cString::Format("Marker found %d",m_TheMarkers[i].id));
//            if (m_outputMode!=1) { m_TheMarkers[i].draw(TheInputImage,Scalar(0,0,255),1);
//                putText(
//                            TheInputImage,
//                            cv::String(cString::Format("%.2f  %.2f  %.2f",m_TheMarkers[i].Tvec.at<float>(0),m_TheMarkers[i].Tvec.at<float>(1),m_TheMarkers[i].Tvec.at<float>(2)).GetPtr()),
//                            Point2f(TheInputImage.cols*0.5,50),
//                            CV_FONT_NORMAL,
//                            1.0,
//                            Scalar(0,0,255));
//            }
            //LOG_WARNING(adtf_util::cString::Format("%d => T: (%.03f,%.03f,%.03f)", m_TheMarkers[i].id, m_TheMarkers[i].Tvec.at<float>(0),m_TheMarkers[i].Tvec.at<float>(1),m_TheMarkers[i].Tvec.at<float>(2)));
            if((validMarkers.find(m_TheMarkers[i].id) != validMarkers.end())&&(norm(m_TheMarkers[i].Tvec)<closestValidMarkerDistance)) {
                cv::Mat rot;
                cv::Rodrigues(m_TheMarkers[i].Rvec, rot);
                float yaw=atan2(rot.at<float>(3),rot.at<float>(0))*RAD_TO_DEG; //roll
                float pitch=atan2(-rot.at<float>(6),sqrt(pow(rot.at<float>(7),2)+pow(rot.at<float>(8),2)))*RAD_TO_DEG;
                float roll=atan2(rot.at<float>(7),rot.at<float>(8))*RAD_TO_DEG; //yaw
                //LOG_WARNING(adtf_util::cString::Format("%.03f %.03f %.03f",yaw,pitch,roll));
//                putText(
//                            TheInputImage,
//                            cv::String(cString::Format("%.2f  %.2f  %.2f",yaw,pitch,roll).GetPtr()),
//                            Point2f(TheInputImage.cols*0.5,100),
//                            CV_FONT_NORMAL,
//                            1.0,
//                            Scalar(0,0,255));
              if(yaw>=70&&yaw<=110 && abs(pitch)<=35){ //  if(yaw>=70&&yaw<=100 && abs(pitch)<=25 && abs(roll)>=130&&abs(roll)<=180) {
                    closestValidMarkerDistance = norm(m_TheMarkers[i].Tvec);
                    closestValidMarkerIndex = i;
                }
            }
        }
        if(closestValidMarkerIndex>=0){

            markerDetected(m_TheMarkers[closestValidMarkerIndex],pISample->GetTime());

            //if(m_outputMode!=1) m_TheMarkers[closestValidMarkerIndex].draw(TheInputImage,Scalar(0,255,0),1);
        }else sendRoadSignStruct(0, Marker(), pISample->GetTime());
    }

    //update new media sample with image data if something has to be transmitted
//    if (m_outputMode!=1)
//    {
//        fixVideoFormat(TheInputImage);
//        pNewSample->Update(pISample->GetTime(), TheInputImage.data, m_sOutputFormat.nSize, 0);
//        m_oPinOutputVideo.Transmit(pNewSample);
//    }
    // send a sample which indicates that no sign was detectec in this frame
    //if (!(m_SignInFrame)) sendRoadSignStruct(0,0,pISample->GetTime());

    RETURN_NOERROR;
}

tResult cMarkerDetectFilter::UpdateInputImageFormat(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {
        m_sInputFormat = (*pFormat);

        LOG_INFO(adtf_util::cString::Format("Marker Detection Filter: Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d", m_sInputFormat.nWidth,m_sInputFormat.nHeight,m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize, m_sInputFormat.nPixelFormat));

    }

    RETURN_NOERROR;
}

tResult cMarkerDetectFilter::UpdateOutputImageFormat(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
    {
        m_sOutputFormat = (*pFormat);

        LOG_INFO(adtf_util::cString::Format("Marker Detection Filter: Output: Size %d x %d ; BPL %d ; Size %d, PixelFormat; %d", m_sOutputFormat.nWidth,m_sOutputFormat.nHeight,m_sOutputFormat.nBytesPerLine, m_sOutputFormat.nSize, m_sOutputFormat.nPixelFormat));

        m_oPinOutputVideo.SetFormat(&m_sOutputFormat, NULL);
    }

    RETURN_NOERROR;
}

tResult cMarkerDetectFilter::markerDetected(Marker marker, tTimeStamp timeOfFrame)
{
    switch (marker.id)
    {
    
    case 166:
        //Vorgeschriebene Fahrtrichtung geradeaus
        sendRoadSignStruct(1, marker, timeOfFrame); break;
    case 371:
        //Vorfahrt an naechster Kreuzung
        sendRoadSignStruct(2, marker, timeOfFrame); break;
    case 140:
        //Halt! Vorfahrt gewaehren (Stop)
        sendRoadSignStruct(3, marker, timeOfFrame); break;
    case 484:
        //Parken
        sendRoadSignStruct(4, marker, timeOfFrame); break;
    case 491:
        //Vorfahrt gewaehren
        sendRoadSignStruct(5, marker, timeOfFrame); break;
    case 466:
        //Kreuzung
        sendRoadSignStruct(6, marker, timeOfFrame); break;
    case 46:
        //Kreisverkehr
        sendRoadSignStruct(7, marker, timeOfFrame); break;
//    case 376:
//        //Fussgaengerueberweg
//        sendRoadSignStruct(7, marker, timeOfFrame); break;
//    case 340:
//        //Ueberholverbot
//        sendRoadSignStruct(9, marker, timeOfFrame); break;
//    case 306:
//        //Verbot der Einfahrt
//        sendRoadSignStruct(10, marker, timeOfFrame); break;
//    case 82:
//        //Einbahnstrasse
//        sendRoadSignStruct(11, marker, timeOfFrame); break;
    default:
        break;

    }
    RETURN_NOERROR;
}

tResult cMarkerDetectFilter::sendRoadSignStruct(tInt8 ID, Marker marker, tTimeStamp timeOfFrame)
{
    if (ID != 0) m_SignInFrame = tTrue;

    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescRoadSign->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();

    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

    cObjectPtr<IMediaCoder> pCoder;
    RETURN_IF_FAILED(m_pCoderDescRoadSign->WriteLock(pMediaSample, &pCoder));

    pCoder->Set("i8Identifier", (tVoid*) &(ID));
    pCoder->Set("fl32TransX", (tVoid*)&marker.Tvec.at<float>(0));
    pCoder->Set("fl32TransY", (tVoid*)&marker.Tvec.at<float>(1));
    pCoder->Set("fl32TransZ", (tVoid*)&marker.Tvec.at<float>(2));
    pCoder->Set("fl32RotX", (tVoid*)&marker.Rvec.at<float>(0));
    pCoder->Set("fl32RotY", (tVoid*)&marker.Rvec.at<float>(1));
    pCoder->Set("fl32RotZ", (tVoid*)&marker.Rvec.at<float>(2));

    RETURN_IF_FAILED(m_pCoderDescRoadSign->Unlock(pCoder));

    RETURN_IF_FAILED(pMediaSample->SetTime(timeOfFrame));
    RETURN_IF_FAILED(m_oPinRoadSign.Transmit(pMediaSample));
    if (m_bDebugModeEnabled) LOG_INFO(cString::Format("Zeichen ID %d erkannt. Size: %f",ID,marker.ssize));

    RETURN_NOERROR;
}

