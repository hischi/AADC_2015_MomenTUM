/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/


/**
 *
 *AADC_Spurerkennung
 *
 *BRICKL_CHRISTOPH, AEV
 */

#include "stdafx.h"
#include "laneDetection.h"

using namespace std;
using namespace cv;

#define PAINT_OUTPUT

ADTF_FILTER_PLUGIN("LaneDetection", OID_ADTF_LaneDetection, cLaneDetection)

#define PROJECTED_IMAGE_HEIGTH 160

cLaneDetection::cLaneDetection(const tChar* __info) : cAsyncDataTriggeredFilter(__info),detector(16,Point(0,40),Point(200,PROJECTED_IMAGE_HEIGTH)),model(true)
{
    m_Busy = false;
    m_LastValue = 0;
}

cLaneDetection::~cLaneDetection()
{

}

tResult cLaneDetection::Init(tInitStage eStage, __exception )
{
    RETURN_IF_FAILED(cAsyncDataTriggeredFilter::Init(eStage, __exception_ptr));

	if (eStage == StageFirst)
	{
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescValue));
        cObjectPtr<IMediaType> pTypeSignalValueOut = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValueOut->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescValueOut));
        cObjectPtr<IMediaType> pTypeSignalValueOt = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValueOt->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescValueOvertaking));

        // State Input
        RETURN_IF_FAILED(m_inputPin_state.Create("state", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_inputPin_state));

        RETURN_IF_FAILED(m_inputPin_useOtherLane.Create("useOtherLane", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_inputPin_useOtherLane));

		// Video Input
		RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));

        //Video Output
        RETURN_IF_FAILED(m_oVideoOutputPin.Create("Video_Output",IPin::PD_Output, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin));

        //Lane Info50
        tChar const * strDescLaneInfoValue = pDescManager->GetMediaDescription("tLaneInfo");
        RETURN_IF_POINTER_NULL(strDescLaneInfoValue);
        cObjectPtr<IMediaType> pTypeLaneInfoValue = new cMediaType(0, 0, 0, "tLaneInfo", strDescLaneInfoValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeLaneInfoValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescLaneInfo));

        RETURN_IF_FAILED(m_oLaneDataOutputPin.Create("laneInfo", pTypeLaneInfoValue  , static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oLaneDataOutputPin));
        
        //IsCurveOutput
        RETURN_IF_FAILED(m_oIsCurvePin.Create("isCurve", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oIsCurvePin));

        //create car motion change input pins

        tChar const * strDescMotionData = pDescManager->GetMediaDescription("tMotionDataExt");
        RETURN_IF_POINTER_NULL(strDescMotionData);
        cObjectPtr<IMediaType> pTypeMotionData = new cMediaType(0, 0, 0, "tMotionDataExt", strDescMotionData,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeMotionData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescMotionData));

        RETURN_IF_FAILED(m_inputPin_motionData.Create("motionData", pTypeMotionData  , static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_inputPin_motionData));

	}
	else if (eStage == StageNormal)
	{
        firstFrame = tTrue;
        imagecount = 0;


        //set the videoformat of the rgb video pin
        m_sBitmapFormat.nWidth = 800;
        m_sBitmapFormat.nHeight = 300;
		m_sBitmapFormat.nBitsPerPixel = 24;
		m_sBitmapFormat.nPixelFormat = cImage::PF_RGB_888;
        m_sBitmapFormat.nBytesPerLine = 800 * 3;
        m_sBitmapFormat.nSize = 3* 800 * 300;
		m_sBitmapFormat.nPaletteSize = 0;
		m_oVideoOutputPin.SetFormat(&m_sBitmapFormat, NULL);

        m_Busy = false;
        resetSystem();
        m_LastValue = 0;
	}
	RETURN_NOERROR;
}

tResult cLaneDetection::PropertyChanged(const char* strProperty)
{
    RETURN_NOERROR;
}

tResult cLaneDetection::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
	switch (eStage)
		{
        case cAsyncDataTriggeredFilter::StageFirst:
		{
			break;
		}
		default:
			break;
		}
    return cAsyncDataTriggeredFilter::Shutdown(eStage,__exception_ptr);
}

tResult cLaneDetection::OnAsyncPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

 	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);

	if(nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
        if(pSource == &m_inputPin_state)
        {
            tFloat32 value;
            tUInt32 timestamp;
            ReceiveSignalValueMessage(pMediaSample,m_pCoderDescValue, &value, &timestamp);

            if(value == 1 && m_LastValue != 1)
            {
                resetSystem();
            }
            m_LastValue = value;
        }

        if(pSource == &m_inputPin_useOtherLane)
        {
            tFloat32 value = 70;
            tUInt32 timestamp;
            int res = (int)ReceiveSignalValueMessage(pMediaSample,m_pCoderDescValueOvertaking, &value, &timestamp);
//            LOG_INFO(cString::Format("CHANGE LANE : %d   %f     cur: %d   res : %d",value,value,model.getCurrentLane(),res));

            if(model.getCurrentLane() != (value))
            {
                LOG_INFO("Change LANE");
                model.changeLane();
            }
        }

		if(pSource == &m_oVideoInputPin)
		{
			//Videoformat
			if (firstFrame)
			{		
				cObjectPtr<IMediaType> pType;
				RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));
				cObjectPtr<IMediaTypeVideo> pTypeVideo;
				RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
				const tBitmapFormat* pFormat = pTypeVideo->GetFormat();								
				if (pFormat == NULL)
				{
					LOG_ERROR("Spurerkennung: No Bitmap information found on pin \"input\"");
					RETURN_ERROR(ERR_NOT_SUPPORTED);
				}
				m_sInputFormat.nPixelFormat = pFormat->nPixelFormat;
				m_sInputFormat.nWidth = pFormat->nWidth;
				m_sInputFormat.nHeight =  pFormat->nHeight;
				m_sInputFormat.nBitsPerPixel = pFormat->nBitsPerPixel;
				m_sInputFormat.nBytesPerLine = pFormat->nBytesPerLine;
				m_sInputFormat.nSize = pFormat->nSize;
				m_sInputFormat.nPaletteSize = pFormat->nPaletteSize;
				firstFrame = tFalse;
			}

			ProcessInput(pMediaSample);
            //RETURN_IF_FAILED(m_oVideoOutputPin.Transmit(pMediaSample));
		} 

        if(pSource == &m_inputPin_motionData)
        {
            tFloat32 dY;
            tFloat32 dPhi;
            tUInt32  timestamp;
            ReceiveMotionDataMessage(pMediaSample, &dY, &dPhi, &timestamp);

            //model.applyCarMovementImproved(dY,-dPhi);
            //circularModel.applyCarMovement(dY,-dPhi);
        }
	}
	if (nEventCode== IPinEventSink::PE_MediaTypeChanged)
    {
            if (pSource == &m_oVideoInputPin)
            {
                    cObjectPtr<IMediaType> pType;
                    RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));
                    cObjectPtr<IMediaTypeVideo> pTypeVideo;
                    RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
                    UpdateImageFormat(m_oVideoInputPin.GetFormat());
            }
    }
RETURN_NOERROR;
}

//re-initialize the whole system
void cLaneDetection::resetSystem()
{
    model = LaneModel(false);
}

tResult cLaneDetection::ProcessInput(IMediaSample* pSample)
{

    pthread_mutex_lock(&m_busy_mux);
        if(m_Busy)
        {
            pthread_mutex_unlock(&m_busy_mux);
            RETURN_NOERROR;
        }
        else
            m_Busy = true;
    pthread_mutex_unlock(&m_busy_mux);

    // VideoInput
    cObjectPtr<IMediaSample> pNewRGBSample;
    const tVoid* l_pSrcBuffer;

    if (IS_OK(pSample->Lock(&l_pSrcBuffer)))
    {

        Mat image;
        image  = Mat(m_sInputFormat.nHeight,m_sInputFormat.nWidth,CV_8UC1,(tVoid*)l_pSrcBuffer,m_sInputFormat.nBytesPerLine);

        Mat transformedImage = image(Rect(0,480,200,PROJECTED_IMAGE_HEIGTH)).clone();
        Mat sobeledImage     = image(Rect(0,480+250,200,PROJECTED_IMAGE_HEIGTH)).clone();
        Mat groundPlane      = image(Rect(0,480+2*250,200,PROJECTED_IMAGE_HEIGTH)).clone();

        pSample->Unlock(l_pSrcBuffer);


        //create an output image for debugging
        Mat generalOutputImage(300,800,CV_8UC3,Scalar(0,0,0));

        //do clahe equalization
        //cv::Ptr<cv::CLAHE> clahe = createCLAHE();
        //clahe->setClipLimit(4);
        //clahe->apply(grayscale,grayscale);

        //do standard equalization
        //equalizeHist(transformedImage, transformedImage);

        //detect lane markings
        vector<Point2d> laneMarkings = detector.detect(transformedImage,sobeledImage,groundPlane);


#ifdef PAINT_OUTPUT
        //---------------------- DEBUG OUTPUT LANE MARKINGS---------------------------------//
        //plot the detected road markings to the debug output image
        Mat transformedImagePaintable = transformedImage.clone();
        cvtColor(transformedImagePaintable,transformedImagePaintable,CV_GRAY2BGR);
        for(int i = 0;i < (int)laneMarkings.size();i++)
        {
            circle(transformedImagePaintable,laneMarkings.at(i),1,Scalar(0,0,255),-1);
        }

        Point2d p1(4,160-1);
        Point2d p2(195,160-1);
        Point2d p3(126,50);
        Point2d p4(74,50);
        line(transformedImagePaintable,p1,p2,Scalar(0,200,0));
        line(transformedImagePaintable,p2,p3,Scalar(0,200,0));
        line(transformedImagePaintable,p3,p4,Scalar(0,200,0));
        line(transformedImagePaintable,p4,p1,Scalar(0,200,0));


        transformedImagePaintable.copyTo(generalOutputImage(Rect(0,0,transformedImagePaintable.cols,transformedImagePaintable.rows)));
        //---------------------- END DEBUG OUTPUT LANE MARKINGS------------------------------//
#endif

        //use the detected lane markings to find contours in the image
        Mat circleImage(PROJECTED_IMAGE_HEIGTH,200,CV_8UC1,Scalar(0));
        for(int i = 0;i < (int)laneMarkings.size();i++)
        {
            circle(circleImage,laneMarkings.at(i),3,Scalar(255),-1);
        }

        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        findContours(circleImage,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point(0,0));
        ContourModel cModel;
        bool midLaneFound = cModel.update(contours,laneMarkings);
        vector<vector<Point2d> > nicelyGroupedPoints = cModel.points;


#ifdef PAINT_OUTPUT
        //---------------------- DEBUG OUTPUT CONTOURS ---------------------------------//
        Mat contourImagePaintable = transformedImage.clone();
        cvtColor(contourImagePaintable,contourImagePaintable,CV_GRAY2BGR);
        //draw the detected contours on the output image
        for(int i = 0;i < (int)cModel.points.size();i++)
        {
            Scalar color(255,255,255);
            if(i == 0)color = Scalar(0,0,255);if(i == 1)color = Scalar(0,255,0);if(i == 2)color = Scalar(255,0,0);
            if(i == 3)color = Scalar(255,255,0);if(i == 4)color = Scalar(255,0,255);if(i == 5)color = Scalar(255,255,0);

            vector<Point2d> pointGroup = cModel.points.at(i);
            for(int j = 0;j < (int)pointGroup.size();j++)
            {
                Point2d currP = pointGroup.at(j);
                currP.x += 100;
                circle(contourImagePaintable,currP,1,color,-1);
            }
        }
        contourImagePaintable.copyTo(generalOutputImage(Rect(200,0,contourImagePaintable.cols,contourImagePaintable.rows)));
        //---------------------- END DEBUG OUTPUT CONTOURS------------------------------//
#endif

        // LOG_INFO(cString::Format("Found: %d",midLaneFound));
        model.improvedUpdate(&nicelyGroupedPoints,midLaneFound);

        // LOG_INFO(cString::Format("CERTAINTY: %d",model.certainty));


        // SEND OUT OLD LANE INFORMATION
        double curvature; //1/cm
        double distanceRightLane; //cm
        double angle; //rad
        bool isCurve;
        model.getCarState(&curvature,&distanceRightLane,&angle,&isCurve);
        SendLaneInfo(curvature, distanceRightLane-1, angle, 0);

        //send is curve info:
        tUInt32 stamp = 0;
        SendSignalValueMessage(&m_oIsCurvePin, isCurve, stamp);


#ifdef PAINT_OUTPUT


        //---------------------- DEBUG OUTPUT LANE MODEL---------------------------------//
        int carOffset = 50;
        Mat laneModelDrawing(PROJECTED_IMAGE_HEIGTH+carOffset,200,CV_8UC3,Scalar(0,0,0));
        Mat transformedImageCopy = transformedImage.clone();
        cvtColor(transformedImageCopy,transformedImageCopy,CV_GRAY2BGR);
        transformedImageCopy.copyTo(laneModelDrawing(Rect(0,carOffset,transformedImageCopy.cols,transformedImageCopy.rows)));
        model.getDebugImage(laneModelDrawing);
        laneModelDrawing.copyTo(generalOutputImage(Rect(400,0,laneModelDrawing.cols,laneModelDrawing.rows)));
        //---------------------- END DEBUG OUTPUT LANE MODEL------------------------------//
#endif


        #ifdef PAINT_OUTPUT
        cObjectPtr<IMediaSample> pNewRGBSample;
        if (IS_OK(AllocMediaSample(&pNewRGBSample)))
        {
            tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
            pNewRGBSample->Update(tmStreamTime, generalOutputImage.data, tInt32(3*800*300), 0);
            m_oVideoOutputPin.Transmit(pNewRGBSample);
        }
        #endif
    }

    pthread_mutex_lock(&m_busy_mux);
        m_Busy = false;
    pthread_mutex_unlock(&m_busy_mux);
    RETURN_NOERROR;
}

tResult cLaneDetection::ProcessFound()
{		
	RETURN_NOERROR;
}

tResult cLaneDetection::ProcessOutput()
{
	RETURN_NOERROR;
}



tResult cLaneDetection::UpdateImageFormat(const tBitmapFormat* pFormat)
{
	if (pFormat != NULL)
	{
		m_sInputFormat = (*pFormat);		
        //LOG_INFO(adtf_util::cString::Format("Spurerkennung Filter Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d",m_sInputFormat.nWidth,m_sInputFormat.nHeight,
            //m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize, m_sInputFormat.nPixelFormat));
	}
	
	RETURN_NOERROR;
}


tResult cLaneDetection::SendLaneInfo(tFloat32 curvature, tFloat32 lanePos, tFloat32 carOrientation, tUInt32 timeStamp)
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

tResult cLaneDetection::ReceiveMotionDataMessage(IMediaSample* pMediaSample, tFloat32 *pdY, tFloat32 *pdPhi, tUInt32 *pTimeStamp)
{
    // read-out the incoming Media Sample
    cObjectPtr<IMediaCoder> pCoderInput;
    RETURN_IF_FAILED(m_pCoderDescValue->Lock(pMediaSample, &pCoderInput));

    *pdY = 0;
    *pdPhi = 0;
    *pTimeStamp = 0;

    //get values from media sample
    pCoderInput->Get("f32dY", (tVoid*)pdY);
    pCoderInput->Get("f32dPhi", (tVoid*)pdPhi);
    pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)pTimeStamp);
    m_pCoderDescValue->Unlock(pCoderInput);

    RETURN_NOERROR;
}

tResult cLaneDetection::ReceiveSignalValueMessage(IMediaSample* pMediaSample,cObjectPtr<IMediaTypeDescription> mediaDesc, tFloat32 *pValue, tUInt32 *pTimeStamp)
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

tResult cLaneDetection::SendSignalValueMessage(cObjectPtr<cOutputPin> pOutputPin, tFloat32 value, tUInt32 timeStamp)
{
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleOut;
    AllocMediaSample((tVoid**)&pMediaSampleOut);
    
    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescValueOut->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleOut->AllocBuffer(nSize);
    
    //write date to the media sample with the coder of the descriptor
    cObjectPtr<IMediaCoder> pCoderOutput;
    m_pCoderDescValueOut->WriteLock(pMediaSampleOut, &pCoderOutput);
    
    pCoderOutput->Set("f32Value", (tVoid*)&(value));
    pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
    m_pCoderDescValueOut->Unlock(pCoderOutput);
    
    //transmit media sample over output pin
    tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
    pMediaSampleOut->SetTime(tmStreamTime);
    pOutputPin->Transmit(pMediaSampleOut);
    
    RETURN_NOERROR;
}


