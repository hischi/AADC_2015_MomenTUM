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
#include "crossingDetection_ML.h"

using namespace std;
using namespace cv;

#define PAINT_OUTPUT

ADTF_FILTER_PLUGIN("CrossingDetection_ML", OID_ADTF_CrossingDetection_ML, cCrossingDetection_ML)

#define PROJECTED_IMAGE_HEIGTH 250

cCrossingDetection_ML::cCrossingDetection_ML(const tChar* __info) : cAsyncDataTriggeredFilter(__info),cModel()
{
    pic_count = 0;
    m_enabled = true;
}

cCrossingDetection_ML::~cCrossingDetection_ML()
{

}

void cCrossingDetection_ML::resetSystem()
{
    cModel = CrossingModel();
}

tResult cCrossingDetection_ML::Init(tInitStage eStage, __exception )
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

        // State Input
        RETURN_IF_FAILED(m_inputPin_state.Create("State", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_inputPin_state));

        // SignLocation Input
        tChar const * strMarkerInfoValue = pDescManager->GetMediaDescription("tRoadSign");
        RETURN_IF_POINTER_NULL(strMarkerInfoValue);
        cObjectPtr<IMediaType> pTypeSignInfoValue = new cMediaType(0, 0, 0, "tRoadSign", strMarkerInfoValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignInfoValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSign));
        RETURN_IF_FAILED(m_inputPin_signLocation.Create("RoadSign", pTypeSignInfoValue  , static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_inputPin_signLocation));

		// Video Input
		RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));

        //Video Output
        RETURN_IF_FAILED(m_oVideoOutputPin.Create("Video_Output",IPin::PD_Output, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin));

        //Distance output
        tChar const * strDescDistanceValue = pDescManager->GetMediaDescription("tCrossingInfo");
        RETURN_IF_POINTER_NULL(strDescDistanceValue);
        cObjectPtr<IMediaType> pTypeDistanceInfoValue = new cMediaType(0, 0, 0, "tCrossingInfo", strDescDistanceValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeDistanceInfoValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescCrossing));

        RETURN_IF_FAILED(m_oDistanceOutputPin.Create("crossing_info", pTypeDistanceInfoValue  , static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oDistanceOutputPin));
        

        //create car motion change input pins

        tChar const * strDescMotionData = pDescManager->GetMediaDescription("tMotionDataExt");
        RETURN_IF_POINTER_NULL(strDescMotionData);
        cObjectPtr<IMediaType> pTypeMotionData = new cMediaType(0, 0, 0, "tMotionDataExt", strDescMotionData,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeMotionData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescMotionData));


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


	}
	RETURN_NOERROR;
}

tResult cCrossingDetection_ML::PropertyChanged(const char* strProperty)
{
    RETURN_NOERROR;
}

tResult cCrossingDetection_ML::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
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

tResult cCrossingDetection_ML::ProcessSign(IMediaSample* pSample)
{
    RETURN_IF_POINTER_NULL(pSample);
    tInt8 id;
    cv::Mat tVec = cv::Mat::zeros(3,1,CV_32FC1);
    cv::Mat rVec = cv::Mat::zeros(3,1,CV_32FC1);
    tUInt32 timeStamp;
    ReceiveRoadSign(pSample, &id, &tVec, &rVec, &timeStamp);

    bool lostSign = false;
    if(id == 0)lostSign = true;
    //LOG_WARNING(adtf_util::cString::Format("(%d) T: (%.03f,%.03f,%.03f) R: (%.03f,%.03f,%.03f)", id, tVec.at<float>(0),tVec.at<float>(1),tVec.at<float>(2), rVec.at<float>(0),rVec.at<float>(1),rVec.at<float>(2)));

    //send sign data in cars frame [cm] => times 100
    cModel.signDataUpdate(tVec.at<float>(0)*100,tVec.at<float>(2)*100,rVec.at<float>(1),lostSign);


    RETURN_NOERROR;
}

tResult cCrossingDetection_ML::OnAsyncPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

 	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);

	if(nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
        if(pSource == &m_inputPin_state)
        {
            tFloat32 value;
            tUInt32 timestamp;
            ReceiveSignalValueMessage(pMediaSample, &value, &timestamp);

            if(value == 1)
            {
                m_enabled = true;
                resetSystem();
            }
            else
            {
                m_enabled = false;
            }
        }

        if(pSource == &m_oVideoInputPin)
        {
            if(!m_enabled)
                RETURN_NOERROR;

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

        if(pSource == &m_inputPin_signLocation)
        {
                ProcessSign(pMediaSample);
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

tResult cCrossingDetection_ML::ProcessInput(IMediaSample* pSample)
{
		// VideoInput
		RETURN_IF_POINTER_NULL(pSample);
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

           // GaussianBlur(transformedImage,transformedImage,Size(5,5),2);

            //do clahe equalization
            //cv::Ptr<cv::CLAHE> clahe = createCLAHE();
            //clahe->setClipLimit(4);
            //clahe->apply(grayscale,grayscale);

            //do standard equalization
            //equalizeHist(transformedImage, transformedImage);

            Mat transformedCol;
            cvtColor(transformedImage,transformedCol,CV_GRAY2BGR);

            cModel.cameraUpdate(transformedImage,sobeledImage,groundPlane);

            tUInt32 stamp = 0;
            tInt8 type = 0;
            //SendSignalValueMessage(&m_oDistanceOutputPin,cModel.getDistToCrossing(),stamp);
            if(cModel.cType == cModel.UNKNOWN)
                type = 0;
            else if(cModel.cType == cModel.X_CROSSING)
                type = 1;
            else if(cModel.cType == cModel.T_CROSSING)
            {
                if(cModel.tcType == UNKNOWN_T)
                    type = 0;
                else if(cModel.tcType == DEG_90) // Dont care about front
                    type = 2;
                else if(cModel.leftIsOpen())
                    type = 3;
                else
                    type = 4;
            }

            sendCrossingInfo(type, cModel.getDistToCrossing(), cModel.angularOffset, stamp);




#ifdef PAINT_OUTPUT
            Point2d carCenter(100,0);
            Mat ipmColor = transformedCol.clone();
            cModel.paintDebugInfo(ipmColor);

            circle(ipmColor,Point(3,ipmColor.rows-1-80),2,Scalar(0,255,255),-1);
            circle(ipmColor,Point(3,ipmColor.rows-1),2,Scalar(0,255,255),-1);
            circle(ipmColor,Point(ipmColor.cols-3,ipmColor.rows-1),2,Scalar(0,255,255),-1);
            circle(ipmColor,Point(ipmColor.cols-3,ipmColor.rows-1-80),2,Scalar(0,255,255),-1);
            circle(ipmColor,Point(126,50),2,Scalar(0,255,255),-1);
            circle(ipmColor,Point(74,50),2,Scalar(0,255,255),-1);

            ipmColor.copyTo(generalOutputImage(Rect(0,0,ipmColor.cols,ipmColor.rows)));

#endif




#ifdef PAINT_OUTPUT
            cObjectPtr<IMediaSample> pNewRGBSample;
            if (IS_OK(AllocMediaSample(&pNewRGBSample)))
            {
                tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
                pNewRGBSample->Update(tmStreamTime, generalOutputImage.data, tInt32(3*800*300), 0);
                RETURN_IF_FAILED(m_oVideoOutputPin.Transmit(pNewRGBSample));
            }
#endif


        }


	RETURN_NOERROR;			
}

tResult cCrossingDetection_ML::ProcessFound()
{		
	RETURN_NOERROR;
}

tResult cCrossingDetection_ML::ProcessOutput()
{
	RETURN_NOERROR;
}



tResult cCrossingDetection_ML::UpdateImageFormat(const tBitmapFormat* pFormat)
{
	if (pFormat != NULL)
	{
		m_sInputFormat = (*pFormat);		
        //LOG_INFO(adtf_util::cString::Format("Spurerkennung Filter Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d",m_sInputFormat.nWidth,m_sInputFormat.nHeight,
            //m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize, m_sInputFormat.nPixelFormat));
	}
	
	RETURN_NOERROR;
}



tResult cCrossingDetection_ML::ReceiveMotionDataMessage(IMediaSample* pMediaSample, tFloat32 *pdY, tFloat32 *pdPhi, tUInt32 *pTimeStamp)
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

tResult cCrossingDetection_ML::ReceiveSignalValueMessage(IMediaSample* pMediaSample, tFloat32 *pValue, tUInt32 *pTimeStamp)
{
    // read-out the incoming Media Sample
    cObjectPtr<IMediaCoder> pCoderInput;
    RETURN_IF_FAILED(m_pCoderDescValue->Lock(pMediaSample, &pCoderInput));

    *pValue = 0;
    *pTimeStamp = 0;

    //get values from media sample
    pCoderInput->Get("f32Value", (tVoid*)pValue);
    pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)pTimeStamp);
    m_pCoderDescValue->Unlock(pCoderInput);

    RETURN_NOERROR;
}

tResult cCrossingDetection_ML::SendSignalValueMessage(cObjectPtr<cOutputPin> pOutputPin, tFloat32 value, tUInt32 timeStamp)
{
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleOut;
    AllocMediaSample((tVoid**)&pMediaSampleOut);
    
    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescValue->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleOut->AllocBuffer(nSize);
    
    //write date to the media sample with the coder of the descriptor
    cObjectPtr<IMediaCoder> pCoderOutput;
    m_pCoderDescValue->WriteLock(pMediaSampleOut, &pCoderOutput);
    
    pCoderOutput->Set("f32Value", (tVoid*)&(value));
    pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
    m_pCoderDescValue->Unlock(pCoderOutput);
    
    //transmit media sample over output pin
    tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
    pMediaSampleOut->SetTime(tmStreamTime);
    pOutputPin->Transmit(pMediaSampleOut);
    
    RETURN_NOERROR;
}

tResult cCrossingDetection_ML::ReceiveRoadSign(IMediaSample* pMediaSample, tInt8 *pId, Mat *pTvec, Mat* pRvec, tUInt32 *pTimeStamp)
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

tResult cCrossingDetection_ML::sendCrossingInfo(tInt8 type, tFloat32 distance, tFloat32 orientation, tTimeStamp timeOfFrame)
{
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescCrossing->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();

    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

    cObjectPtr<IMediaCoder> pCoder;
    RETURN_IF_FAILED(m_pCoderDescCrossing->WriteLock(pMediaSample, &pCoder));
    pCoder->Set("i8Type", (tVoid*) &(type));
    pCoder->Set("f32Distance", (tVoid*)&distance);
    pCoder->Set("f32Orientation", (tVoid*)&orientation);
    pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timeOfFrame);
    RETURN_IF_FAILED(m_pCoderDescCrossing->Unlock(pCoder));

    RETURN_IF_FAILED(pMediaSample->SetTime(pMediaSample->GetTime()));
    RETURN_IF_FAILED(m_oDistanceOutputPin.Transmit(pMediaSample));
    RETURN_NOERROR;
}


