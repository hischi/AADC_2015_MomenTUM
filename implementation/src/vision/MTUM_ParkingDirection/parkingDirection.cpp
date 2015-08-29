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
#include "parkingDirection.h"

using namespace std;
using namespace cv;

#define PAINT_OUTPUT

ADTF_FILTER_PLUGIN("ParkingDirection", OID_ADTF_ParkingDirection, cParkingDirection)

#define PROJECTED_IMAGE_HEIGTH 250

cParkingDirection::cParkingDirection(const tChar* __info) : cFilter(__info),pModel()
{
    pic_count = 0;
    enabled = false;
}

cParkingDirection::~cParkingDirection()
{

}

void cParkingDirection::resetSystem()
{
    pModel.reset();
}

tResult cParkingDirection::Init(tInitStage eStage, __exception )
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

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

		// Video Input
		RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));

        //Video Output
        RETURN_IF_FAILED(m_oVideoOutputPin.Create("Video_Output",IPin::PD_Output, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin));

        //Distance output
        tChar const * strDescDistanceValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescDistanceValue);

        pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescDistanceValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescValue_send));

        RETURN_IF_FAILED(m_oDirectionOutputPin.Create("direction", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oDirectionOutputPin));
        

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

        pic_count = 0;
        enabled = false;
	}
	RETURN_NOERROR;
}

tResult cParkingDirection::PropertyChanged(const char* strProperty)
{
    RETURN_NOERROR;
}

tResult cParkingDirection::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
	switch (eStage)
		{
        case cFilter::StageFirst:
		{
			break;
		}
		default:
			break;
		}
    return cFilter::Shutdown(eStage,__exception_ptr);
}


tResult cParkingDirection::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
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
                enabled = true;
                resetSystem();
            }
            else
                enabled = false;
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

            if(enabled)
                ProcessInput(pMediaSample);
            //RETURN_IF_FAILED(m_oVideoOutputPin.Transmit(pMediaSample));
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

tResult cParkingDirection::ProcessInput(IMediaSample* pSample)
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

            Mat transformedCol;
            cvtColor(transformedImage,transformedCol,CV_GRAY2BGR);

            /*
            tUInt32 stamp = 0;
            SendSignalValueMessage(&m_oDistanceOutputPin,cModel.getDistToCrossing(),stamp);*/

            pModel.cameraUpdate(transformedImage,sobeledImage,groundPlane);

            tFloat32 parallel = pModel.certaintyParallel / (pModel.certaintyParallel+pModel.certaintyCross);
            tFloat32 cross = pModel.certaintyCross / (pModel.certaintyParallel+pModel.certaintyCross);


                //LOG_INFO(cString::Format("ParallelC: %f, CrossC: %f",parallel,cross));

            if(parallel > cross)
                SendSignalValueMessage(&m_oDirectionOutputPin, 2, 0);
            else if(cross > parallel)
                SendSignalValueMessage(&m_oDirectionOutputPin, 1, 0);
            else
                SendSignalValueMessage(&m_oDirectionOutputPin, 0, 0);



#ifdef PAINT_OUTPUT
            Point2d carCenter(100,0);
            Mat ipmColor = transformedCol.clone();
            pModel.paintDebugInfo(ipmColor);

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

tResult cParkingDirection::ProcessFound()
{		
	RETURN_NOERROR;
}

tResult cParkingDirection::ProcessOutput()
{
	RETURN_NOERROR;
}

tResult cParkingDirection::UpdateImageFormat(const tBitmapFormat* pFormat)
{
	if (pFormat != NULL)
	{
		m_sInputFormat = (*pFormat);		
        //LOG_INFO(adtf_util::cString::Format("Spurerkennung Filter Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d",m_sInputFormat.nWidth,m_sInputFormat.nHeight,
            //m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize, m_sInputFormat.nPixelFormat));
	}
	
	RETURN_NOERROR;
}


tResult cParkingDirection::ReceiveSignalValueMessage(IMediaSample* pMediaSample, tFloat32 *pValue, tUInt32 *pTimeStamp)
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

tResult cParkingDirection::SendSignalValueMessage(cObjectPtr<cOutputPin> pOutputPin, tFloat32 value, tUInt32 timeStamp)
{
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleOut;
    AllocMediaSample((tVoid**)&pMediaSampleOut);
    
    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescValue_send->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleOut->AllocBuffer(nSize);
    
    //write date to the media sample with the coder of the descriptor
    cObjectPtr<IMediaCoder> pCoderOutput;
    m_pCoderDescValue_send->WriteLock(pMediaSampleOut, &pCoderOutput);
    
    pCoderOutput->Set("f32Value", (tVoid*)&(value));
    pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
    m_pCoderDescValue_send->Unlock(pCoderOutput);
    
    //transmit media sample over output pin
    tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
    pMediaSampleOut->SetTime(tmStreamTime);
    pOutputPin->Transmit(pMediaSampleOut);
    
    RETURN_NOERROR;
}
