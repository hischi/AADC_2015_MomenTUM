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
#include "parkingSpaceDetection.h"

using namespace std;
using namespace cv;

#define PAINT_OUTPUT

ADTF_FILTER_PLUGIN("ParkingDetection", OID_ADTF_PARKING_DETECTION, cParkingDetection)

#define PROJECTED_IMAGE_HEIGTH 250

cParkingDetection::cParkingDetection(const tChar* __info) : cFilter(__info),pDetector(24,Point(100,40),Point(200,PROJECTED_IMAGE_HEIGTH)),pModel(ParkingSpaceModel::CROSS)
{
    doDetect = false;
}

cParkingDetection::~cParkingDetection()
{

}

tResult cParkingDetection::Init(tInitStage eStage, __exception )
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

	if (eStage == StageFirst)
	{
		// Video Input
		RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));

        //Video Output
        RETURN_IF_FAILED(m_oVideoOutputPin.Create("Video_Output",IPin::PD_Output, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin));

        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal));

        tChar const * strDescParking = pDescManager->GetMediaDescription("tParkingData");
        RETURN_IF_POINTER_NULL(strDescParking);
        cObjectPtr<IMediaType> pTypeParking = new cMediaType(0, 0, 0, "tParkingData", strDescParking,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeParking->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescParking));

        RETURN_IF_FAILED(m_outputPin_distToLot.Create("Parking_Data", pTypeParking  , static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_outputPin_distToLot));

        RETURN_IF_FAILED(m_inputPin_reset.Create("reset", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_inputPin_reset));
        //Motion Data input
        tChar const * strDescMotionData = pDescManager->GetMediaDescription("tMotionDataExt");
        RETURN_IF_POINTER_NULL(strDescMotionData);
        cObjectPtr<IMediaType> pTypeMotionData = new cMediaType(0, 0, 0, "tMotionDataExt", strDescMotionData,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeMotionData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescMotionData));

        RETURN_IF_FAILED(m_inputPin_Motion.Create("Motion_Data", pTypeMotionData  , static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_inputPin_Motion));

	}
	else if (eStage == StageNormal)
	{
        firstFrame = tTrue;

        //set the videoformat of the rgb video pin (output)
        m_sBitmapFormat.nWidth = 800;
        m_sBitmapFormat.nHeight = 300;
		m_sBitmapFormat.nBitsPerPixel = 24;
		m_sBitmapFormat.nPixelFormat = cImage::PF_RGB_888;
        m_sBitmapFormat.nBytesPerLine = 800 * 3;
        m_sBitmapFormat.nSize = 3* 800 * 300;
		m_sBitmapFormat.nPaletteSize = 0;
		m_oVideoOutputPin.SetFormat(&m_sBitmapFormat, NULL);

        //input
        cObjectPtr<IMediaType> pType;
        RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));
        cObjectPtr<IMediaTypeVideo> pTypeVideo;
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
        const tBitmapFormat* pFormat = pTypeVideo->GetFormat();
        if (pFormat == NULL)
        {
            LOG_ERROR("Parkin detection: No Bitmap information found on pin \"input\"");
            RETURN_ERROR(ERR_NOT_SUPPORTED);
        }
//        m_sInputFormat.nPixelFormat = cImage::PF_RGB_888;
//        m_sInputFormat.nWidth = 640;
//        m_sInputFormat.nHeight =  480;
//        m_sInputFormat.nBitsPerPixel = 24;
//        m_sInputFormat.nBytesPerLine = 640*3;
//        m_sInputFormat.nSize = 3*640*480;
//        m_sInputFormat.nPaletteSize = 0;
//        m_oVideoInputPin.SetFormat(&m_sInputFormat,NULL);

        m_sInputFormat.nPixelFormat = pFormat->nPixelFormat;
        m_sInputFormat.nWidth = pFormat->nWidth;
        m_sInputFormat.nHeight =  pFormat->nHeight;
        m_sInputFormat.nBitsPerPixel = pFormat->nBitsPerPixel;
        m_sInputFormat.nBytesPerLine = pFormat->nBytesPerLine;
        m_sInputFormat.nSize = pFormat->nSize;
        m_sInputFormat.nPaletteSize = pFormat->nPaletteSize;
	}
	RETURN_NOERROR;
}

tResult cParkingDetection::PropertyChanged(const char* strProperty)
{
    RETURN_NOERROR;
}

tResult cParkingDetection::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
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

tResult cParkingDetection::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

 	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);

	if(nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		if(pSource == &m_oVideoInputPin)
		{
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
        else if( pSource == &m_inputPin_Motion){

            // read-out the incoming Media Sample of tMotionDataExt type
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescMotionData->Lock(pMediaSample, &pCoderInput));
            tFloat32 phi = 0;
            tFloat32 inDY = 0;
            tUInt32 timeStamp = 0;
            pCoderInput->Get("f32dY", (tVoid*)&inDY);
            pCoderInput->Get("f32dPhi", (tVoid*)&phi);
            pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderDescMotionData->Unlock(pCoderInput);

            if(!doDetect)RETURN_NOERROR;

                pModel.setMotion(inDY,phi);



        }
        else if(pSource == &m_inputPin_reset)
        {
            cObjectPtr<IMediaCoder> pCoderInput;
            RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoderInput));

            //write values with zero
            tFloat32 resetSignal = 0;
            tUInt32 timeStamp = 0;

            //get values from media sample
            pCoderInput->Get("f32Value", (tVoid*)&resetSignal);
            pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderDescSignal->Unlock(pCoderInput);

            if(resetSignal == 1)
            {
                doDetect = true;
                pModel = ParkingSpaceModel(ParkingSpaceModel::CROSS);
            }
            else if(resetSignal == 2)
            {
                doDetect = true;
                pModel = ParkingSpaceModel(ParkingSpaceModel::PARALLEL);
            }
//            else if(resetSignal == 3)
//            {
//                doDetect = true;
//                pModel = ParkingSpaceModel(ParkingSpaceModel::RESET_CROSS);
//            }
//            else if(resetSignal == 4)
//            {
//                doDetect = true;
//                pModel = ParkingSpaceModel(ParkingSpaceModel::RESET_PARALLEL);
//            }
            else
            {
                doDetect = false;
            }
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

tResult cParkingDetection::ProcessInput(IMediaSample* pSample)
{
    if(!doDetect)RETURN_NOERROR;

		// VideoInput
		RETURN_IF_POINTER_NULL(pSample);
		const tVoid* l_pSrcBuffer;
		if (IS_OK(pSample->Lock(&l_pSrcBuffer)))
        {

            Mat image;
            image  = Mat(m_sInputFormat.nHeight,m_sInputFormat.nWidth,CV_8UC1,(tVoid*)l_pSrcBuffer,m_sInputFormat.nBytesPerLine);

            Mat transformedImage = image(Rect(0,480,200,PROJECTED_IMAGE_HEIGTH)).clone();
            Mat sobeledImage     = image(Rect(0,480+250,200,PROJECTED_IMAGE_HEIGTH)).clone();
            Mat groundPlane      = image(Rect(0,480+2*250,200,PROJECTED_IMAGE_HEIGTH)).clone();

            pSample->Unlock(l_pSrcBuffer);
            //convert to opencv mat
//            Mat image;
//            image  = Mat(m_sInputFormat.nHeight,m_sInputFormat.nWidth,CV_8UC3,(tVoid*)l_pSrcBuffer,m_sInputFormat.nBytesPerLine);
//            //save the original image data
//            Mat originalImage = image.clone();
//            pSample->Unlock(l_pSrcBuffer);

            //create an output image for debugging
            Mat generalOutputImage(300,800,CV_8UC3,Scalar(0,0,0));

//            //convert inputcurvature image to grayscale
//            Mat debugColor;
//            cvtColor(grayscale,debugColor,COLOR_GRAY2BGR);

//            Mat transformedImage = ipmapper.remap(grayscale);

            GaussianBlur(transformedImage,transformedImage,Size(3,3),1.0);


            vector<Point2d> points = pDetector.detect(transformedImage,sobeledImage,groundPlane);


#ifdef PAINT_OUTPUT
            //---------------------- DEBUG OUTPUT LANE MARKINGS---------------------------------//
            //plot the detected road markings to the debug output image
            Mat transformedImagePaintable = transformedImage.clone();
            cvtColor(transformedImagePaintable,transformedImagePaintable,CV_GRAY2BGR);

            //draw the points to the remapped image and circle image
            for(int i = 0;i < points.size();i++)
            {
                circle(transformedImagePaintable, points.at(i), 1, Scalar(0,0,255),-1);
            }

            transformedImagePaintable.copyTo(generalOutputImage(Rect(0,0,transformedImagePaintable.cols,transformedImagePaintable.rows)));
            //---------------------- END DEBUG OUTPUT LANE MARKINGS------------------------------//
#endif


            pModel.updateModel(points);

            tInt32 lot_index;
            tFloat32 dist = (tFloat32) pModel.getDistanceNextParkingLot(lot_index);
            tFloat32 angle = pModel.getAngle();

            tUInt32 timeStamp = 0;
//            if(pModel.parkingType == ParkingSpaceModel::CROSS)
//                LOG_INFO(cString::Format("distance next lot: %f , lotID: %d, angle: %f-- numlost: %d,  type: CROSS",dist,lot_index,angle,pModel.numLost));
//            else if(pModel.parkingType == ParkingSpaceModel::PARALLEL)
//                LOG_INFO(cString::Format("distance next lot: %f , lotID: %d, angle: %f-- numlost: %d,  type: PARALLEL",dist,lot_index,angle,pModel.numLost));
//            else
//                LOG_WARNING("parking type error");

            //create new media sample
            cObjectPtr<IMediaSample> pMediaSampleOut;
            AllocMediaSample((tVoid**)&pMediaSampleOut);

            //allocate memory with the size given by the descriptor
            cObjectPtr<IMediaSerializer> pSerializer;
            m_pCoderDescParking->GetMediaSampleSerializer(&pSerializer);
            tInt nSize = pSerializer->GetDeserializedSize();
            pMediaSampleOut->AllocBuffer(nSize);

            //write date to the media sample with the coder of the descriptor
            cObjectPtr<IMediaCoder> pCoderOutput;
            m_pCoderDescParking->WriteLock(pMediaSampleOut, &pCoderOutput);

            pCoderOutput->Set("f32dist", (tVoid*)&(dist));
            pCoderOutput->Set("f32angle", (tVoid*)&(angle));
            pCoderOutput->Set("i32lotID", (tVoid*)&(lot_index));
            pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            m_pCoderDescParking->Unlock(pCoderOutput);

            //transmit media sample over output pin
            pMediaSampleOut->SetTime(GetTime());
            m_outputPin_distToLot.Transmit(pMediaSampleOut);



#ifdef PAINT_OUTPUT
            //---------------------- DEBUG OUTPUT PARKING MODEL---------------------------------//
            //plot the detected road markings to the debug output image
            Mat debug_mat_model = pModel.getDebugImage();
            debug_mat_model.copyTo(generalOutputImage(Rect(200,0,debug_mat_model.cols,debug_mat_model.rows)));
            //---------------------- END DEBUG OUTPUT LANE MARKINGS------------------------------//
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

tResult cParkingDetection::ProcessFound()
{		
	RETURN_NOERROR;
}

tResult cParkingDetection::ProcessOutput()
{
	RETURN_NOERROR;
}


tResult cParkingDetection::UpdateImageFormat(const tBitmapFormat* pFormat)
{
	if (pFormat != NULL)
	{
		m_sInputFormat = (*pFormat);		
        //LOG_INFO(adtf_util::cString::Format("Spurerkennung Filter Input: Size %d x %d ; BPL %d ; Size %d , PixelFormat; %d",m_sInputFormat.nWidth,m_sInputFormat.nHeight,
            //m_sInputFormat.nBytesPerLine, m_sInputFormat.nSize, m_sInputFormat.nPixelFormat));
	}
	
	RETURN_NOERROR;
}

tTimeStamp cParkingDetection::GetTime()
{
    return (_clock != NULL) ? _clock->GetStreamTime(): adtf_util::cHighResTimer::GetTime();
}
