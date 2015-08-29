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
#include "imageProcessing.h"

using namespace std;
using namespace cv;

#define PROJECTED_IMAGE_HEIGTH 250

ADTF_FILTER_PLUGIN("ImageProcessing", OID_ADTF_ImageProcessing, cImageProcessing)

cImageProcessing::cImageProcessing(const tChar* __info) : cFilter(__info),ipMapper()
{
    outputImage = cv::Mat(cv::Size(640,480+3*PROJECTED_IMAGE_HEIGTH+240), CV_8UC1);
    floorMask = cv::Mat(cv::Size(320,240), CV_8UC1,cv::Scalar::all(1));

    computeSobel=computeRemap=computeMask=true;

    SetPropertyBool("Compute Sobel", computeSobel);
    SetPropertyBool("Compute Remap", computeRemap);
    SetPropertyBool("Compute Floor Mask", computeMask);
    SetPropertyStr("Depth Image for Floor Mask","/home/odroid/AADC/calibration_files/floor.png");
    SetPropertyBool("Depth Image for Floor Mask" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Depth Image for Floor Mask" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "PNG Files (*.png)");
    SetPropertyStr("Camera Calibration Parameters","/home/odroid/AADC/calibration_files/cameraParameters.xml");
    SetPropertyBool("Camera Calibration Parameters" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Camera Calibration Parameters" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyBool("Compute Downscale", computeDownscale);
    SetPropertyFloat("Camera Pitch",16.0);
    SetPropertyFloat("Camera Pitch" NSSUBPROP_REQUIRED, tTrue);
}

cImageProcessing::~cImageProcessing()
{
#if defined(WIN32)
    cv::destroyWindow("imageProcessing");
#endif
}

tResult cImageProcessing::LoadConfigurationData(cFilename m_fileConfig)
{
    if (m_fileConfig.IsEmpty())
    {
        LOG_ERROR("ImageProcessing: Camera Calibration file not found");
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    //Load file, parse configuration, print the data
    if (cFileSystem::Exists(m_fileConfig))
    {
        cDOM oDOM;
        oDOM.Load(m_fileConfig);
        cDOMElement* pConfigElement;

        float fu=-1.0, fv=-1.0, cx=-1.0, cy=-1.0;

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
        if(fu<0 || fv<0 || cx<0 || cy<0)
        {
            LOG_ERROR("ImageProcessing: Camera Calibration file is missing parameters");
            RETURN_ERROR(ERR_INVALID_FILE);
        }
        ipMapper.initialize(200,PROJECTED_IMAGE_HEIGTH, fu, fv, cx, cy, pitch);
    }
    else
    {
        LOG_ERROR("ImageProcessing: Camera Calibration file not found");
        RETURN_ERROR(ERR_INVALID_FILE);
    }
    RETURN_NOERROR;
}

void cImageProcessing::fixVideoFormat(cv::Mat image)
{
    if(image.cols == m_sBitmapFormat.nWidth && image.rows == m_sBitmapFormat.nHeight && (image.channels()*8==m_sBitmapFormat.nBitsPerPixel))
        return;
    LOG_WARNING(adtf_util::cString::Format("Changing image format: [%dx%d], %d channels",image.cols,image.rows,image.channels()));
    m_sBitmapFormat.nWidth = image.cols;
    m_sBitmapFormat.nHeight = image.rows;
    m_sBitmapFormat.nBitsPerPixel = 8 * image.channels();
    m_sBitmapFormat.nPixelFormat = (image.channels() == 3) ? cImage::PF_RGB_888 : cImage::PF_GREYSCALE_8;
    m_sBitmapFormat.nBytesPerLine = image.cols * image.channels();
    m_sBitmapFormat.nSize = m_sBitmapFormat.nBytesPerLine * image.rows;
    m_sBitmapFormat.nPaletteSize = 0;
    m_oVideoOutputPin.SetFormat(&m_sBitmapFormat, NULL);
}

tResult cImageProcessing::Init(tInitStage eStage, __exception )
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        // Video Input
        RETURN_IF_FAILED(m_oVideoInputPin.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoInputPin));
        //Depth Input
        RETURN_IF_FAILED(m_iDepth.Create("Depth_Input", IPin::PD_Input, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_iDepth));

        //Video Output
        RETURN_IF_FAILED(m_oVideoOutputPin.Create("Video_Output",IPin::PD_Output, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideoOutputPin));
    }
    else if (eStage == StageNormal)
    {
        firstFrame = tTrue;

        //set the videoformat of the rgb video pin
        m_sBitmapFormat.nWidth = 640;
        m_sBitmapFormat.nHeight = 480;
        m_sBitmapFormat.nBitsPerPixel = 24;
        m_sBitmapFormat.nPixelFormat = cImage::PF_RGB_888;
        m_sBitmapFormat.nBytesPerLine = 640 * 3;
        m_sBitmapFormat.nSize = m_sBitmapFormat.nBytesPerLine * 480;
        m_sBitmapFormat.nPaletteSize = 0;
        m_oVideoOutputPin.SetFormat(&m_sBitmapFormat, NULL);

        m_sBitmapFormatDepth.nWidth = 320;
        m_sBitmapFormatDepth.nHeight = 240;
        m_sBitmapFormatDepth.nBitsPerPixel = 8 * 2;
        m_sBitmapFormatDepth.nPixelFormat = cImage::PF_GREYSCALE_16;
        m_sBitmapFormatDepth.nBytesPerLine = m_sBitmapFormatDepth.nWidth * 2;
        m_sBitmapFormatDepth.nSize = m_sBitmapFormatDepth.nBytesPerLine * m_sBitmapFormatDepth.nHeight;
        m_sBitmapFormatDepth.nPaletteSize = 0;
        //m_oDepth.SetFormat(&m_sBitmapFormatDepth, NULL);

        computeSobel = GetPropertyBool("Compute Sobel");
        computeRemap = GetPropertyBool("Compute Remap");
        computeMask = GetPropertyBool("Compute Floor Mask");
        computeDownscale = GetPropertyBool("Compute Downscale");
        pitch = GetPropertyFloat("Camera Pitch");

        cFilename fileConfig = GetPropertyStr("Depth Image for Floor Mask");

        ADTF_GET_CONFIG_FILENAME(fileConfig);
        fileConfig = fileConfig.CreateAbsolutePath(".");

        if (fileConfig.IsEmpty() || !(cFileSystem::Exists(fileConfig)))
        {
            LOG_ERROR("Depth Image Floor Mask not found");
            RETURN_ERROR(ERR_INVALID_FILE);
        }

        cFilename cameraCalib = GetPropertyStr("Camera Calibration Parameters");
        RETURN_IF_FAILED(LoadConfigurationData(cameraCalib));

        LOG_WARNING(adtf_util::cString::Format("ImageProcessing config: computeSobel: %s, computeRemap: %s, computeMask: %s, computeDownscale: %s, pitch: %f", computeSobel ? "true" : "false", computeRemap ? "true" : "false", computeMask ? "true" : "false", computeDownscale ? "true" : "false", pitch));
        depthAvg = cv::imread(string(fileConfig));
    }
    RETURN_NOERROR;
}

tResult cImageProcessing::PropertyChanged(const char* strProperty)
{
    RETURN_NOERROR;
}

tResult cImageProcessing::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
    switch (eStage)
    {
        case cFilter::StageFirst:
            break;
        default:
            break;
    }
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cImageProcessing::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
     RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if(nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        if(pSource == &m_oVideoInputPin) {
            if (firstFrame) {
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
        } else if(pSource == &m_iDepth) {
            if(computeMask)
                ProcessDepth(pMediaSample);
        }
    }
    if (nEventCode== IPinEventSink::PE_MediaTypeChanged) {
        if (pSource == &m_oVideoInputPin) {
            cObjectPtr<IMediaType> pType;
            RETURN_IF_FAILED(m_oVideoInputPin.GetMediaType(&pType));
            cObjectPtr<IMediaTypeVideo> pTypeVideo;
            RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
            UpdateImageFormat(m_oVideoInputPin.GetFormat());
        }
    }
    RETURN_NOERROR;
}

tResult cImageProcessing::ProcessInput(IMediaSample* pSample)
{
    // VideoInput
    RETURN_IF_POINTER_NULL(pSample);

    const tVoid* l_pSrcBuffer;

    if (IS_OK(pSample->Lock(&l_pSrcBuffer)))
    {
        IplImage* img = cvCreateImageHeader(cvSize(m_sInputFormat.nWidth, m_sInputFormat.nHeight), IPL_DEPTH_8U, 3);
        img->imageData = (char*)l_pSrcBuffer;
        Mat image(cvarrToMat(img));
        cvReleaseImage(&img);
        pSample->Unlock(l_pSrcBuffer);

        Mat gray     = outputImage(Rect(0,0,image.cols,image.rows));
        Mat remapped = outputImage(Rect(0,gray.rows,200,PROJECTED_IMAGE_HEIGTH));
        Mat sobel_remapped  = outputImage(Rect(0,gray.rows+remapped.rows,200,PROJECTED_IMAGE_HEIGTH));
        Mat mask_remapped = outputImage(Rect(0,gray.rows+remapped.rows+sobel_remapped.rows,200,PROJECTED_IMAGE_HEIGTH));
        Mat downscaled = outputImage(Rect(0,gray.rows+remapped.rows+sobel_remapped.rows+mask_remapped.rows,320,240));
        cvtColor(image,gray,COLOR_BGR2GRAY);

        if(computeRemap || computeSobel) {
            ipMapper.remap(gray, &remapped);
        }

        if(computeSobel) {
            Mat grad_x,grad_y;
            Mat abs_grad_x,abs_grad_y;
            Sobel(remapped,grad_x,remapped.depth(),1,0);
            convertScaleAbs(grad_x,abs_grad_x);
            Sobel(remapped,grad_y,remapped.depth(),0,1);
            convertScaleAbs(grad_y,abs_grad_y);
            addWeighted(abs_grad_x,0.5,abs_grad_y,0.5,0,sobel_remapped);
        }

        if(computeMask) {
            ipMapper.remap(floorMask, &mask_remapped, false);
        }

        if(computeDownscale) {
            resize(gray,downscaled,downscaled.size());
        }

        image = outputImage;
        cObjectPtr<IMediaSample> pNewRGBSample;
        if (IS_OK(AllocMediaSample(&pNewRGBSample)))
        {                            
            tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
            pNewRGBSample->Update(tmStreamTime, image.data, tInt32(image.channels()*image.cols*image.rows), 0);
            fixVideoFormat(image);
            RETURN_IF_FAILED(m_oVideoOutputPin.Transmit(pNewRGBSample));
        }    
    }

    RETURN_NOERROR;            
}

tResult cImageProcessing::ProcessDepth(IMediaSample* pSample)
{
    RETURN_IF_POINTER_NULL(pSample);
    adtf_util::cImage oImage;
    const tVoid* l_pSrcBuffer;
    if (IS_OK(pSample->Lock(&l_pSrcBuffer))) {
        oImage.SetBits((tUInt8*) l_pSrcBuffer, &m_sBitmapFormatDepth);
        pSample->Unlock(l_pSrcBuffer);
    }
    cv::Mat depthImage(cv::Size(m_sBitmapFormatDepth.nWidth,m_sBitmapFormatDepth.nHeight), CV_16UC1);

    memcpy(depthImage.data,oImage.GetBitmap(),m_sBitmapFormatDepth.nSize);
    for(int i=0; i<depthImage.cols;i++) {
        for(int j=0; j<depthImage.rows; j++) {
            if (depthImage.at<tUInt16>(j,i) != 0) {
                tUInt16 diff = abs(depthAvg.at<tUInt16>(j,i)-depthImage.at<tUInt16>(j,i));
                floorMask.at<tUInt8>(j,i) = (diff>=6000) ? 0 : 255;
            } else
                floorMask.at<tUInt8>(j,i) = 255; //Invalid pixels
        }
    }
    RETURN_NOERROR;
}

tResult cImageProcessing::ProcessFound()
{        
    RETURN_NOERROR;
}

tResult cImageProcessing::ProcessOutput()
{
    RETURN_NOERROR;
}

tResult cImageProcessing::UpdateImageFormat(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
        m_sInputFormat = (*pFormat);        
    RETURN_NOERROR;
}

