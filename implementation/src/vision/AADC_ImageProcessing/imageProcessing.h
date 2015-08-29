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

#ifndef _ImageProcessing_FILTER_HEADER_
#define _ImageProcessing_FILTER_HEADER_
#include "IPMapper.h"

#define OID_ADTF_ImageProcessing  "adtf.aadc.ImageProcessing"

class cImageProcessing : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_ImageProcessing, "AADC ImageProcessing", adtf::OBJCAT_Tool, "AADC ImageProcessing", 1, 0, 0, "Beta Version");    
protected:
    cVideoPin    m_oVideoInputPin;    /**< the input pin for the video*/
    cVideoPin    m_iDepth;            /**< the input pin from the depth image */

    //RGB image output pin
    cVideoPin      m_oVideoOutputPin; /**< the output for the video showing the detected lines*/ 

    tResult ReceiveSignalValueMessage(IMediaSample* pMediaSample, tFloat32 *pValue, tUInt32 *pTimeStamp);

public:
    cImageProcessing(const tChar*);
    virtual ~cImageProcessing();
    tResult Init(tInitStage eStage, __exception=NULL);
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr=NULL);
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
    tResult ProcessInput(IMediaSample* pSample);
    tResult ProcessDepth(IMediaSample* pSample);
    tResult ProcessFound();
    tResult ProcessOutput();
    tResult PropertyChanged(const char* strProperty);
    tResult UpdateImageFormat(const tBitmapFormat* pFormat);
    void fixVideoFormat(cv::Mat image);
    tResult LoadConfigurationData(cFilename m_fileConfig);

    tBitmapFormat    m_sBitmapFormat;      /**< bitmap format for the RGB video */
    tBitmapFormat    m_sBitmapFormatDepth; /**< bitmap format for the Depth video */

private:
    tBool firstFrame;                    /**< flag for the first frame*/
    tBitmapFormat m_sInputFormat;        /**< bitmap format of the input image*/
    cv::Mat outputImage;
    IPMapper ipMapper;
    cv::Mat depthAvg;
    cv::Mat floorMask;
    tBool computeSobel;
    tBool computeRemap;
    tBool computeMask;
    tBool computeDownscale;
    tFloat pitch;

    cObjectPtr<IMediaTypeDescription> m_pCoderDescValue;
};

#endif 
