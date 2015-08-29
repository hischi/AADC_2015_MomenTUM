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

// DepthGrid.cpp : Transforms information from the depth image to an obstacle map
//

#include "DepthGrid.h"

ADTF_FILTER_PLUGIN("MTUM Depth Grid", OID_MTUM_DEPTHGRID, DepthGrid)

DepthGrid::DepthGrid(const tChar* __info) : cAsyncDataTriggeredFilter(__info)
{
	SetPropertyStr("Configurationfile depthsensor", "/home/odroid/AADC/calibration_files/calibrationDepth.xml");
	SetPropertyBool("Configurationfile depthsensor" NSSUBPROP_FILENAME, tTrue);
	SetPropertyStr("Configurationfile depthsensor" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");

    m_dY = 0;

    m_motion_mux = PTHREAD_MUTEX_INITIALIZER;
    m_input_mux = PTHREAD_MUTEX_INITIALIZER;

}

DepthGrid::~DepthGrid()
{

}


tResult DepthGrid::CreateInputPins(__exception)
{	
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
    tChar const * strDescMotion = pDescManager->GetMediaDescription("tMotionDataExt");
    RETURN_IF_POINTER_NULL(strDescMotion);
    cObjectPtr<IMediaType> pTypeMotion = new cMediaType(0, 0, 0, "tMotionDataExt", strDescMotion,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeMotion->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescMotion));

    //depth image
    RETURN_IF_FAILED(m_inputPin_depthImage.Create("depth_image", adtf::IPin::PD_Input  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_depthImage));

    //motion
    RETURN_IF_FAILED(m_inputPin_motion.Create("motion_data", pTypeMotion, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_motion));

	RETURN_NOERROR;
}

tResult DepthGrid::CreateOutputPins(__exception)
{
    RETURN_IF_FAILED(m_outputPin_depth_grid.Create("depth_grid", adtf::IPin::PD_Output  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_depth_grid));
 
    RETURN_NOERROR;
}

tResult DepthGrid::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cAsyncDataTriggeredFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
	{
		CreateInputPins(__exception_ptr);
		CreateOutputPins(__exception_ptr);
	}
    else if (eStage == StageNormal)
    {
        int width = TILE_COUNT_LEFT + TILE_COUNT_RIGHT;
        int height = TILE_COUNT_FRONT + TILE_COUNT_REAR;

        //preset mat
        m_depthGrid = *(new Mat(height,width,CV_8UC1,Scalar(GRID_BASE_VALUE)));

        //setup bitmap format
        m_sBitmapFormatGrid.nWidth = width;
        m_sBitmapFormatGrid.nHeight = height;
        m_sBitmapFormatGrid.nBitsPerPixel = 8;
        m_sBitmapFormatGrid.nPixelFormat = cImage::PF_GREYSCALE_8;
        m_sBitmapFormatGrid.nBytesPerLine = width;
        m_sBitmapFormatGrid.nSize = width*height;
        m_sBitmapFormatGrid.nPaletteSize = 0;

        m_sBitmapFormatDepthIn.nWidth = 320;
        m_sBitmapFormatDepthIn.nHeight = 240;
        m_sBitmapFormatDepthIn.nBitsPerPixel = 16;
        m_sBitmapFormatDepthIn.nPixelFormat = cImage::PF_GREYSCALE_16;
        m_sBitmapFormatDepthIn.nBytesPerLine = 640;
        m_sBitmapFormatDepthIn.nSize = 640*240;
        m_sBitmapFormatDepthIn.nPaletteSize = 0;

        //apply bitmap format to video pins
        m_outputPin_depth_grid.SetFormat(&m_sBitmapFormatGrid, NULL);
        m_inputPin_depthImage.SetFormat(&m_sBitmapFormatDepthIn, NULL);

        sensDepth = new DepthSensor();
        sensDepth->LoadConfigurationData(GetPropertyStr("Configurationfile depthsensor"),m_sBitmapFormatDepthIn);
        m_depthImage = *(new Mat(m_sBitmapFormatDepthIn.nHeight,m_sBitmapFormatDepthIn.nWidth,CV_16UC1));
    }
    RETURN_NOERROR;
}

tResult DepthGrid::Start(__exception)
{
    return cAsyncDataTriggeredFilter::Start(__exception_ptr);
}

tResult DepthGrid::Stop(__exception)
{
    return cAsyncDataTriggeredFilter::Stop(__exception_ptr);
}

tResult DepthGrid::Shutdown(tInitStage eStage, __exception)
{
    return cAsyncDataTriggeredFilter::Shutdown(eStage,__exception_ptr);
}
tResult DepthGrid:: OnAsyncPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample)
{
    if(pSource == &m_inputPin_motion)
    {
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

        pthread_mutex_lock(&m_motion_mux);
        m_dY -= inDY;
        m_phi -= phi;// *360/2/PI;             //phi from radiant to degree
        pthread_mutex_unlock(&m_motion_mux);

    }

    if (pSource == &m_inputPin_depthImage)
    {
        RETURN_IF_POINTER_NULL(pMediaSample);
        if(pthread_mutex_trylock(&m_input_mux) != 0)
        {
            LOG_INFO("SKIPPED");
            RETURN_NOERROR;
        }
        //read media sample and write to depth image data (openCV Mat)
        const tVoid* l_pSrcBuffer;
        if (IS_OK(pMediaSample->Lock(&l_pSrcBuffer)))
        {
            m_depthImage = Mat(m_sBitmapFormatDepthIn.nHeight,m_sBitmapFormatDepthIn.nWidth,CV_8UC1,(tVoid*)l_pSrcBuffer,m_sBitmapFormatDepthIn.nBytesPerLine);

            //rotation

            if(abs(m_dY/GRID_CM_PER_UNIT) > m_depthGrid.cols/2.2 -1)
            {
                pthread_mutex_lock(&m_motion_mux);
                m_dY = 0;
                m_phi = 0;
                pthread_mutex_unlock(&m_motion_mux);
            }
            if(abs(m_dY) > 1)
            {
                //build representing triangle to estimate shift
                Point2f srcTri[3],dstTri[3];
                tFloat32 triShift = 20;

                srcTri[0] = Point2f(TILE_COUNT_RIGHT,     TILE_COUNT_REAR);
                srcTri[1] = srcTri[0] + Point2f(sin(PI/4) * triShift, cos(PI/4) * triShift);
                srcTri[2] = srcTri[0] + Point2f(sin(-PI/4) * triShift, cos(-PI/4) * triShift);

                dstTri[0] = srcTri[0] + Point2f(sin(m_phi) * m_dY,cos(m_phi) * m_dY);
                dstTri[1] = dstTri[0] + Point2f(sin(PI/4 + m_phi) * triShift, cos(PI/4 + m_phi) * triShift);
                dstTri[2] = dstTri[0] + Point2f(sin(-PI/4 + m_phi) * triShift, cos(-PI/4 + m_phi) * triShift);

                Mat t = getAffineTransform(srcTri,dstTri);  // calculate translation matrix from preset triangle shift

                warpAffine(m_depthGrid,m_depthGrid,t,m_depthGrid.size(),INTER_CUBIC,BORDER_CONSTANT,Scalar(128));  //apply translation matrix

                pthread_mutex_lock(&m_motion_mux);
                m_dY = 0;
                m_phi = 0;
                pthread_mutex_unlock(&m_motion_mux);

            }

            m_decayCounter++;
            if(m_decayCounter > 20)
            {
                GaussianBlur(m_depthGrid,m_depthGrid,Size(3,3),1.0);
                m_decayCounter = 0;
            }

            sensDepth->ProcessDepthImage(m_depthImage,m_depthGrid,m_depthGrid,GetPropertyBool("process depth grid"));
            pMediaSample->Unlock(l_pSrcBuffer);
        }
        //write obstacle grid to output pin
        if (m_outputPin_depth_grid.IsConnected())
        {
            cObjectPtr<IMediaSample> pNewDepthSample;
            if (IS_OK(AllocMediaSample(&pNewDepthSample)))
            {
                tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
                pNewDepthSample->Update(tmStreamTime, m_depthGrid.data, tInt32(m_depthGrid.rows*m_depthGrid.cols), 0);
                m_outputPin_depth_grid.Transmit(pNewDepthSample);
            }
        }
        pthread_mutex_unlock(&m_input_mux);
    }
        RETURN_NOERROR;
}

