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
#include "ObstacleGrid.h"
#include <math.h>


ADTF_FILTER_PLUGIN("MTUM Obstacle Grid", OID_MTUM_OBSTACLEGRID, ObstacleGrid)

ObstacleGrid::ObstacleGrid(const tChar* __info) : cFilter(__info)
{
    CreatePropertys();

    m_decayCounter = 0;
	m_updateCounter = 0;
    m_dY = 0;

    /* Init Sensors */
    INIT_P(sensUSFL);
    INIT_P(sensUSFR);
    INIT_P(sensUSRL);
    INIT_P(sensUSRR);
    lastUSFRValue = 0;
    lastUSFLValue = 0;
    lastUSRRValue = 0;
    lastUSRLValue = 0;

    INIT_P(sensIRF);
    INIT_P(sensIRFL);
    INIT_P(sensIRFR);
    INIT_P(sensIRR);
    INIT_P(sensIRL);
    INIT_P(sensIRC);
}

ObstacleGrid::~ObstacleGrid()
{
    m_obstacleGrid.release();

    DELETE_IF_INIT(sensUSFL);
    DELETE_IF_INIT(sensUSFR);
    DELETE_IF_INIT(sensUSRL);
    DELETE_IF_INIT(sensUSRR);

    DELETE_IF_INIT(sensIRF);
    DELETE_IF_INIT(sensIRFL);
    DELETE_IF_INIT(sensIRFR);
    DELETE_IF_INIT(sensIRR);
    DELETE_IF_INIT(sensIRL);
    DELETE_IF_INIT(sensIRC);

}

tResult ObstacleGrid::CreatePropertys()
{
    SetPropertyInt("Grid Height [GridCells]",(TILE_COUNT_FRONT+TILE_COUNT_REAR));
    SetPropertyInt("Grid Width [GridCells]",(TILE_COUNT_LEFT+TILE_COUNT_RIGHT));
    SetPropertyInt("Grid Car Position Cross [GridCells]",(TILE_COUNT_RIGHT));
    SetPropertyInt("Grid Car Position Driving Direction [GridCells]",(TILE_COUNT_REAR));


    SetPropertyStr("Configurationfile infrared front left shortrange","/home/odroid/AADC/calibration_files/calibrationIrFLS.xml");
    SetPropertyBool("Configurationfile infrared front left shortrange" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Configurationfile infrared front left shortrange" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyStr("Configurationfile infrared front left longrange","/home/odroid/AADC/calibration_files/calibrationIrFLL.xml");
    SetPropertyBool("Configurationfile infrared front left longrange" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Configurationfile infrared front left longrange" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");

    SetPropertyStr("Configurationfile infrared front center longrange","/home/odroid/AADC/calibration_files/calibrationIrFCL.xml");
    SetPropertyBool("Configurationfile infrared front center longrange" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Configurationfile infrared front center longrange" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyStr("Configurationfile infrared front center shortrange","/home/odroid/AADC/calibration_files/calibrationIrFCS.xml");
    SetPropertyBool("Configurationfile infrared front center shortrange" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Configurationfile infrared front center shortrange" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");

    SetPropertyStr("Configurationfile infrared front right longrange","/home/odroid/AADC/calibration_files/calibrationIrFRL.xml");
    SetPropertyBool("Configurationfile infrared front right longrange" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Configurationfile infrared front right longrange" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyStr("Configurationfile infrared front right shortrange","/home/odroid/AADC/calibration_files/calibrationIrFRS.xml");
    SetPropertyBool("Configurationfile infrared front right shortrange" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Configurationfile infrared front right shortrange" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");

    SetPropertyStr("Configurationfile infrared rear right shortrange","/home/odroid/AADC/calibration_files/calibrationIrRRS.xml");
    SetPropertyBool("Configurationfile infrared rear right shortrange" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Configurationfile infrared rear right shortrange" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyStr("Configurationfile infrared rear center shortrange","/home/odroid/AADC/calibration_files/calibrationIrRCS.xml");
    SetPropertyBool("Configurationfile infrared rear center shortrange" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Configurationfile infrared rear center shortrange" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyStr("Configurationfile infrared rear left shortrange","/home/odroid/AADC/calibration_files/calibrationIrRLS.xml");
    SetPropertyBool("Configurationfile infrared rear left shortrange" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Configurationfile infrared rear left shortrange" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");

    SetPropertyStr("Configurationfile ultrasound front left","/home/odroid/AADC/calibration_files/calibrationUsFL.xml");
    SetPropertyBool("Configurationfile ultrasound front left" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Configurationfile ultrasound front left" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyStr("Configurationfile ultrasound front right","/home/odroid/AADC/calibration_files/calibrationUsFR.xml");
    SetPropertyBool("Configurationfile ultrasound front right" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Configurationfile ultrasound front right" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyStr("Configurationfile ultrasound rear left","/home/odroid/AADC/calibration_files/calibrationUsRL.xml");
    SetPropertyBool("Configurationfile ultrasound rear left" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Configurationfile ultrasound rear left" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyStr("Configurationfile ultrasound rear right","/home/odroid/AADC/calibration_files/calibrationUsRR.xml");
    SetPropertyBool("Configurationfile ultrasound rear right" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Configurationfile ultrasound rear right" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");

    RETURN_NOERROR;
}

tResult ObstacleGrid::CreateInputPins(__exception)
{	
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
    tChar const * strDescMotion = pDescManager->GetMediaDescription("tMotionDataExt");
    RETURN_IF_POINTER_NULL(strDescMotion);
    cObjectPtr<IMediaType> pTypeMotion = new cMediaType(0, 0, 0, "tMotionDataExt", strDescMotion,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeMotion->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescMotion));

    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal));

    //Infrared
	RETURN_IF_FAILED(m_inputPin_ir_front_center_longrange.Create("ir_voltage_front_center_longrange",  new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_inputPin_ir_front_center_longrange));		
	RETURN_IF_FAILED(m_inputPin_ir_front_center_shortrange.Create("ir_voltage_front_center_shortrange",  new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_inputPin_ir_front_center_shortrange));
	RETURN_IF_FAILED(m_inputPin_ir_front_left_longrange.Create("ir_voltage_front_left_longrange",  new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_inputPin_ir_front_left_longrange));	
    RETURN_IF_FAILED(m_inputPin_ir_front_left_shortrange.Create("ir_voltage_front_left_shortrange",  new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_inputPin_ir_front_left_shortrange));
	RETURN_IF_FAILED(m_inputPin_ir_front_right_longrange.Create("ir_voltage_front_right_longrange", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_inputPin_ir_front_right_longrange));	
	RETURN_IF_FAILED(m_inputPin_ir_front_right_shortrange.Create("ir_voltage_front_right_shortrange", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_inputPin_ir_front_right_shortrange));
	RETURN_IF_FAILED(m_inputPin_ir_rear_center_shortrange.Create("ir_voltage_rear_center_shortrange", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_inputPin_ir_rear_center_shortrange));
	RETURN_IF_FAILED(m_inputPin_ir_rear_left_shortrange.Create("ir_voltage_rear_left_shortrange", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_inputPin_ir_rear_left_shortrange));
	RETURN_IF_FAILED(m_inputPin_ir_rear_right_shortrange.Create("ir_voltage_rear_right_shortrange", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_inputPin_ir_rear_right_shortrange));

    //Ultrasound
    RETURN_IF_FAILED(m_inputPin_us_front_left.Create("us_range_front_left", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_us_front_left));
    RETURN_IF_FAILED(m_inputPin_us_front_right.Create("us_range_front_right", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_us_front_right));
    RETURN_IF_FAILED(m_inputPin_us_rear_left.Create("us_range_rear_left", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_us_rear_left));
    RETURN_IF_FAILED(m_inputPin_us_rear_right.Create("us_range_rear_right", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_us_rear_right));

    //motion
    RETURN_IF_FAILED(m_inputPin_motion.Create("motion_data", pTypeMotion, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_motion));

	RETURN_NOERROR;
}

tResult ObstacleGrid::CreateOutputPins(__exception)
{
    RETURN_IF_FAILED(m_outputPin_obstacle_grid.Create("obstacle_grid", adtf::IPin::PD_Output  , static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_outputPin_obstacle_grid));
 
    RETURN_NOERROR;
}

tResult ObstacleGrid::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

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
        m_obstacleGrid = *(new Mat(height,width,CV_8UC1,Scalar(GRID_BASE_VALUE)));

        //setup bitmap format
        m_sBitmapFormatGrid.nWidth = width;
        m_sBitmapFormatGrid.nHeight = height;
        m_sBitmapFormatGrid.nBitsPerPixel = 8;
        m_sBitmapFormatGrid.nPixelFormat = cImage::PF_GREYSCALE_8;
        m_sBitmapFormatGrid.nBytesPerLine = width;
        m_sBitmapFormatGrid.nSize = width*height;
        m_sBitmapFormatGrid.nPaletteSize = 0;

        //apply bitmap format to video pins
        m_outputPin_obstacle_grid.SetFormat(&m_sBitmapFormatGrid, NULL);

        //load sensor configuration
        sensUSFR = new Sensor();
        sensUSFR->LoadConfigurationData(GetPropertyStr("Configurationfile ultrasound front right"));
        sensUSFL = new Sensor();
        sensUSFL->LoadConfigurationData(GetPropertyStr("Configurationfile ultrasound front left"));
        sensUSRL = new Sensor();
        sensUSRL->LoadConfigurationData(GetPropertyStr("Configurationfile ultrasound rear left"));
        sensUSRR = new Sensor();
        sensUSRR->LoadConfigurationData(GetPropertyStr("Configurationfile ultrasound rear right"));

        sensIRF = new SensorIR_Combined();
        sensIRF->LoadConfigurationData(GetPropertyStr("Configurationfile infrared front center shortrange"), GetPropertyStr("Configurationfile infrared front center longrange"));

        sensIRFR = new SensorIR_Combined();
        sensIRFR->LoadConfigurationData(GetPropertyStr("Configurationfile infrared front right shortrange"), GetPropertyStr("Configurationfile infrared front right longrange"));

        sensIRFL = new SensorIR_Combined();
        sensIRFL->LoadConfigurationData(GetPropertyStr("Configurationfile infrared front left shortrange"), GetPropertyStr("Configurationfile infrared front left longrange"));

        sensIRR = new SensorIR_Short();
        sensIRR->LoadConfigurationData(GetPropertyStr("Configurationfile infrared rear right shortrange"));
        sensIRL = new SensorIR_Short();
        sensIRL->LoadConfigurationData(GetPropertyStr("Configurationfile infrared rear left shortrange"));
        sensIRC = new SensorIR_Short();
        sensIRC->LoadConfigurationData(GetPropertyStr("Configurationfile infrared rear center shortrange"));
    }
    RETURN_NOERROR;
}

tResult ObstacleGrid::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult ObstacleGrid::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult ObstacleGrid::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}
tResult ObstacleGrid:: OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample)
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

        m_dY -= inDY;
        m_phi -= phi;

        if(abs(m_dY/GRID_CM_PER_UNIT) > m_obstacleGrid.cols/2.2 -1)
        {
            m_dY = 0;
            m_phi = 0;
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

            warpAffine(m_obstacleGrid,m_obstacleGrid,t,m_obstacleGrid.size(),INTER_CUBIC,BORDER_CONSTANT,Scalar(128));  //apply translation matrix

            m_dY = 0;
            m_phi = 0;
        }

        //do simulated us sens update
        lastUSFRValue = MAX(lastUSFRValue-inDY,0);
        lastUSFLValue = MAX(lastUSFLValue-inDY,0);
        lastUSRRValue = MAX(lastUSRRValue-inDY,0);
        lastUSRLValue = MAX(lastUSRLValue-inDY,0);
        sensUSFL->UpdateGrid(lastUSFLValue,m_obstacleGrid);
        sensUSFR->UpdateGrid(lastUSFRValue,m_obstacleGrid);
        sensUSRL->UpdateGrid(lastUSRLValue,m_obstacleGrid);
        sensUSRR->UpdateGrid(lastUSRRValue,m_obstacleGrid);
    }
    else
    {
        if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
        {
            RETURN_IF_POINTER_NULL(pMediaSample);


            if (pMediaSample != NULL && m_pCoderDescSignal != NULL)
            {
                // read-out the incoming Media Sample of type Signal value
                cObjectPtr<IMediaCoder> pCoderInput;
                RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoderInput));
                tFloat32 value = 0;
                tUInt32 timeStamp = 0;
                pCoderInput->Get("f32Value", (tVoid*)&value);
                pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
                m_pCoderDescSignal->Unlock(pCoderInput);

                if(m_decayCounter >= 16)
                {
                    GaussianBlur(m_obstacleGrid,m_obstacleGrid,Size(3,3),1.0);
                    m_decayCounter = 0;
                }

                //Infrared updated
                if (pSource == &m_inputPin_ir_front_center_longrange)
                {
                    sensIRF->UpdateGrid(value,m_obstacleGrid, false);
                }
                if (pSource == &m_inputPin_ir_front_center_shortrange)
                {
                    sensIRF->UpdateGrid(value,m_obstacleGrid, true);
                }
                if (pSource == &m_inputPin_ir_front_left_longrange)
                {
                    sensIRFL->UpdateGrid(value,m_obstacleGrid, false);
                }
                if (pSource == &m_inputPin_ir_front_left_shortrange)
                {
                    sensIRFL->UpdateGrid(value,m_obstacleGrid, true);
                }
                if (pSource == &m_inputPin_ir_front_right_longrange)
                {
                    sensIRFR->UpdateGrid(value,m_obstacleGrid, false);
                }
                if (pSource == &m_inputPin_ir_front_right_shortrange)
                {
                    sensIRFR->UpdateGrid(value,m_obstacleGrid, true);
                }
                if (pSource == &m_inputPin_ir_rear_left_shortrange)
                {
                    sensIRL->UpdateGrid(value,m_obstacleGrid);
                }
                if (pSource == &m_inputPin_ir_rear_right_shortrange)
                {
                    sensIRR->UpdateGrid(value,m_obstacleGrid);
                }
                if (pSource == &m_inputPin_ir_rear_center_shortrange)
                {
                    sensIRC->UpdateGrid(value,m_obstacleGrid);
                }

                //ultrasound update
                if (pSource == &m_inputPin_us_front_right)
                {
                    m_decayCounter++;
                    sensUSFR->UpdateGrid(value,m_obstacleGrid);
                    lastUSFRValue = value;
                }
                if (pSource == &m_inputPin_us_front_left)
                {
                    m_decayCounter++;
                    sensUSFL->UpdateGrid(value,m_obstacleGrid);
                    lastUSFLValue = value;
                }
                if (pSource == &m_inputPin_us_rear_right)
                {
                    m_decayCounter++;
                    sensUSRR->UpdateGrid(value,m_obstacleGrid);
                    lastUSRRValue = value;
                }
                if (pSource == &m_inputPin_us_rear_left)
                {
                    m_decayCounter++;
                    sensUSRL->UpdateGrid(value,m_obstacleGrid);
                    lastUSRLValue = value;
                }

				m_updateCounter++;
				if (m_updateCounter > 10)
				{
                    m_updateCounter = 0;
					//write obstacle grid to output pin
					if (m_outputPin_obstacle_grid.IsConnected())
					{
						cObjectPtr<IMediaSample> pNewDepthSample;
						if (IS_OK(AllocMediaSample(&pNewDepthSample)))
						{
							tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
							pNewDepthSample->Update(tmStreamTime, m_obstacleGrid.data, tInt32(m_obstacleGrid.rows*m_obstacleGrid.cols), 0);
							m_outputPin_obstacle_grid.Transmit(pNewDepthSample);
						}
					}
				}
            }

        }
    }
        RETURN_NOERROR;
}

