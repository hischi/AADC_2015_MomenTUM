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

/*! \brief Depth Grid
 *         
 *  Transforms the given depth image to an obstacle grid like map
 */
#ifndef _DEPTHGRID_H_
#define _DEPTHGRID_H_

#include "stdafx.h"
#include <math.h>
#include "momenTUM_const.h"
#include "momenTUM_globalGrid.h"
#include "DepthSensor.h"
#include <pthread.h>


#define OID_MTUM_DEPTHGRID "adtf.mtum.DepthGrid"


int g_gridScale = GRID_CM_PER_UNIT;
char g_carPosX = TILE_COUNT_RIGHT;
char g_carPosY = TILE_COUNT_REAR;

/*!
 * This filter transforms the given depth image to an obstacle grid like map using odometry data. It represents
 * the local obstacle environment around the car. For more information about the grid based sensor fusion see
 * ObstacleGrid.h.
 *
 * Additional features are ground plane extraction and resolving the depth-information 0 to either 'to near' or
 * 'to far ahead'.
 *
 * This filter is marked async to release the camera thread!
 */
class DepthGrid : public adtf::cAsyncDataTriggeredFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_MTUM_DEPTHGRID, "MTUM Depth Grid", OBJCAT_DataFilter, "Depth Grid", 1, 0, 0, "<Beta Version");

        cVideoPin m_inputPin_depthImage;           /**< input pin for the depth image */
        cInputPin m_inputPin_motion;			   /**< input pin for the motion data */

        cVideoPin m_outputPin_depth_grid;          //output pin for the dept obstacle grid

        
    public:
        DepthGrid(const tChar* __info);
        virtual ~DepthGrid();
	
    protected: // overwrites cAsyncDataTriggeredFilter
        tResult Init(tInitStage eStage, __exception = NULL);
        tResult Start(__exception = NULL);
        tResult Stop(__exception = NULL);
        tResult Shutdown(tInitStage eStage, __exception = NULL);        
        tResult OnAsyncPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);
	private:
		/*! creates all the input Pins*/
		tResult CreateInputPins(__exception = NULL);
		/*! creates all the output Pins*/
		tResult CreateOutputPins(__exception = NULL);
        /*! creates all the filter propertys*/

        Mat m_depthGrid;

        pthread_mutex_t m_motion_mux;
        pthread_mutex_t m_input_mux;


        Mat m_depthImage;


        //depthimage Sensor
        DepthSensor *sensDepth;

        tFloat32 m_dY;
        tFloat32 m_phi;
        tInt32 m_decayCounter;

        tBitmapFormat	m_sBitmapFormatGrid;
        tBitmapFormat	m_sBitmapFormatDepthIn;

		/*! Coder Descriptor for the pins*/
        cObjectPtr<IMediaTypeDescription> m_pCoderDescMotion;
		
};



//*************************************************************************************************

#endif // _DEPTHGRID_H_

