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


/*! \brief Obstacle Grid
 *  This filter merges the given sensor data into an opencv matrix using sensor values and odometry data.
 */
#ifndef _OBSTACLEGRID_H_
#define _OBSTACLEGRID_H_

#include "momenTUM_const.h"
#include "Sensor.h"
#include "SensorIR_Short.h"
#include "SensorIR_Combined.h"

#define OID_MTUM_OBSTACLEGRID "adtf.mtum.ObstacleGrid"

#ifndef DELETE_IF_INIT
#  define DELETE_IF_INIT(a)  if(a != NULL)\
                             {  delete(a);\
                                a = NULL;\
                             }
#endif

#ifndef INIT_P
#  define INIT_P(a)  (a = NULL)
#endif



/*!
 * This filter merges the given sensor data into an opencv matrix using sensor values and odometry data. It represents
 * the local obstacle environment around the car based on the range sensors (ultrasonic and infrared sensors).
 *
 * The size of the grid can be adjusted in the global constant file (momenTUM_const.h). The grid can be interpreted as
 * a grey-value image. A black pixel represents an obstacle at the corresponding realworld position. On the other hand,
 * a white pixel represents free space. A grey value (of 127) represents that there is not enough information wether the
 * area is allocated by an obstacle or not.
 *
 * The incomming odometry data will shift and rotate the grid such that the position of the car inside the grid stays
 * always the same. To model the stochastic error of motion and odometry-data a Gaussian-Blurr is applied to the grid.
 * Additionally, is a pixel not updated by any sensor it will become more gray to model the outdating of sensor-
 * information.
 */
class ObstacleGrid : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_MTUM_OBSTACLEGRID, "MTUM Obstacle Grid", OBJCAT_DataFilter, "Obstacle Grid", 1, 0, 0, "<Beta Version");

        cInputPin m_inputPin_ir_rear_center_shortrange;		/**< input pin for the rear center short range data */
	    cInputPin m_inputPin_ir_rear_left_shortrange;		/**< input pin for the rear left short range data */
	    cInputPin m_inputPin_ir_rear_right_shortrange;		/**< input pin for the rear right short range data */
        cInputPin m_inputPin_ir_front_left_shortrange;		/**< input pin for the front left short range data */
        cInputPin m_inputPin_ir_front_left_longrange;		/**< input pin for the front left long range data */
	    cInputPin m_inputPin_ir_front_center_shortrange;	/**< input pin for the front center short range data */
	    cInputPin m_inputPin_ir_front_center_longrange;		/**< input pin for the front center long range data */
	    cInputPin m_inputPin_ir_front_right_shortrange;		/**< input pin for the front right short range data */
	    cInputPin m_inputPin_ir_front_right_longrange;		/**< input pin for the front right long range data */

        cInputPin m_inputPin_us_front_left;	    /**< input pin for uss front left data */
        cInputPin m_inputPin_us_front_right;	/**< input pin for uss front right data */
        cInputPin m_inputPin_us_rear_left;	    /**< input pin for uss rear left data */
        cInputPin m_inputPin_us_rear_right;	    /**< input pin for uss rear right data */

        cInputPin m_inputPin_motion;			/**< input pin for the motion data */
        
        cVideoPin m_outputPin_obstacle_grid;    /**< output pin for the obstcale grid */
        
    public:
        ObstacleGrid(const tChar* __info);
        virtual ~ObstacleGrid();
	
    protected: // overwrites cFilter
        tResult Init(tInitStage eStage, __exception = NULL);
        tResult Start(__exception = NULL);
        tResult Stop(__exception = NULL);
        tResult Shutdown(tInitStage eStage, __exception = NULL);        
        tResult OnPinEvent(IPin *pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample *pMediaSample);
	private:
		/*! creates all the input Pins*/
		tResult CreateInputPins(__exception = NULL);
		/*! creates all the output Pins*/
		tResult CreateOutputPins(__exception = NULL);
        /*! creates all the filter propertys*/
        tResult CreatePropertys();

        Mat m_obstacleGrid;

        //Ultrasound US## --
        Sensor *sensUSFR;
        Sensor *sensUSFL;
        Sensor *sensUSRR;
        Sensor *sensUSRL;

        tFloat32 lastUSFRValue;
        tFloat32 lastUSFLValue;
        tFloat32 lastUSRRValue;
        tFloat32 lastUSRLValue;

        //Infrared
        SensorIR_Combined *sensIRF;
        SensorIR_Combined *sensIRFR;
        SensorIR_Combined *sensIRFL;
        SensorIR_Short *sensIRR;
        SensorIR_Short *sensIRC;
        SensorIR_Short *sensIRL;

        tUInt m_decayCounter;
		tInt32 m_updateCounter;
        tFloat32 m_dY;
        tFloat32 m_phi;

        tBitmapFormat	m_sBitmapFormatGrid;

		/*! Coder Descriptor for the pins*/
	    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;			
        cObjectPtr<IMediaTypeDescription> m_pCoderDescMotion;
		
};



//*************************************************************************************************

#endif // _OBSTACLEGRID_H_

