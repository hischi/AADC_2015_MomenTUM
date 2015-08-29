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

/*! \brief CarController
 *
 *  This filter communicates with the jury and maintains the maneuver list.
 */

#ifndef _CARCONTROLLER_H_
#define _CARCONTROLLER_H_

#include "momenTUM_const.h"
#include "../../../../../adtfBase/AADC_ADTF_BaseFilters/include/ManeuverList.h"
#include <pthread.h>


#define OID_ADTF_CARCONTROLLER "adtf.aadc.CarController"

/*!
 * This filter communicates with the jury and maintains the maneuver list.
 * This is not based on the provided JuryModul but reimplements its communication interface! The filter is based on
 * the ADTF TimeTriggeredFilter. The cycle method is used to (re)send the status-information to the jury with an
 * adjustable frequency.
 *
 * Additionally it collects status information of connected filters. Only if all filters reported a ready state, this
 * information is forwarded to the jury. The headlight is on if all car filters are ready.
 */
class CarController : public adtf::cTimeTriggeredFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_CARCONTROLLER, "AADC CarController", OBJCAT_DataFilter, "CarController", 1, 0, 0, "<Beta Version");


    cInputPin m_inputPin_juryStruct;		/**< input pin for the jury struct output of the jury module >*/
    cInputPin m_inputPin_statusIn;          /**< input pin for the status feedback of other filters >*/

    cInputPin m_inputPin_maneuverFinished;  /**< input pin for the finished signal of segmentDriver >*/

    cOutputPin m_outputPin_statusOut;       /**< output pin for the internal state signal to active filters >*/
    cOutputPin m_outputPin_driverStruct;    /**< output pin for the jury state signal to active filters >*/

    cOutputPin m_outputPin_maneuver;        /**< output pin for the current active maneuver >*/
    cOutputPin m_outputPin_sr_enable;       /**< output pin to activate/deactivate speed recommender >*/

    cOutputPin	m_outputPin_head_light_enabled; /**< output pin to activate/deactivate head light >*/


    public:
        CarController(const tChar* __info);
        virtual ~CarController();
	
    protected: // overwrites cFilter
        tResult Init(tInitStage eStage, __exception = NULL);
        tResult Start(__exception = NULL);
        tResult Stop(__exception = NULL);
        tResult Shutdown(tInitStage eStage, __exception = NULL);
        tResult Cycle(__exception);
        tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
	
	private:
		/*! creates all the input Pins*/
		tResult CreateInputPins(__exception = NULL);
		/*! creates all the output Pins*/
		tResult CreateOutputPins(__exception = NULL);
        /*! processes mediasamples from the jury filter, changes the car state (reset, ready, ...) and requests new maneuvers
         * (send to SegmentDriver)*/
        tResult ProcessJuryUpdate(IMediaSample* pMediaSample);
        /*! processes mediasamples from the status input pin, if X (adjustable) filters send 1 the filter will switch to ready
         * and if one filter sends 0 the car will be stopped, set to idle and reset.*/
        tResult ProcessStatusUpdate(IMediaSample* pMediaSample);

        /*! if the car is ready the next maneuver in the maneuver-list will be started (StartManeuver)*/
        tResult NextManeuver();
        /*! if the maneuver-index points to a valid maneuver it is requested, otherwise the list seems to be finished*/
        tResult StartManeuver();
        /*! maps the maneuver-index to the maneuver-type using the maneuver-list*/
        tResult SearchManeuverIdx(tInt16 maneuver, tInt *pSectorIdx, tInt *pManeuverIdx);
        /*! translates the given string to one of the maneuver-types in the ACTION enumeration (defined in momenTUM_const.h) */
        tInt    String2Action(cString str);
        /*! sets the car state to RESET, notifies all other filters and initialises the reset_counter with 0 (see ProcessStatusUpdate) */
        tResult ResetCar();
        /*! stopps the car by disabling the SpeedRecommender*/
        tResult StopCar();

        /*! sends a signal-value (float) media-sample through the given output pin*/
        tResult SendSignalValueMessage(cObjectPtr<cOutputPin> pOutputPin, tFloat32 value, tUInt32 timeStamp);
        /*! sends a bool-value media-sample through the given output pin */
        tResult SendBoolSignalValueMessage(cObjectPtr<cOutputPin> pOutputPin, bool value, tUInt32 timeStamp);
        /*! decodes a signal-value (float) media-sample*/
        tResult ReceiveSignalValueMessage(IMediaSample* pMediaSample, tFloat32 *pValue, tUInt32 *pTimeStamp);
        /*! decodes a jury-struct media-sample*/
        tResult ReceiveJuryStructMessage(IMediaSample* pMediaSample, tInt8 *pActionID, tInt16 *pManeuverEntry);
        /*! sends a driver-struct media-sample*/
        tResult SendDriverStructMessage(tInt8 stateID, tInt16 maneuverEntry);
        /*! loads the (in ADTF) selected maneuver-list*/
        tResult loadManeuverList();

    // some mutexes to ensure thread-safe access (no guarantee here!)
        pthread_mutex_t m_driverStatus_mux;
        pthread_mutex_t m_status_mux;
        pthread_mutex_t m_send_mux;

    // internal status and reset variables
        tUInt8 m_status;
        enum eStatus {STATUS_IDLE=0,STATUS_RESET=1,STATUS_READY=2};
        tUInt32 m_reset_counter;            /*! current number of filter which have already reported 'ready'*/
        tUInt32 m_reset_counter_desired;    /*! expected number of filters which should report their state to this filter*/

    // current driver state:
        tInt8 m_DriverState;
        tInt8 m_DriverStateLastSend;
        enum eDriverStates {DS_STARTUP=-2,DS_ERROR=-1,DS_READY=0,DS_RUNNING=1,DS_FINISHED=2};

        bool m_maneuverFinished; // currently not used

        tInt16 m_currManeuver;      /*! current maneuver-index (index for the maneuver-list)*/
        tInt16 m_StartAtManeuver;   /*! used to restart sector */

        /*! this is the filename of the maneuver list*/
        cFilename m_maneuverListFile;
        /*! this is the list with all the loaded sections from the maneuver list*/
        std::vector<tSector> m_sectorList;

		/*! Coder Descriptor for the pins*/
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescJuryStruct;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescDriverStruct;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescBoolSignal;
		
};



//*************************************************************************************************

#endif // _CARCONTROLLER_H_

