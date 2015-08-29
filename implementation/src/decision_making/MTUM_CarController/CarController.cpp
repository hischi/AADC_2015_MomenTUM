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

#include "math.h"
#include "CarController.h"
#include <iostream>


ADTF_FILTER_PLUGIN("AADC CarController", OID_ADTF_CARCONTROLLER, CarController)


CarController::CarController(const tChar* __info) : cTimeTriggeredFilter(__info)
{
    //SetPropertyInt("Grid Car Position Cross [GridCells]",20);
    //SetPropertyInt("Grid Car Position Driving [GridCells]",50);
    //SetPropertyFloat("Obstacle max probability",0.75);

    SetPropertyStr("ManeuverFile", "");
    SetPropertyBool("ManeuverFile" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("ManeuverFile" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");

    m_reset_counter_desired = 1;
    SetPropertyInt("Reset End Counter",m_reset_counter_desired);

    SetPropertyInt("RepeateRate", 2);

    m_reset_counter = 0;

    m_currManeuver = -1;
    m_status = STATUS_IDLE;
    m_DriverState = DS_ERROR;
}

CarController::~CarController()
{

}

tResult CarController::CreateInputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

    // Jury Struct Output
    tChar const * strDescJuryRun = pDescManager->GetMediaDescription("tJuryStruct");
    RETURN_IF_POINTER_NULL(strDescJuryRun);
    cObjectPtr<IMediaType> pTypeJuryRun = new cMediaType(0, 0, 0, "tJuryStruct", strDescJuryRun, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(m_inputPin_juryStruct.Create("Jury_Struct", pTypeJuryRun, this));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_juryStruct));
    RETURN_IF_FAILED(pTypeJuryRun->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescJuryStruct));

    RETURN_IF_FAILED(m_inputPin_maneuverFinished.Create("maneuverFinished", new cMediaType(0, 0, 0, "tSignalValue")  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_maneuverFinished));

    RETURN_IF_FAILED(m_inputPin_statusIn.Create("statusIn", new cMediaType(0, 0, 0, "tSignalValue") , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_statusIn));


	RETURN_NOERROR;
}

tResult CarController::CreateOutputPins(__exception)
{ 
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    tChar const * strDescDriverState = pDescManager->GetMediaDescription("tDriverStruct");
    RETURN_IF_POINTER_NULL(strDescDriverState);
    cObjectPtr<IMediaType> pTypeDriverState = new cMediaType(0, 0, 0, "tDriverStruct", strDescDriverState,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeDriverState->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescDriverStruct));
    RETURN_IF_FAILED(m_outputPin_driverStruct.Create("Driver_Struct", pTypeDriverState  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_driverStruct));


    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal));

    RETURN_IF_FAILED(m_outputPin_statusOut.Create("statusOut", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_statusOut));

    RETURN_IF_FAILED(m_outputPin_maneuver.Create("maneuver", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_maneuver));

    RETURN_IF_FAILED(m_outputPin_sr_enable.Create("sr_enable", pTypeSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_sr_enable));

    tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");
    RETURN_IF_POINTER_NULL(strDescBoolSignalValue);
    cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescBoolSignal));

    RETURN_IF_FAILED(m_outputPin_head_light_enabled.Create("head_light", pTypeBoolSignalValue  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_head_light_enabled));

    RETURN_NOERROR;
}

tResult CarController::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cTimeTriggeredFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
	{
		CreateInputPins(__exception_ptr);
		CreateOutputPins(__exception_ptr);
	}
    else if (eStage == StageNormal)
    {
        loadManeuverList();

        m_reset_counter_desired = GetPropertyInt("Reset End Counter");
        if(m_reset_counter_desired <= 0)
            m_status = STATUS_READY;

        tUInt interval = (tUInt) GetPropertyInt("RepeateRate");
        interval = 1000000 / interval;
        SetInterval(interval);
        cout << "JuryRepeater-Interval set to " << interval << endl;

        m_status_mux = PTHREAD_MUTEX_INITIALIZER;
        m_send_mux = PTHREAD_MUTEX_INITIALIZER;
        m_driverStatus_mux = PTHREAD_MUTEX_INITIALIZER;

        m_DriverStateLastSend = -10;
    }

    RETURN_NOERROR;
}

tResult CarController::Start(__exception)
{
    return cTimeTriggeredFilter::Start(__exception_ptr);
}

tResult CarController::Stop(__exception)
{
    return cTimeTriggeredFilter::Stop(__exception_ptr);
}

tResult CarController::Shutdown(tInitStage eStage, __exception)
{
    return cTimeTriggeredFilter::Shutdown(eStage,__exception_ptr);
}

tResult CarController::Cycle(__exception)
{
    tInt currentDriverState = 0;

    pthread_mutex_lock(&m_driverStatus_mux);
        currentDriverState = m_DriverState;
    pthread_mutex_unlock(&m_driverStatus_mux);

    // Answer with the current DriverState:
    if((currentDriverState == DS_ERROR && m_DriverStateLastSend != DS_ERROR) || (currentDriverState == DS_FINISHED && m_DriverStateLastSend != DS_FINISHED) || currentDriverState == DS_STARTUP || currentDriverState == DS_RUNNING || currentDriverState == DS_READY )
        SendDriverStructMessage(m_DriverState, m_currManeuver);
    m_DriverStateLastSend = currentDriverState;
}


tResult CarController::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    if (pMediaSample != NULL && m_pCoderDescSignal != NULL)
    {
        if(pSource == &m_inputPin_juryStruct)
        {
            ProcessJuryUpdate(pMediaSample);
        }
        else if(pSource == &m_inputPin_maneuverFinished)
        {
            NextManeuver();
        }
        else if(pSource == &m_inputPin_statusIn)
        {
            ProcessStatusUpdate(pMediaSample);
        }


    }

	RETURN_NOERROR;
}


tResult CarController::ProcessJuryUpdate(IMediaSample* pMediaSample)
{
    tInt8 i8ActionID = 0;
    tInt16 maneuver = 0;
    ReceiveJuryStructMessage(pMediaSample, &i8ActionID, &maneuver);

    tInt currentDriverState = 0;

    pthread_mutex_lock(&m_driverStatus_mux);
        currentDriverState = m_DriverState;
    pthread_mutex_unlock(&m_driverStatus_mux);

    pthread_mutex_lock(&m_status_mux);
        if(m_status == STATUS_RESET || m_status == STATUS_IDLE)
            currentDriverState = DS_ERROR;
    pthread_mutex_unlock(&m_status_mux);

    if (i8ActionID==-1) // STOP
    {
        StopCar();

        switch(currentDriverState)
        {
        case DS_ERROR:
        case DS_RUNNING:
            pthread_mutex_lock(&m_status_mux);
                if(m_status != STATUS_RESET)
                {
                    pthread_mutex_unlock(&m_status_mux);
                    Reset();
                }
                else
                    pthread_mutex_unlock(&m_status_mux);
            currentDriverState = DS_ERROR;
            break;

        case DS_READY:
        case DS_FINISHED:
            currentDriverState = DS_READY;
            break;

        default:
            currentDriverState = DS_ERROR;
        }
    }
    else if (i8ActionID==1) // Start
    {
        switch(currentDriverState)
        {
        case DS_ERROR:
            currentDriverState = DS_ERROR;
            break;

        case DS_READY:
            // Starte erstes Maneuver
            m_currManeuver = maneuver;
            m_StartAtManeuver = maneuver;

            SendSignalValueMessage(&m_outputPin_sr_enable, 1, 0);
            currentDriverState = DS_RUNNING;
            StartManeuver();
            break;

        case DS_RUNNING:
            if(m_StartAtManeuver != maneuver)
            {
                pthread_mutex_lock(&m_status_mux);
                    if(m_status != STATUS_RESET)
                    {
                        pthread_mutex_unlock(&m_status_mux);
                        Stop();
                        Reset();
                    }
                    else
                        pthread_mutex_unlock(&m_status_mux);
                currentDriverState = DS_ERROR;
            }
            else
                currentDriverState = DS_RUNNING;
            break;

        case DS_FINISHED:
            currentDriverState = DS_FINISHED;
            break;

        default:
            currentDriverState = DS_ERROR;
        }
    }
    else
    {
        switch(currentDriverState)
        {
        case DS_ERROR:
            pthread_mutex_lock(&m_status_mux);
                if(m_status == STATUS_READY)
                    currentDriverState = DS_READY;
                else
                    currentDriverState = DS_ERROR;
            pthread_mutex_unlock(&m_status_mux);
            break;

        case DS_READY:
            currentDriverState = DS_READY;
            break;

        case DS_RUNNING:
            currentDriverState = DS_RUNNING;
            break;

        case DS_FINISHED:
            currentDriverState = DS_FINISHED;
            break;

        default:
            currentDriverState = DS_ERROR;
        }
    }

    pthread_mutex_lock(&m_driverStatus_mux);
        m_DriverState = currentDriverState;
    pthread_mutex_unlock(&m_driverStatus_mux);

    RETURN_NOERROR;
}

tResult CarController::ProcessStatusUpdate(IMediaSample* pMediaSample)
{
    tFloat32 value = 0;
    tUInt32 timeStamp = 0;

    ReceiveSignalValueMessage(pMediaSample, &value, &timeStamp);

    pthread_mutex_lock(&m_status_mux);
        if(m_status == STATUS_RESET || m_status == STATUS_IDLE)
        {
            if(value == 1)  // NOERROR
                m_reset_counter++;

            if(m_reset_counter >= m_reset_counter_desired)
            {
                m_status = STATUS_READY;
                SendBoolSignalValueMessage(&m_outputPin_head_light_enabled, true, 0);
                //SendSignalValueMessage(&m_outputPin_sr_enable, 1, 0);
            }
            pthread_mutex_unlock(&m_status_mux);
        }
        else
        {
            pthread_mutex_unlock(&m_status_mux);

            if(value == 0)  // ERROR
            {
                StopCar();
                ResetCar();
            }
        }

    RETURN_NOERROR;
}

tResult CarController::NextManeuver()
{
    if(m_status != STATUS_READY)
        RETURN_NOERROR;

    LOG_INFO("ManeuverFinsihed");

    m_currManeuver++;
    StartManeuver();

    RETURN_NOERROR;
}

tResult CarController::StartManeuver()
{
    tInt sectorIdx = -1;
    tInt maneuverIdx = -1;

    SearchManeuverIdx(m_currManeuver, &sectorIdx, &maneuverIdx);

    if(sectorIdx >= 0 && sectorIdx < m_sectorList.size() && maneuverIdx >= 0 && maneuverIdx < m_sectorList.at(sectorIdx).maneuverList.size()) // Is there a valid maneuver?
    {
        tInt action = String2Action(m_sectorList.at(sectorIdx).maneuverList.at(maneuverIdx).action);
        return SendSignalValueMessage(&m_outputPin_maneuver, action, 0);  // Request maneuver
    }
    else
    {
        LOG_INFO("All sectors finished!");
        pthread_mutex_lock(&m_driverStatus_mux);
            m_DriverState = DS_FINISHED;
        pthread_mutex_unlock(&m_driverStatus_mux);

        RETURN_NOERROR;
    }
}

tResult CarController::StopCar()
{
    SendSignalValueMessage(&m_outputPin_sr_enable, 0, 0);
}

tResult CarController::ResetCar()
{
    pthread_mutex_lock(&m_status_mux);
        m_status = STATUS_RESET;
        m_reset_counter = 0;
    pthread_mutex_unlock(&m_status_mux);

    SendSignalValueMessage(&m_outputPin_statusOut, -1.0, 0);

    RETURN_NOERROR;
}

tResult CarController::SearchManeuverIdx(tInt16 maneuver, tInt *pSectorIdx, tInt *pManeuverIdx)
{
    for(int i = 0; i < m_sectorList.size(); i++)
    {
        if(m_sectorList.at(i).maneuverList.size() > 0)
        {
            for(int j = 0; j < m_sectorList.at(i).maneuverList.size(); j++)
            {
                if(m_sectorList.at(i).maneuverList.at(j).id == maneuver)
                {
                    *pSectorIdx = i;
                    *pManeuverIdx = j;
                    RETURN_NOERROR;
                }
            }
        }
    }

    *pSectorIdx = -1;
    *pManeuverIdx = -1;
    RETURN_NOERROR;
}

tInt CarController::String2Action(cString str)
{
    if(str.Compare("left") == 0)
        return ACTION_LEFT;
    else if(str.Compare("straight") == 0)
        return ACTION_STRAIGHT;
    else if(str.Compare("right") == 0)
        return ACTION_RIGHT;
    else if(str.Compare("cross_parking") == 0)
        return ACTION_CROSS_PARKING;
    else if(str.Compare("parallel_parking") == 0)
        return ACTION_PARALLEL_PARKING;
    else if(str.Compare("pull_out_left") == 0)
        return ACTION_PULL_OUT_LEFT;
    else if(str.Compare("pull_out_right") == 0)
        return ACTION_PULL_OUT_RIGHT;
    else
        return ACTION_STOP;
}


tResult CarController::SendSignalValueMessage(cObjectPtr<cOutputPin> pOutputPin, tFloat32 value, tUInt32 timeStamp)
{
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleOut;
    AllocMediaSample((tVoid**)&pMediaSampleOut);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleOut->AllocBuffer(nSize);

    pthread_mutex_lock(&m_send_mux);
        //write date to the media sample with the coder of the descriptor
        cObjectPtr<IMediaCoder> pCoderOutput;
        m_pCoderDescSignal->WriteLock(pMediaSampleOut, &pCoderOutput);

        pCoderOutput->Set("f32Value", (tVoid*)&(value));
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
        m_pCoderDescSignal->Unlock(pCoderOutput);

   pthread_mutex_unlock(&m_send_mux);

    //transmit media sample over output pin
    pMediaSampleOut->SetTime(pMediaSampleOut->GetTime());
    pOutputPin->Transmit(pMediaSampleOut);

    RETURN_NOERROR;
}

tResult CarController::SendBoolSignalValueMessage(cObjectPtr<cOutputPin> pOutputPin, bool value, tUInt32 timeStamp)
{
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleOut;
    AllocMediaSample((tVoid**)&pMediaSampleOut);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescBoolSignal->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleOut->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    cObjectPtr<IMediaCoder> pCoderOutput;
    m_pCoderDescBoolSignal->WriteLock(pMediaSampleOut, &pCoderOutput);

    pCoderOutput->Set("bValue", (tVoid*)&(value));
    pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
    m_pCoderDescBoolSignal->Unlock(pCoderOutput);

    //transmit media sample over output pin
    pMediaSampleOut->SetTime(pMediaSampleOut->GetTime());
    pOutputPin->Transmit(pMediaSampleOut);

    RETURN_NOERROR;
}

tResult CarController::ReceiveSignalValueMessage(IMediaSample* pMediaSample, tFloat32 *pValue, tUInt32 *pTimeStamp)
{
    // read-out the incoming Media Sample
    cObjectPtr<IMediaCoder> pCoderInput;
    RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoderInput));

    *pValue = 0;
    *pTimeStamp = 0;

    //get values from media sample
    pCoderInput->Get("f32Value", (tVoid*)pValue);
    pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)pTimeStamp);
    m_pCoderDescSignal->Unlock(pCoderInput);

    RETURN_NOERROR;
}


tResult CarController::ReceiveJuryStructMessage(IMediaSample* pMediaSample, tInt8 *pActionID, tInt16 *pManeuverEntry)
{
    // read-out the incoming Media Sample
    cObjectPtr<IMediaCoder> pCoderInput;
    RETURN_IF_FAILED(m_pCoderDescJuryStruct->Lock(pMediaSample, &pCoderInput));

    *pManeuverEntry = 0;

    //get values from media sample
    pCoderInput->Get("i8ActionID", (tVoid*)pActionID);
    pCoderInput->Get("i16ManeuverEntry", (tVoid*)pManeuverEntry);
    m_pCoderDescJuryStruct->Unlock(pCoderInput);

    RETURN_NOERROR;
}

tResult CarController::SendDriverStructMessage(tInt8 stateID, tInt16 maneuverEntry)
{
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleOut;
    AllocMediaSample((tVoid**)&pMediaSampleOut);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescDriverStruct->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleOut->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    cObjectPtr<IMediaCoder> pCoderOutput;
    m_pCoderDescDriverStruct->WriteLock(pMediaSampleOut, &pCoderOutput);

    pCoderOutput->Set("i8StateID", (tVoid*)&(stateID));
    pCoderOutput->Set("i16ManeuverEntry", (tVoid*)&maneuverEntry);
    m_pCoderDescDriverStruct->Unlock(pCoderOutput);

    //transmit media sample over output pin
    pMediaSampleOut->SetTime(pMediaSampleOut->GetTime());
    m_outputPin_driverStruct.Transmit(pMediaSampleOut);

    RETURN_NOERROR;
}

tResult CarController::loadManeuverList()
{

    m_maneuverListFile = GetPropertyStr("ManeuverFile");

    if (m_maneuverListFile.IsEmpty())
    {
        LOG_ERROR("DriverFilter: Maneuver file not found");
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    ADTF_GET_CONFIG_FILENAME(m_maneuverListFile);

    m_maneuverListFile = m_maneuverListFile.CreateAbsolutePath(".");

    //Load file, parse configuration, print the data

    if (cFileSystem::Exists(m_maneuverListFile))
    {
        cDOM oDOM;
        oDOM.Load(m_maneuverListFile);
        cDOMElementRefList oSectorElems;
        cDOMElementRefList oManeuverElems;

        //read first Sector Elem
        if(IS_OK(oDOM.FindNodes("AADC-Maneuver-List/AADC-Sector", oSectorElems)))
        {
            //iterate through sectors
            for (cDOMElementRefList::iterator itSectorElem = oSectorElems.begin(); itSectorElem != oSectorElems.end(); ++itSectorElem)
            {
                //if sector found
                tSector sector;
                sector.id = (*itSectorElem)->GetAttributeUInt32("id");

                if(IS_OK((*itSectorElem)->FindNodes("AADC-Maneuver", oManeuverElems)))
                {
                    //iterate through maneuvers
                    for(cDOMElementRefList::iterator itManeuverElem = oManeuverElems.begin(); itManeuverElem != oManeuverElems.end(); ++itManeuverElem)
                    {
                        tAADC_Maneuver man;
                        man.id = (*itManeuverElem)->GetAttributeUInt32("id");
                        man.action = (*itManeuverElem)->GetAttribute("action");
                        sector.maneuverList.push_back(man);
                    }
                }

                m_sectorList.push_back(sector);
            }
        }
        if (oSectorElems.size() > 0)
        {
            LOG_INFO("DriverFilter: Loaded Maneuver file successfully.");
        }
        else
        {
            LOG_ERROR("DriverFilter: no valid Maneuver Data found!");
            RETURN_ERROR(ERR_INVALID_FILE);
        }
    }
    else
    {
        LOG_ERROR("DriverFilter: no valid Maneuver File found!");
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    //OUTPUT DATA TO LOG
    //for(int i = 0; i < m_sectorList.size(); i++)
    //{
    //	LOG_INFO(cString::Format("Driver Module: Sector ID %d",m_sectorList[i].id));
    //	for(int j = 0; j < m_sectorList[i].maneuverList.size(); j++)
    //	{
    //		LOG_INFO(cString::Format("\tManeuver ID: %d", m_sectorList[i].maneuverList.at(j).id));
    //		LOG_INFO(cString::Format("\tManeuver action: %s", m_sectorList[i].maneuverList.at(j).action.GetPtr()));
    //	}
    //}

    LOG_INFO(cString::Format("Size of Data: %d Bytes ", sizeof(m_sectorList)));

    //checks if data are valid
    //RETURN_IF_FAILED(CheckConfigurationData());

    RETURN_NOERROR;
}




