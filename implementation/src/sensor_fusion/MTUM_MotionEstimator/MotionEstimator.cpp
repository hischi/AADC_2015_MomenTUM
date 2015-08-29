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
#include "MotionEstimator.h"
#include "math.h"




ADTF_FILTER_PLUGIN("AADC Motion Estimator Ext", OID_ADTF_MTUMMOTIONESTIMATOREXT, MotionEstimatorExt)

MotionEstimatorExt::MotionEstimatorExt(const tChar* __info) : cFilter(__info),m_ticks2Dist(WHEEL_RADIUS*2*PI/WHEEL_TICKS_PER_ROUND)
{
    SetPropertyStr("Configurationfile","/home/odroid/AADC/calibration_files/MotionEstimatorSettings.xml");
    SetPropertyBool("Configurationfile" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Configurationfile" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");

    SetPropertyBool("DebugOutput",false);


    m_initEnableCounter = 0;
    m_state = STATE_IDLE;

    m_dir_mux = PTHREAD_MUTEX_INITIALIZER;
    m_waitStop_mux = PTHREAD_MUTEX_INITIALIZER;
    m_reset_mux = PTHREAD_MUTEX_INITIALIZER;
    m_waitStop = true;
}

MotionEstimatorExt::~MotionEstimatorExt()
{

}

tResult MotionEstimatorExt::CreateInputPins(__exception)
{	
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);
    cObjectPtr<IMediaType> pTypeSignalValueARDU = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValueARDU->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalARDU));
    cObjectPtr<IMediaType> pTypeSignalValueCAM = new cMediaType(0,0,0,"tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValueCAM->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalCAM));
    cObjectPtr<IMediaType> pTypeSignalValueSCPT = new cMediaType(0,0,0,"tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValueSCPT->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalSCPT));


    RETURN_IF_FAILED(m_inputPin_RPM_Left.Create("wheel_speed_left",  pTypeSignalValueARDU, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_RPM_Left));
    RETURN_IF_FAILED(m_inputPin_RPM_Right.Create("wheel_speed_right",  pTypeSignalValueARDU, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_RPM_Right));
    RETURN_IF_FAILED(m_inputPin_yaw.Create("yaw",  pTypeSignalValueARDU, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_yaw));
    RETURN_IF_FAILED(m_InputPin_acc_act.Create("acceleration",  pTypeSignalValueCAM, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_InputPin_acc_act));
    RETURN_IF_FAILED(m_inputPin_Yacc.Create("Y_acc",pTypeSignalValueARDU, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_inputPin_Yacc));
    RETURN_IF_FAILED(m_InputPin_dir.Create("Driving_dir", pTypeSignalValueCAM, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_InputPin_dir));
    RETURN_IF_FAILED(m_InputPin_reset.Create("reset", pTypeSignalValueSCPT, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_InputPin_reset));

    RETURN_NOERROR;
}

tResult MotionEstimatorExt::CreateOutputPins(__exception)
{

    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    tChar const * strDescSignalValue2 = pDescManager->GetMediaDescription("tMotionDataExt");
    RETURN_IF_POINTER_NULL(strDescSignalValue2);
    cObjectPtr<IMediaType> pTypeSignalValue2 = new cMediaType(0, 0, 0, "tMotionDataExt", strDescSignalValue2,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_FAILED(pTypeSignalValue2->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescMotion));

    RETURN_IF_FAILED(m_outputPin_motion.Create("motion_data", pTypeSignalValue2  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_motion));

    RETURN_IF_FAILED(m_outputPin_status.Create("status", new cMediaType(0, 0, 0, "tSignalValue")  , static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_outputPin_status));

    RETURN_NOERROR;
}

tResult MotionEstimatorExt::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

            if (eStage == StageFirst)
    {
        CreateInputPins(__exception_ptr);
        CreateOutputPins(__exception_ptr);
    }
    else if (eStage == StageNormal)
    {
        pthread_mutex_lock(&m_reset_mux);
        m_yaw_mean_pre = 0;
        m_dir =0;
        m_accy =0;
        m_offset =0;
        m_n = 0;
        m_M2 =0;


        float dt = 0.05;
        // intialization of KF...                           dt   1/2 dt^2
        kalman = new KalmanFilter(3,3,0);
        kalman->transitionMatrix = (Mat_<float>(3, 3) << 1,dt,0.5*dt*dt,   0,1,dt,  0,0,1);
        //Mat_<float> measurement(2,1); measurement.setTo(Scalar(0));
        m_estimated = (Mat_<float>(3,1) << 0,0,0);
        kalman->statePre.at<float>(0)=0;
        kalman->statePre.at<float>(1)=0;
        kalman->statePre.at<float>(2)=0;

        kalman->measurementMatrix = (Mat_<float>(3,3) << 0,1,0,    0, 1, 0,   0, 0, 1);//(Mat_<float>(2,3) << 0, 1,0, 0,0,1 );

        m_debug = GetPropertyBool("DebugOutput");
        pthread_mutex_lock(&m_dir_mux);
        m_dir = 0;
        pthread_mutex_unlock(&m_dir_mux);


        m_lastr =-1;
        m_lastl =-1;
        m_rdist =0;
        m_ldist =0;
        pthread_mutex_lock(&m_waitStop_mux);
        m_standCounter=0;
        m_maxstandCounter =0;
        m_waitStop = true;
        pthread_mutex_unlock(&m_waitStop_mux);
        m_lastaccs = vector<float>();
        LoadConfigurationData();
        LOG_INFO("MotionEstimator: config data gelesen");
        pthread_mutex_unlock(&m_reset_mux);
        m_dirPinActive = false;
    }
    RETURN_NOERROR;
}



tResult MotionEstimatorExt::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult MotionEstimatorExt::Stop(__exception)
{
    pthread_mutex_unlock(&m_dir_mux);
    pthread_mutex_unlock(&m_waitStop_mux);
    pthread_mutex_unlock(&m_reset_mux);
    return cFilter::Stop(__exception_ptr);
}

tResult MotionEstimatorExt::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}

void MotionEstimatorExt::checkWaitStop(float v)
{
    pthread_mutex_lock(&m_waitStop_mux);
    if(!m_waitStop ||m_dir ==0){
        pthread_mutex_unlock(&m_waitStop_mux);
        return;
    }

    if(v !=0){

        m_standCounter =10;

        pthread_mutex_unlock(&m_waitStop_mux);
        return;
    }

    if(!m_standCounter || !m_maxstandCounter){
        pthread_mutex_lock(&m_dir_mux);
        m_dir =0;
        pthread_mutex_unlock(&m_dir_mux);

        pthread_mutex_unlock(&m_waitStop_mux);
        return;
    }

    if(m_standCounter)
        m_standCounter--;


    if(m_maxstandCounter)
        m_maxstandCounter--;

    pthread_mutex_unlock(&m_waitStop_mux);

}

void MotionEstimatorExt::insertAccMean(tFloat32 value)
{
    if(m_lastaccs.size()>10){
        m_lastaccs.erase(m_lastaccs.begin());
    }
    m_lastaccs.push_back(value);
}
float MotionEstimatorExt::getAccMean(){
    float sum =0;
    for(size_t i = 0; i< m_lastaccs.size(); i++){
        sum+=m_lastaccs[i];
    }
    return(m_lastaccs.size()>0)?sum/m_lastaccs.size():0;
}

tResult MotionEstimatorExt::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{	

    if (nEventCode != IPinEventSink::PE_MediaSampleReceived)
        RETURN_ERROR(ERR_INVALID_TYPE);
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(m_pCoderDescMotion);
    RETURN_IF_POINTER_NULL(m_pCoderDescSignalCAM);
    RETURN_IF_POINTER_NULL(m_pCoderDescSignalARDU);
    RETURN_IF_POINTER_NULL(m_pCoderDescSignalSCPT);
    tFloat32 value = 0;
    tUInt32 timeStamp = 0;
    if(pSource == &m_InputPin_reset){
        cObjectPtr<IMediaCoder> pCoderInput;
        RETURN_IF_FAILED(m_pCoderDescSignalSCPT->Lock(pMediaSample, &pCoderInput));

        //get values from media sample
        pCoderInput->Get("f32Value", (tVoid*)&value);
        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
        m_pCoderDescSignalSCPT->Unlock(pCoderInput);
        pthread_mutex_lock(&m_reset_mux);

        m_dir =0;
        m_standCounter=0;
        m_maxstandCounter =0;
        m_waitStop = true;
        kalman->statePre.at<float>(0)=0;
        kalman->statePre.at<float>(1)=0;
        kalman->statePre.at<float>(2)=0;

        m_initEnableCounter = 0;
        m_state = STATE_IDLE;

        m_dir_mux = PTHREAD_MUTEX_INITIALIZER;
        m_waitStop_mux = PTHREAD_MUTEX_INITIALIZER;
        pthread_mutex_unlock(&m_reset_mux);



    }
    if(pSource == &m_InputPin_dir || pSource == &m_InputPin_acc_act){
        pthread_mutex_lock(&m_reset_mux);
        pthread_mutex_unlock(&m_reset_mux);
        // read-out the incoming Media Sample
        cObjectPtr<IMediaCoder> pCoderInput;
        RETURN_IF_FAILED(m_pCoderDescSignalCAM->Lock(pMediaSample, &pCoderInput));

        //get values from media sample
        pCoderInput->Get("f32Value", (tVoid*)&value);
        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
        m_pCoderDescSignalCAM->Unlock(pCoderInput);
        if(pSource == &m_InputPin_dir){
            m_dirPinActive = true;
            if(value >= 0){
                pthread_mutex_lock(&m_dir_mux);
                m_dir =1;
                pthread_mutex_unlock(&m_dir_mux);
            }else{
                pthread_mutex_lock(&m_dir_mux);
                m_dir = -1;
                pthread_mutex_unlock(&m_dir_mux);
            }
        }
        if(pSource == &m_InputPin_acc_act)
        {
            pthread_mutex_lock(&m_dir_mux);
            if(m_dir ==0 || !m_dirPinActive){
                if(value>0)
                    m_dir =1;
                if(value<0)
                    m_dir = -1;

            }

            pthread_mutex_unlock(&m_dir_mux);
            pthread_mutex_lock(&m_waitStop_mux);
            if(value == 0)
            {
                m_waitStop = true;
            }
            else
            {
                m_waitStop = false;
                m_standCounter = 10;
                m_maxstandCounter = max(m_maxstandCounter, value*2);

            }
            pthread_mutex_unlock(&m_waitStop_mux);


        }
    }
    else
    {
        pthread_mutex_lock(&m_reset_mux);
        pthread_mutex_unlock(&m_reset_mux);

        cObjectPtr<IMediaCoder> pCoderInput;
        RETURN_IF_FAILED(m_pCoderDescSignalARDU->Lock(pMediaSample, &pCoderInput));

        //get values from media sample
        pCoderInput->Get("f32Value", (tVoid*)&value);
        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
        m_pCoderDescSignalARDU->Unlock(pCoderInput);


        if (pSource == &m_inputPin_RPM_Left)
        {

            if(m_lastl == -1)
                m_lastl = value;
            float v = value- m_lastl;
            v = (v < 0)?0:v;
            v = (v > 10)?10:v;
            checkWaitStop(v);
            pthread_mutex_lock(&m_dir_mux);
            if(m_dir<0)
                v = -v;
            if(m_dir && m_state != STATE_IDLE)
                m_ldist += v;
            pthread_mutex_unlock(&m_dir_mux);
            m_lastl = value;


        }
        if (pSource == &m_inputPin_RPM_Right)
        {
            if(m_lastr == -1)
                m_lastr = value;
            float v = value-m_lastr;
            v = (v < 0)?0:v;
            v = (v > 10)?10:v;
            checkWaitStop(v);
            pthread_mutex_lock(&m_dir_mux);
            if(m_dir<0)
                v =-v;
            if(m_dir && m_state != STATE_IDLE)
                m_rdist+=v;
            pthread_mutex_unlock(&m_dir_mux);
            m_lastr = value;
        }
        if(pSource == &m_inputPin_Yacc){
            m_accy = value;

            if(m_state == STATE_IDLE){
                m_switched_to_enabled =true;
                m_n ++;
                float delta = value - m_offset;
                m_offset += delta/m_n;
                m_M2 +=  delta*(value-m_offset);
                insertAccMean(value);
            } else if (m_state == STATE_ENABLED){
                if(m_switched_to_enabled){
                    m_switched_to_enabled = false;
                    float variance = m_M2 /(m_n -1);
                    LOG_INFO(cString::Format("mean acceleration: %f, or %f variance: %f",m_offset, getAccMean(),variance));
                    m_dy =0;

                }
                m_accy -= getAccMean();

                pthread_mutex_lock(&m_dir_mux);
                if(m_dir==0)insertAccMean(value);
                pthread_mutex_unlock(&m_dir_mux);

            }
        }
        if (pSource == &m_inputPin_yaw)
        {
            tUInt32 timediff = GetTime()-  m_t;
            m_yaw_mean = value;
            m_t = GetTime();
            if(m_state == STATE_IDLE)
            {

                tFloat32 yawChange = m_yaw_mean - m_yaw_mean_pre_pre;
                m_yaw_mean_pre_pre = m_yaw_mean_pre;
                m_yaw_mean_pre = m_yaw_mean;
                m_initEnableCounter++;
                if(yawChange < GYRO_EPSILON && yawChange > -GYRO_EPSILON && m_initEnableCounter > 10)
                {
                    LOG_INFO(cString::Format("MotionEstimatorExt: AUTO-ENABLE      yawChange = %f   after %d ticks",yawChange,m_initEnableCounter));
                    SendSignalValueMessage(&m_outputPin_status, 1, 0);
                    m_state = STATE_ENABLED;
                }
                if(m_initEnableCounter > 1000)
                {
                    LOG_ERROR("MotionEstimatorExt: AUTO-ENABLE timeout");
                    SendSignalValueMessage(&m_outputPin_status, 0, 0);
                    //m_state = STATE_ENABLED;
                }

            }
            else if(m_state == STATE_ENABLED) // 25 ms
            {

                if(timediff > 100000){
                    // one step got lost, do double prediction to handle this.
                    if(m_debug)LOG_INFO("long timeslot, do double prediction..");
                    kalman->predict();
                }


                tFloat32 dPhi = 0;
                tFloat32 dY = 0;

                // Calculate angle update:
                dPhi = m_yaw_mean - m_yaw_mean_pre;
                m_yaw_mean_pre = m_yaw_mean;
                if(dPhi > M_PI){
                    dPhi-= 2*M_PI;
                }
                if(dPhi < -M_PI){
                    dPhi += 2*M_PI;
                }

                float dy;
                float vel;
                if(timediff > 25000){

                    Mat_<float> prediction;
                    prediction = kalman->predict();
                    //if(m_debug)LOG_INFO(cString::Format("%s: values of prediction: y: %f, vel: %f, acc: %f, estimated vel: %f, acc vel: %f",m_name.GetPtr(), prediction(0),prediction(1),prediction(2), vel_est, vel));

                    Mat_<float> measurement(3,1);
                    measurement(0) = m_ldist;
                    m_ldist =0;// estDist/0.05; //m_dist*0.01;
                    measurement(1) = m_rdist;
                    m_rdist =0;
                    measurement(2) = m_accy;


                    m_estimated = kalman->correct(measurement);

                    dy =  m_estimated(0)*100;
                    vel = m_estimated(1)*100;

                    timeStamp = 0;
                    dY = dy - m_dy ;
                    m_dy = dy;
                }
                else
                {
                    dY =0;
                    vel = m_estimated(1)*100;
                }





                //create new media sample
                cObjectPtr<IMediaSample> pMediaSampleOut;
                AllocMediaSample((tVoid**)&pMediaSampleOut);

                //allocate memory with the size given by the descriptor
                cObjectPtr<IMediaSerializer> pSerializer;
                m_pCoderDescMotion->GetMediaSampleSerializer(&pSerializer);
                tInt nSize = pSerializer->GetDeserializedSize();
                pMediaSampleOut->AllocBuffer(nSize);

                //write date to the media sample with the coder of the descriptor
                cObjectPtr<IMediaCoder> pCoderOutput;
                m_pCoderDescMotion->WriteLock(pMediaSampleOut, &pCoderOutput);

                pCoderOutput->Set("f32dY", (tVoid*)&(dY));
                pCoderOutput->Set("f32dPhi", (tVoid*)&(dPhi));
                pCoderOutput->Set("f32vel",(tVoid*)&(vel));
                pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
                m_pCoderDescMotion->Unlock(pCoderOutput);

                //transmit media sample over output pin
                tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
                pMediaSampleOut->SetTime(tmStreamTime);
                m_outputPin_motion.Transmit(pMediaSampleOut);
            }

        }

    }
    RETURN_NOERROR;
}


tResult MotionEstimatorExt::SendSignalValueMessage(cObjectPtr<cOutputPin> pOutputPin, tFloat32 value, tUInt32 timeStamp)
{
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSampleOut;
    AllocMediaSample((tVoid**)&pMediaSampleOut);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSignalARDU->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSampleOut->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    cObjectPtr<IMediaCoder> pCoderOutput;
    m_pCoderDescSignalARDU->WriteLock(pMediaSampleOut, &pCoderOutput);

    pCoderOutput->Set("f32Value", (tVoid*)&(value));
    pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
    m_pCoderDescSignalARDU->Unlock(pCoderOutput);

    //transmit media sample over output pin
    tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
    pMediaSampleOut->SetTime(tmStreamTime);
    pOutputPin->Transmit(pMediaSampleOut);

    RETURN_NOERROR;
}




void MotionEstimatorExt::fillMat(Mat_<float>& measurement, cDOMElementRefList& oElems)
{
    for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem)
    {
        cDOMElementRefList oRows;
        (*itElem)->FindNodes("row",oRows);
        int j =0;
        for(cDOMElementRefList::iterator itRow = oRows.begin();itRow != oRows.end();++itRow){
            cString row = (*itRow)->GetData();
            if(itRow == oRows.begin()){
                int rows =oRows.size();
                int cols =  row.CountString(",")+1;
                measurement = Mat_<float>(rows,cols);
            }
            cStringList collist;
            row.Split(collist,',');
            for(int i =0; i < collist.GetItemCount(); i++){
                cString cell = collist[i];
                measurement(j,i) = float(cell.AsFloat64());
            }
            j++;
        }
    }

}

void MotionEstimatorExt::printMat(Mat_<float> measurement)
{
    cString str;
    for(int row =0; row <measurement.rows; row++){
        str.Append("[ ");
        for(int col =0; col < measurement.cols; col++)
            str.Append(cString::Format("[%f] ",measurement(row,col)));
        str.Append("]\n");
    }
    LOG_INFO(str);
}

tResult MotionEstimatorExt::LoadConfigurationData()
{

    //Get path of configuration file
    cFilename m_fileConfig = GetPropertyStr("Configurationfile");
    if (m_fileConfig.IsEmpty())
    {
        LOG_ERROR("Motion Estimator: Configured configuration file not found");
        RETURN_ERROR(ERR_INVALID_FILE);
    }
    ADTF_GET_CONFIG_FILENAME(m_fileConfig);
    m_fileConfig = m_fileConfig.CreateAbsolutePath(".");

    //Load file, parse configuration, print the data

    if (cFileSystem::Exists(m_fileConfig))
    {
        cDOM oDOM;
        oDOM.Load(m_fileConfig);
        //load settings for calibration mode
        cDOMElementRefList oElemsSettings;
        if(IS_OK(oDOM.FindNodes("MotionEstSettings/measurementMat", oElemsSettings)))
        {
            LOG_INFO("-------found Measurement Mat in given file...");
            Mat_<float> measurement;
            fillMat(measurement, oElemsSettings);

            printMat(measurement);
            kalman->measurementMatrix=measurement;
        }
        if(IS_OK(oDOM.FindNodes("MotionEstSettings/measurementCovarianceMat",oElemsSettings)))
        {
            LOG_INFO("-------found Measurement Noise Covariance Mat in given file...");
            Mat_<float> covariance;
            fillMat(covariance,oElemsSettings);

            printMat(covariance);
            kalman->measurementNoiseCov = covariance;
        }
        if(IS_OK(oDOM.FindNodes("MotionEstSettings/processNoiseCov",oElemsSettings)))
        {
            LOG_INFO("-------found ProcessNoiseCov Mat in given file...");
            Mat_<float> processNoiseCov;
            fillMat(processNoiseCov,oElemsSettings);
            printMat(processNoiseCov);
            kalman->processNoiseCov=processNoiseCov;
        }

    }
    else
    {
        LOG_ERROR("MotionEstimator: Configured configuration file does not exist");
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    RETURN_NOERROR;
}


