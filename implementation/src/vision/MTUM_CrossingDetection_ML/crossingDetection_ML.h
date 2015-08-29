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
// Christoph Brickl, AEV:

/*! \brief cSpurerkennung
 *         
•	Zur Erkennung einer Spur wird das RGB Bild der Asus Xtion eingelesen und verarbeitet
•	Weiteres Vorgehen:
•	Zuschneiden des Orginal Bildes auf die eingestellte Größe
•	Erzeugen eines Graustufen Bildes
•	Anwenden eines Schwellwertfilters
•	Kantendedektion
•	Suchen nach Kanten auf den eingestellten cm-Marken
•	Auswerten der gefundenen Punkte ob sinnvolle Linie erstellt werden kann
•	Anzeigen der Linie mit Hilfe der GLC

 */

#ifndef _LaneDetection_FILTER_HEADER_
#define _LaneDetection_FILTER_HEADER_

#define OID_ADTF_CrossingDetection_ML  "mtum.aadc.CrossingDetection_ML"

#include "CrossingModel.h"
#include "LaneDetector.h"
#include "momenTUM_const.h"

class cCrossingDetection_ML : public adtf::cAsyncDataTriggeredFilter
{
     ADTF_DECLARE_FILTER_VERSION(OID_ADTF_CrossingDetection_ML, "MTUM CrossingDetection ML", adtf::OBJCAT_Tool, "MTUM CrossingDetection ML", 1, 0, 0, "Beta Version");
protected:
//RGB image input pin
		cVideoPin		m_oVideoInputPin;		/**< the input pin for the video*/


        cInputPin m_inputPin_state;
        cInputPin m_inputPin_signLocation;

//RGB image output pin
        cVideoPin      m_oVideoOutputPin;       /**< the output for the video showing the detected lines*/ 
        cOutputPin     m_oDistanceOutputPin;

        tResult ReceiveMotionDataMessage(IMediaSample* pMediaSample, tFloat32 *pdY, tFloat32 *pdPhi, tUInt32 *pTimeStamp);
        tResult ReceiveSignalValueMessage(IMediaSample* pMediaSample, tFloat32 *pValue, tUInt32 *pTimeStamp);
        tResult SendSignalValueMessage(cObjectPtr<cOutputPin> pOutputPin, tFloat32 value, tUInt32 timeStamp);

public:

        void resetSystem();

        cCrossingDetection_ML(const tChar*);
        virtual ~cCrossingDetection_ML();
		tResult Init(tInitStage eStage, __exception=NULL);
		tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr=NULL);
        tResult OnAsyncPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

		/*! processing
		@param pSample the input media sample
		*/tResult ProcessInput(IMediaSample* pSample);
		/*! processing the image
		*/
		tResult ProcessFound();
		/*! processing the image
		*/
		tResult ProcessOutput();
		/*! changed a property
		@param strProperty char pointer to property
		*/
		tResult PropertyChanged(const char* strProperty);
		
		/*! updates the image format
		@param pFormat the new image format
		*/
        tResult UpdateImageFormat(const tBitmapFormat* pFormat);
        tResult ProcessSign(IMediaSample* pSample);
        tResult ReceiveRoadSign(IMediaSample* pMediaSample, tInt8 *pId, Mat *pTvec, Mat* pRvec, tUInt32 *pTimeStamp);
        tResult sendCrossingInfo(tInt8 type, tFloat32 distance, tFloat32 orientation, tTimeStamp timeOfFrame);



        tBitmapFormat	m_sBitmapFormat;		/**< bitmap format for the RGB video */


        CrossingModel cModel;
        


private:

        int pic_count;
        tBool m_enabled;

        tResult SendLaneInfo(tFloat32 curvature, tFloat32 lanePos, tFloat32 carOrientation, tUInt32 timeStamp);

		tBool firstFrame;				/**< flag for the first frame*/
		tUInt8 imagecount;				/**< counter for the imaes*/
		tBitmapFormat m_sInputFormat;	/**< bitmap format of the input image*/

        cObjectPtr<IMediaTypeDescription> m_pCoderDescCrossing;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescMotionData;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescValue;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSign;
};

#endif 
