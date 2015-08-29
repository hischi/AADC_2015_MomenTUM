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

#ifndef _PARKING_DETECTION_FILTER_HEADER_
#define _PARKING_DETECTION_FILTER_HEADER_

#define OID_ADTF_PARKING_DETECTION  "adtf.mtum.ParkingDetection"

#include "momenTUM_const.h"
#include "ParkingSpaceDetector.h"
#include "ParkingSpaceModel.h"

class cParkingDetection : public adtf::cFilter
{
     ADTF_DECLARE_FILTER_VERSION(OID_ADTF_PARKING_DETECTION, "AADC ParkingDetection", adtf::OBJCAT_Tool, "AADC ParkingDetection", 1, 0, 0, "Beta Version");
protected:
//RGB image input pin
		cVideoPin		m_oVideoInputPin;		/**< the input pin for the video*/

//RGB image output pin
        cVideoPin      m_oVideoOutputPin;       /**< the output for the video showing the detected lines*/ 
        cOutputPin	   m_outputPin_distToLot;
        cInputPin	   m_inputPin_reset;
        cInputPin      m_inputPin_Motion;


public:

        cParkingDetection(const tChar*);
        virtual ~cParkingDetection();

		tResult Init(tInitStage eStage, __exception=NULL);
		tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr=NULL);
        tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

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

        tBitmapFormat	m_sBitmapFormat;		/**< bitmap format for the RGB video */

        tTimeStamp GetTime();

private:

		tBitmapFormat m_sInputFormat;	/**< bitmap format of the input image*/

        tBool firstFrame;				/**< flag for the first frame*/

        cObjectPtr<IMediaTypeDescription> m_pCoderDescLaneInfo;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescMotionData;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
        cObjectPtr<IMediaTypeDescription> m_pCoderDescParking;

        ParkingSpaceDetector pDetector;
        ParkingSpaceModel    pModel;

        bool doDetect;
};

#endif 
