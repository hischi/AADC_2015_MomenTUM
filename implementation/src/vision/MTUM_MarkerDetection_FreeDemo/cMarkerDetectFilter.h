/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: forchhe, MomenTUM#$  $Date:: 2014-12-11 14:02:50#$ $Rev:: 29769   $
**********************************************************************/

/*! \brief Marker Detect Filter
 *         
 *   .
 */
#ifndef _MARKERDETECTFILTER_FREE_DEMO_HEADER
#define _MARKERDETECTFILTER_FREE_DEMO_HEADER
#define OID_MTUM_MARKERDETECTFILTER_FREEDEMO "adtf.mtum.markerDetectionFilter_FreeDemo"

//! class for filter ofroad sign detection
	/*!
	*/
class cMarkerDetectFilter : public cFilter
{
     ADTF_DECLARE_FILTER_VERSION(OID_MTUM_MARKERDETECTFILTER_FREEDEMO, "MTUM Marker Detection Free Demo", adtf::OBJCAT_Tool, "MTUM Marker Detection Free Demo", 0, 5, 0, "Beta Version");
public:
        
	cMarkerDetectFilter(const tChar* __info);
        virtual ~cMarkerDetectFilter();
	tResult Init(tInitStage eStage, __exception);	
       	tResult OnPinEvent(adtf::IPin* pSource, tInt nEventCode,tInt nParam1,tInt nParam2,adtf::IMediaSample* pMediaSample);
	
	tResult Shutdown(tInitStage eStage, __exception = NULL);

protected:
	/*! input Pin for video */
	cVideoPin m_oPinInputVideo;				 
	/*! output Pin for video */
	cVideoPin m_oPinOutputVideo;				
	/*! output Pin for detected Sign as tInt */
	cOutputPin m_oPinRoadSign;			
    cInputPin m_inputPin_enable;

	/*!  */	
	UCOM_DECLARE_TIMING_SPOT(m_oProcessStart)
	/*!  */		
	UCOM_DECLARE_TIMING_SPOT(m_oProcessEnd)

private:
	/*! bitmapformat of input image */	
	tBitmapFormat      m_sInputFormat;						
	/*! bitmapformat of output image */	
	tBitmapFormat      m_sOutputFormat;						
	/*! indicates wheter information is printed to the console or not */
	tBool m_bDebugModeEnabled;
	/* indicates what is transmitted over the output pin*/
	tInt m_outputMode;
	/* flag to check if any sign was detected in flag */
    tBool m_SignInFrame;
    tBool doDetect;
    tBool m_bDownsampleImage;
	
    tInt computeEveryXFrame;
    int frameCounter;

	/*! Coder Descriptor */
	cObjectPtr<IMediaTypeDescription> m_pCoderDescRoadSign;		
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;

	/*! function process the video data
	@param pSample the new media sample to be processed
	*/	
	tResult ProcessVideo(IMediaSample* pSample);
	
	/*! function to set the m_sProcessFormat and the  m_sInputFormat variables
	@param pFormat the new format for the input and input pin
	*/
	tResult UpdateInputImageFormat(const tBitmapFormat* pFormat);

	/*! function to set the m_output image format
	@param pFormat the new format for the output pin
	*/
	tResult UpdateOutputImageFormat(const tBitmapFormat* pFormat);	
	
	/*! function to convert the marker number to a traffic sign ID
	@param ID ID of the sign
	@param size size of the markers sides in meters
	@param timeOfFrame the timestamp of the frame where the sign was detected	
	*/
    tResult markerDetected(Marker marker, tTimeStamp timeOfFrame);
	/*! function to transmit a detected road sign
	@param ID ID of the sign
	@param size size of the markers sides in meters
	@param timeOfFrame the timestamp of the frame where the sign was detected
	*/
    tResult sendRoadSignStruct(tInt8 ID, Marker marker,tTimeStamp timeOfFrame);

    tResult LoadConfigurationData(cFilename m_fileConfig);
    void fixVideoFormat(cv::Mat image);
	
	/*! the aruco elements: */
	/*! the aruco detector for the markers*/
	MarkerDetector m_MDetector;
	/*! the aruco markers */
	vector<Marker> m_TheMarkers;	
	/*! size of the markers*/ 
	tFloat m_TheMarkerSize;
	/*! threshold parameter 1 for aruco detection*/		
	tInt32 m_iThresParam1;
	/*! threshold parameter 2 for aruco detection*/	
	tInt32 m_iThresParam2;
	/*! the intrinsic parameter of the camera*/
	CameraParameters m_TheCameraParameters;
	/*! the dictionary for the aruco lib*/ 
	Dictionary m_Dictionary;
    /*! vector of valid markers*/
    set<int> validMarkers;

};
#endif //_MARKERDETECTFILTER_HEADER
