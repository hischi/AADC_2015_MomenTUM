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

#ifndef _CrossingDetection_FILTER_HEADER_
#define _CrossingDetection_FILTER_HEADER_

#define OID_ADTF_CrossingDetection  "adtf.aadc.CrossingDetection"

struct LineSample {
    float m;
    float y0;
    float length;
    unsigned int age;
    LineSample() { m=y0=age=length=0; }
    LineSample(float _m, float _y0, float _length=0, unsigned int _age=0) { m = _m; y0 = _y0; length = _length; age = _age; }
};

class Camera {
public:
    struct Parameters {
        float fx,fy;
        float cx,cy;
        float k1,k2,k3;
        float p1,p2;
        Parameters(float _fx, float _fy,
                   float _cx, float _cy,
                   float _k1=0.0, float _k2=0.0, float _k3=0.0,
                   float _p1=0.0, float _p2=0.0) : fx(_fx),fy(_fy),cx(_cx),cy(_cy),k1(_k1),k2(_k2),k3(_k3),p1(_p1),p2(_p2) {}
        Parameters() {}
    };
    Camera() {}
    Camera(float fx, float fy,
           float cx, float cy,
           float k1=0.0, float k2=0.0, float k3=0.0,
           float p1=0.0, float p2=0.0):params(fx,fy,cx,cy,k1,k2,k3,p1,p2),useDistortion(false) {}
    Parameters params;
    bool useDistortion;
    void distortPoint(float x, float y, float *u, float *v);
    void undistortPoint(float u, float v, float *x, float *y);
};

class cCrossingDetection : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_CrossingDetection, "AADC CrossingDetection", adtf::OBJCAT_Tool, "AADC CrossingDetection", 1, 0, 0, "Beta Version");
protected:
    cVideoPin   m_iVideo;    /**< the input pin from the video*/
    cInputPin   m_iSign;     /**< the input pin from the marker detector*/
    cInputPin   m_iMotion;   /**< the input pin from the motion estimator*/
    cInputPin   m_iLane;     /**< the input pin from the line detector*/

    cVideoPin   m_oVideo;    /**< the output for the video showing the detected lines*/
    cOutputPin  m_oCrossing;

    tResult ReceiveRoadSign(IMediaSample* pMediaSample, tInt8 *pId, Mat *pTvec, Mat* pRvec, tUInt32 *pTimeStamp);
    tResult ReceiveMotionDataMessage(IMediaSample* pMediaSample, tFloat32 *pdY, tFloat32 *pdPhi, tUInt32 *pTimeStamp);
    tResult ReceiveLaneInfo(IMediaSample* pMediaSample, tFloat32 *curvature, tFloat32 *position, tFloat32 *orientation, tUInt32 *pTimeStamp);

public:
    cCrossingDetection(const tChar*);
    virtual ~cCrossingDetection();
    tResult Init(tInitStage eStage, __exception=NULL);
    tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr=NULL);
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
    tResult ProcessFound() { RETURN_NOERROR; }
    tResult ProcessOutput() { RETURN_NOERROR; }
    tResult PropertyChanged(const char* strProperty);
    tResult UpdateImageFormat(const tBitmapFormat* pFormat);
    bool isOpenForTurning(Mat *image, Point p1, Point p2, Point p3, Point p4, bool horizontal=true);
    void updateLineGroups();
    void updateCornersFromMotion(float dY, float dPhi);
    void updatePointFromMotion(float dY, float dPhi, cv::Point2f* point);
    int getSegmentColor(Mat image, float x1, float y1, float x2, float y2, bool isVertical);

    tBitmapFormat    m_sBitmapFormat;      /**< bitmap format for the RGB video */
    tBitmapFormat    m_sBitmapFormatDepth; /**< bitmap format for the Depth video */

private:
    tBool           firstFrame;            /**< flag for the first video frame*/
    tBitmapFormat   m_sInputFormat;        /**< bitmap format of the input image*/

    cv::Mat         map_x;
    cv::Mat         map_y;
    cv::Mat         inv_map_x;
    cv::Mat         inv_map_y;
    tResult ProcessVideoFrame(IMediaSample* pSample);
    tResult ProcessMotionData(IMediaSample* pSample);
    tResult ProcessSign(IMediaSample* pSample);
    tResult ProcessLaneInfo(IMediaSample* pSample);
    tResult sendCrossingInfo(tInt8 type, tFloat32 distance,tTimeStamp timeOfFrame);
    void computeRemappingLUT(cv::Mat image, cv::Mat *map_x, cv::Mat *map_y);
    void computeInvRemappingLUT(cv::Mat image, cv::Mat *map_x, cv::Mat *map_y);
    void remapAndFlip(cv::Mat &I);
    void logZForCalibration(cv::Mat &image);
    tInt8 evaluateCrossing(cv::Mat &croppedRemaped, cv::Mat *croppedColor, cv::Mat *corners=NULL, cv::Mat rotation=(Mat_<float>(3,3) << 1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0), cv::Mat translation=(Mat_<float>(3,1) << 0.0,0.0,0.0), bool saveCorners=false);
    void fixVideoFormat(cv::Mat image);
    cv::Mat lastT;           /**< last seen marker translation*/
    cv::Mat lastR;           /**< last seen marker rotation*/
    tInt8 lastId;            /**< last seen marker ID*/
    unsigned short h_cutout; /**< Kinect horizontal cutout height*/
    time_t timeAtLastSign;
    cv::Point2f remappedCorners[4];
    Camera cam;
    tBool positionFromMarker;
    tBool calibrateZ;
    float distanceToCrossing;

    cObjectPtr<IMediaTypeDescription> m_pCoderDescMotion;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSign;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescLane;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescCrossing;

    std::vector<std::vector<LineSample> > horizontalLineGroups;
    std::vector<std::vector<LineSample> > verticalLineGroups;

};

#endif 
