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

// CrossingDetection
// Output description:
//   i8Type:       {0: no turn possible, 1: right turn possible, 2: left turn possible, 3: right and left turn possible, 4: must turn (T crossing) }
//   f32Distance:  Distance in meters between camera and oposing crossing line

#include "stdafx.h"
#include "crossingDetection.h"
const double DEG_TO_RAD=M_PI/180;
const double RAD_TO_DEG=180/M_PI;

ADTF_FILTER_PLUGIN("CrossingDetection", OID_ADTF_CrossingDetection, cCrossingDetection)

cCrossingDetection::cCrossingDetection(const tChar* __info) : cFilter(__info)
{
    lastT = cv::Mat::zeros(3,1,CV_32FC1);
    lastR = cv::Mat::zeros(3,1,CV_32FC1);
    lastId = 0;
    h_cutout = 234; //Kinect horizontal cutout height (What is higher is not relevant for the floor)
    timeAtLastSign = 0;
    cam = Camera(524.692545, 524.692545,319.5, 239.5, 0.041, -0.15,0.00673,0.0,0.0);
    calibrateZ = false; positionFromMarker=true;
    SetPropertyBool("Position from marker", positionFromMarker);
    SetPropertyBool("Calibrate Z", calibrateZ);
    for(int i=0;i<4;i++) remappedCorners[i] = Point2f(0.,0.);
}

cCrossingDetection::~cCrossingDetection()
{
#if defined(WIN32)
    cv::destroyWindow("crossingDetection");
#endif
}

tResult cCrossingDetection::Init(tInitStage eStage, __exception )
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        //Video Input
        RETURN_IF_FAILED(m_iVideo.Create("Video_Input", IPin::PD_Input, static_cast<IPinEventSink*>(this)));
        RETURN_IF_FAILED(RegisterPin(&m_iVideo));

        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
        //Sign Detector
        tChar const * strMarkerInfoValue = pDescManager->GetMediaDescription("tRoadSign");
        RETURN_IF_POINTER_NULL(strMarkerInfoValue);
        cObjectPtr<IMediaType> pTypeSignInfoValue = new cMediaType(0, 0, 0, "tRoadSign", strMarkerInfoValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeSignInfoValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSign));
        RETURN_IF_FAILED(m_iSign.Create("RoadSign", pTypeSignInfoValue  , static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_iSign));

        //Motion Estimator
        tChar const * strDescMotionData = pDescManager->GetMediaDescription("tMotionDataExt");
        RETURN_IF_POINTER_NULL(strDescMotionData);
        cObjectPtr<IMediaType> pTypeMotionData = new cMediaType(0, 0, 0, "tMotionDataExt", strDescMotionData,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeMotionData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescMotion));
        RETURN_IF_FAILED(m_iMotion.Create("motionData", pTypeMotionData, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_iMotion));

        //Lane Information
        tChar const * strDescLaneInfoValue = pDescManager->GetMediaDescription("tLaneInfo");
        RETURN_IF_POINTER_NULL(strDescLaneInfoValue);
        cObjectPtr<IMediaType> pTypeLaneInfoValue = new cMediaType(0, 0, 0, "tLaneInfo", strDescLaneInfoValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(pTypeLaneInfoValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescLane));
        RETURN_IF_FAILED(m_iLane.Create("laneInfo", pTypeLaneInfoValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_iLane));

        //Video Output
        RETURN_IF_FAILED(m_oVideo.Create("Video_Output",IPin::PD_Output, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oVideo));

        //this is a OutputPin
        tChar const * strDesc = pDescManager->GetMediaDescription("tCrossingInfo");
        if(!strDesc) LOG_ERROR(cString(OIGetInstanceName()) + ": Could not load mediadescription tCrossingInfo, check path");
        RETURN_IF_POINTER_NULL(strDesc);
        cObjectPtr<IMediaType> pType = new cMediaType(0, 0, 0, "tCrossingInfo", strDesc, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_oCrossing.Create("CrossingInfo", pType, this));
        RETURN_IF_FAILED(RegisterPin(&m_oCrossing));
        RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescCrossing));
    }
    else if (eStage == StageNormal)
    {
        firstFrame = tTrue;
        positionFromMarker = GetPropertyBool("Position from marker");
        calibrateZ = GetPropertyBool("Calibrate Z");
        LOG_WARNING(adtf_util::cString::Format("Options: CalibrateFromZ: %d, PositionFromMarker: %d", calibrateZ, positionFromMarker));
    }
    RETURN_NOERROR;
}

tResult cCrossingDetection::sendCrossingInfo(tInt8 type, tFloat32 distance,tTimeStamp timeOfFrame)
{
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescCrossing->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();

    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));

    cObjectPtr<IMediaCoder> pCoder;
    RETURN_IF_FAILED(m_pCoderDescCrossing->WriteLock(pMediaSample, &pCoder));
    pCoder->Set("i8Type", (tVoid*) &(type));
    pCoder->Set("f32Distance", (tVoid*)&distance);
    pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timeOfFrame);
    RETURN_IF_FAILED(m_pCoderDescCrossing->Unlock(pCoder));

    RETURN_IF_FAILED(pMediaSample->SetTime(pMediaSample->GetTime()));
    RETURN_IF_FAILED(m_oCrossing.Transmit(pMediaSample));
    RETURN_NOERROR;
}

void cCrossingDetection::fixVideoFormat(cv::Mat image)
{
    if(image.cols == m_sBitmapFormat.nWidth && image.rows == m_sBitmapFormat.nHeight && (image.channels()*8==m_sBitmapFormat.nBitsPerPixel))
        return;
    LOG_WARNING(adtf_util::cString::Format("Changing image format: [%dx%d], %d channels",image.cols,image.rows,image.channels()));
    m_sBitmapFormat.nWidth = image.cols;
    m_sBitmapFormat.nHeight = image.rows;
    m_sBitmapFormat.nBitsPerPixel = 8 * image.channels();
    m_sBitmapFormat.nPixelFormat = (image.channels() == 3) ? cImage::PF_RGB_888 : cImage::PF_GREYSCALE_8;
    m_sBitmapFormat.nBytesPerLine = image.cols * image.channels();
    m_sBitmapFormat.nSize = m_sBitmapFormat.nBytesPerLine * image.rows;
    m_sBitmapFormat.nPaletteSize = 0;
    m_oVideo.SetFormat(&m_sBitmapFormat, NULL);
}

tResult cCrossingDetection::PropertyChanged(const char* strProperty)
{
    RETURN_NOERROR;
}

tResult cCrossingDetection::Shutdown(tInitStage eStage, ucom::IException** __exception_ptr)
{
    switch (eStage)
    {
    case cFilter::StageFirst:
        break;
    default:
        break;
    }
    return cFilter::Shutdown(eStage,__exception_ptr);
}

void cCrossingDetection::updateCornersFromMotion(float dY, float dPhi)
{
    for(size_t i=0; i<4;i++)
        updatePointFromMotion(dY,dPhi,&remappedCorners[i]);
}

void cCrossingDetection::updatePointFromMotion(float dY, float dPhi, cv::Point2f* point)
{
    if(dY != 0.0 || dPhi!=0.0) {
        if(abs(dY)>100.0 || abs(dPhi)>0.7) {
            LOG_WARNING(adtf_util::cString::Format("CrossingDetection: WARNING Motion Estimator input too high: (dY: %.03f dPhi: %.03f). Skipping...", dY, dPhi));
            return;
        }
        float sin_phi = -sin(dPhi);
        float cos_phi = cos(dPhi);
        Point2f cameraToIMU(0.,29.); //[cm]
        point->x = (point->x + cameraToIMU.x)*cos_phi - (point->y + cameraToIMU.y)*sin_phi - cameraToIMU.x;
        point->y = (point->x + cameraToIMU.x)*sin_phi + (point->y + cameraToIMU.y)*cos_phi - cameraToIMU.y - dY;
    }
}

tResult cCrossingDetection::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);
    if(nEventCode == IPinEventSink::PE_MediaSampleReceived) {
        if(pSource == &m_iVideo) {
            //Videoformat
            if (firstFrame) {
                cObjectPtr<IMediaType> pType;
                RETURN_IF_FAILED(m_iVideo.GetMediaType(&pType));
                cObjectPtr<IMediaTypeVideo> pTypeVideo;
                RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
                const tBitmapFormat* pFormat = pTypeVideo->GetFormat();
                if (pFormat == NULL) {
                    LOG_ERROR("CrossingDetection: No Bitmap information found on pin");
                    RETURN_ERROR(ERR_NOT_SUPPORTED);
                }
                m_sInputFormat.nPixelFormat = pFormat->nPixelFormat;
                m_sInputFormat.nWidth = pFormat->nWidth;
                m_sInputFormat.nHeight =  pFormat->nHeight;
                m_sInputFormat.nBitsPerPixel = pFormat->nBitsPerPixel;
                m_sInputFormat.nBytesPerLine = pFormat->nBytesPerLine;
                m_sInputFormat.nSize = pFormat->nSize;
                m_sInputFormat.nPaletteSize = pFormat->nPaletteSize;
                firstFrame = tFalse;
            }
            ProcessVideoFrame(pMediaSample);
        } else if(pSource == &m_iSign) {
            ProcessSign(pMediaSample);
        } else if(pSource == &m_iMotion) {
            ProcessMotionData(pMediaSample);
        } else if(pSource == &m_iLane) {
            ProcessLaneInfo(pMediaSample);
        }
    }
    if (nEventCode== IPinEventSink::PE_MediaTypeChanged) {
        if (pSource == &m_iVideo) {
            cObjectPtr<IMediaType> pType;
            RETURN_IF_FAILED(m_iVideo.GetMediaType(&pType));
            cObjectPtr<IMediaTypeVideo> pTypeVideo;
            RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_VIDEO, (tVoid**)&pTypeVideo));
            UpdateImageFormat(m_iVideo.GetFormat());
        }
    }
    RETURN_NOERROR;
}

/**
 * @brief lineGroupComparator
 * Compares pair of line groups by total length
 */
bool lineGroupComparator(std::vector<LineSample> i, std::vector<LineSample> j) {
    float length_i=0,length_j=0;
    for(size_t ii=0;ii<i.size();ii++) length_i+=i[ii].length;
    for(size_t jj=0;jj<j.size();jj++) length_j+=j[jj].length;
    return(length_i>length_j);
}

bool cCrossingDetection::isOpenForTurning(Mat *image, Point p1, Point p2, Point p3, Point p4, bool horizontal)
{
    Mat mask(image->size(), CV_8UC1,Scalar::all(0));
    Point points[1][4];
    points[0][0] = p1;
    points[0][1] = p2;
    points[0][2] = p3;
    points[0][3] = p4;
    const Point* ppt[1] = { points[0] };
    int npt[] = { 4 };
    fillPoly( mask, ppt, npt, 1, Scalar(255,255,255),8);
    Mat masked(image->size(), CV_8UC1,Scalar::all(0));
    image->copyTo(masked, mask);
    threshold(masked,masked, 130, 255,THRESH_TOZERO);
    Canny(masked,masked,100,200,3,false);
    std::vector<Vec4i> lines;
    HoughLinesP(masked,lines,1,1*CV_PI/180,20,10,5);
    float hough_length_sq=0;
    for(size_t i=0;i<lines.size();i++) {
        Vec4i l = lines[i];
        float y2 = l[3], y1 = l[1], x2=l[2], x1=l[0];
        cv::line(*image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255,0,0), 1, cv::LINE_AA);
        hough_length_sq += pow(y2-y1,2)+pow(x2-x1,2);
    }
    float length_fraction = (horizontal) ? sqrt(hough_length_sq/(pow(p3.y-p2.y,2)+pow(p3.x-p2.x,2))) : sqrt(hough_length_sq/(pow(p2.y-p1.y,2)+pow(p2.x-p1.x,2)));
    return (length_fraction<0.5);
}

void Camera::distortPoint(float x, float y, float *u, float *v)
{
    float xp = x, yp=y;
    if(useDistortion) {
        float r2 = pow(x,2) + pow(y,2);
        xp = xp*(1 + params.k1*r2 + params.k2*r2*r2 + params.k3*r2*r2*r2) + 2*params.p1*xp*yp + params.p2*(r2 + 2*pow(xp,2));
        yp = yp*(1 + params.k1*r2 + params.k2*r2*r2 + params.k3*r2*r2*r2) + 2*params.p2*xp*yp + params.p1*(r2 + 2*pow(yp,2));
    }
    *u = params.fx*xp + params.cx;
    *v = params.fy*yp + params.cy;
}

void Camera::undistortPoint(float u, float v, float *x, float *y)
{
    *x = (u - params.cx)/params.fx;
    *y = (v - params.cy)/params.fy;
}

tResult cCrossingDetection::ProcessMotionData(IMediaSample* pSample)
{
    RETURN_IF_POINTER_NULL(pSample);
    tFloat32 dY;
    tFloat32 dPhi;
    tUInt32  timestamp;
    ReceiveMotionDataMessage(pSample, &dY, &dPhi, &timestamp);
    updateCornersFromMotion(dY, dPhi);
    RETURN_NOERROR;
}

tResult cCrossingDetection::ProcessSign(IMediaSample* pSample)
{
    RETURN_IF_POINTER_NULL(pSample);
    tInt8 id;
    cv::Mat tVec = cv::Mat::zeros(3,1,CV_32FC1);
    cv::Mat rVec = cv::Mat::zeros(3,1,CV_32FC1);
    tUInt32 timeStamp;
    ReceiveRoadSign(pSample, &id, &tVec, &rVec, &timeStamp);
    //if(id) LOG_WARNING(adtf_util::cString::Format("(%d) T: (%.03f,%.03f,%.03f) R: (%.03f,%.03f,%.03f)", id, tVec.at<float>(0),tVec.at<float>(1),tVec.at<float>(2), rVec.at<float>(0),rVec.at<float>(1),rVec.at<float>(2)));
    lastId = id;
    if(lastId) {
        lastT = tVec;
        lastR = rVec;
    }
    RETURN_NOERROR;
}

tResult cCrossingDetection::ProcessLaneInfo(IMediaSample* pSample)
{
    RETURN_IF_POINTER_NULL(pSample);
    tFloat32 curvature;
    tFloat32 position;
    tFloat32 orientation;
    tUInt32 timeStamp;
    ReceiveLaneInfo(pSample, &curvature, &position, &orientation, &timeStamp);
    LOG_WARNING(adtf_util::cString::Format("Lane Info: %.03f %.03f", position, orientation*RAD_TO_DEG));
    RETURN_NOERROR;
}

int cCrossingDetection::getSegmentColor(Mat image, float x1, float y1, float x2, float y2, bool isVertical)
{
    int pix_count=0;
    int sum=0;
    float a=0,b=0,c=0;
    if (isVertical) { //Vertical
        if (y1<y2) { a = y1; b=y2; c=x1; }
        else       { a = y2; b=y1; c=x2; }
        float m_inv = (x2-x1)/(y2-y1);
        for (float i=a;i<b;i++) {
            float j = (i-a)*m_inv + c;
            for(float u=j-1;u<=j+1;u++) {
                if(u<image.cols && u>=0) {
                    sum += (int) image.at<unsigned char>((int)i,(int)j);
                    pix_count++;
                }
            }
        }
    } else { //Horizontal
        if (x1<x2) { a = x1; b=x2; c=y1; }
        else       { a = x2; b=x1; c=y2; }
        float m = (y2-y1)/(x2-x1);
        for (float i=a;i<b;i++) {
            float j = (i-a)*m + c;
            for(float v=j-1;v<=j+1;v++) {
                if(v<image.rows && v>=0) {
                    sum += (int) image.at<unsigned char>((int)j,(int)i);
                    pix_count++;
                }
            }
        }
    }
    return ((int)sum/pix_count);
}

tResult cCrossingDetection::ProcessVideoFrame(IMediaSample* pSample)
{
    RETURN_IF_POINTER_NULL(pSample);
    const tVoid* l_pSrcBuffer;
    if (IS_OK(pSample->Lock(&l_pSrcBuffer))) {
        Mat image = Mat(m_sInputFormat.nHeight,m_sInputFormat.nWidth,CV_8UC1,(tVoid*)l_pSrcBuffer,m_sInputFormat.nBytesPerLine);
        Mat gray = image(Rect(0,0,640,480));
        Mat transformedImage = image(Rect(0,480,200,250)).clone();
        Mat sobeledImage     = image(Rect(0,480+250,200,250)).clone();
        Mat groundPlane      = image(Rect(0,480+2*250,200,250)).clone();

        pSample->Unlock(l_pSrcBuffer);
//        if(m_sInputFormat.nWidth != 640 || m_sInputFormat.nHeight != 480) {
//            LOG_WARNING(adtf_util::cString::Format("Incompatible input image size: (%d,%d). Expected: [640x480]. Skipping...",m_sInputFormat.nWidth,m_sInputFormat.nHeight));
//            RETURN_ERROR(ERR_NOT_SUPPORTED);
//        }
//        Mat gray(m_sInputFormat.nWidth,m_sInputFormat.nHeight,CV_8UC1,Scalar(0,0,0));
//        cvtColor(image,gray,COLOR_BGR2GRAY);
        //Mat gray = transformedImage; //image(Rect(0,0,200,480+250));
        Mat remapped = image(Rect(0,480,200,250));
        //cv::Mat croppedImage = gray(cv::Rect(cv::Point2i(0,h_cutout),cv::Point2i(m_sInputFormat.nWidth,m_sInputFormat.nHeight)));
        if(map_x.size() != gray.size() || map_y.size() != gray.size()) {
            map_x.create(gray.size(),CV_32FC1); map_y.create(gray.size(),CV_32FC1);
            inv_map_x.create(gray.size(),CV_32FC1); inv_map_y.create(gray.size(),CV_32FC1);
            computeRemappingLUT(gray,&map_x,&map_y);
            computeInvRemappingLUT(gray, &inv_map_x, &inv_map_y);
        }
        Mat croppedEqualized;
        Mat cannyImage;
        Mat croppedRemaped = image(Rect(0,480,200,250));
        Mat croppedColor;
        croppedColor=gray;
        if((calibrateZ || lastId)&&false) { //Slow
//            cv::Ptr<cv::CLAHE> clahe = createCLAHE();
//            clahe->setClipLimit(4);
//            clahe->apply(croppedImage,croppedEqualized);
            //croppedEqualized = croppedImage;
            //cv::Canny(croppedEqualized, cannyImage, 350,600,3, true); //1FPS
            //cv::remap(croppedEqualized, croppedRemaped, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_DEFAULT, cv::Scalar(0,0,0)); //5FPS
            //croppedRemaped = image(Rect(0,480,200,250));
            cvtColor(croppedRemaped,croppedColor,COLOR_GRAY2RGB);
        }

        if (calibrateZ) {
            logZForCalibration(croppedColor);
        } else if(lastId) { //Sign detected
            Mat rMat;
            Rodrigues(lastR,rMat);
            Mat corners[4];
            corners[0] = (Mat_<float>(3,1) << 0.1,0.1,0.35); //  x:down, y:right, z:towards camera
            corners[1] = corners[0] + (Mat_<float>(3,1) << 0.,1.,0.);
            corners[2] = corners[0] + (Mat_<float>(3,1) << 0.,0.,1.);
            corners[3] = corners[2] + (Mat_<float>(3,1) << 0.,1.,0.);
//            Mat nonRot = (Mat_<float>(3,3) << 0, 1, 0,
//                                              1, 0, 0,
//                                              0, 0,-1);
            float yaw=M_PI/2, pitch=0.0, roll=atan2(rMat.at<float>(7),rMat.at<float>(8));//M_PI;
            //Mat rot = (Mat_<float>(3,3) << cos(yaw)*cos(pitch),cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll),cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll),sin(yaw)*cos(pitch),sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll),sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll),-sin(pitch),cos(pitch)*sin(roll),cos(pitch)*cos(roll));
            Mat rot = (Mat_<float>(3,3) << 0,-cos(roll),sin(roll),
                                           1,0,0,
                                           0,sin(roll),cos(roll));
            //Compute distance
            Mat x = /*nonRot*/rot*(Mat_<float>(3,1) << 0.1,0.15,0.35) + lastT;
            distanceToCrossing=x.at<float>(2);

            //evaluateCrossing(croppedRemaped,&croppedColor,corners,/*rMat*/rot,lastT,true);
            timeAtLastSign = time(0);
        } else if(false) {
            tInt8 crossingType = 0;
            // This section finds strong lines to detect the crossing without the marker
            std::vector<cv::Vec4i> lines;
            cv::HoughLinesP(cannyImage, lines, 1, 1*CV_PI/180, 20, 20, 10);
            std::vector<LineSample > horizontalLines, verticalLines;
            for( size_t i = 0; i < lines.size(); i++ ) { // Save detected vertical and horizontal lines
                cv::Vec4i l = lines[i];
                float x2 = inv_map_x.at<float>(l[3],l[2]);
                float y2 = inv_map_y.at<float>(l[3],l[2]);
                float x1 = inv_map_x.at<float>(l[1],l[0]);
                float y1 = inv_map_y.at<float>(l[1],l[0]);
                bool isHorizontal=false, isVertical=false;
                float m=0.0, m_inv=0.0;
                if(x2 != x1) {
                    m = (y2-y1)/(x2-x1);
                    if(y1 != y2) m_inv = (x2-x1)/(y2-y1);
                    if (abs(m)<0.5) isHorizontal = true;  // <26�
                    else if(abs(m)>1.732) isVertical = true; // >60�
                } else {
                    isVertical = true;
                }

                if(isHorizontal && (y1>10) && (y2>10)) { //Horizontal line & not the bottom line
                    //if(getSegmentColor(croppedRemaped,x1,y1,x2,y2,false)>100) { //Only add line if avg color over threshold
                    float y0 = -m*x1+y1;
                    horizontalLines.push_back(LineSample(m,y0,sqrt(pow(y2-y1,2)+pow(x2-x1,2))));
                    //}
                } else if(isVertical) {
                    //if(getSegmentColor(croppedRemaped,x1,y1,x2,y2,true)>140) {
                    float x0 = -m_inv*y1+x1;
                    verticalLines.push_back(LineSample(m_inv,x0,sqrt(pow(y2-y1,2)+pow(x2-x1,2))));
                    //}
                }
                if(false) { //Shows detected segments
                    if(x1>0&&x2>0&&y1>0&&y2>0&&x1<croppedColor.cols&&x2<croppedColor.cols&&y1<croppedColor.rows&&y2<croppedColor.rows) {
                        if(isHorizontal) {
                            int color = getSegmentColor(croppedRemaped,x1,y1,x2,y2, false);
                            cv::line(croppedColor, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(color,0,0), 3, cv::LINE_AA);
                        } else if (isVertical) {
                            int color = getSegmentColor(croppedRemaped,x1,y1,x2,y2, true);
                            cv::line( croppedColor, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0,0,color), 3, cv::LINE_AA);
                        }
                    }
                }
            }
            //Group and select strong horizontal lines
            std::vector<LineSample> selectedHorizontal;
            if(horizontalLines.size()>0) {
                for (size_t i=1;i<horizontalLines.size();i++) {
                    bool added = false;
                    for (size_t j=0;j<horizontalLineGroups.size();j++) { //Adds line i to line group j if within tolerance
                        float pointDiff = abs((horizontalLines[i].m*croppedRemaped.cols/2+horizontalLines[i].y0) - (horizontalLineGroups[j][0].m*croppedRemaped.cols/2+horizontalLineGroups[j][0].y0));
                        float slopeDiff = abs(atan(horizontalLines[i].y0)-atan(horizontalLineGroups[j][0].y0));
                        if(pointDiff<12 && slopeDiff < 0.2) { //10px, 10grad
                            horizontalLineGroups[j].push_back(horizontalLines[i]);
                            added = true;
                            break;
                        }
                    }
                    if(!added) {
                        std::vector<LineSample > group;
                        group.push_back(horizontalLines[i]);
                        horizontalLineGroups.push_back(group);
                    }
                }
                std::sort(horizontalLineGroups.begin(),horizontalLineGroups.end(),lineGroupComparator);
                for (size_t i=0,j=0;(i<horizontalLineGroups.size())&&(j<4);i++,j++) { // (j<4) take the best 4
                    float m=0,y0=0;
                    if(horizontalLineGroups[i].size()<5) continue;
                    float totalLength=0;
                    for (size_t j=0; j<horizontalLineGroups[i].size();j++) {
                        m+=horizontalLineGroups[i][j].m*horizontalLineGroups[i][j].length;
                        y0+=horizontalLineGroups[i][j].y0*horizontalLineGroups[i][j].length;
                        totalLength += horizontalLineGroups[i][j].length;
                    }
                    m/=totalLength;
                    y0/=totalLength;
                    selectedHorizontal.push_back(LineSample(m,y0,totalLength));
                    //int thickness =  1;
                    //cv::line(croppedColor, cv::Point(0, y0), cv::Point(croppedColor.cols-1, (croppedColor.cols-1)*m+y0), cv::Scalar(0,255,0), thickness, cv::LINE_AA);
                }
            }
            //Group and select strong vertical lines
            std::vector<LineSample> selectedVertical;
            if(verticalLines.size()>0) {
                for (size_t i=1;i<verticalLines.size();i++) {
                    bool added = false;
                    for (size_t j=0;j<verticalLineGroups.size();j++) { //Adds line i to line group j if within tolerance
                        float pointDiff = abs((verticalLines[i].m*croppedRemaped.rows/2+verticalLines[i].y0) - (verticalLineGroups[j][0].m*croppedRemaped.rows/2+verticalLineGroups[j][0].y0));
                        float slopeDiff = abs(atan(verticalLines[i].y0)-atan(verticalLineGroups[j][0].y0));
                        if(pointDiff<10 && slopeDiff < 0.2) { //10px, 10grad
                            verticalLineGroups[j].push_back(verticalLines[i]);
                            added = true;
                            break;
                        }
                    }
                    if(!added) {
                        std::vector<LineSample > group;
                        group.push_back(verticalLines[i]);
                        verticalLineGroups.push_back(group);
                    }
                }
                std::sort(verticalLineGroups.begin(),verticalLineGroups.end(),lineGroupComparator);
                for (size_t i=0,j=0;(i<verticalLineGroups.size())&&(j<3);i++,j++) {
                    float m_inv=0,x0=0;
                    if(verticalLineGroups[i].size()<5) continue;
                    float totalLength=0;
                    for (size_t j=0; j<verticalLineGroups[i].size();j++) {
                        m_inv+=verticalLineGroups[i][j].m*verticalLineGroups[i][j].length;
                        x0+=verticalLineGroups[i][j].y0*verticalLineGroups[i][j].length;
                        totalLength += verticalLineGroups[i][j].length;
                    }
                    m_inv/=totalLength;
                    x0/=totalLength;
                    selectedVertical.push_back(LineSample(m_inv,x0,totalLength));
                    //int thickness =  1;
                    //cv::line(croppedColor, cv::Point(x0, 0), cv::Point((croppedColor.rows-1)*m_inv+x0,croppedColor.rows-1), cv::Scalar(0,255,0), thickness, cv::LINE_AA);
                }
            }
            //Find intercept corresponding to bottom-right corner
            float max_x=-1,max_y=-1, min_dist2=640000;
            for(size_t i=0;i<selectedHorizontal.size();i++) {
                for(size_t j=0;j<selectedVertical.size();j++) {
                    float y = (selectedHorizontal[i].m*selectedVertical[j].y0 + selectedHorizontal[i].y0)/(1-selectedHorizontal[i].m*selectedVertical[j].m); // y=(m*x0+y0)/(1-m*m_inv)
                    float x = selectedVertical[j].m*y+selectedVertical[j].y0;
                    float dist2 = pow(croppedColor.cols-x,2) + y*y;
                    if(dist2<min_dist2) {
                        min_dist2 = dist2; max_x = x; max_y=y;
                    }
                }
            }
            // Reconstruct 3D structure of crossing and evaluate
            if(max_x>=0 && max_y>=0) {
                //cv::circle(croppedColor, Point(max_x,max_y), 1, Scalar(0,0,255),5);
                //Recovering 3D coordinate from max_x,max_y
                float unwarped_u = map_x.at<float>(max_y,max_x);
                float unwarped_v = map_y.at<float>(max_y,max_x)+h_cutout;
                float unxp,unyp;
                cam.undistortPoint(unwarped_u,unwarped_v,&unxp,&unyp);
                //float unzp = (max_y+19.8635)/71.6477;  //Linear regression done between Z and pixel height (points are in one plane)
                //float unzp = (max_y-0.41318)/61.5931;  //Linear regression done between Z and pixel height (points are in one plane)
                float unzp = (max_y+10.747)/73.1686;
                Mat corners[4];
                corners[3] = (Mat_<float>(3,1) << unxp*unzp,unyp*unzp,unzp);
                corners[2] = corners[3] + (Mat_<float>(3,1) << -1.,0.,0.);
                corners[1] = corners[3] + (Mat_<float>(3,1) << 0.,0.,1.);
                corners[0] = corners[1] + (Mat_<float>(3,1) << -1.,0.,0.);
                crossingType = evaluateCrossing(croppedRemaped,&croppedColor,corners);
            }

            sendCrossingInfo(crossingType,distanceToCrossing,pSample->GetTime());
            updateLineGroups();
        }
        if(lastId) {
            sendCrossingInfo(0,distanceToCrossing,pSample->GetTime());
            //LOG_WARNING(adtf_util::cString::Format("dist: %.03f", distanceToCrossing));
        }
        if(calibrateZ || lastId) {
            //cv::resize(croppedColor(cv::Rect(0,0,420,330)),croppedColor,cv::Size(0,0),2,2);
            //cv::flip(croppedColor,croppedColor,0);
            Mat transImage = transformedImage;
            cObjectPtr<IMediaSample> pNewRGBSample;
            if (IS_OK(AllocMediaSample(&pNewRGBSample)))
            {
                tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
                pNewRGBSample->Update(tmStreamTime, transImage.data, tInt32(transImage.channels()*transImage.cols*transImage.rows), 0);
                fixVideoFormat(transImage);
                RETURN_IF_FAILED(m_oVideo.Transmit(pNewRGBSample));
            }
        }
    }
    RETURN_NOERROR;
}

tInt8 cCrossingDetection::evaluateCrossing(cv::Mat &croppedRemaped, cv::Mat *croppedColor, cv::Mat *corners, cv::Mat rotation, cv::Mat translation, bool saveCorners)
{
    float rx[4],ry[4];
    bool rightTurnVisible=true;
    bool rightTurnPossible=true;
    bool leftTurnVisible=true;
    bool leftTurnPossible=true;
    bool frontLineVisible=true;
    tInt8 retval = 0;
    for(size_t i=0;i<4;i++) {
        if(corners!=NULL) {
            Mat x = rotation*corners[i] + translation; //If we want to add a rotation this would be the place
            float xp = x.at<float>(0)/x.at<float>(2);
            float yp = x.at<float>(1)/x.at<float>(2);
            float u, v;
            cam.distortPoint(xp,yp,&u,&v);
            if((u>=0 && u<inv_map_x.cols)&&((v-h_cutout)>=0 && (v-h_cutout)<inv_map_x.rows)) { //if(u>0 && v>0 && u<m_sInputFormat.nWidth && v<m_sInputFormat.nHeight) {
                float remaped_u = inv_map_x.at<float>(v-h_cutout,u);
                float remaped_v = inv_map_y.at<float>(v-h_cutout,u);
                rx[i]=remaped_u; ry[i]=remaped_v;
                if(saveCorners) {
                    remappedCorners[i].x = remaped_u; remappedCorners[i].y = remaped_v;
                }
                if (rx[i]<=0 || ry[i]<=0 || rx[i]>=croppedRemaped.cols || ry[i]>=croppedRemaped.rows) {
                    if (i==1 || i==3) rightTurnVisible=false;
                    if (i==0 || i==2) leftTurnVisible=false;
                    if (i==0 || i==1) frontLineVisible=false;
                }
                else if(saveCorners) cv::circle(*croppedColor, Point(rx[i],ry[i]), 1, Scalar(0,255,0),5); //Draws circle on the four corners of the crossing
            } else {
                if (i==1 || i==3) rightTurnVisible=false;
                if (i==0 || i==2) leftTurnVisible=false;
                if (i==0 || i==1) frontLineVisible=false;
            }
        } else { //Use saved corners (possibly updated by the motion estimator)
            rx[i] = remappedCorners[i].x; ry[i] = remappedCorners[i].y;
            if (rx[i]<=0 || ry[i]<=0 || rx[i]>=croppedRemaped.cols || ry[i]>=croppedRemaped.rows) {
                if (i==1 || i==3) rightTurnVisible=false;
                if (i==0 || i==2) leftTurnVisible=false;
                if (i==0 || i==1) frontLineVisible=false;
            }
            //else cv::circle(*croppedColor, Point(rx[i],ry[i]), 1, Scalar(0,0,255),5); //Draws circle on the four corners of the crossing
        }
    }
    if(!saveCorners) { // Now using save corners just to restrict the range of found corners
        float dist = sqrt(pow(remappedCorners[3].x-rx[3],2)+pow(remappedCorners[3].y-ry[3],2));
        if(dist>20) { //Distance between detected corners and marker too large, using marker
            rightTurnVisible=true; leftTurnVisible=true;
            for(size_t i=0;i<4;i++) {
                rx[i] = remappedCorners[i].x; ry[i] = remappedCorners[i].y;
                if (rx[i]<=0 || ry[i]<=0 || rx[i]>=croppedRemaped.cols || ry[i]>=croppedRemaped.rows) {
                    if (i==1 || i==3) rightTurnVisible=false;
                    if (i==0 || i==2) leftTurnVisible=false;
                    if (i==0 || i==1) frontLineVisible=false;
                }
            }
        } else distanceToCrossing=corners[1].at<float>(2);
        if(rightTurnVisible) {
            rightTurnPossible = isOpenForTurning(&croppedRemaped, Point((rx[1]+rx[3])/2-5,(ry[1]+ry[3])/2), Point((rx[1]+rx[3])/2+5,(ry[1]+ry[3])/2), Point(rx[3]+5,ry[3]), Point(rx[3]-5,ry[3]));
            cv::line(*croppedColor, cv::Point((rx[1]+rx[3])/2,(ry[1]+ry[3])/2), cv::Point(rx[3], ry[3]), (rightTurnPossible) ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255), 3, cv::LINE_AA);
            if (rightTurnPossible) retval += 1;
        }
        if(leftTurnVisible) {
            leftTurnPossible = isOpenForTurning(&croppedRemaped, Point(rx[0]-10,ry[0]), Point(rx[0]+10,ry[0]), Point((rx[0]+rx[2])/2+10,(ry[0]+ry[2])/2), Point((rx[0]+rx[2])/2-10,(ry[0]+ry[2])/2));
            cv::line(*croppedColor, cv::Point(rx[0], ry[0]), cv::Point((rx[0]+rx[2])/2,(ry[0]+ry[2])/2), (leftTurnPossible) ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255), 3, cv::LINE_AA);
            if (rightTurnPossible) retval += 2;
        }
        if(frontLineVisible) {
            bool mustTurn = isOpenForTurning(&croppedRemaped, Point((rx[0]+rx[1])/2,(ry[0]+ry[1])/2-10), Point(rx[1],(ry[0]+ry[1])/2-10), Point(rx[1],(ry[0]+ry[1])/2+10), Point((rx[0]+rx[1])/2,(ry[0]+ry[1])/2+10),false);
            cv::line(*croppedColor, cv::Point((rx[0]+rx[1])/2, (ry[0]+ry[1])/2), cv::Point(rx[1],(ry[0]+ry[1])/2), (mustTurn) ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255), 3, cv::LINE_AA);
            if (mustTurn) retval += 4;
        }
    }
    return retval;
}

void cCrossingDetection::logZForCalibration(cv::Mat &image)
{
    Mat zer = (Mat_<float>(3,1) << 0.1,0.0,0.0);
    Mat nonRot = (Mat_<float>(3,3) << 0, 1, 0,
                  1, 0, 0,
                  0, 0,-1);
    Mat xx = nonRot*zer + lastT;
    float xp = xx.at<float>(0)/xx.at<float>(2);
    float yp = xx.at<float>(1)/xx.at<float>(2);
    float u, v;
    cam.distortPoint(xp,yp,&u,&v);
    if((u>=0 && u<inv_map_x.cols)&&((v-h_cutout)>=0 && (v-h_cutout)<inv_map_x.rows)) {
        float remaped_u = inv_map_x.at<float>(v-h_cutout,u);
        float remaped_v = inv_map_y.at<float>(v-h_cutout,u);
        cv::circle(image, Point(remaped_u,remaped_v), 1, Scalar(0,255,0),5);
        LOG_WARNING(adtf_util::cString::Format("%.03f\",\"%.03f",xx.at<float>(2),remaped_v));
    }
}

/**
 * @brief cCrossingDetection::updateLineGroups
 * Deletes old samples from line groups
 */
void cCrossingDetection::updateLineGroups() {
    for(std::vector<std::vector<LineSample> >::iterator it_i=horizontalLineGroups.begin();it_i!=horizontalLineGroups.end();) {
        for(std::vector<LineSample>::iterator it_j=it_i->begin();it_j!=it_i->end();) {
            it_j->age++;
            if(it_j->age>4)
                it_j = it_i->erase(it_j);
            else
                ++it_j;
        }
        if(it_i->size()==0)
            it_i = horizontalLineGroups.erase(it_i);
        else
            ++it_i;
    }
    for(std::vector<std::vector<LineSample> >::iterator it_i=verticalLineGroups.begin();it_i!=verticalLineGroups.end();) {
        for(std::vector<LineSample>::iterator it_j=it_i->begin();it_j!=it_i->end();) {
            it_j->age++;
            if(it_j->age>4)
                it_j = it_i->erase(it_j);
            else
                ++it_j;
        }
        if(it_i->size()==0)
            it_i = verticalLineGroups.erase(it_i);
        else
            ++it_i;
    }
}

tResult cCrossingDetection::UpdateImageFormat(const tBitmapFormat* pFormat)
{
    if (pFormat != NULL)
        m_sInputFormat = (*pFormat);
    RETURN_NOERROR;
}

tResult cCrossingDetection::ReceiveRoadSign(IMediaSample* pMediaSample, tInt8 *pId, Mat *pTvec, Mat* pRvec, tUInt32 *pTimeStamp)
{
    cObjectPtr<IMediaCoder> pCoderInput;
    RETURN_IF_FAILED(m_pCoderDescSign->Lock(pMediaSample, &pCoderInput));
    pCoderInput->Get("i8Identifier", (tVoid*)pId);
    pCoderInput->Get("fl32TransX", (tVoid*)&pTvec->at<float>(0));
    pCoderInput->Get("fl32TransY", (tVoid*)&pTvec->at<float>(1));
    pCoderInput->Get("fl32TransZ", (tVoid*)&pTvec->at<float>(2));
    pCoderInput->Get("fl32RotX", (tVoid*)&pRvec->at<float>(0));
    pCoderInput->Get("fl32RotY", (tVoid*)&pRvec->at<float>(1));
    pCoderInput->Get("fl32RotZ", (tVoid*)&pRvec->at<float>(2));
    pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)pTimeStamp);
    m_pCoderDescSign->Unlock(pCoderInput);
    RETURN_NOERROR;
}

tResult cCrossingDetection::ReceiveMotionDataMessage(IMediaSample* pMediaSample, tFloat32 *pdY, tFloat32 *pdPhi, tUInt32 *pTimeStamp)
{
    cObjectPtr<IMediaCoder> pCoderInput;
    RETURN_IF_FAILED(m_pCoderDescMotion->Lock(pMediaSample, &pCoderInput));
    pCoderInput->Get("f32dY", (tVoid*)pdY); //in cm
    pCoderInput->Get("f32dPhi", (tVoid*)pdPhi);
    pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)pTimeStamp);
    m_pCoderDescMotion->Unlock(pCoderInput);
    RETURN_NOERROR;
}

tResult cCrossingDetection::ReceiveLaneInfo(IMediaSample* pMediaSample, tFloat32 *curvature, tFloat32 *position, tFloat32 *orientation, tUInt32 *pTimeStamp)
{
    cObjectPtr<IMediaCoder> pCoderInput;
    RETURN_IF_FAILED(m_pCoderDescLane->Lock(pMediaSample, &pCoderInput));
    pCoderInput->Get("f32Curvature", (tVoid*)curvature);
    pCoderInput->Get("f32LanePos", (tVoid*)position);
    pCoderInput->Get("f32CarOrientation", (tVoid*)orientation);
    pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)pTimeStamp);
    m_pCoderDescLane->Unlock(pCoderInput);
    RETURN_NOERROR;
}

void cCrossingDetection::computeRemappingLUT(cv::Mat image, cv::Mat *map_x, cv::Mat *map_y) {
    //Focal Length (mm)
    double f_u = cam.params.fx, f_v = cam.params.fy;
    //Principal Point
    double c_u = cam.params.cx, c_v = cam.params.cy;
    double pitch_angle = 1*DEG_TO_RAD;
    //cos(pitch_angle), cos(yaw_angle)
    double c_1 = cos(pitch_angle);
    double c_2 = 1.00;
    //sin(pitch_angle), sin(yaw_angle)
    double s_1 = sin(pitch_angle);
    double s_2 = 0.0;
    //height of camera (cm)
    double h = 22.5;

    //Transformation matrix T > Image points to ground plane
    cv::Mat T = (cv::Mat_<double>(4,4) << -c_2/f_u,  s_1*s_2/f_v,  c_u*c_2/f_u-c_v*s_1*s_2/f_v-c_1*s_2, 0,
                 s_2/f_u,  s_1*c_1/f_v, -c_u*s_2/f_u-c_v*s_1*c_2/f_v-c_1*c_2, 0,
                 0,      c_1/f_v,                     -c_v*c_1/f_v+s_1, 0,
                 0, -c_1/(f_v*h),                c_v*c_1/(h*f_v)-s_1/h, 0);
    T = h * T;

    cv::Mat T_inv = (cv::Mat_<double>(4,4) <<   f_u*c_2+c_u*c_1*s_2,   c_u*c_1*c_2-s_2*f_u,         -c_u*s_1, 0,
                     s_2*(c_v*c_1-f_v*s_1), c_2*(c_v*c_1-f_v*s_1), -f_v*c_1-c_v*s_1, 0,
                     c_1*s_2,               c_1*c_2,             -s_1, 0,
                     c_1*s_2,               c_1*c_2,             -s_1, 0);

    for (int p_y=0;p_y<480;p_y++) {
        for (int p_x=0;p_x<640;p_x++) {
            cv::Mat P_G = (cv::Mat_<double>(4,1) << p_x-210, p_y+40, -h ,1);
            cv::Mat P_I = T_inv*P_G;
            P_I /= P_I.at<double>(3,0);
            float x = (float) P_I.at<double>(0,0);
            float y = (float) P_I.at<double>(1,0);
            if(x>=0 && x<image.cols && y>=h_cutout && y<image.rows ) {
                (*map_y).at<float>(p_y,p_x) = y-h_cutout;  // y displacement, this LUT if for the cropped image.
                (*map_x).at<float>(p_y,p_x) = x;
            }
        }
    }
}

void cCrossingDetection::computeInvRemappingLUT(cv::Mat image, cv::Mat *map_x, cv::Mat *map_y)
{
    //Focal Length (mm)
    double f_u = cam.params.fx, f_v = cam.params.fy;
    //Principal Point
    double c_u = cam.params.cx, c_v = cam.params.cy;
    double pitch_angle = 2*DEG_TO_RAD;
    //cos(pitch_angle), cos(yaw_angle)
    double c_1 = cos(pitch_angle);
    double c_2 = 1.00;
    //sin(pitch_angle), sin(yaw_angle)
    double s_1 = sin(pitch_angle);
    double s_2 = 0.0;
    //height of camera (cm)
    double h = 22.5;

    //Transformation matrix T > Image points to ground plane
    cv::Mat T = (cv::Mat_<double>(4,4) << -c_2/f_u,  s_1*s_2/f_v,  c_u*c_2/f_u-c_v*s_1*s_2/f_v-c_1*s_2, 0,
                 s_2/f_u,  s_1*c_1/f_v, -c_u*s_2/f_u-c_v*s_1*c_2/f_v-c_1*c_2, 0,
                 0,      c_1/f_v,                     -c_v*c_1/f_v+s_1, 0,
                 0, -c_1/(f_v*h),                c_v*c_1/(h*f_v)-s_1/h, 0);
    T = h * T;

    for (int y=0;y<image.rows;y++) {
        for(int x=0; x<image.cols;x++) {
            cv::Mat P_I = (cv::Mat_<double>(4,1) << x,y,1,1);
            cv::Mat P_G = T*P_I;
            P_G /= P_G.at<double>(3,0);
            float x_value = (float) P_G.at<double>(0,0);
            float y_value = (float) P_G.at<double>(1,0);
            x_value += 210; //To get the frame in the image
            y_value -= 40;
            if(x_value>=0 && y_value>=0 && x_value < image.cols && y_value < image.rows && y >= h_cutout) {
                (*map_y).at<float>(y-h_cutout,x) = y_value;  //y-line_height: For the cropped image
                (*map_x).at<float>(y-h_cutout,x) = x_value;
            }
        }
    }
}

void cCrossingDetection::remapAndFlip(cv::Mat &I)
{
    cv::remap(I, I, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_DEFAULT, cv::Scalar(0,0,0));
    cv::resize(I(cv::Rect(0,0,420,330)),I,cv::Size(0,0),2,2);
    cv::flip(I,I,0);
}
