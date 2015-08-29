//
//  ParkingSpaceDetector.cpp
//  TestExtendedDetector
//
/**
Copyright (c)
Audi Autonomous Driving Cup. Team MomenTUM . All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 **/

/***************************************************************************
 * $Author:: Paul Bergmann $  $Date:: 2015-01-15 13:29:48#$ $Rev:: 26104  $*
 ***************************************************************************/

#include "ParkingSpaceDetector.h"

#define PATH_30FEATURES "/home/florian/AADC/Wettbewerb2016/AADC/src/adtfUser/implementation/src/vision/MTUM_ParkingSpaceDetection/strongClassifiers/classifier_parking.txt"
#define PATH_2FEATURES "/home/florian/AADC/Wettbewerb2016/AADC/src/adtfUser/implementation/src/vision/MTUM_ParkingSpaceDetection/strongClassifiers/classifier_parking_2.txt"

ParkingSpaceDetector::ParkingSpaceDetector(int size,Point p1,Point p2) : classifier_2features(PATH_2FEATURES,0.5),classifier_30features(PATH_30FEATURES,0.6)
{
    detectorSize = size;
    foe_1 = p1;
    foe_2 = p2;
}

vector<Point2d> ParkingSpaceDetector::detect(Mat grayscaleImage,Mat sobeledImage,Mat groundplaneImage)
{
    vector<Point2d> detectedMarkers;
    
    int totalDetectors = 0;
    int detectorsEval  = 0;

//    //sobel the input image
//    Mat sobeledImage;

//    Mat grad_x,grad_y;
//    Mat abs_grad_x,abs_grad_y;

//    Sobel(grayscaleImage,grad_x,grayscaleImage.depth(),1,0);
//    convertScaleAbs(grad_x,abs_grad_x);

//    Sobel(grayscaleImage,grad_y,grayscaleImage.depth(),0,1);
//    convertScaleAbs(grad_y,abs_grad_y);

//    addWeighted(abs_grad_x,0.5,abs_grad_y,0.5,0,sobeledImage);

    vector<Point> checkContour;
    checkContour.push_back(Point(100,grayscaleImage.rows-1));
    checkContour.push_back(Point(195,grayscaleImage.rows-1));
    checkContour.push_back(Point(126,50));
    checkContour.push_back(Point(100,50));
    
    for(int i = foe_1.y;i < foe_2.y-detectorSize;i+=3)
    {
        for(int j = foe_1.x;j < foe_2.x-detectorSize;j+=1)
        {

            if(pointPolygonTest(checkContour, Point(j,i),false) < 0)
            {
                continue;
            }

            totalDetectors++;

            if(groundplaneImage.at<uchar>(i+8,j+8) == 0)continue;
            if(sobeledImage.at<uchar>(i+8,j+8) < 100)continue;

            Rect x(j,i,detectorSize,detectorSize);
            Mat detectorGray = grayscaleImage(x);


            Mat integralImage;Mat rotatedIntegralImage;Mat dummy;
            cv::integral(detectorGray, integralImage, dummy, rotatedIntegralImage);
          //  int classifyResult = classifier_2features.classifyImage(integralImage, rotatedIntegralImage);
//
          //  if(classifyResult == 0)continue;

            int classifyResult = classifier_30features.classifyImage(integralImage, rotatedIntegralImage);

            detectorsEval++;

            if(classifyResult == 1)
            {
                //imshow("CORRECT",detectorGray);
                //waitKey(0);

                Point2d newPoint(j+8,i+8);
                detectedMarkers.push_back(newPoint);
                detectedMarkers.push_back(newPoint+Point2d(0,2));
                detectedMarkers.push_back(newPoint+Point2d(0,-2));

                detectedMarkers.push_back(newPoint+Point2d(-1,0));
                detectedMarkers.push_back(newPoint+Point2d(-1,2));
                detectedMarkers.push_back(newPoint+Point2d(-1,-2));

                detectedMarkers.push_back(newPoint+Point2d(1,0));
                detectedMarkers.push_back(newPoint+Point2d(1,2));
                detectedMarkers.push_back(newPoint+Point2d(1,-2));
                //detectedMarkers.push_back(newPoint+Point2d(0,-2));
                //detectedMarkers.push_back(newPoint+Point2d(0,2));
            }
        }
    }


    Mat circleImage(grayscaleImage.rows,grayscaleImage.cols,CV_8UC1,Scalar(0));
    for(int i = 0;i < detectedMarkers.size();i++)
    {
        circle(circleImage,detectedMarkers.at(i),2,Scalar(255),-1);
    }

    vector<vector<Point > > contours;
    vector<Vec4i> hierarchy;

    findContours(circleImage,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point(0,0));

    //find large contours:

    vector<vector<Point> > largeContours;

    int areaThreshold = 30;
    for(int i = 0;i < contours.size();i++)
    {
        vector<Point> currentCont = contours.at(i);
        double area = contourArea(currentCont);
        if(area < areaThreshold)
        {
            continue;
        }
        largeContours.push_back(currentCont);
    }

    vector<Point2d> mass_centers;
    for(int i = 0;i < largeContours.size();i++)
    {
        vector<Moments> contoursMoments;
        Moments currMom = moments(largeContours.at(i));
        double x_cent = currMom.m10/currMom.m00;
        double y_cent = currMom.m01/currMom.m00;
        Point2d mass_center(x_cent,y_cent);
        mass_centers.push_back(mass_center);
    }
    
    return mass_centers;
}


