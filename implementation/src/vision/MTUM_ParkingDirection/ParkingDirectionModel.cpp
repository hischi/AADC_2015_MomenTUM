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

#include "ParkingDirectionModel.h"

ParkingDirectionModel::ParkingDirectionModel() : detector(16,Point(0,40),Point(200,250))
{
    certaintyParallel = 0.0;
    certaintyCross    = 0.0;
}

void ParkingDirectionModel::reset()
{
    certaintyParallel = 0.0;
    certaintyCross    = 0.0;
    pointsForDrawing.clear();
}


//sort 2 contours by y-center value
bool acompare(vector<Point> c1,vector<Point> c2)
{
    Moments currMoments = moments(c1);
    double y_cent_1 = currMoments.m01 / currMoments.m00;

    currMoments = moments(c2);
    double y_cent_2 = currMoments.m01 / currMoments.m00;

    return y_cent_1 < y_cent_2;
}

//check weather to merge two contours based on the center points and directions
bool ParkingDirectionModel::mergeContours(Point2d centre1,Point2d dir1,Point2d centre2,Point2d dir2)
{
    Point centreDir = centre2-centre1;

    //calc angle centre1/2 centreDir and threshold this to boolean

    double scalarProd1 = (centreDir.x*dir1.x+centreDir.y*dir1.y);
    double absDir1    = sqrt(dir1.x*dir1.x+dir1.y*dir1.y);
    double absCent    = sqrt(centreDir.x*centreDir.x+centreDir.y*centreDir.y);

    double cos1 = scalarProd1/(absDir1*absCent);

    double scalarProd2 = (centreDir.x*dir2.x+centreDir.y*dir2.y);
    double absDir2    = sqrt(dir2.x*dir2.x+dir2.y*dir2.y);

    double cos2 = scalarProd2/(absDir2*absCent);

    double angle1 = acos(cos1)*(180/3.1415926);
    double angle2 = acos(cos2)*(180/3.1415926);

    if(angle1 > 90) angle1 = 180-angle1;
    if(angle2 > 90) angle2 = 180-angle2;


    int angleThresh = 10;
    return (abs(angle1) < angleThresh && abs(angle2) < angleThresh);
}

void ParkingDirectionModel::cameraUpdate(Mat gray,Mat sobel,Mat ground)
{
    int pointNrThresh = 20;

    Point2d carOffset(100,0);
    vector<Point> checkContour;
    Point p1(Point2d(-50,45)+carOffset);
    Point p2(Point2d(0,45)+carOffset);
    Point p3(Point2d(0,90)+carOffset);
    Point p4(Point2d(-50,90)+carOffset);
    checkContour.push_back(p1);
    checkContour.push_back(p2);
    checkContour.push_back(p3);
    checkContour.push_back(p4);

    double offGroundPoints;
    vector<Point2d> points = detector.detect(gray,sobel,ground,checkContour,&offGroundPoints);
    pointsForDrawing = points;
    if(points.size() < pointNrThresh)
    {
        certaintyCross++;
        return;
    }

    double avg_x = 0.0;
    double avg_y = 0.0;

    for(int i = 0;i < points.size();i++)
    {
        avg_x += points.at(i).x;
        avg_y += points.at(i).y;
    }

    avg_x /= points.size();
    avg_y /= points.size();

    double deviation_x = 0.0;
    double deviation_y = 0.0;

    for(int i = 0;i < points.size();i++)
    {
        deviation_x += abs(points.at(i).x-avg_x);
        deviation_y += abs(points.at(i).y-avg_y);
    }

    if(deviation_x > deviation_y)
    {
        certaintyCross++;
        return;
    }
    else
    {
        certaintyParallel++;
        return;
    }
}

void ParkingDirectionModel::paintDebugInfo(Mat colorIPM)
{
    Point2d carOffset(100,0);

    //show working detector
    for(int i = 0;i < (int)pointsForDrawing.size();i++)
    {
        circle(colorIPM,pointsForDrawing.at(i),1,Scalar(0,0,255),-1);
    }

    Point p1(Point2d(-50,45)+carOffset);
    Point p2(Point2d(0,45)+carOffset);
    Point p3(Point2d(0,90)+carOffset);
    Point p4(Point2d(-50,90)+carOffset);
    line(colorIPM,p1,p2,Scalar(255,0,255),2);
    line(colorIPM,p2,p3,Scalar(255,0,255),2);
    line(colorIPM,p3,p4,Scalar(255,0,255),2);
    line(colorIPM,p4,p1,Scalar(255,0,255),2);

    return;
}
