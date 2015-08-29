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

#include "CrossingModel.h"

#define T_0_BOX_WIDTH 40
#define T_0_BOX_HEIGHT 100
#define T_0_BOX_Y_OFFSET -4

CrossingModel::CrossingModel() : detector(16,Point(0,40),Point(200,250))
{
    cType = UNKNOWN;
    tcType = UNKNOWN_T;
    xVotes = tVotes = 0;
    middleLineLocked = false;
    middleLineX = 0;
    xStoppingLinePresent = false;
    angularOffset = 0.0;
    lastSignDist = 1234;
    lostSign = false;

    leftOpenVotes = 0;
    rightOpenVotes = 0;
}

void CrossingModel::signDataUpdate(double dx,double dy,double alpha,bool lost)
{
    if(lost)
    {
        lostSign = true;
        return;
    }

    lastSignDist = dy;

    if(dy > 230)
    {
        lostSign = true;
        return;
    }

        lostSign = false;
        Point2d crossingMid(dx-SIGN_X_OFFSET,dy-SIGN_Y_OFFSET);
        crossing_center = crossingMid;
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

void CrossingModel::cameraUpdate(Mat gray,Mat sobel,Mat ground)
{
    Point2d carOffset(100,0);

    //not sure about type yet => use road points to find middle line
    if(cType == UNKNOWN)
    {
        double offGroundPoints = 0;

        int tCrossingThreshold = 10;

        //cout << "CENTER DISTANCE: " << crossing_center.y << endl;
        if(crossing_center.y > 150 || crossing_center.y < 30)return;

        //construct crossing area
        vector<Point> crossingArea;
        double areaSize = CHECK_RECT_SIZE;
        Point p1(crossing_center+Point2d(-areaSize,-areaSize)+carOffset);
        Point p2(crossing_center+Point2d(-areaSize,+areaSize)+carOffset);
        Point p3(crossing_center+Point2d(+areaSize,+areaSize)+carOffset);
        Point p4(crossing_center+Point2d(+areaSize,-areaSize)+carOffset);
        crossingArea.push_back(p1);
        crossingArea.push_back(p2);
        crossingArea.push_back(p3);
        crossingArea.push_back(p4);

        vector<Point2d> points = detector.detect(gray,sobel,ground,crossingArea,&offGroundPoints);
        if(offGroundPoints > 0.2)
        {
            pointsForDrawing.clear();
            return;
        }

        pointsForDrawing = points;

        if((int)points.size() > tCrossingThreshold)
        {
            tVotes++;
            tCrossingPoints.push_back(points);
            if(tVotes > 10)
            {
                cType = T_CROSSING;
                getTCrossingType();
            }
        }
        else
        {
            xVotes++;
            if(xVotes > 10)
            {
                cType = X_CROSSING;
            }
        }
    }

    //track the 90 degree middle line of the orthogonal crossing
    if(cType == T_CROSSING && tcType == DEG_90)
    {
        vector<Point> contour;
        contour.push_back(Point2d(-50,distToCrossing-10)+carOffset);
        contour.push_back(Point2d(-50,distToCrossing+10)+carOffset);
        contour.push_back(Point2d(10,distToCrossing+10)+carOffset);
        contour.push_back(Point2d(10,distToCrossing-10)+carOffset);
        double offGround = 0;
        vector<Point2d> newPoints = detector.detect(gray,sobel,ground,contour,&offGround);


        if(newPoints.size() > 5)
        {
            pointsForDrawing.clear();
            pointsForDrawing = newPoints;
            updateAngularOffset(newPoints,false);
            double newDist = 0;
            for(int i = 0; i< (int)newPoints.size();i++)
            {
                newDist += newPoints.at(i).y;
            }
            distToCrossing = newDist / newPoints.size();
        }
    }

    if(cType == T_CROSSING && tcType == DEG_0)
    {
        if(lostSign)
        {
            crossing_center.y = distToCrossing-5;
        }
        if(!middleLineLocked)
        {
            //run the detector in the crossing area and take the second contour to lock it
            vector<Point> contour;
            contour.push_back(crossing_center+carOffset+Point2d(-T_0_BOX_WIDTH/2,T_0_BOX_Y_OFFSET+T_0_BOX_HEIGHT));
            contour.push_back(crossing_center+carOffset+Point2d(+T_0_BOX_WIDTH/2,T_0_BOX_Y_OFFSET+T_0_BOX_HEIGHT));
            contour.push_back(crossing_center+carOffset+Point2d(+T_0_BOX_WIDTH/2,T_0_BOX_Y_OFFSET));
            contour.push_back(crossing_center+carOffset+Point2d(-T_0_BOX_WIDTH/2,T_0_BOX_Y_OFFSET));
            double offGround = 0;
            vector<Point2d> newPoints = detector.detect(gray,sobel,ground,contour,&offGround);

            //only if dist to crossing > 100cm
            if(distToCrossing > 80)
            {
                vector<Point> checkLeftContour;
                /*checkLeftContour.push_back(crossing_center+carOffset+Point2d(-20,10));
                checkLeftContour.push_back(crossing_center+carOffset+Point2d(-65,10));
                checkLeftContour.push_back(crossing_center+carOffset+Point2d(-65,25));
                checkLeftContour.push_back(crossing_center+carOffset+Point2d(-20,25));*/

                checkLeftContour.push_back(crossing_center+carOffset+Point2d(20,-8));
                checkLeftContour.push_back(crossing_center+carOffset+Point2d(65,-8));
                checkLeftContour.push_back(crossing_center+carOffset+Point2d(65,-16));
                checkLeftContour.push_back(crossing_center+carOffset+Point2d(20,-16));

                double offGround2 = 0;
                vector<Point2d> checkLeftPoints = detector.detect(gray,sobel,ground,checkLeftContour,&offGround2);
                leftOpenPointsDebug = checkLeftPoints;

                if(checkLeftPoints.size() > 5)
                {
                    leftOpenVotes++;
                }
                else
                {
                    rightOpenVotes++;
                }
            }


            if(newPoints.size() > 5)
            {
                pointsForDrawing = newPoints;
                updateAngularOffset(newPoints,true);
                //find contours
                Mat circleImage(250,200,CV_8UC1,Scalar(0));
                for(int i = 0;i < (int)newPoints.size();i++)
                {
                    circle(circleImage,newPoints.at(i),2,Scalar(255),-1);
                }
                vector<vector<Point> > contours;
                vector<Vec4i> hierarchy;
                findContours(circleImage,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point(0,0));

                vector<vector<Point> > goodContours;
                int areaThreshold = 80;
                //erase small contours:
                for(int i = 0;i < (int)contours.size();i++)
                {
                    vector<Point> currCont = contours.at(i);
                    double area = contourArea(contours.at(i));

                Point2d minPoint(0,100000);
                Point2d maxPoint(0,-100000);
                for(int i = 0;i < (int)currCont.size();i++)
                {
                    if(currCont.at(i).y < minPoint.y)
                    {
                        minPoint = currCont.at(i);
                    }
                    if(currCont.at(i).y > maxPoint.y)
                    {
                        maxPoint = currCont.at(i);
                    }
                }
                double len = abs(maxPoint.y - minPoint.y);
                    
                 if(area > areaThreshold && len > 25)
                 {
                     goodContours.push_back(currCont);
                 }
                }
                contours = goodContours;

                if(contours.size() < 1)return;

                std::sort(contours.begin(),contours.end(),acompare);
                vector<Point> secondLine = contours.at(0);                   

                double avg_x = 0;
                Point2d minPoint(0,100000);
                for(int i = 0;i < (int)secondLine.size();i++)
                {
                    avg_x += secondLine.at(i).x;
                    if(secondLine.at(i).y < minPoint.y)
                    {
                        minPoint = secondLine.at(i);
                    }
                }

                avg_x /= secondLine.size();
                middleLineX = avg_x;
                crossing_center.x = avg_x-100;

                distToCrossing = minPoint.y;

            }
        }
        //middle line is already locked
        else
        {
            LOG_INFO("MIDDLE LINE LOCKED T CROSSING 0");
            //run the detector in the crossing area and take the second contour to lock it
            vector<Point> contour;
            contour.push_back(Point2d(middleLineX-10,distToCrossing-20));
            contour.push_back(Point2d(middleLineX-10,distToCrossing+20));
            contour.push_back(Point2d(middleLineX+10,distToCrossing+20));
            contour.push_back(Point2d(middleLineX-10,distToCrossing-20));
            double offGround = 0;
            vector<Point2d> newPoints = detector.detect(gray,sobel,ground,contour,&offGround);

            if(newPoints.size() > 5)
            {
                double avg_x = 0;
                pointsForDrawing = newPoints;
                updateAngularOffset(newPoints,true);
                //find min y of those points
                double min_y = 12345;
                for(int i = 0;i < (int)newPoints.size();i++)
                {
                    Point2d currP = newPoints.at(i);
                    avg_x += currP.x;

                    if(currP.y < min_y)
                        min_y = currP.y;


                }
                avg_x /= newPoints.size();


                distToCrossing = min_y;

                middleLineX = avg_x;
            }
        }


    }

    if(cType == X_CROSSING)
    {
        if(lostSign && xStoppingLinePresent)
        {
            crossing_center.y = distToCrossing-7-50;
        }
        if(lostSign && !xStoppingLinePresent)
        {
            crossing_center.y = distToCrossing-12-50;
        }
        if(!middleLineLocked)
        {
            //run the detector in the crossing area and take the second contour to lock it
            vector<Point> contour;
            contour.push_back(crossing_center+carOffset+Point2d(-15,40));
            contour.push_back(crossing_center+carOffset+Point2d(-15,100));
            contour.push_back(crossing_center+carOffset+Point2d(15,100));
            contour.push_back(crossing_center+carOffset+Point2d(15,40));
            double offGround = 0;
            vector<Point2d> newPoints = detector.detect(gray,sobel,ground,contour,&offGround);

            //check if there is a stopping line
            contour.clear();
            contour.push_back(crossing_center+carOffset+Point2d(-15,40));
            contour.push_back(crossing_center+carOffset+Point2d(-15,90));
            contour.push_back(crossing_center+carOffset+Point2d(-35,90));
            contour.push_back(crossing_center+carOffset+Point2d(-35,40));
            double offGroundStoppingLine;
            vector<Point2d> stoppingLinePoints = detector.detect(gray,sobel,ground,contour,&offGroundStoppingLine);
            xStoppingLinePoints = stoppingLinePoints;
            if(offGroundStoppingLine < 0.3 && stoppingLinePoints.size() > 5)
            {
                xStoppingLinePresent = true;
            }
            else if(offGroundStoppingLine < 0.3)
            {
                xStoppingLinePresent = false;
            }

            if(newPoints.size() > 5)
            {
                pointsForDrawing = newPoints;


                //stopping line present => filter out points that make angular estmation bad
                if(xStoppingLinePresent)
                {
                    vector<Point2d> angularEstimation;

                    //find stopping line y-range
                    double yBegin = 10000;
                    double yEnd   = -10000;
                    for(int i = 0;i < (int)xStoppingLinePoints.size();i++)
                    {
                        Point2d currP = xStoppingLinePoints.at(i);
                        if(currP.y < yBegin)    yBegin = currP.y;
                        if(currP.y > yEnd)      yEnd = currP.y;
                    }
                    //add some uncertainty range
                    yBegin -= 1;
                    yEnd += 3;

                    for(int i = 0;i < (int)newPoints.size();i++)
                    {
                        Point2d currP = newPoints.at(i);
                        if(currP.y > yEnd)angularEstimation.push_back(currP);
                    }

                    updateAngularOffset(angularEstimation,true);

                }
                else
                {
                    updateAngularOffset(newPoints,true);
                }
                //find contours
                Mat circleImage(250,200,CV_8UC1,Scalar(0));
                for(int i = 0;i < (int)newPoints.size();i++)
                {
                    circle(circleImage,newPoints.at(i),2,Scalar(255),-1);
                }
                vector<vector<Point> > contours;
                vector<Vec4i> hierarchy;
                findContours(circleImage,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point(0,0));

                vector<vector<Point> > goodContours;
                int areaThreshold = 5;
                //erase small contours:
                for(int i = 0;i < (int)contours.size();i++)
                {
                    vector<Point> currCont = contours.at(i);
                    double area = contourArea(contours.at(i));
                    if(area > areaThreshold)
                    {
                        goodContours.push_back(currCont);
                    }
                }
                contours = goodContours;

                if(contours.size() < 1)return;


                std::sort(contours.begin(),contours.end(),acompare);
                vector<Point> firstLine = contours.at(0);

                double avg_x = 0;
                Point2d minPoint(0,100000);
                for(int i = 0;i < (int)firstLine.size();i++)
                {
                    avg_x += firstLine.at(i).x;
                    if(firstLine.at(i).y < minPoint.y)
                    {
                        minPoint = firstLine.at(i);
                    }
                }

                if(firstLine.size() > 0)
                {
                    avg_x /= firstLine.size();
                    middleLineX = avg_x;
                    crossing_center.x = avg_x-100;

                    distToCrossing = minPoint.y;
                }

            }
        }
    }

}

//using the point history to estimate the type of the t crossing, 90 deg or 0 deg rotation
void CrossingModel::getTCrossingType()
{
    int deg_0_votes = 0;
    int deg_90_votes = 0;

    for(int i = 0;i < (int)tCrossingPoints.size();i++)
    {
        double xAvg = 0.0;
        double yAvg = 0.0;

        vector<Point2d> points = tCrossingPoints.at(i);
        for(int j = 0;j < (int)points.size();j++)
        {
            xAvg += points.at(j).x;
            yAvg += points.at(j).y;
        }

        xAvg /= points.size();
        yAvg /= points.size();

        double deviation_x = 0.0;
        double deviation_y = 0.0;

        for(int j = 0;j < (int)points.size();j++)
        {
            deviation_x += abs(points.at(j).x-xAvg);
            deviation_y += abs(points.at(j).y-yAvg);
        }

        if(2*deviation_x < deviation_y)
        {
            deg_0_votes++;
        }
        else if(2*deviation_y < deviation_x)
        {
            deg_90_votes++;
        }
    }

    if(deg_90_votes > deg_0_votes)
    {
        tcType = DEG_90;
        double dist = 0;
        vector<Point2d> lastPoints = tCrossingPoints.at(tCrossingPoints.size()-1);
        for(int i = 0;i < (int)lastPoints.size();i++)
        {
            dist += lastPoints.at(i).y;
        }
        dist /= lastPoints.size();
        distToCrossing = dist;
    }
    else
    {
        tcType = DEG_0;
        middleLineLocked = false;
    }

}

void CrossingModel::paintDebugInfo(Mat colorIPM)
{
    Point2d carCenter(100,0);

    if(cType == UNKNOWN)
    {
        //draw the crossing rectangle
        circle(colorIPM,crossing_center+carCenter,3,Scalar(0,255,0),-1);

        double areaSize = CHECK_RECT_SIZE;
        Point2d p1(crossing_center+Point2d(-areaSize,-areaSize));
        Point2d p2(crossing_center+Point2d(-areaSize,+areaSize));
        Point2d p3(crossing_center+Point2d(+areaSize,+areaSize));
        Point2d p4(crossing_center+Point2d(+areaSize,-areaSize));


        Scalar regionColor(0,0,255);

        line(colorIPM,p1+carCenter,p2+carCenter,regionColor,1);
        line(colorIPM,p2+carCenter,p3+carCenter,regionColor,1);
        line(colorIPM,p3+carCenter,p4+carCenter,regionColor,1);
        line(colorIPM,p4+carCenter,p1+carCenter,regionColor,1);

    }
    else if(cType == T_CROSSING)
    {
        //Print T on top of picture
        //putText(colorIPM,"T-Crossing",Point(10,45),FONT_HERSHEY_COMPLEX_SMALL,0.8,Scalar(0,255,0),1,CV_AA);

        if(tcType == DEG_90)
        {
            //PRINT 90
            //putText(colorIPM,"(90)",Point(150,45),FONT_HERSHEY_COMPLEX_SMALL,0.8,Scalar(0,255,0),1,CV_AA);

            Point2d check_1(-50,distToCrossing-10);
            Point2d check_2(-50,distToCrossing+10);
            Point2d check_3(10,distToCrossing+10);
            Point2d check_4(10,distToCrossing-10);

            line(colorIPM,check_1+carCenter,check_2+carCenter,Scalar(0,255,255),1);
            line(colorIPM,check_2+carCenter,check_3+carCenter,Scalar(0,255,255),1);
            line(colorIPM,check_3+carCenter,check_4+carCenter,Scalar(0,255,255),1);
            line(colorIPM,check_4+carCenter,check_1+carCenter,Scalar(0,255,255),1);

        }
        else if(tcType == DEG_0)
        {
            //PRINT 0
            //putText(colorIPM,"(0)",Point(150,45),FONT_HERSHEY_COMPLEX_SMALL,0.8,Scalar(0,255,0),1,CV_AA);

            //show left open points debug
            for(int i = 0;i < leftOpenPointsDebug.size();i++)
            {
                circle(colorIPM,leftOpenPointsDebug.at(i),1,Scalar(255,0,255),-1);
            }

            if(!middleLineLocked)
            {
                //Print point search area
                Point2d check_1(crossing_center+Point2d(-T_0_BOX_WIDTH/2,T_0_BOX_Y_OFFSET));
                Point2d check_2(crossing_center+Point2d(-T_0_BOX_WIDTH/2,T_0_BOX_Y_OFFSET+T_0_BOX_HEIGHT));
                Point2d check_3(crossing_center+Point2d(T_0_BOX_WIDTH/2,T_0_BOX_Y_OFFSET+T_0_BOX_HEIGHT));
                Point2d check_4(crossing_center+Point2d(T_0_BOX_WIDTH/2,T_0_BOX_Y_OFFSET));

                Point2d check_left_1(crossing_center+Point2d(+20,-8));
                Point2d check_left_2(crossing_center+Point2d(+65,-8));
                Point2d check_left_3(crossing_center+Point2d(65,-16));
                Point2d check_left_4(crossing_center+Point2d(20,-16));





                //searching area
                line(colorIPM,check_1+carCenter,check_2+carCenter,Scalar(0,255,255),1);
                line(colorIPM,check_2+carCenter,check_3+carCenter,Scalar(0,255,255),1);
                line(colorIPM,check_3+carCenter,check_4+carCenter,Scalar(0,255,255),1);
                line(colorIPM,check_4+carCenter,check_1+carCenter,Scalar(0,255,255),1);

                //check left area
                line(colorIPM,check_left_1+carCenter,check_left_2+carCenter,Scalar(0,255,255),1);
                line(colorIPM,check_left_2+carCenter,check_left_3+carCenter,Scalar(0,255,255),1);
                line(colorIPM,check_left_3+carCenter,check_left_4+carCenter,Scalar(0,255,255),1);
                line(colorIPM,check_left_4+carCenter,check_left_1+carCenter,Scalar(0,255,255),1);
            }
            else
            {
                //Print tracking area
                Point2d check_1(Point2d(middleLineX-10,distToCrossing-5));
                Point2d check_2(Point2d(middleLineX-10,distToCrossing+20));
                Point2d check_3(Point2d(middleLineX+10,distToCrossing+20));
                Point2d check_4(Point2d(middleLineX+10,distToCrossing-5));


                //searching area
                line(colorIPM,check_1,check_2,Scalar(0,255,255),1);
                line(colorIPM,check_2,check_3,Scalar(0,255,255),1);
                line(colorIPM,check_3,check_4,Scalar(0,255,255),1);
                line(colorIPM,check_4,check_1,Scalar(0,255,255),1);
            }


        }


    }
    else if(cType == X_CROSSING)
    {
        //Print T on top of picture
        //putText(colorIPM,"X-Crossing",Point(10,45),FONT_HERSHEY_COMPLEX_SMALL,0.8,Scalar(0,255,0),1,CV_AA);

        //Print tracking area
        Point2d check_1(Point2d(crossing_center+Point2d(-15,40)));
        Point2d check_2(Point2d(crossing_center+Point2d(-15,100)));
        Point2d check_3(Point2d(crossing_center+Point2d(15,100)));
        Point2d check_4(Point2d(crossing_center+Point2d(15,40)));

        //searching area
        line(colorIPM,check_1+carCenter,check_2+carCenter,Scalar(0,255,255),1);
        line(colorIPM,check_2+carCenter,check_3+carCenter,Scalar(0,255,255),1);
        line(colorIPM,check_3+carCenter,check_4+carCenter,Scalar(0,255,255),1);
        line(colorIPM,check_4+carCenter,check_1+carCenter,Scalar(0,255,255),1);

        //xCrossingWindow
        Point2d check_5(crossing_center+Point2d(-15,40));
        Point2d check_6(crossing_center+Point2d(-15,90));
        Point2d check_7(crossing_center+Point2d(-35,90));
        Point2d check_8(crossing_center+Point2d(-35,40));

        //searching area
        line(colorIPM,check_5+carCenter,check_6+carCenter,Scalar(0,255,255),1);
        line(colorIPM,check_6+carCenter,check_7+carCenter,Scalar(0,255,255),1);
        line(colorIPM,check_7+carCenter,check_8+carCenter,Scalar(0,255,255),1);
        line(colorIPM,check_8+carCenter,check_5+carCenter,Scalar(0,255,255),1);

        if(xStoppingLinePresent){
            line(colorIPM,Point2d(150,20),Point2d(90,20),Scalar(0,255,0),4);
        }
        else
        {
            line(colorIPM,Point2d(150,20),Point2d(90,20),Scalar(0,0,255),4);
        }

        for(int i = 0;i < (int)xStoppingLinePoints.size();i++)
        {
            circle(colorIPM,xStoppingLinePoints.at(i),1,Scalar(0,255,255),-1);
        }
    }


    //show dist to crossing
    std::ostringstream strs;
    strs << "Dist: " << distToCrossing;
    std::string str = strs.str();
    //putText(colorIPM,str,Point(10,25),FONT_HERSHEY_COMPLEX_SMALL,0.8,Scalar(0,255,0),1,CV_AA);


    //show angular offset:
    std::ostringstream strsAngle;
    double angleGrad = angularOffset*(180/3.1415926);
    strsAngle << "Angle: " << angleGrad;
    std::string angleString = strsAngle.str();
    //putText(colorIPM,angleString,Point(10,170),FONT_HERSHEY_COMPLEX_SMALL,0.8,Scalar(0,255,0),1,CV_AA);



    //show real dist to crossing
    std::ostringstream strs2;
    double dist = getDistToCrossing();
    strs2 << "Dist(mapped): " << dist;
    std::string myString = strs2.str();
    //putText(colorIPM,myString,Point(10,200),FONT_HERSHEY_COMPLEX_SMALL,0.8,Scalar(0,255,0),1,CV_AA);

    //draw dist to crossing line
    line(colorIPM,Point2d(-80,distToCrossing)+carCenter,Point2d(10,distToCrossing)+carCenter,Scalar(0,0,255),1);

    //show working detector
    for(int i = 0;i < (int)pointsForDrawing.size();i++)
    {
        circle(colorIPM,pointsForDrawing.at(i),1,Scalar(0,0,255),-1);
    }

    //show if  crossing sign visible
    if(!lostSign)
    {
        circle(colorIPM,Point(colorIPM.cols-20,20),5,Scalar(0,255,0));
    }
    else
    {
        circle(colorIPM,Point(colorIPM.cols-20,20),5,Scalar(0,0,255));
    }

    return;
}

#define OFFSET_T_90 60.25
#define OFFSET_T_0  70.25
#define OFFSET_X    120.5
#define WIDTH_STOPPINGLINE 5
#define CAMERA_OFFSET 10

double CrossingModel::getDistToCrossing()
{
    /*
    //double distToCrossingTemp = cos(angularOffset)*distToCrossing;
    double distToCrossingTemp = 1*distToCrossing;
    double mappedDistance = 1234;
    if(cType == T_CROSSING)
    {
       if(tcType == DEG_90)mappedDistance = (distToCrossingTemp-OFFSET_T_90-CAMERA_OFFSET);
       if(tcType == DEG_0)mappedDistance = (distToCrossingTemp-OFFSET_T_90-CAMERA_OFFSET);
    }
    else if(cType == X_CROSSING)
    {
        if(xStoppingLinePresent)mappedDistance = distToCrossingTemp-(OFFSET_X-WIDTH_STOPPINGLINE)-CAMERA_OFFSET;
        else mappedDistance = distToCrossingTemp-OFFSET_X-CAMERA_OFFSET;
    }
    */
    double result = 1234;

    if(cType == T_CROSSING)
    {
       if(tcType == DEG_90)result = distToCrossing;
       if(tcType == DEG_0)result = distToCrossing-10;
    }
    else if(cType == X_CROSSING)
    {
        if(xStoppingLinePresent)result = distToCrossing-45-7-5;
        else                    result = distToCrossing-45-7;
    }

    if(!lostSign)
    {
        if( abs((lastSignDist-SIGN_Y_OFFSET)-result) > 15)
        {
            return 1000;
        }
    }

    return result;
}

//TODO: FILTER OUT Points at stopping line

void CrossingModel::updateAngularOffset(vector<Point2d> points,bool straight)
{
    if(points.size() <= 5)return;

    //fit line through points
    Vec4f line;
    fitLine(points,line,DIST_L2,0,0.01,0.01);
    Point2d dir(line[0],line[1]);       //direction vector of the line
    Point2d p(line[2],line[3]);         //point on the line

    double angle = -atan(dir.x/dir.y);

    //handle T-90 angle differently
    if(!straight)
    {
        double res = 3.1415926/2-abs(angle);
        if(angle > 0)res*=-1;
        angle = res;
    }

    angularOffset = angle;

}

bool CrossingModel::leftIsOpen()
{
    return leftOpenVotes > rightOpenVotes;
}


