//
//  ParkingSpaceModel.cpp
//  ParkingSpaceDetector
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

#include "ParkingSpaceModel.h"

#define INVALID_X_VALUE 12345.0


ParkingSpaceModel::ParkingSpaceModel(ParkingSpaceType type)
{
    parkingType = type;
    markersSeen = 0;
    numLost = 0;
    trackedPoints.clear();
    motionDAngle =0;
    motionDY=0;
    trackedLastPoint = Point2f(INVALID_X_VALUE,0);
}

bool acompare(Point2d lEntry,Point2d rEntry)
{
    return lEntry.y > rEntry.y;
}

void ParkingSpaceModel::updateModel(vector<Point2d> detectedPoints)
{    
   //update the certainties of the currently visible points, push the points that are not tracked
   //yet to the current history
    int requiredReliability = 3;
    int distanceThreshold = 10;

    //do point reasoning
    for(int i = 0;i < trackedPoints.size();i++)
    {
        for(int j = i+1;j < trackedPoints.size();j++)
        {
            Point2d dir = trackedPoints.at(i).point-trackedPoints.at(j).point;
            Point2d normDir = Point2d(-dir.y,dir.x);


            double scalarProduct = normDir.x;
            double len = sqrt(normDir.x*normDir.x+normDir.y*normDir.y);

            double cosValue = scalarProduct/len;
            double angle = acos(cosValue)*(180/3.1415926);

            if(angle > 90)angle = 180-angle;
            int angleThresh = 15;

            if(abs(angle) > angleThresh || len < 30 || len > 150)
            {
                if(trackedPoints.at(i).point.x < trackedPoints.at(j).point.x)
                {
                    trackedPoints.at(j).certainty -= 3;
                }
                else
                {
                    trackedPoints.at(i).certainty -= 3;
                }
            }
        }
    }

    for(int i = 0;i < detectedPoints.size();i++)
    {
        Point2d currP = detectedPoints.at(i);
        bool isInside = false;
        //is there a point already in the history?
        for(int j = 0;j < trackedPoints.size();j++)
        {
            Point2d trackedPoint = trackedPoints.at(j).point;
            Point2d diff = trackedPoint-currP;
            double distance = sqrt(diff.x*diff.x+diff.y*diff.y);

            //this point is alraedy in the history
            if(distance < distanceThreshold)
            {
                trackedPoints.at(j).point = currP;
                trackedPoints.at(j).certainty++;
                trackedPoints.at(j).lastFrameUpdated = true;
                trackedPoints.at(j).timeNotSeen = 0;

                if(trackedPoints.at(j).certainty > requiredReliability)
                {
                    trackedPoints.at(j).reachedTopCertainty = true;
                    markersSeen++;
                }

                isInside = true;
            }
        }

        if(!isInside)
        {
            //this point is not in the history yet:
            parkingPoint newPoint;
            newPoint.point = currP;
            newPoint.certainty = 3;
            newPoint.lastFrameUpdated = true;
            newPoint.reachedTopCertainty = false;
            newPoint.timeNotSeen = 0;
            trackedPoints.push_back(newPoint);
        }
    }

    for(int i = 0;i < trackedPoints.size();i++)
    {
        if(!trackedPoints.at(i).lastFrameUpdated)
        {
            trackedPoints.at(i).certainty--;
            trackedPoints.at(i).timeNotSeen++;
        }
        else
        {
            trackedPoints.at(i).lastFrameUpdated = false;
        }
    }

    for(int i = 0;i < trackedPoints.size();i++)
    {
        if(trackedPoints.at(i).certainty < 0)
        {
            trackedPoints.erase(trackedPoints.begin()+i);
            i--;
        }
    }

    for(int i = 0;i < trackedPoints.size();i++)
    {
        if(trackedPoints.at(i).timeNotSeen > 2)
        {
            if(/*(parkingType == CROSS || parkingType == PARALLEL) &&*/ trackedPoints.at(i).reachedTopCertainty && trackedPoints.at(i).point.y < 100)
            {
                numLost++;

                if(parkingType == CROSS && numLost == 5)
                {
                    trackedLastPoint = Point2d(trackedPoints.at(i).point.x,trackedPoints.at(i).point.y);
                    numLost = 4;
                }
                if(parkingType == PARALLEL && numLost == 6)
                {
                    trackedLastPoint = Point2d(trackedPoints.at(i).point.x,trackedPoints.at(i).point.y);
                    numLost = 5;
                }
                if((int)trackedPoints.size() == 1)
                {
                    if(parkingType == CROSS)
                        numLost = 4;
                    else
                        numLost = 5;
                    trackedLastPoint =  Point2d(trackedPoints.at(i).point.x,trackedPoints.at(i).point.y);
                }
            }
//            else if((parkingType == RESET_CROSS || parkingType == RESET_PARALLEL) && trackedPoints.at(i).reachedTopCertainty && trackedPoints.at(i).point.y > 175)
//            {
//                numLost ++;
//                if(parkingType == RESET_CROSS && /*TOTALNUMCOUNT check*/)
//                {

//                }
//            }

            trackedPoints.erase(trackedPoints.begin()+i);
        }
    }
}

void ParkingSpaceModel::setMotion(double dy, double dAngle)
{
  /*  pthread_mutex_lock(motion_mux);
    motionDY += dy;
    motionDAngle += dAngle;
    pthread_mutex_unlock(motion_mux);*/

    if(trackedLastPoint.x != INVALID_X_VALUE)
    {
        trackedLastPoint.y -= dy;
        //cout << "dy" << dy << "     tlp" << trackedLastPoint.y << endl;
    }
}

Mat ParkingSpaceModel::getDebugImage()
{
    Point2d carOrigin(100,100);
    Mat outputImage(300,200,CV_8UC3,Scalar(0,0,0));
    
    for(int i = 0;i < trackedPoints.size();i++)
    {
        int colorValue = 150+trackedPoints.at(i).certainty*10;
        if(colorValue > 255)colorValue = 255;
        if(!trackedPoints.at(i).reachedTopCertainty)
        {
            circle(outputImage, trackedPoints.at(i).point, 5, Scalar(0,0,colorValue),-1);
        }
        else
        {
            if(trackedPoints.at(i).point.y < 70)
                 circle(outputImage, trackedPoints.at(i).point, 5, Scalar(0,255,0),-1);
            else
                circle(outputImage, trackedPoints.at(i).point, 5, Scalar(255,255,255),-1);
        }
    }

    //numLost
    vector<Point2d> goodPoints;
    for(int i = 0;i < trackedPoints.size();i++)
    {
        if(trackedPoints.at(i).reachedTopCertainty)
        {
            goodPoints.push_back(trackedPoints.at(i).point);
        }
    }

    if(goodPoints.size() > 1)
    {
        //sort the points by y-value
        std::sort(goodPoints.begin(),goodPoints.end(),acompare);

        Point2d p1 = goodPoints.at(0);
        Point2d p2 = goodPoints.at(1);

        Point2d dir = p2-p1;

        for(int i = 0;i < numLost;i++)
        {
            circle(outputImage, p1-(i+1)*dir, 5, Scalar(0,255,255),-1);
        }

        if(parkingType == PARALLEL)
        {
            int numPointsNeeded = 6;
            numPointsNeeded -= numLost;
            numPointsNeeded -= goodPoints.size();

            for(int i = 0;i < numPointsNeeded;i++)
            {
                circle(outputImage, goodPoints.at(goodPoints.size()-1)+(i+1)*dir, 5, Scalar(0,255,255),-1);
            }

        }

    }

    for(int i = 0;i < m_debugPoints.size();i++)
    {
        circle(outputImage, m_debugPoints.at(i) - Point2d(20,0), 5, Scalar(0,0,255),-1);
    }
    m_debugPoints.clear();
    return outputImage;
}

double ParkingSpaceModel::getAngle(){

    vector<Point2d> goodPoints;
    for(int i = 0;i < trackedPoints.size();i++)
    {
        if(trackedPoints.at(i).reachedTopCertainty)
        {
            goodPoints.push_back(trackedPoints.at(i).point);
        }
    }
    std::sort(goodPoints.begin(),goodPoints.end(),acompare);

    if(goodPoints.size() > 1)
    {
        Point2d p1 = goodPoints.at(0);
        Point2d p2 = goodPoints.at(1);
        Point2d dir = p2-p1;
        return atan2(dir.y,dir.x);
    }
    else
    {
        return 0.0;
    }
}

double ParkingSpaceModel::getDistanceNextParkingLot(int& lot_index){

    Point2d rearAxis(0,-39);
    Point2d rearAxisRef(0,-39);
    lot_index = -1;

    if(trackedLastPoint.x != INVALID_X_VALUE)
    {

//        cout << "TRACKED LAST POINT SET" << endl;
//        if(parkingType == CROSS){lot_index = 5;}
//        else if(parkingType == PARALLEL){lot_index = 6;}
//        return trackedLastPoint.y-rearAxis.y;
    }

    vector<Point2d> parkingPoints;
    if(trackedLastPoint.x != INVALID_X_VALUE && trackedPoints.size() == 0)
    {
        parkingPoints.push_back(trackedLastPoint);
    }
    for(int i = 0;i < trackedPoints.size();i++)
    {
        if(trackedPoints.at(i).reachedTopCertainty)
        {
            parkingPoints.push_back(trackedPoints.at(i).point);
        }
    }
    std::sort(parkingPoints.begin(),parkingPoints.end(),acompare); //sort desc

    if(parkingPoints.size() < 1)return INVALID_X_VALUE;

    //vector<Point2d> parkingPoints;
//    for(int i = 0;i < parkingPoints.size();i++)
//    {
//        parkingPoints.push_back(goodPoints.at(i));
//    }

    Point2d direction(0,-1);

    if(parkingPoints.size() > 1)
    {
        Point2d p1 = parkingPoints.at(0);
        Point2d p2 = parkingPoints.at(1);
        //direction = p2-p1;
        double directionLen = sqrt(direction.x*direction.x+direction.y*direction.y);
        //cout << p1.y - p2.y << endl;
        //direction *= (1.0/directionLen);
    }

    if(parkingType == CROSS)
    {
        //cout << "cross" << endl;
        direction *= DISTANCE_CROSS_PARKING;
        rearAxisRef.y = -49;

    }
    else if(parkingType == PARALLEL)
    {
        //cout << "parallel" << endl;
        direction *= DISTANCE_PARALLEL_PARKING;
        rearAxisRef.y = -69;
    }

    //cout << "ptype" << parkingType << "     direction" << direction << endl;

    //push already seen points
    for(int i = 0;i < numLost;i++)
    {
        //Point2d newPoint = parkingPoints.at((int)parkingPoints.size()-1) - (i+1)*direction;
        //newPoint.y -= 2; //line offset

        //push this point to the FRONT of the parkingPoint vector
        //vector<Point2d> temp;
//        if(i == numLost - 3)
//        {
//            parkingPoints.push_back(parkingPoints.at((int)parkingPoints.size()-1) + direction + Point2d(0,-3));
//            //cout << "i = " << i << "numlost = " << numLost << "pp last y = " << parkingPoints.at(parkingPoints.size()-1) << endl;
//        }
//        else if (i == numLost -2)
//        {
//            parkingPoints.push_back(parkingPoints.at(int)parkingPoints.size()-1) + direction + Point2d(0,);
//        }else
            parkingPoints.push_back(parkingPoints.at((int)parkingPoints.size()-1) + direction);
        //temp.insert(temp.end(),parkingPoints.begin(),parkingPoints.end());

        //parkingPoints = temp;
    }

    for (int i = 0; i < (int)parkingPoints.size(); ++i) {
        m_debugPoints.push_back(parkingPoints.at(i));
    }


//    int desiredPoints = 5;
//    //push not seen yet
//    if(parkingType == CROSS)
//    {
//        desiredPoints = 5;
//    }
//    if(parkingType == PARALLEL)
//    {
//        desiredPoints = 6;
//    }

//    desiredPoints -= parkingPoints.size();
//    for(int i = 0;i < desiredPoints;i++)
//    {
//        Point2d currP = parkingPoints.at(parkingPoints.size()-1);
//        currP += direction;
//        parkingPoints.push_back(currP);
//    }



    double currMinDistance = 123456567;
    Point2d pointClosestToRearAxis(1234,1234);
    double returnDist = INVALID_X_VALUE;
    int pointIndex = 0;
    for(int i = 0;i < parkingPoints.size();i++)
    {
        Point2d currP = parkingPoints.at(i);
        //cout << "CURRP: " << currP << endl;

        double distance = abs(currP.y-rearAxisRef.y);
        if(distance < currMinDistance)
        {
            pointIndex = parkingPoints.size() - i;
            returnDist = currP.y-rearAxis.y;
            if(pointIndex == 1)
                returnDist -= 2;
            else if(pointIndex == 2)
                returnDist -= 4;
            //pointClosestToRearAxis = currP;
            currMinDistance = distance;
        }
    }

    lot_index = pointIndex;

    return returnDist;
}



