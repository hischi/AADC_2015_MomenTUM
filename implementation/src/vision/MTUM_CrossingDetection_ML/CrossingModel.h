//
//  CrossingModel.h
//  CrossingDetection
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

#ifndef __CrossingDetection__CrossingModel__
#define __CrossingDetection__CrossingModel__

#include "LaneDetector.h"
#include <stdio.h>
#include <stdafx.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>
#include <pthread.h>

using namespace std;
using namespace cv;

#define SIGN_X_OFFSET -65
#define SIGN_Y_OFFSET 85

#define CROSSING_WIDTH    100
#define CHECK_RECT_SIZE   CROSSING_WIDTH/3.7


enum TCrossingType{
    UNKNOWN_T,
    DEG_90,
    DEG_0
};

class CrossingModel{
public:

    enum CrossingType{
        UNKNOWN,
        X_CROSSING,
        T_CROSSING
    };



    CrossingModel();

    void paintDebugInfo(Mat colorIPM);
    void signDataUpdate(double dx,double dy,double alpha,bool lost);
    void cameraUpdate(Mat gray,Mat sobel,Mat ground);
    double getDistToCrossing();
    double angularOffset;


    
//private:

    Point2d crossing_center;
    double crossing_alpha;

    CrossingType cType;
    int xVotes;
    int tVotes;

    TCrossingType tcType;
    bool middleLineLocked;
    double middleLineX;
    bool lostSign;

    bool xStoppingLinePresent;

    LaneDetector  detector;

    vector<Point2d> pointsForDrawing;
    vector<Point2d> xStoppingLinePoints;
    vector<Point2d> leftOpenPointsDebug;

    vector<vector<Point2d> > tCrossingPoints;
    void getTCrossingType();

    double distToCrossing;

    void updateAngularOffset(vector<Point2d> points,bool straight);


    double lastSignDist;

    int leftOpenVotes;
    int rightOpenVotes;
    bool leftIsOpen();





    RNG rand;
};

#endif /* defined(__LaneDetection__LaneModel__) */

