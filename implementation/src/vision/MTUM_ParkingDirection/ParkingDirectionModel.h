//
//  ParkingDirectionModel.h
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

#ifndef __ParkingDirection__ParkingDirectionModel__
#define __ParkingDirection__ParkingDirectionModel__

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

class ParkingDirectionModel{
public:

    ParkingDirectionModel();

    void paintDebugInfo(Mat colorIPM);
    void cameraUpdate(Mat gray,Mat sobel,Mat ground);
    void reset();


//private:

    double certaintyParallel;
    double certaintyCross;
    LaneDetector  detector;
    vector<Point2d> pointsForDrawing;
    vector<vector<vector<Point> > > mergedContoursForDrawing;

    bool mergeContours(Point2d centre1,Point2d dir1,Point2d centre2,Point2d dir2);

};

#endif /* defined(__LaneDetection__LaneModel__) */

