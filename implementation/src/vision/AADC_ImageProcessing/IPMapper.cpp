//
//  IPMapper.cpp
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

#include "IPMapper.h"

const double DEG_TO_RAD = M_PI/180;

void IPMapper::remap(Mat input, Mat* output, bool blur)
{
    if(input.cols == 320 && input.rows == 240) {
        for(int y = 0;y < outputHeight;y++) {
            for(int x = 0;x < outputWidth;x++) {
                Point toCopy = mappingMatrix(y,x);
                if(toCopy.x >= 0 && toCopy.y >= 0 && toCopy.y < 480 && toCopy.x < 640) {
                    output->at<uchar>(y,x) = input.at<uchar>(toCopy.y/2,toCopy.x/2);
                }
            }
        }
    } else {
        for(int y = 0;y < outputHeight;y++) {
            for(int x = 0;x < outputWidth;x++) {
                Point toCopy = mappingMatrix(y,x);
                if(toCopy.x >= 0 && toCopy.y >= 0 && toCopy.y < input.rows && toCopy.x < input.cols) {
                    output->at<uchar>(y,x) = input.at<uchar>(toCopy.y,toCopy.x);
                }
            }
        }
    }

    if(blur) GaussianBlur(*output, *output, Size(5,5),1);
}

IPMapper::IPMapper(int ow,int oh, float fu, float fv, float cx, float cy, double pitch)
{
    initialize(ow,oh,fu,fv,cx,cy,pitch);
}

void IPMapper::initialize(int ow,int oh, float fu, float fv, float cx, float cy, double pitch)
{
    outputWidth  = ow;
    outputHeight = oh;

    //CAMERA PARAMETERS:
    f_u = fu;             //focal lense values (mm)
    f_v = fv;

    c_u = cx;             //camera optical center
    c_v = cy;

    c_1 = cos(pitch*DEG_TO_RAD);  //cos(alpha : pitch angle),cos(beta : yaw angle)
    c_2 = 1.0;

    s_1 = sin(pitch*DEG_TO_RAD);  //sin(alpha : pitch angle),sin(beta : yaw angle)
    s_2 = 0.0;

    cam_h = 21.5;

    //init projection matrices
    T = (Mat_<double>(4,4) <<   -c_2/f_u, s_1*s_2/f_v, c_u*c_2/f_u-c_v*s_1*s_2/f_v-c_1*s_2, 0,
         s_2/f_u, s_1*c_1/f_v, -c_u*s_2/f_u-c_v*s_1*c_2/f_v-c_1*c_2, 0,
         0, c_1/f_v, -c_v*c_1/f_v+s_1, 0,
         0, -c_1/(f_v*cam_h), c_v*c_1/(cam_h*f_v)-s_1/cam_h, 0);
    T = cam_h * T;

    T_INV = (Mat_<double>(4,4) << f_u*c_2+c_u*c_1*s_2,c_u*c_1*c_2-s_2*f_u,-c_u*s_1,0,
             s_2*(c_v*c_1-f_v*s_1),c_2*(c_v*c_1-f_v*s_1),-f_v*c_1-c_v*s_1,0,
             c_1*s_2,c_1*c_2,-s_1,0,
             c_1*s_2,c_1*c_2,-s_1,0);

    mappingMatrix = Mat_<cv::Point>(outputHeight, outputWidth, cv::Point(0, 0));
    initMappingMatrix(&mappingMatrix);
}

void IPMapper::initMappingMatrix(Mat_<cv::Point>* pointMatrix)
{
    for(int y = 0;y < pointMatrix->rows;y++)
    {
        for(int x = 0;x < pointMatrix->cols;x++)
        {
            Point toTransform(x-pointMatrix->cols/2,y);
            
            vector<Point> toMap;
            toMap.push_back(toTransform);
            vector<Point2d> result;
            invProjectPoints(&toMap, &result);
            
            if(result.at(0).x >= 0 && result.at(0).x < 640 && result.at(0).y >= 0 && result.at(0).y < 480)
            {
                pointMatrix->at<Point>(y,x) = (Point)result.at(0);
            }
            
        }
    }
}

void IPMapper::projectPoints(vector<Point>* points,vector<Point2d>* result)
{
    //for each lane marking
    for(size_t i = 0;i < points->size();i++)
    {
        Point currPoint = points->at(i);
        Mat P_I = (Mat_<double>(4,1) << currPoint.x,currPoint.y,1,1);
        Mat P_G = T*P_I;
        P_G /= P_G.at<double>(3);
        
        Point2d transformedPoint(P_G.at<double>(0),P_G.at<double>(1));
        result->push_back(transformedPoint);
    }
}

void IPMapper::invProjectPoints(vector<Point>* points,vector<Point2d>* result)
{
    for(size_t i = 0;i < points->size();i++)
    {
        Point currPoint = points->at(i);
        Mat P_G = (Mat_<double>(4,1) << currPoint.x,currPoint.y,-cam_h,1);
        Mat P_I = T_INV * P_G;
        P_I = P_I / P_I.at<double>(3);
        Point2d invTransformedPoint(P_I.at<double>(0),P_I.at<double>(1));
        result->push_back(invTransformedPoint);
        
    }
}