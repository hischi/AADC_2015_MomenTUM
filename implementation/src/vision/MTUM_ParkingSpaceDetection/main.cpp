//
//  main.cpp
//  ParkingSpaceDetector
//
//  Created by Paul on 23.02.15.
//  Copyright (c) 2015 Paul Bergmann. All rights reserved.
//

#include <iostream>
#include "IPMapper.h"
#include "ParkingSpaceDetector.h"
#include "ParkingSpaceModel.h"

//opencv includes
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>

using namespace std;
using namespace cv;

struct ParkingLot
{
    char type;
    Point2d begin;
    Point2d end;
};

//make it possible to sort a vector of points based on y values
bool acompare(Point2d lEntry, Point2d rEntry)
{return lEntry.y < rEntry.y;}

int main(int argc, const char * argv[]) {

    int imageHeigth = 300;
    
    Mat colorInput;
    Mat grayscaleInput;
    
    IPMapper mapper(200,imageHeigth);
    ParkingSpaceDetector detector(24,Point(1,40),Point(200,imageHeigth));
    VideoCapture capture(0);
    
    int imgNum = 1;
    
    ParkingSpaceModel pModel(ParkingSpaceModel::PARALLEL);
    
    while(true)
    {
        capture >> colorInput;
        
        
        resize(colorInput, colorInput, Size(640,480));
        
        //colorInput = imread("img_parking/"+std::to_string(imgNum)+".png");
        imgNum++;
        
        cvtColor(colorInput, grayscaleInput, COLOR_BGR2GRAY);
        
        imshow("GRAY",grayscaleInput);
        
        //remap the image and create a paintable canvas for visual output
        Mat remapped = mapper.remap(grayscaleInput);
        Mat remappedPaintable = remapped.clone();
        cvtColor(remappedPaintable, remappedPaintable, COLOR_GRAY2BGR);
        
        //detect lane markers on the blurred image
        GaussianBlur(remapped, remapped, Size(5,5),1);
        vector<Point2d> points = detector.detect(remapped);
        
        //draw the points to the remapped image and circle image
        for(int i = 0;i < points.size();i++)
        {
            circle(remappedPaintable, points.at(i), 1, Scalar(0,0,255),-1);
        }
        
        imshow("REM", remappedPaintable);
        
        
        
        Mat circleImage(imageHeigth,200, CV_8UC1, Scalar(0));
        for(int i = 0;i < points.size();i++)
        {
            circle(circleImage, points.at(i),1, Scalar(255),-1);
        }
        
        imshow("circ",circleImage);
        
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        
        /// Find contours
        findContours( circleImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
                
        //drawContours(circleImage, contours[i], <#int contourIdx#>, <#const Scalar &color#>)
        
        cvtColor(circleImage, circleImage, COLOR_GRAY2BGR);
        
        vector<vector<Point> > largeContours;
        
        int areaThreshold = 30;
        for( int i = 0; i< contours.size(); i++ )
        {
            //check if contour is large enough
            vector<Point> currCont = contours.at(i);
            double area = contourArea(contours.at(i));
            if(area < areaThreshold)
            {
                continue;
            }
            largeContours.push_back(currCont);
        }
        
        for(int i = 0;i < largeContours.size();i++)
        {
            Scalar color(0,0,255);
            drawContours(circleImage, largeContours, i, color, 1, 8, hierarchy, 0, Point());
        }
        
        vector<Point2d> mass_centers;
        for(int i = 0;i < largeContours.size();i++)
        {
            //calculate the mass center for each contour
            vector<Moments> contourMoments;
            Moments currMoments = moments(largeContours.at(i));
            double x_cent = currMoments.m10 / currMoments.m00;
            double y_cent = currMoments.m01 / currMoments.m00;
            Point2d mass_cent(x_cent,y_cent);
            mass_centers.push_back(mass_cent);
        }

        //sort mass vector by y values (small to large)
        std::sort(mass_centers.begin(),mass_centers.end(),acompare);

        
        vector<ParkingLot> detectedLots;
        for(int i = 0;i < mass_centers.size();i++)
        {
            if(i == mass_centers.size()-1)break;
            
            Point2d currCenter_i = mass_centers.at(i);
            Point2d currCenter_j = mass_centers.at(i+1);
   
            //calculate distance
            Point2d dir = currCenter_j-currCenter_i;
            double distance = sqrt(dir.x*dir.x + dir.y*dir.y);

            if(distance > 40 && distance < 80)
            {
              //  cout << "CROSS PARKING DETECTED" << endl;
                ParkingLot p;
                p.type = 'c';
                p.begin = currCenter_i;
                p.end = currCenter_j;
                detectedLots.push_back(p);
            }
            else if(distance >= 60 && distance < 150)
            {
               // cout << "PARALLEL PARKING DETECTED" << endl;
                ParkingLot p;
                p.type = 'p';
                p.begin = currCenter_i;
                p.begin = currCenter_j;
                detectedLots.push_back(p);
            }
        }
        
        cout << detectedLots.size() << endl;
        
        for(int i = 0; i < mass_centers.size();i++)
        {
            if(i+1 < mass_centers.size())
            {
                line(remappedPaintable, mass_centers.at(i), mass_centers.at(i+1), Scalar(255,0,0),2);
                Point2d diff = mass_centers.at(i+1)-mass_centers.at(i);
                Point2d norm(diff.y,-diff.x);
                double len = sqrt(norm.x*norm.x+norm.y*norm.y);
                norm *= (120/len);
                
                
                
                line(remappedPaintable, mass_centers.at(i), mass_centers.at(i)+norm, Scalar(255,0,0),2);
                
                line(remappedPaintable, mass_centers.at(i+1), mass_centers.at(i+1)+norm, Scalar(255,0,0),2);
                
            }
            circle(remappedPaintable, mass_centers.at(i), 2, Scalar(0,255,0),-1);
        }
        imshow("circ2",circleImage);
        imshow("detection",remappedPaintable);

        pModel.updateModel(mass_centers);
        Mat debugImage = pModel.getDebugImage();
        imshow("DEBUG", debugImage);
        
        waitKey(1);
    }

    
    
    
    return 0;
}
