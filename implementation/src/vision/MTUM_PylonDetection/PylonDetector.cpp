//
//  PylonDetector.cpp
//  PylonDetection
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
#include "PylonDetector.h"

bool rectanglesIntersect(Rect r1,Rect r2)
{
    Point2d tl = r1.tl();
    Point2d br = r1.br();
    
    //for each of these 4 points check if inside rectangle:
    Point2d tl2 = r2.tl();
    Point2d tr2 = tl2 + Point2d(r2.width,0);
    Point2d br2 = r2.br();
    Point2d bl2 = br2 - Point2d(r2.width,0);
    
    double xFrom = tl.x;
    double xTo   = tl.x+r1.width;
    
    double yFrom = tl.y;
    double yTo   = tl.y + r1.height;
    
    //check 4 points wether they are inside the rectangle
    if(tl2.x >= xFrom && tl2.x < xTo && tl2.y >= yFrom && tl2.y < yTo)
    {
        return true;
    }
    if(tr2.x >= xFrom && tr2.x < xTo && tr2.y >= yFrom && tr2.y < yTo)
    {
        return true;
    }
    if(br2.x >= xFrom && br2.x < xTo && br2.y >= yFrom && br2.y < yTo)
    {
        return true;
    }
    if(bl2.x >= xFrom && bl2.x < xTo && bl2.y >= yFrom && bl2.y < yTo)
    {
        return true;
    }
    
    return false;
}

bool pointDownwards(Point2d p1,Point2d p2)
{
    Point2d topPoint;
    Point2d botPoint;
    
    if(p1.y < p2.y)
    {
        topPoint = p1;
        botPoint = p2;
    }
    else
    {
        topPoint = p2;
        botPoint = p1;
    }
    
    
    Point2d dirTemp = botPoint-topPoint;
    Point2d dir(-dirTemp.y,dirTemp.x);
    Point2d floorDir(1,0);
    
    double scalarProd = dir.x;
    double absDir = sqrt(dir.x*dir.x+dir.y*dir.y);
    
    double cosValue = scalarProd/absDir;
    double angle    = acos(cosValue)*(180/3.1415926);
    
    if(angle > 90) angle = 180-angle;
    int angleThresh = 20;
        
    if(abs(angle) > angleThresh)return false;
    else                        return true;
}

bool isInLine(Point2d p1,Point2d p2,Point2d p3)
{
    Point2d topPoint;
    Point2d midPoint;
    Point2d botPoint;

    //order points from top to bottom
    if(p1.y < p2.y && p1.y < p3.y)
    {
        topPoint = p1;
        if(p2.y < p3.y) {midPoint = p2;botPoint = p3;}
        else            {midPoint = p3;botPoint = p2;}
    }

    if(p2.y < p1.y && p2.y < p3.y)
    {
        topPoint = p2;
        if(p1.y < p3.y) {midPoint = p1;botPoint = p3;}
        else            {midPoint = p3;botPoint = p1;}
    }

    if(p3.y < p1.y && p3.y < p2.y)
    {
        topPoint = p3;
        if(p1.y < p2.y) {midPoint = p1;botPoint = p2;}
        else            {midPoint = p2;botPoint = p1;}
    }

    Point2d dir1 = midPoint - topPoint;
    Point2d dir2 = botPoint - midPoint;

    double scalarProd = dir1.x*dir2.x + dir1.y*dir2.y;
    double absDir1 = sqrt(dir1.x*dir1.x+dir1.y*dir1.y);
    double absDir2 = sqrt(dir2.x*dir2.x+dir2.y*dir2.y);

    if(absDir1/absDir2 < 0.8 || absDir1/absDir2 > 1.2)
        return false;

    double cosValue = scalarProd/(absDir1*absDir2);
    double angle    = acos(cosValue)*(180/3.1415926);

    if(angle > 90) angle = 180-angle;

    int angleThresh = 10;
    return abs(angle) < angleThresh;
}


struct pylon{
    vector<Point> cont1;
    vector<Point> cont2;
    vector<Point> cont3;
    
    Point2d c1;
    Point2d c2;
    Point2d c3;
    
    Rect boundingRect;
};



/*
 *  Receive an rbg image from the camera, afterwards apply the following:
 *  => Convert from BGR to HSV color space
 *  => Perform color segmentation (orange and white together and separately -> 3 images)
 *  => check for large regions in combined image 
 *  => check for color differences in this large region by the other two images => striping shape
 *  => get the (rotated) boundary rectangles of the found regions and return them
 */
vector<Rect> PylonDetector::detect(Mat image,Scalar color_from,Scalar color_to)
{

    //resulting rectangles found
    vector<Rect> result;
    
    //convert to hsv color space
    Mat hsvImage;
    cvtColor(image, hsvImage, COLOR_BGR2HSV);

    //blur hsv image
    //GaussianBlur(image, image,Size(9,9), 1);


    //filter hsv data
   // Scalar orange_from(0,150,100);
    // Scalar orange_to(18,255,255);

   // Scalar orange_from(0,150,100);
   // Scalar orange_to(25,255,255);

    //Values from 16_03_2015
//    Scalar orange_from(0,131,217);
//    Scalar orange_to(255,255,255);

    //Values from 17_03_2015
//    Scalar orange_from(0,146,180);
//    Scalar orange_to(180,255,255);

    //Values from 19_03_2015
    //Scalar orange_from(0,118,80);
    //Scalar orange_to(255,255,255);


    //do color segmentation to orange image
    Mat orangeImage;
    inRange(hsvImage, color_from, color_to, orangeImage);

    thresholdImageDebug = orangeImage.clone();

    //find contours in this image
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(orangeImage, contours, hierarchy,RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
    
    //find large contours
    int areaThreshold = 15;
    vector<vector<Point> > largeContours;
    
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
    
    //last step: reason about the large contours:
    //three contours needed: + in a row

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
    
    vector<pylon> found_pylons;
    for(int i = 0;i < mass_centers.size();i++)
    {
        Point2d c1 = mass_centers.at(i);
        for(int j = i+1;j < mass_centers.size();j++)
        {
            Point2d c2 = mass_centers.at(j);
           // for(int k = j+1;k < mass_centers.size();k++)
           // {
            //    Point2d c3 = mass_centers.at(k);
                
                //check for c1,c2,c3 wether they are in one line!
                //if(isInLine(c1,c2,c3) && pointDownwards(c1, c2) && pointDownwards(c2, c3))
                if(pointDownwards(c1, c2))
                {
                    //check if the distances are roughly the same
                    pylon newPylon;
                    newPylon.cont1 = largeContours.at(i);
                    newPylon.cont2 = largeContours.at(j);
                    
                    newPylon.c1 = c1;
                    newPylon.c2 = c2;

                    found_pylons.push_back(newPylon);
                }
         //  }
        }
    }
    
    /*
    for(int i = 0;i < found_pylons.size();i++)
    {
        vector<Point> c1 = found_pylons.at(i).cont1;
        vector<Point> c2 = found_pylons.at(i).cont2;
        vector<Point> c3 = found_pylons.at(i).cont3;
        vector<vector<Point> > contours;
        contours.push_back(c1);
        contours.push_back(c2);
        contours.push_back(c3);
        
        Scalar color(0,0,255);
        drawContours( orangeImage, contours, 0, color, 2, 8);
        drawContours( orangeImage, contours, 1, color, 2, 8);
        drawContours( orangeImage, contours, 2, color, 2, 8);
        
        circle(orangeImage, found_pylons.at(i).c1, 5, Scalar(0,0,255),-1);
        circle(orangeImage, found_pylons.at(i).c2, 5, Scalar(0,0,255),-1);
        circle(orangeImage, found_pylons.at(i).c3, 5, Scalar(0,0,255),-1);
    }*/
    
    //rotated rectangles for each pylon
    vector<Rect> minRect;
    
    for(int i = 0;i < found_pylons.size();i++)
    {
        pylon currPylon = found_pylons.at(i);
        
        vector<Point> cont1 = currPylon.cont1;
        vector<Point> cont2 = currPylon.cont2;
       // vector<Point> cont3 = currPylon.cont3;
        
        vector<Point> overallCont;
        overallCont.insert(overallCont.end(), cont1.begin(),cont1.end());
        overallCont.insert(overallCont.end(), cont2.begin(),cont2.end());
        //overallCont.insert(overallCont.end(), cont3.begin(),cont3.end());
        
        Rect rect = boundingRect(Mat(overallCont));
        found_pylons.at(i).boundingRect = rect;
        result.push_back(rect);
    }
    
    /*
    //check wether some rectangles intersect
    for(int i = 0;i < found_pylons.size();i++)
    {
        for(int j = i+1;j < found_pylons.size();j++)
        {
            bool doIntersect = rectanglesIntersect(found_pylons.at(i).boundingRect,found_pylons.at(j).boundingRect);
            
            if(doIntersect)
            {
                double iSize = found_pylons.at(i).boundingRect.width*found_pylons.at(i).boundingRect.height;
                double jSize = found_pylons.at(j).boundingRect.width*found_pylons.at(j).boundingRect.height;
                
                //erase the smaller of the two pylons => maybe change with depth grid information
                if(iSize > jSize)found_pylons.erase(found_pylons.begin()+j);
                else             found_pylons.erase(found_pylons.begin()+i);
                
                //reset the loop
                i = 0;
                j = 1;
            }
        }
    }*/
    
  /*  for(int i = 0;i < found_pylons.size();i++)
    {
        Rect currRect = found_pylons.at(i).boundingRect;
        rectangle(image, currRect.tl(), currRect.br(), Scalar(255,0,0), 2, 8, 0 );
    }*/
        
    return result;
}
