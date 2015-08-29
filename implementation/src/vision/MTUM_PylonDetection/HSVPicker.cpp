//
//  HSVPicker.cpp
//  PylonDetection
//
//  Created by Paul on 21.02.15.
//  Copyright (c) 2015 Paul Bergmann. All rights reserved.
//

#include "HSVPicker.h"

void on_trackbar(int pos,void*)
{
    cout << "POS: " << pos << endl;
}

void HSVPicker::run()
{
    VideoCapture cap(0);
    if(!cap.isOpened())
    {
        cout << "cant open camera" << endl;
        return;
    }
    
    namedWindow("Control",WINDOW_AUTOSIZE);
    
    int h_low  = 0;
    int h_high = 179;
    
    int s_low = 0;
    int s_high = 255;
    
    int v_low = 0;
    int v_high = 255;
    
    //create trackbars in control window
    
    createTrackbar("LowH", "Control", &h_low, 255,on_trackbar); //Hue (0 - 179)
    createTrackbar("HighH", "Control", &h_high, 255,on_trackbar);
    
    createTrackbar("LowS", "Control", &s_low, 255,on_trackbar); //Saturation (0 - 255)
    createTrackbar("HighS", "Control", &s_high , 255,on_trackbar);
    
    createTrackbar("LowV", "Control", &v_low, 255,on_trackbar); //Value (0 - 255)
    createTrackbar("HighV", "Control", &v_high, 255,on_trackbar);
    
    while(true)
    {
        Mat input_image;
        bool success_reading = cap.read(input_image);
        if(!success_reading)
        {
            cout << "error capturing from video" << endl;
            return;
        }
        
        resize(input_image, input_image, Size(640,480));
        
        Mat hsv_image;
        cvtColor(input_image, hsv_image, COLOR_BGR2HSV);
        
        Mat thresh_image;
        
        inRange(hsv_image, Scalar(h_low, s_low, v_low), Scalar(h_high, s_high, v_high), thresh_image); //Threshold the image
        
        erode(thresh_image,thresh_image,getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
        dilate(thresh_image,thresh_image,getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
        
        dilate(thresh_image,thresh_image,getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
        erode(thresh_image,thresh_image,getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
        
        imshow("Thresholded:",thresh_image);
        imshow("Original:",input_image);
        
        if(waitKey(30) == 27)
        {
            cout << "ESC" << endl;
            return;
        }
        
        
    }
    
    return;
}