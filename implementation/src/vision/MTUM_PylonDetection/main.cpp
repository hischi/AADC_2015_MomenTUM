//
//  main.cpp
//  PylonDetection
//
//  Created by Paul on 17.02.15.
//  Copyright (c) 2015 Paul Bergmann. All rights reserved.
//

#include <iostream>
#include "PylonDetector.h"
#include "HSVPicker.h"

//opencv includes
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>

using namespace std;
using namespace cv;

int main(int argc, const char * argv[]) {
  
  // HSVPicker picker;
    //picker.run();
    
    
    
    VideoCapture cap(0);
    if(!cap.isOpened())
    {
        cout << "cant open camera" << endl;
        return -1;
    }

    PylonDetector d;
    Mat input_image;
    
    while(true)
    {
        bool success_reading = cap.read(input_image);
        if(!success_reading)
        {
            cout << "error capturing from video" << endl;
            return -1;
        }
        resize(input_image, input_image, Size(640,480));
        
        d.detect(input_image);
    }

    

    return 0;
}
