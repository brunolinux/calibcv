
#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace calibration
{

    enum
    {  
        PATTERN_TYPE_CHESSBOARD = 0,
        PATTERN_TYPE_SYMMETRIC_CIRCLES = 1,
        PATTERN_TYPE_ASYMMETRIC_CIRCLES = 2,
        PATTERN_TYPE_CONCENTRIC_CIRCLES = 3
    };
    
    struct PatternInfo
    {
        int type;
        
        float cb_squareLength;
        cv::Size cb_size;
    };


    struct CalibrationData
    {
    	cv::Mat cameraMatrix;
    	cv::Mat distanceCoeffMatrix;

    	vector< cv::Mat > frames;
    	vector< vector< cv::Point2f > > allCorners;
    };
}