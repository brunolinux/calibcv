
#pragma once

#include <iostream>
#include <algorithm>
#include <fstream>
#include <string>
#include <cassert>
#include <vector>
#include <chrono>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;

#define TAG_FRAME_WIDTH "FrameWidth"
#define TAG_FRAME_HEIGHT "FrameHeight"
#define TAG_CALIBRATION_ERROR_RMS "CalibrationRMSerror"
#define TAG_CALIBRATION_ERROR_REPROJECTION "CalibrationReprojectionError"
#define TAG_CALIBRATION_ERROR_COLINEARITY_OLD "CalibrationColinearityErrorOld"
#define TAG_CALIBRATION_ERROR_COLINEARITY_NEW "CalibrationColinearityErrorNew"
#define TAG_CAMERA_MATRIX "CameraMatrix"
#define TAG_DISTORTION_COEFFICIENTS "DistortionCoefficients"

#define DIST_VIS_X_DIV 20
#define DIST_VIS_Y_DIV 20

#define VIZ_CALIB_TYPE_SIMPLE 0
#define VIZ_CALIB_TYPE_REFINED 1

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
        cv::Size size;
    };

    struct DetectionInfo
    {
        vector< cv::Point2f > roi;
        // To use in the refination step
        bool useRefining;
        cv::Mat cameraMatrix;
        cv::Mat distortionCoefficients;
    };

    class CpuTimer
    {

        private :

        chrono::high_resolution_clock::time_point m_timeStart;
        chrono::high_resolution_clock::time_point m_timeStop;

        public :

        CpuTimer()
        {
            start();
        }

        void start()
        {
            m_timeStart = chrono::high_resolution_clock::now();
        }

        float stop()
        {
            m_timeStop = chrono::high_resolution_clock::now();
            chrono::duration< float > _delta = chrono::duration_cast< chrono::duration< float > >( m_timeStop - m_timeStart );

            return _delta.count();
        }

    };
}