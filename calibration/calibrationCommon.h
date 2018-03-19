
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
#define TAG_FRAMES_IN_CALIBRATION "FramesInCalibration"
#define TAG_CALIBRATION_ERROR_RMS "CalibrationRMSerror"
#define TAG_CALIBRATION_ERROR_REPROJECTION "CalibrationReprojectionError"
#define TAG_CALIBRATION_ERROR_COLINEARITY_OLD "CalibrationColinearityErrorOld"
#define TAG_CALIBRATION_ERROR_COLINEARITY_NEW "CalibrationColinearityErrorNew"
#define TAG_CAMERA_MATRIX "CameraMatrix"
#define TAG_DISTORTION_COEFFICIENTS "DistortionCoefficients"

#define TAG_CALIBRATION_FRAME_PATTERN_IMAGE "CalibrationFramePatternImage"
#define TAG_CALIBRATION_FRAME_PATTERN_WORLD "CalibrationFramePatternWorld"
#define TAG_CALIBRATION_FRAME_ERROR_RMS "CalibrationFrameRMSerror"
#define TAG_CALIBRATION_FRAME_ERROR_COLINEARITY "CalibrationFrameColinearityError"
#define TAG_CALIBRATION_FRAME_EXT_ROTATION "CalibrationFrameExtRotation"
#define TAG_CALIBRATION_FRAME_EXT_TRANSLATION "CalibrationFrameExtTranslation"

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

        string getTypeString()
        {
            if ( type == PATTERN_TYPE_CHESSBOARD )
            {
                return string( "chessboard" );
            }
            else if ( type == PATTERN_TYPE_SYMMETRIC_CIRCLES )
            {
                return string( "symmetric_circles" );
            }
            else if ( type == PATTERN_TYPE_ASYMMETRIC_CIRCLES )
            {
                return string( "asymmetric_circles" );
            }
            else if ( type == PATTERN_TYPE_CONCENTRIC_CIRCLES )
            {
                return string( "concentric" );
            }

            return string( "unknown" );
        }
    };

    struct DetectionInfo
    {
        vector< cv::Point2f > roi;
    };

    struct CalibrationBucket
    {
        int videoIndx;          // to grab from the video
        cv::Mat rotation;       // rotation in rodriguez form
        cv::Mat translation;    // translation in 3d
        vector< cv::Point2f > points; // pattern control points
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