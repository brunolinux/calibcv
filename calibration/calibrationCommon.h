
#pragma once

#include <iostream>
#include <algorithm>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;

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

    struct DetectionInfo
    {
        vector< cv::Point2f > roi;
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