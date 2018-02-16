
#pragma once

#include "../Common.h"

#include <camcore/SCommon.h>
#include <chrono>

using namespace std;

namespace calibcv
{

    void imgRgb2cvMat( cv::Mat& dst, const cam::SImageRGB& rgbImage );

    class SCpuTimer
    {

        private :

        chrono::high_resolution_clock::time_point m_timeStart;
        chrono::high_resolution_clock::time_point m_timeStop;

        SCpuTimer();

        public :

        static SCpuTimer* create();
        static SCpuTimer* INSTANCE;

        void start();
        float stop();

    };

}
