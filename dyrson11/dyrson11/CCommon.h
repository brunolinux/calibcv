
#pragma once

#define KEY_ESCAPE 27
#define KEY_SPACE 32

// #define KEY_ESCAPE 1048603
// #define KEY_SPACE 1048608

#include <vector>
#include <opencv2/opencv.hpp>

namespace calibcv
{

    typedef std::vector< cv::Point > CTypeContour;
    typedef std::vector< std::vector< cv::Point > > CTypeContours;
    typedef std::vector< cv::Vec4i > CTypeHierarchy;

}