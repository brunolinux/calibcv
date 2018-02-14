
#pragma once

#include "../Common.h"

#include <camcore/SCommon.h>


namespace calibcv
{

    void imgRgb2cvMat( cv::Mat& dst, const cam::SImageRGB& rgbImage );


}