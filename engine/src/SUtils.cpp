
#include <SUtils.h>
#include <cstring>

using namespace std;


namespace calibcv
{

    void imgRgb2cvMat( cv::Mat& dst, const cam::SImageRGB& rgbImage )
    {
        cam::u8* _buff = new cam::u8[ rgbImage.size ];
        memcpy( _buff, rgbImage.data, rgbImage.size );

        dst = cv::Mat( rgbImage.height, rgbImage.width, CV_8UC3,
                       _buff, cv::Mat::AUTO_STEP );
    }


}