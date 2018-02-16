
#include <stages/SComputingStageMasking.h>


namespace calibcv
{

    SComputingStageMasking::SComputingStageMasking()
        : SComputingStage()
    {

    }

    SComputingStageMasking::~SComputingStageMasking()
    {

    }

    void SComputingStageMasking::_run( const cv::Mat& input )
    {
        cv::Mat _grayScale;

        cv::cvtColor( input, _grayScale, CV_RGB2GRAY );
        cv::adaptiveThreshold( _grayScale, m_stageResult, MASKING_STAGE_MAX_VALUE,
                               cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV,
                               MASKING_STAGE_BLOCKSIZE, MASKING_STAGE_C );
    }

}
