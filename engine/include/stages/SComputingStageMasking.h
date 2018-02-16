
#pragma once

#include <SComputingStage.h>

#define MASKING_STAGE_MAX_VALUE 255
#define MASKING_STAGE_BLOCKSIZE 41
#define MASKING_STAGE_C         15

namespace calibcv
{

    class SComputingStageMasking : public SComputingStage
    {

        protected :

        void _run( const cv::Mat& input ) override;

        public :

        SComputingStageMasking();
        ~SComputingStageMasking();

    };

}
