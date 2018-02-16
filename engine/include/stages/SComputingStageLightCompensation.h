

#pragma once

#include <SComputingStage.h>

namespace calibcv
{

    class SComputingStageLightCompensation : public SComputingStage
    {
        protected :

        void _run( const cv::Mat& input ) override;

        public :

        SComputingStageLightCompensation();
        ~SComputingStageLightCompensation();

    };

}
