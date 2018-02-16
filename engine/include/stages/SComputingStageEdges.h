

#pragma once

#include <SComputingStage.h>

#define EDGES_STAGE_SCALE 1
#define EDGES_STAGE_DELTA 0

namespace calibcv
{

    class SComputingStageEdges : public SComputingStage
    {

        protected :

        void _run( const cv::Mat& input ) override;

        public :

        SComputingStageEdges();
        ~SComputingStageEdges();

    };

}
