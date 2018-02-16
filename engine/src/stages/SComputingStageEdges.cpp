
#include <stages/SComputingStageEdges.h>

namespace calibcv
{


    SComputingStageEdges::SComputingStageEdges()
        : SComputingStage()
    {

    }

    SComputingStageEdges::~SComputingStageEdges()
    {

    }


    void SComputingStageEdges::_run( const cv::Mat& input )
    {
        cv::Mat _gradX, _gradY, _gradAbsX, _gradAbsY;

        cv::Scharr( input, _gradX, CV_16S, 1, 0,
                    EDGES_STAGE_SCALE, EDGES_STAGE_DELTA, cv::BORDER_DEFAULT );
        cv::convertScaleAbs( _gradX, _gradAbsX );

        cv::Scharr( input, _gradY, CV_16S, 0, 1,
                    EDGES_STAGE_SCALE, EDGES_STAGE_DELTA, cv::BORDER_DEFAULT );
        cv::convertScaleAbs( _gradY, _gradAbsY );

        cv::addWeighted( _gradAbsX, 0.5, _gradAbsY, 0.5, 0, m_stageResult );
    }
}
