
#include <SComputingStage.h>


namespace calibcv
{


    SComputingStage::SComputingStage()
    {
        m_timeCost = 0;
    }

    SComputingStage::~SComputingStage()
    {
        
    }

    void SComputingStage::run( const cv::Mat& input )
    {
        _run( input );
    }

    void SComputingStage::run( SComputingStage* parent )
    {
        _run( parent );
    }

}