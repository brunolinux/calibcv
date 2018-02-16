
#include <SComputingStage.h>


namespace calibcv
{


    SComputingStage::SComputingStage()
    {
        m_timeCost = 0;
        m_enabled = true;
    }

    SComputingStage::~SComputingStage()
    {

    }

    void SComputingStage::run( const cv::Mat& input )
    {
        SCpuTimer::INSTANCE->start();
        if ( m_enabled )
        {
            _run( input );
        }
        else
        {
            m_stageResult = input;
        }

        m_timeCost = SCpuTimer::INSTANCE->stop();
    }

}
