
#include <SComputingPipeline.h>

using namespace std;

namespace calibcv
{


    SComputingPipeline::SComputingPipeline()
    {
        m_totalCost = 0;
        m_isInitializing = true;
    }

    SComputingPipeline::~SComputingPipeline()
    {
        for ( SComputingStage* _stage : m_stages )
        {
            delete _stage;
        }

        m_stages.clear();
    }

    void SComputingPipeline::addStage( SComputingStage* stage )
    {
        m_stages.push_back( stage );
    }

    void SComputingPipeline::_preProcessing( const cv::Mat& input, cv::Mat& output )
    {
        std::cout << "DOING PRE-PROCESSING" << std::endl;
    }

    void SComputingPipeline::_postProcessing( SComputingStage* lastStage )
    {
        std::cout << "DOING POST-PROCESSING" << std::endl;
    }

    void SComputingPipeline::_runInitialCalibration( const cv::Mat& input, const SPipelineParams& params )
    {
        // std::cout << "running calibration" << std::endl;
    }

    void SComputingPipeline::_runPipeline( const cv::Mat& input, const SPipelineParams& params )
    {
        // std::cout << "running pipeline" << std::endl;
        cv::Mat _pInput;

        for ( auto _stage : m_stages )
        {
            _stage->setOriginalFrame( input );
        }

        _preProcessing( input, _pInput );

        for ( int q = 0; q < m_stages.size(); q++ )
        {
            m_stages[q]->begin( _pInput );
        }

        m_matInput = _pInput;
        SCpuTimer::INSTANCE->start();
        m_stages[0]->run( _pInput );
        m_totalCost = SCpuTimer::INSTANCE->stop();
        m_stages[0]->setTimeCost( m_totalCost );

        for ( int q = 1; q < m_stages.size(); q++ )
        {
            SCpuTimer::INSTANCE->start();
            m_stages[q]->run( m_stages[ q - 1 ] );
            float _cost = SCpuTimer::INSTANCE->stop();
            m_stages[q]->setTimeCost( m_totalCost );
            m_totalCost += _cost;
        }

        m_matOutput = m_stages.back()->getStageResult();

        _postProcessing( m_stages.back() );
    }

    void SComputingPipeline::run( const cv::Mat& input, const SPipelineParams& params  )
    {
        if ( m_isInitializing )
        {
            _runInitialCalibration( input, params );
        }
        else
        {
            _runPipeline( input, params );
        }
    }

}