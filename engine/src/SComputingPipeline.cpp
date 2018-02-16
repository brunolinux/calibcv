
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
        m_stages[0]->run( _pInput );
        m_totalCost = m_stages[0]->getTimeCost();

        for ( int q = 1; q < m_stages.size(); q++ )
        {
            m_stages[q]->grabParamsFromParent( m_stages[ q - 1 ] );
            m_stages[q]->run( m_stages[ q - 1 ]->getStageResult() );
            m_totalCost += m_stages[q]->getTimeCost();
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
