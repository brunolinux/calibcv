
#pragma once

#include "SComputingStage.h"

using namespace std;

namespace calibcv
{

    struct SPipelineParams
    {
        cv::Rect2i roi;
    };


    class SComputingPipeline
    {

        protected :

        vector< SComputingStage* > m_stages;

        float m_totalCost;

        cv::Mat m_matInput;
        cv::Mat m_matOutput;
        cv::Mat m_matResult;

        virtual void _preProcessing( const cv::Mat& input, cv::Mat& output );
        virtual void _postProcessing( SComputingStage* lastStage );
        virtual void _runInitialCalibration( const cv::Mat& input, const SPipelineParams& params );

        virtual void _runPipeline( const cv::Mat& input, const SPipelineParams& params );

        bool m_isInitializing;

        public :

        SComputingPipeline();
        ~SComputingPipeline();

        void addStage( SComputingStage* stage );

        void run( const cv::Mat& input, const SPipelineParams& params );

        float getTotalCost() { return m_totalCost; }

    };




}