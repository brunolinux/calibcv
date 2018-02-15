
#pragma once

#include "SUtils.h"

namespace calibcv
{

    /**
    * @brief : generic computing stage for pipeline integration
    */
    class SComputingStage
    {

        protected :

        float m_timeCost;
        cv::Mat m_stageResult;
        cv::Mat m_frame;
        cv::Mat m_originalFrame;

        virtual void _run( const cv::Mat& input ) {}
        virtual void _run( SComputingStage* parent ) {}

        public :

        SComputingStage();
        ~SComputingStage();

        void setOriginalFrame( const cv::Mat& originalFrame ) { m_originalFrame = originalFrame; }
        void begin( const cv::Mat& frame ) { m_frame = frame; }

        void run( const cv::Mat& input );
        void run( SComputingStage* parent );

        void setTimeCost( float cost ) { m_timeCost = cost; }
        float getTimeCost() { return m_timeCost; }
        cv::Mat getStageResult() { return m_stageResult; }

    };




}