
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

        bool m_enabled;
        bool m_success;

        public :

        SComputingStage();
        ~SComputingStage();

        void setOriginalFrame( const cv::Mat& originalFrame ) { m_originalFrame = originalFrame; }
        void begin( const cv::Mat& frame ) { m_frame = frame; }

        virtual void grabParamsFromParent( SComputingStage* parent ) {}
        virtual void reset() {}
        void run( const cv::Mat& input );

        void setTimeCost( float cost ) { m_timeCost = cost; }
        float getTimeCost() { return m_timeCost; }
        cv::Mat getStageResult() { return m_stageResult; }

        bool isEnabled() { return m_enabled; }

        virtual bool success() { return m_success; }
    };




}
