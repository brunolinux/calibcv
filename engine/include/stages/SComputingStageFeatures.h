
#pragma once

#include <SComputingStage.h>

using namespace std;

namespace calibcv
{

    class SComputingStageFeatures : public SComputingStage
    {

        protected :

        vector< cv::KeyPoint > m_keypoints;
        cv::Ptr< cv::SimpleBlobDetector > m_blobsDetector;

        void _run( const cv::Mat& input ) override;

        public :

        SComputingStageFeatures();
        ~SComputingStageFeatures();

        vector< cv::KeyPoint > getKeypoints() { return m_keypoints; }

    };





}
