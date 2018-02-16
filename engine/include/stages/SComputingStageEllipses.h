
#pragma once

#include <SComputingStage.h>

#define ELLIPSE_MIN_SIZE 15
#define ELLIPSE_MAX_SIZE 52
#define ELLIPSE_MIN_RATIO 0.6
#define ELLIPSE_MAX_RATIO 1.3

using namespace std;

namespace calibcv
{

    class SComputingStageEllipses : public SComputingStage
    {
        private :

        vector< cv::KeyPoint > m_keypoints;

        protected :

        void _run( const cv::Mat& input ) override;

        public :

        SComputingStageEllipses();
        ~SComputingStageEllipses();

        vector< cv::KeyPoint > getKeypoints() { return m_keypoints; }

    };





}
