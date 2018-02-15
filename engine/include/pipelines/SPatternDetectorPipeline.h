
#pragma once

#include <SComputingPipeline.h>

#include <stages/SComputingStageMasking.h>
#include <stages/SComputingStageEdges.h>
#include <stages/SComputingStageFeatures.h>
#include <stages/SComputingStageTracking.h>

#include <panels/SPatternDetectorPanel.h>

using namespace std;

namespace calibcv
{

    static cv::Point2f s_middle = cv::Point2f( 0, 0 );

    bool comparator1( cv::KeyPoint a, cv::KeyPoint b );
    bool comparator2( cv::KeyPoint a, cv::KeyPoint b );
    bool sort3( cv::KeyPoint &a, cv::KeyPoint &b, cv::KeyPoint &c );

    float distanceToLine( cv::Point2f start, cv::Point2f end, cv::Point2f from );
    float distanceToPoint( cv::Point2f start, cv::Point2f end );
    float computeBorders( vector< cv::KeyPoint > &keypoints, cv::KeyPoint &st, cv::KeyPoint &en, cv::KeyPoint &minUp, cv::KeyPoint &minDown );
    float computeLines( vector< cv::KeyPoint > &keypoints, cv::KeyPoint &st, cv::KeyPoint &en, cv::KeyPoint &minUp, cv::KeyPoint &minMid, cv::KeyPoint &minDown );

    class SPatternDetectorPipeline : public SComputingPipeline
    {

        private :

        void _computeInitialPattern( vector< cv::KeyPoint >& keypoints );

        protected :

        void _preProcessing( const cv::Mat& input, cv::Mat& output ) override;
        void _postProcessing( SComputingStage* lastStage ) override;
        void _runInitialCalibration( const cv::Mat& input, const SPipelineParams& params ) override;

        public :

        SPatternDetectorPipeline();
        ~SPatternDetectorPipeline();

        
    };




}