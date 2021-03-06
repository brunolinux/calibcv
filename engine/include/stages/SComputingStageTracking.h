
#pragma once

#include <SComputingStage.h>

using namespace std;

#define WINDOW_RECT_DELTA 40

namespace calibcv
{

    enum _trackingMode
    {
        TRACKING_MODE_DISABLED,
        TRACKING_MODE_FULL_TRACKING,
        TRACKING_MODE_RECOVERING
    };

    struct STrackingPoint
    {
        cv::Point2f pos;
        cv::Point2f vel;
        bool found;
    };

    static cv::Point2f s_detectionPoint = cv::Point2f( 0, 0 );

    float dist( cv::Point2f start, cv::Point2f end );

    class SComputingStageTracking : public SComputingStage
    {

        private :

        vector< cv::KeyPoint > m_detectedKeypoints;

        vector< STrackingPoint > m_trackingPoints;

        _trackingMode m_trackingMode;

        cv::Point2f m_cropOrigin;
        cv::Rect2f m_cropROI;

        void _estimateLostPattern();
        void _estimateLostSingle( int indx );

        protected :

        void _run( const cv::Mat& input ) override;

        public :

        SComputingStageTracking();
        ~SComputingStageTracking();

        void setTrackingMode( _trackingMode tmode ) { m_trackingMode = tmode; }
        _trackingMode trackingMode() { return m_trackingMode; }

        void grabParamsFromParent( SComputingStage* parent ) override;

        void initialize( vector< cv::KeyPoint > positions );
        void setDetectedKeypoints( const vector< cv::KeyPoint >& keypoints );
        void getCroppedByWindow( const cv::Mat& src, cv::Mat& dst );

        void reset() override;
    };




}
